import { ref, computed } from 'vue'

export enum WsState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  RECONNECTING = 'reconnecting'
}

interface WsConnectionState {
  state: WsState
  connected: boolean
  attempt: number
  reason: string
}

interface WsRequestCallback {
  resolve: (value: any) => void
  reject: (reason: Error) => void
  timeoutTimer: number
  action: string
}

// 全局 WebSocket 状态
let ws: WebSocket | null = null
let wsReconnectTimer: number | null = null
let wsReconnectAttempts = 0
let wsReconnectEnabled = true
let wsActiveConnectionId = 0
let wsInitialized = false

const wsRequestCallbacks = new Map<number, WsRequestCallback>()
let wsRequestIdCounter = 1

const wsEventListeners = new Map<string, Array<(data: any) => void>>()

const WS_RECONNECT_MIN_DELAY = 1000
const WS_RECONNECT_MAX_DELAY = 30000
const WS_RECONNECT_FACTOR = 2

// 连接状态必须是模块级单例：之前放在 useWebSocket() 内部，每个组件各拿一份
// 互不相干的状态，只有调用 connectWebSocket() 的那个组件会更新它，
// 于是 LogWindow 之类的组件永远显示“已断开”。
const connectionState = ref<WsConnectionState>({
  state: WsState.DISCONNECTED,
  connected: false,
  attempt: 0,
  reason: ''
})

export function useWebSocket() {
  const isConnected = computed(() => connectionState.value.connected)

  /**
   * 等待连接就绪（子组件的 onMounted 会早于 App 的 connectWebSocket 执行，
   * 直接发送会拿到“WebSocket 未连接”）
   */
  function waitForConnection(timeout: number): Promise<void> {
    return new Promise((resolve, reject) => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        resolve()
        return
      }

      // 还没有开始连接（例如组件先于 App 挂载）就主动发起一次
      if (!ws) {
        connectWebSocket()
      }

      const startedAt = Date.now()
      const timer = window.setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
          window.clearInterval(timer)
          resolve()
        } else if (Date.now() - startedAt > timeout) {
          window.clearInterval(timer)
          reject(new Error('WebSocket 未连接'))
        }
      }, 100)
    })
  }

  /**
   * 发送 WebSocket 请求
   */
  function sendWsRequest<T = any>(
    action: string,
    data: Record<string, any> = {},
    timeout = 30000
  ): Promise<T> {
    return new Promise((resolve, reject) => {
      if (!ws || ws.readyState !== WebSocket.OPEN) {
        // 连接未就绪时先等（最长 timeout），再重试发送
        waitForConnection(timeout)
          .then(() => sendWsRequest<T>(action, data, timeout))
          .then(resolve, reject)
        return
      }

      const requestId = wsRequestIdCounter++

      const timer = window.setTimeout(() => {
        if (wsRequestCallbacks.has(requestId)) {
          wsRequestCallbacks.delete(requestId)
          reject(new Error('请求超时'))
        }
      }, timeout)

      wsRequestCallbacks.set(requestId, {
        resolve,
        reject,
        timeoutTimer: timer,
        action
      })

      const message = {
        action,
        requestId,
        ...data
      }

      try {
        ws.send(JSON.stringify(message))
      } catch (e) {
        clearTimeout(timer)
        wsRequestCallbacks.delete(requestId)
        reject(e instanceof Error ? e : new Error(String(e)))
      }
    })
  }

  /**
   * 注册事件监听器
   */
  function onWsEvent(action: string, callback: (data: any) => void) {
    if (!wsEventListeners.has(action)) {
      wsEventListeners.set(action, [])
    }
    wsEventListeners.get(action)!.push(callback)
  }

  /**
   * 移除事件监听器
   */
  function offWsEvent(action: string, callback: (data: any) => void) {
    const listeners = wsEventListeners.get(action)
    if (listeners) {
      wsEventListeners.set(
        action,
        listeners.filter(cb => cb !== callback)
      )
    }
  }

  /**
   * 拒绝所有待处理的请求
   */
  function rejectPendingWsRequests(message: string) {
    const error = new Error(message)
    wsRequestCallbacks.forEach((callback, requestId) => {
      clearTimeout(callback.timeoutTimer)
      callback.reject(error)
    })
    wsRequestCallbacks.clear()
  }

  /**
   * 连接 WebSocket
   */
  function connectWebSocket() {
    if (ws && (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN)) {
      return
    }

    const connectionId = ++wsActiveConnectionId
    wsInitialized = true

    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const wsUrl = `${protocol}//${window.location.host}/ws`

    connectionState.value.state = wsReconnectAttempts > 0 ? WsState.RECONNECTING : WsState.CONNECTING
    connectionState.value.connected = false
    connectionState.value.attempt = wsReconnectAttempts

    ws = new WebSocket(wsUrl)

    ws.onopen = () => {
      if (connectionId !== wsActiveConnectionId) return

      console.log('WebSocket 连接成功')
      wsReconnectAttempts = 0
      connectionState.value.state = WsState.CONNECTED
      connectionState.value.connected = true
      connectionState.value.attempt = 0
      connectionState.value.reason = ''

      // 触发连接事件
      const listeners = wsEventListeners.get('connected')
      if (listeners) {
        listeners.forEach(cb => cb({}))
      }
    }

    ws.onmessage = (event) => {
      if (connectionId !== wsActiveConnectionId) return

      try {
        const message = JSON.parse(event.data)

        // 处理请求响应
        if (message.requestId !== undefined) {
          const callback = wsRequestCallbacks.get(message.requestId)
          if (callback) {
            clearTimeout(callback.timeoutTimer)
            wsRequestCallbacks.delete(message.requestId)

            if (message.success) {
              callback.resolve(message.data || {})
            } else {
              callback.reject(new Error(message.error || '请求失败'))
            }
          }
          return
        }

        // 处理事件通知
        if (message.action) {
          const listeners = wsEventListeners.get(message.action)
          if (listeners) {
            listeners.forEach(cb => cb(message.data || message))
          }
        }
      } catch (e) {
        console.error('WebSocket 消息解析失败:', e)
      }
    }

    ws.onerror = (error) => {
      if (connectionId !== wsActiveConnectionId) return
      console.error('WebSocket 错误:', error)
    }

    ws.onclose = (event) => {
      if (connectionId !== wsActiveConnectionId) return

      console.log('WebSocket 连接关闭:', event.code, event.reason)

      connectionState.value.state = WsState.DISCONNECTED
      connectionState.value.connected = false
      connectionState.value.reason = event.reason || `关闭码: ${event.code}`

      rejectPendingWsRequests('连接已关闭')

      // 触发断开事件
      const listeners = wsEventListeners.get('disconnected')
      if (listeners) {
        listeners.forEach(cb => cb({ code: event.code, reason: event.reason }))
      }

      // 自动重连
      if (wsReconnectEnabled && !event.wasClean) {
        scheduleReconnect()
      }
    }
  }

  /**
   * 计划重连
   */
  function scheduleReconnect() {
    if (wsReconnectTimer !== null) return

    wsReconnectAttempts++
    const baseDelay = Math.min(
      WS_RECONNECT_MIN_DELAY * Math.pow(WS_RECONNECT_FACTOR, wsReconnectAttempts - 1),
      WS_RECONNECT_MAX_DELAY
    )
    const delay = baseDelay + Math.random() * 1000

    console.log(`尝试重连 #${wsReconnectAttempts}，${Math.round(delay)}ms 后重试...`)

    wsReconnectTimer = window.setTimeout(() => {
      wsReconnectTimer = null
      connectWebSocket()
    }, delay)
  }

  /**
   * 手动重连
   */
  function reconnectWebSocket() {
    if (wsReconnectTimer !== null) {
      clearTimeout(wsReconnectTimer)
      wsReconnectTimer = null
    }
    wsReconnectAttempts = 0
    connectWebSocket()
  }

  /**
   * 断开连接
   */
  function disconnectWebSocket() {
    wsReconnectEnabled = false
    if (wsReconnectTimer !== null) {
      clearTimeout(wsReconnectTimer)
      wsReconnectTimer = null
    }
    if (ws) {
      ws.close()
      ws = null
    }
  }

  return {
    connectionState,
    isConnected,
    connectWebSocket,
    reconnectWebSocket,
    disconnectWebSocket,
    sendWsRequest,
    onWsEvent,
    offWsEvent
  }
}
