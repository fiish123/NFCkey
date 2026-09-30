import { ref, computed, watch } from 'vue'
import { useWebSocket } from './useWebSocket'
import { showConfirm } from './useDialog'

export enum LogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3,
  VERBOSE = 4
}

export interface LogEntry {
  id?: number
  sessionId?: number
  level: LogLevel
  tag?: string
  message: string
  timestamp?: number
  browserTimestamp: number
  /** 设备重启分隔条（不是真实日志，仅用于在列表中区分新旧日志） */
  divider?: boolean
}

const LOG_LEVEL_LABELS = ['ERROR', 'WARN', 'INFO', 'DEBUG', 'VERBOSE']
const LOG_MAX_ENTRIES = 500
const LOG_DEDUPE_WINDOW = 1000
const LOG_REPLAY_STORAGE_KEY = 'logLastId'
const LOG_SESSION_STORAGE_KEY = 'logSessionId'
const LOG_HISTORY_STORAGE_KEY = 'logHistorySnapshot'

// 全局状态
const allLogs = ref<LogEntry[]>([])
const unreadLogs = ref<LogEntry[]>([])
const currentFilterLevel = ref<number>(3) // 0=全部, 1=ERROR, 2=WARN+, 3=INFO+, 4=DEBUG+, 5=VERBOSE+

let recentLogIds: string[] = []
const seenLogIds = new Set<string>()
let latestLogId = 0
let activeLogSessionId: number | null = null
let latestDeviceLogTimestamp: number | null = null
let latestBrowserLogTimestamp: number | null = null
let timestampAnchorSessionId: number | null = null

export function useLogger() {
  const { sendWsRequest, onWsEvent, offWsEvent } = useWebSocket()

  // 计算属性
  const visibleLogs = computed(() => {
    // 分隔条始终显示，不受级别过滤影响，否则“新旧日志分割”会消失
    return allLogs.value.filter(
      log => log.divider || shouldShowLog(log.level, currentFilterLevel.value)
    )
  })

  const unreadCount = computed(() => {
    return unreadLogs.value.filter(log => 
      shouldShowLog(log.level, currentFilterLevel.value)
    ).length
  })

  /**
   * 初始化日志系统
   */
  function initLogger() {
    // 从 localStorage 读取过滤级别
    const savedLevel = localStorage.getItem('logFilterLevel')
    if (savedLevel !== null) {
      currentFilterLevel.value = parseInt(savedLevel)
    }

    // 恢复历史日志
    restorePersistedLogHistory()

    // 监听日志事件
    onWsEvent('log', handleLogEvent)
    onWsEvent('connected', handleConnected)
  }

  /**
   * 清理日志系统
   */
  function cleanupLogger() {
    offWsEvent('log', handleLogEvent)
    offWsEvent('connected', handleConnected)
  }

  /**
   * 处理日志事件
   */
  function handleLogEvent(data: any) {
    if (Array.isArray(data)) {
      // 历史日志数组
      data.forEach(log => addLogEntry(log))
    } else if (data.level !== undefined) {
      // 单条日志
      addLogEntry(data)
    }
  }

  /**
   * 处理连接成功事件
   */
  async function handleConnected() {
    // 请求日志回放
    await requestLogReplay()
  }

  /**
   * 请求日志回放
   */
  async function requestLogReplay() {
    try {
      const lastLogId = getStoredLastLogId()
      const lastSessionId = getStoredLogSessionId()

      const replayRequest: any = {}
      if (lastLogId > 0) {
        replayRequest.lastLogId = lastLogId
      }
      if (lastSessionId !== null) {
        replayRequest.lastSessionId = lastSessionId
      }

      const data = await sendWsRequest('log/replay', replayRequest, 10000)
      const replayLogs = data && Array.isArray(data.logs) ? data.logs : []
      const replaySessionId = normalizeLogSessionId(data?.sessionId)

      // 判断设备是否重启过：固件用 sessionChanged 告知，本地再比对一次会话号
      const sessionChanged =
        data?.sessionChanged === true ||
        (replaySessionId !== null &&
          activeLogSessionId !== null &&
          replaySessionId !== activeLogSessionId)

      if (sessionChanged && allLogs.value.length > 0) {
        // 询问是否清空重启前的旧日志
        const clear = await showConfirm(
          '检测到设备已重启，是否清空重启前的旧日志？',
          {
            title: '设备已重启',
            type: 'warning',
            confirmText: '清空旧日志',
            cancelText: '保留'
          }
        )
        if (clear) {
          clearLogs({ preserveReplayCursor: true, preserveSession: true })
        }
        addSessionDivider(replaySessionId)
      } else if (sessionChanged) {
        addSessionDivider(replaySessionId)
      }

      // 添加回放的日志
      replayLogs.forEach((log: any) => {
        if (replaySessionId !== null && normalizeLogSessionId(log.sessionId) === null) {
          log.sessionId = replaySessionId
        }
        addLogEntry(log)
      })
    } catch (error) {
      console.error('请求日志回放失败:', error)
    }
  }

  /**
   * 在日志列表中插入一条“设备已重启”分隔条（不进入未读计数）
   */
  function addSessionDivider(sessionId: number | null) {
    const divider: LogEntry = {
      level: LogLevel.INFO,
      message: sessionId !== null ? '设备已重启 · 以下为新日志' : '设备已重启',
      sessionId: sessionId ?? undefined,
      browserTimestamp: Date.now(),
      divider: true
    }
    allLogs.value.push(divider)
    if (allLogs.value.length > LOG_MAX_ENTRIES) {
      allLogs.value.shift()
    }
    persistLogHistory()
  }

  /**
   * 判断日志是否应该显示
   */
  function shouldShowLog(logLevel: LogLevel, filterLevel: number): boolean {
    // filterLevel: 0=全部, 1=ERROR, 2=WARN+, 3=INFO+, 4=DEBUG+, 5=VERBOSE+
    // logLevel: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG, 4=VERBOSE

    // 0=全部 或 5=VERBOSE+，显示所有日志
    if (filterLevel === 0 || filterLevel === 5) {
      return true
    }

    // filterLevel 1-4 映射到对应级别
    // filterLevel=1 只显示 ERROR (logLevel=0)
    // filterLevel=2 显示 ERROR+WARN (logLevel<=1)
    // filterLevel=3 显示 ERROR+WARN+INFO (logLevel<=2)
    // filterLevel=4 显示 ERROR+WARN+INFO+DEBUG (logLevel<=3)
    return logLevel <= filterLevel - 1
  }

  /**
   * 添加日志条目
   */
  function addLogEntry(logData: any) {
    const fallbackSessionId = activeLogSessionId
    const logSessionId = normalizeLogSession(logData, fallbackSessionId)
    const logIdentity = getLogIdentity(logData)

    // 去重
    if (logIdentity !== null && seenLogIds.has(logIdentity)) {
      return
    }

    // 更新 session
    if (logSessionId !== null && logSessionId !== activeLogSessionId) {
      storeLogSessionId(logSessionId)
      resetLogTimestampAnchor(logSessionId)
    }

    rememberLogId(logData)

    // 解析浏览器时间戳
    logData.browserTimestamp = resolveLogBrowserTimestamp(logData)

    // 存储到数组
    allLogs.value.push(logData)

    // 限制日志数量
    if (allLogs.value.length > LOG_MAX_ENTRIES) {
      allLogs.value.shift()
    }

    // 存储未读日志
    unreadLogs.value.push(logData)
    if (unreadLogs.value.length > LOG_MAX_ENTRIES) {
      unreadLogs.value.shift()
    }

    // 持久化
    persistLogHistory()
  }

  /**
   * 清空未读日志
   */
  function clearUnread() {
    unreadLogs.value = []
  }

  /**
   * 清空所有日志
   */
  function clearLogs(options: { 
    preserveReplayCursor?: boolean
    preserveSession?: boolean 
  } = {}) {
    const preserveReplayCursor = options.preserveReplayCursor !== false
    const preserveSession = options.preserveSession !== false

    allLogs.value = []
    unreadLogs.value = []
    recentLogIds = []
    seenLogIds.clear()

    if (!preserveSession) {
      storeLogSessionId(null)
    }

    resetLogTimestampAnchor(preserveSession ? activeLogSessionId : null)

    if (preserveReplayCursor && latestLogId > 0) {
      storeLastLogId(latestLogId)
    } else {
      latestLogId = 0
      sessionStorage.removeItem(LOG_REPLAY_STORAGE_KEY)
    }

    sessionStorage.removeItem(LOG_HISTORY_STORAGE_KEY)

    console.log('日志已清空')
  }

  /**
   * 设置过滤级别
   */
  function setFilterLevel(level: number) {
    currentFilterLevel.value = level
    localStorage.setItem('logFilterLevel', String(level))
  }

  /**
   * 获取日志级别标签
   */
  function getLogLevelLabel(level: LogLevel): string {
    return LOG_LEVEL_LABELS[level] || 'INFO'
  }

  /**
   * 获取日志级别 CSS 类名
   */
  function getLogClass(level: LogLevel): string {
    switch (level) {
      case LogLevel.ERROR: return 'error'
      case LogLevel.WARN: return 'warn'
      case LogLevel.INFO: return 'info'
      case LogLevel.DEBUG: return 'debug'
      case LogLevel.VERBOSE: return 'verbose'
      default: return 'info'
    }
  }

  /**
   * 格式化时间戳
   */
  function formatTimestamp(timestamp: number): string {
    if (!timestamp) return ''

    const date = new Date(timestamp)
    const hours = String(date.getHours()).padStart(2, '0')
    const minutes = String(date.getMinutes()).padStart(2, '0')
    const seconds = String(date.getSeconds()).padStart(2, '0')

    return `${hours}:${minutes}:${seconds}`
  }

  /**
   * 标准化 tag
   */
  function normalizeLogTag(tag?: string, levelLabel?: string): string {
    const normalizedTag = String(tag || '').trim()

    if (!normalizedTag) {
      return 'SYSTEM'
    }

    if (levelLabel && normalizedTag.toUpperCase() === levelLabel) {
      return ''
    }

    return normalizedTag
  }

  // ==================== 内部辅助函数 ====================

  function getStoredLastLogId(): number {
    const rawValue = sessionStorage.getItem(LOG_REPLAY_STORAGE_KEY)
    if (rawValue === null) {
      return 0
    }
    const parsedValue = Number.parseInt(rawValue, 10)
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
      sessionStorage.removeItem(LOG_REPLAY_STORAGE_KEY)
      return 0
    }
    return parsedValue
  }

  function getStoredLogSessionId(): number | null {
    const rawValue = sessionStorage.getItem(LOG_SESSION_STORAGE_KEY)
    if (rawValue === null) {
      return null
    }
    const parsedValue = Number.parseInt(rawValue, 10)
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
      sessionStorage.removeItem(LOG_SESSION_STORAGE_KEY)
      return null
    }
    return parsedValue
  }

  function storeLastLogId(logId: number) {
    if (!Number.isFinite(logId) || logId <= 0) {
      return
    }
    latestLogId = logId
    sessionStorage.setItem(LOG_REPLAY_STORAGE_KEY, String(latestLogId))
  }

  function storeLogSessionId(sessionId: number | null) {
    activeLogSessionId = sessionId
    if (sessionId === null) {
      sessionStorage.removeItem(LOG_SESSION_STORAGE_KEY)
      return
    }
    sessionStorage.setItem(LOG_SESSION_STORAGE_KEY, String(sessionId))
  }

  function resetLogTimestampAnchor(sessionId: number | null = null) {
    timestampAnchorSessionId = sessionId
    latestDeviceLogTimestamp = null
    latestBrowserLogTimestamp = null
  }

  function normalizeLogSessionId(sessionId: any): number | null {
    if (sessionId === undefined || sessionId === null) {
      return null
    }
    const parsedValue = Number.parseInt(sessionId, 10)
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
      return null
    }
    return parsedValue
  }

  function getLogIdentity(logData: any): string | null {
    const logId = normalizeLogId(logData)
    if (logId === null) {
      return null
    }
    const logSessionId = normalizeLogSessionId(logData?.sessionId)
    return logSessionId !== null ? `${logSessionId}:${logId}` : String(logId)
  }

  function normalizeLogSession(logData: any, fallbackSessionId: number | null = activeLogSessionId): number | null {
    if (!logData || typeof logData !== 'object') {
      return fallbackSessionId
    }

    const normalizedSessionId = normalizeLogSessionId(logData.sessionId)
    if (normalizedSessionId !== null) {
      logData.sessionId = normalizedSessionId
      return normalizedSessionId
    }

    if (fallbackSessionId !== null) {
      logData.sessionId = fallbackSessionId
      return fallbackSessionId
    }

    delete logData.sessionId
    return null
  }

  function persistLogHistory() {
    const snapshot = {
      logs: allLogs.value.slice(-LOG_MAX_ENTRIES),
      unreadLogs: unreadLogs.value.slice(-LOG_MAX_ENTRIES)
    }

    try {
      sessionStorage.setItem(LOG_HISTORY_STORAGE_KEY, JSON.stringify(snapshot))
    } catch (error) {
      console.warn('保存日志历史失败:', error)
    }
  }

  function restorePersistedLogHistory() {
    const rawValue = sessionStorage.getItem(LOG_HISTORY_STORAGE_KEY)
    if (rawValue === null) {
      return
    }

    try {
      const snapshot = JSON.parse(rawValue)
      const restoredSessionId = getStoredLogSessionId()
      const restoredLogs = Array.isArray(snapshot?.logs) ? snapshot.logs.slice(-LOG_MAX_ENTRIES) : []
      const restoredUnreadLogs = Array.isArray(snapshot?.unreadLogs) ? snapshot.unreadLogs.slice(-LOG_MAX_ENTRIES) : []

      if (restoredLogs.length === 0) {
        return
      }

      // 标准化 session
      restoredLogs.forEach((logData: any) => {
        normalizeLogSession(logData, restoredSessionId)
        logData.browserTimestamp = resolveLogBrowserTimestamp(logData)
      })

      unreadLogs.value.forEach((logData: any) => {
        normalizeLogSession(logData, restoredSessionId)
        logData.browserTimestamp = resolveLogBrowserTimestamp(logData)
      })

      // 恢复会话号与回放游标，否则重启检测和增量回放都会失效
      if (restoredSessionId !== null) {
        storeLogSessionId(restoredSessionId)
      }
      latestLogId = getStoredLastLogId()

      // 记录已见 ID
      restoredLogs.forEach((logData: any) => {
        rememberLogId(logData)
      })

      allLogs.value = restoredLogs
      unreadLogs.value = restoredUnreadLogs
    } catch (error) {
      console.warn('恢复日志历史失败:', error)
      sessionStorage.removeItem(LOG_HISTORY_STORAGE_KEY)
    }
  }

  function normalizeLogId(logData: any): number | null {
    if (!logData || logData.id === undefined || logData.id === null) {
      return null
    }

    const parsedValue = Number.parseInt(logData.id, 10)
    if (!Number.isFinite(parsedValue) || parsedValue <= 0) {
      return null
    }

    return parsedValue
  }

  function rememberLogId(logData: any) {
    const logId = normalizeLogId(logData)
    if (logId === null) {
      return
    }

    const logIdentity = getLogIdentity(logData)
    if (logIdentity === null || seenLogIds.has(logIdentity)) {
      return
    }

    seenLogIds.add(logIdentity)
    recentLogIds.push(logIdentity)

    if (recentLogIds.length > LOG_DEDUPE_WINDOW) {
      const removed = recentLogIds.shift()
      if (removed) seenLogIds.delete(removed)
    }

    if (normalizeLogSessionId(logData.sessionId) === activeLogSessionId) {
      storeLastLogId(logId)
    }
  }

  function normalizeLogTimestamp(timestamp: any): number | null {
    if (timestamp === undefined || timestamp === null) {
      return null
    }

    const parsedTimestamp = Number(timestamp)
    if (!Number.isFinite(parsedTimestamp) || parsedTimestamp < 0) {
      return null
    }

    return parsedTimestamp
  }

  function resolveLogBrowserTimestamp(logData: any): number {
    const storedBrowserTimestamp = normalizeLogTimestamp(logData.browserTimestamp)
    if (storedBrowserTimestamp !== null) {
      return storedBrowserTimestamp
    }

    const logSessionId = normalizeLogSession(logData, activeLogSessionId)
    const deviceTimestamp = normalizeLogTimestamp(logData.timestamp)
    const browserNow = Date.now()

    if (logSessionId !== null && timestampAnchorSessionId !== logSessionId) {
      resetLogTimestampAnchor(logSessionId)
    }

    if (deviceTimestamp === null) {
      return browserNow
    }

    if (latestDeviceLogTimestamp === null || deviceTimestamp >= latestDeviceLogTimestamp) {
      latestDeviceLogTimestamp = deviceTimestamp
      latestBrowserLogTimestamp = browserNow
    }

    if (latestBrowserLogTimestamp === null || latestDeviceLogTimestamp === null) {
      return browserNow
    }

    return latestBrowserLogTimestamp - (latestDeviceLogTimestamp - deviceTimestamp)
  }

  return {
    allLogs,
    visibleLogs,
    unreadLogs,
    unreadCount,
    currentFilterLevel,
    initLogger,
    cleanupLogger,
    clearLogs,
    clearUnread,
    setFilterLevel,
    getLogLevelLabel,
    getLogClass,
    formatTimestamp,
    normalizeLogTag
  }
}
