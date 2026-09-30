<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted, nextTick } from 'vue'
import { useLogger, LogLevel, type LogEntry } from '../composables/useLogger'
import { useWebSocket, WsState } from '../composables/useWebSocket'

const { 
  visibleLogs, 
  allLogs,
  unreadCount, 
  currentFilterLevel,
  clearLogs, 
  clearUnread,
  setFilterLevel,
  getLogLevelLabel,
  getLogClass,
  formatTimestamp,
  normalizeLogTag
} = useLogger()

const { connectionState } = useWebSocket()

const visible = ref(false)
const minimized = ref(false)
const dragging = ref(false)
const position = ref({ x: 0, y: 0 })
const dragOffset = ref({ x: 0, y: 0 })

const logOutputRef = ref<HTMLElement | null>(null)
const logWindowRef = ref<HTMLElement | null>(null)

let lastRenderedSessionId: number | null = null

// 计算可见的未读数
const visibleUnreadCount = computed(() => unreadCount.value)

// 监听日志变化，自动滚动
watch(() => visibleLogs.value.length, async () => {
  if (!visible.value || minimized.value) return
  
  await nextTick()
  const output = logOutputRef.value
  if (output && isNearBottom(output)) {
    scrollToBottom()
  }
})

// 窗口开着时，新日志直接算已读——否则未读数一直累积，得关掉再打开才会清零
watch(() => allLogs.value.length, () => {
  if (visible.value && !minimized.value) {
    clearUnread()
  }
})

// 切换显示/隐藏
function toggle() {
  visible.value = !visible.value
  
  if (visible.value) {
    clearUnread()
    // 显示之后才能量到尺寸，此时按当前视口校正一次位置
    nextTick().then(clampToViewport)
  }
  
  saveState()
}

// 最小化/展开
function minimize() {
  minimized.value = !minimized.value
  nextTick().then(clampToViewport)
  
  // 展开时把期间累积的未读清掉
  if (!minimized.value) {
    clearUnread()
  }
  
  saveState()
}

// 过滤级别变化
function handleFilterChange(event: Event) {
  const target = event.target as HTMLSelectElement
  setFilterLevel(parseInt(target.value))
}

// 清空日志
function handleClearLogs() {
  clearLogs()
}

// 判断是否接近底部
function isNearBottom(element: HTMLElement): boolean {
  const threshold = 24
  return element.scrollHeight - element.scrollTop - element.clientHeight <= threshold
}

// 滚动到底部
function scrollToBottom() {
  const output = logOutputRef.value
  if (output) {
    output.scrollTop = output.scrollHeight
  }
}

// 拖拽相关
function handleMouseDown(event: MouseEvent) {
  if (!logWindowRef.value) return
  
  dragging.value = true
  const rect = logWindowRef.value.getBoundingClientRect()
  dragOffset.value = {
    x: event.clientX - rect.left,
    y: event.clientY - rect.top
  }
}

/** 面板四周至少留出的边距 */
const VIEWPORT_MARGIN = 8

/** 把坐标限制在视口内（面板比视口大时贴左上角） */
function clampPosition(x: number, y: number): { x: number; y: number } {
  const el = logWindowRef.value
  if (!el) return { x, y }

  const width = el.offsetWidth
  const height = el.offsetHeight
  const maxX = Math.max(VIEWPORT_MARGIN, window.innerWidth - width - VIEWPORT_MARGIN)
  const maxY = Math.max(VIEWPORT_MARGIN, window.innerHeight - height - VIEWPORT_MARGIN)

  return {
    x: Math.min(Math.max(x, VIEWPORT_MARGIN), maxX),
    y: Math.min(Math.max(y, VIEWPORT_MARGIN), maxY)
  }
}

/**
 * 窗口尺寸变化（缩放、旋转屏幕、手机地址栏收起）后重新校正位置，
 * 否则面板会停在旧的 left/top 上，跑到视口外看不见。
 */
function clampToViewport() {
  // {0,0} 是“未拖动”的哨兵值：此时靠 CSS 锚定在右下角，不需要干预
  if (position.value.x === 0 && position.value.y === 0) return
  if (!visible.value) return // 隐藏时量不到尺寸，等下次打开再校正

  const el = logWindowRef.value
  if (!el || !el.offsetWidth || !el.offsetHeight) return

  const next = clampPosition(position.value.x, position.value.y)
  if (next.x !== position.value.x || next.y !== position.value.y) {
    position.value = next
    saveState()
  }
}

let resizeTimer: number | null = null

function handleViewportResize() {
  if (resizeTimer !== null) {
    window.clearTimeout(resizeTimer)
  }
  // 连续缩放时只处理最后一次，避免频繁读写布局
  resizeTimer = window.setTimeout(() => {
    resizeTimer = null
    clampToViewport()
  }, 120)
}

function handleMouseMove(event: MouseEvent) {
  if (!dragging.value || !logWindowRef.value) return

  const newX = event.clientX - dragOffset.value.x
  const newY = event.clientY - dragOffset.value.y

  position.value = clampPosition(newX, newY)
}

function handleMouseUp() {
  if (dragging.value) {
    dragging.value = false
    saveState()
  }
}

// 保存状态
function saveState() {
  const state = {
    visible: visible.value,
    minimized: minimized.value,
    position: position.value
  }
  localStorage.setItem('logWindowState', JSON.stringify(state))
}

// 恢复状态
function restoreState() {
  const saved = localStorage.getItem('logWindowState')
  if (saved) {
    try {
      const state = JSON.parse(saved)
      visible.value = state.visible || false
      minimized.value = state.minimized || false
      if (state.position) {
        position.value = state.position
      }
      
      if (visible.value) {
        clearUnread()
      }
    } catch (e) {
      console.error('恢复日志窗口状态失败:', e)
    }
  }
}

// 键盘快捷键
function handleKeyDown(event: KeyboardEvent) {
  // 按 'L' 键切换日志窗口（当没有聚焦在输入框时）
  if (event.key === 'l' || event.key === 'L') {
    const target = event.target as HTMLElement
    if (!['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName)) {
      event.preventDefault()
      toggle()
    }
  }
  
  // 按 Escape 键关闭日志窗口
  if (event.key === 'Escape' && visible.value) {
    visible.value = false
    saveState()
  }
}

// 计算样式
const windowStyle = computed(() => {
  if (position.value.x === 0 && position.value.y === 0) {
    return {}
  }
  return {
    right: 'auto',
    bottom: 'auto',
    left: `${position.value.x}px`,
    top: `${position.value.y}px`
  }
})

// 连接状态文本
const connectionStateText = computed(() => {
  const state = connectionState.value.state
  const stateMap: Record<string, string> = {
    [WsState.CONNECTED]: '已连接',
    [WsState.CONNECTING]: '连接中',
    [WsState.RECONNECTING]: `重连中 #${connectionState.value.attempt}`,
    [WsState.DISCONNECTED]: '已断开'
  }
  return stateMap[state] || '未知'
})

onMounted(() => {
  restoreState()
  document.addEventListener('mousemove', handleMouseMove)
  document.addEventListener('mouseup', handleMouseUp)
  document.addEventListener('keydown', handleKeyDown)
  window.addEventListener('resize', handleViewportResize)
  window.addEventListener('orientationchange', handleViewportResize)
  // 恢复的历史位置可能在更小的窗口里已经越界
  nextTick().then(clampToViewport)
})

onUnmounted(() => {
  document.removeEventListener('mousemove', handleMouseMove)
  document.removeEventListener('mouseup', handleMouseUp)
  document.removeEventListener('keydown', handleKeyDown)
  window.removeEventListener('resize', handleViewportResize)
  window.removeEventListener('orientationchange', handleViewportResize)
  if (resizeTimer !== null) {
    window.clearTimeout(resizeTimer)
    resizeTimer = null
  }
})
</script>

<template>
  <!-- 浮动切换按钮 -->
  <button
    type="button"
    class="log-toggle-btn"
    :data-ws-state="connectionState.state"
    @click="toggle"
    aria-label="显示/隐藏日志"
    :aria-controls="visible ? 'log-float-window' : undefined"
    :aria-expanded="visible"
  >
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
      <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
      <polyline points="14 2 14 8 20 8"></polyline>
      <line x1="16" y1="13" x2="8" y2="13"></line>
      <line x1="16" y1="17" x2="8" y2="17"></line>
      <polyline points="10 9 9 9 8 9"></polyline>
    </svg>
    <span class="log-toggle-status-ring"></span>
    <span v-if="visibleUnreadCount > 0 && !visible" class="log-toggle-badge">
      {{ visibleUnreadCount > 99 ? '99+' : visibleUnreadCount }}
    </span>
  </button>

  <!-- 日志浮窗 -->
  <Teleport to="body">
    <Transition name="log-window">
      <div
        v-if="visible"
        id="log-float-window"
        ref="logWindowRef"
        class="log-float-window"
        :class="{ minimized }"
        :style="windowStyle"
        role="dialog"
        aria-labelledby="log-float-title"
      >
        <div class="log-float-header" @mousedown="handleMouseDown">
        <div class="log-float-title">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
            <path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z"></path>
            <polyline points="14 2 14 8 20 8"></polyline>
          </svg>
          <span id="log-float-title">系统日志</span>
          <span v-if="visibleUnreadCount > 0 && !minimized" class="log-header-unread">
            {{ visibleUnreadCount }} 条未读
          </span>
        </div>
        <div class="log-float-controls">
          <button type="button" class="log-float-minimize-btn" @click.stop="minimize" aria-label="最小化" title="最小化">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <line x1="5" y1="12" x2="19" y2="12"></line>
            </svg>
          </button>
          <button type="button" class="log-float-close-btn" @click.stop="toggle" aria-label="关闭" title="关闭">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>
      </div>

      <div v-if="!minimized" class="log-float-body">
        <div class="log-float-toolbar" role="toolbar" aria-label="日志工具栏">
          <select
            :value="currentFilterLevel"
            @change="handleFilterChange"
            aria-label="日志级别过滤"
          >
            <option value="0">全部</option>
            <option value="1">ERROR</option>
            <option value="2">WARN+</option>
            <option value="3">INFO+</option>
            <option value="4">DEBUG+</option>
            <option value="5">VERBOSE+</option>
          </select>
          <button type="button" @click="handleClearLogs" aria-label="清空日志">清空</button>
          <div class="log-meta">
            <span class="log-meta-chip" :class="`connection-${connectionState.state}`">
              {{ connectionStateText }}
            </span>
            <span class="log-meta-chip">
              {{ visibleLogs.length }} 条日志
            </span>
            <span v-if="visibleUnreadCount > 0" class="log-meta-chip">
              {{ visibleUnreadCount }} 条未读
            </span>
          </div>
        </div>

        <div
          ref="logOutputRef"
          class="log-float-output"
          role="log"
          aria-live="polite"
          aria-atomic="false"
        >
          <template v-for="(log, index) in visibleLogs" :key="index">
            <!-- 设备重启分隔条：区分重启前后的日志 -->
            <div v-if="log.divider" class="log-divider">
              <span class="log-divider-line"></span>
              <span class="log-divider-label">{{ log.message }}</span>
              <span class="log-divider-line"></span>
            </div>
            <div
              v-else
              class="log-entry"
              :class="getLogClass(log.level)"
            >
              <div class="log-entry-meta">
                <span class="log-level-badge">{{ getLogLevelLabel(log.level) }}</span>
                <span class="log-time">{{ formatTimestamp(log.browserTimestamp) }}</span>
                <span v-if="normalizeLogTag(log.tag, getLogLevelLabel(log.level))" class="log-tag">
                  {{ normalizeLogTag(log.tag, getLogLevelLabel(log.level)) }}
                </span>
              </div>
              <div class="log-message">{{ log.message }}</div>
            </div>
          </template>
        </div>
      </div>
    </div>
    </Transition>
  </Teleport>
</template>

<style scoped>
/* 日志窗口动画 */
.log-window-enter-active,
.log-window-leave-active {
  transition: opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1), transform 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.log-window-enter-from,
.log-window-leave-to {
  opacity: 0;
  transform: translateY(0.5rem);
}
</style>
