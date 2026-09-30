<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRouter } from 'vue-router'
import Dialog from '@/components/Dialog.vue'
import Icon from '@/components/Icon.vue'
import { useWebSocket } from '@/composables/useWebSocket'
import { useToast } from '@/composables/useToast'
import { showAlert, showConfirm } from '@/composables/useDialog'

const router = useRouter()
const toast = useToast()
const { sendWsRequest, onWsEvent, offWsEvent } = useWebSocket()

interface WifiNetwork {
  ssid: string
  rssi: number
  encryption: number
  channel: number
}

interface WifiStatus {
  mode: string
  ssid: string
  ip: string
  rssi: number
  connected: boolean
}

const wifiStatus = ref<WifiStatus | null>(null)
const networks = ref<WifiNetwork[]>([])
const selectedNetwork = ref<string | null>(null)
const password = ref('')
const showPassword = ref(false)
const scanning = ref(false)
const statusLoading = ref(true)
const showConnectDialog = ref(false)
const saving = ref(false)

type TestState = 'idle' | 'testing' | 'success' | 'error'
const testState = ref<TestState>('idle')
const testMessage = ref('')

// 扫描结果保存在固件端，最长约 30 秒；这里留一点余量做兜底
const SCAN_TIMEOUT_MS = 45000
const TEST_TIMEOUT_MS = 60000

let autoRefreshTimer: number | null = null
let scanTimeoutTimer: number | null = null
let testPollTimer: number | null = null

const selectedNetworkInfo = computed(
  () => networks.value.find(n => n.ssid === selectedNetwork.value) ?? null
)
// encryption === 0 为开放网络，不需要密码
const requiresPassword = computed(() => (selectedNetworkInfo.value?.encryption ?? 1) !== 0)

onMounted(async () => {
  // 扫描/测试结果都是 WebSocket 事件推送（固件在异步任务里完成）
  onWsEvent('wifi/scanResult', handleScanResult)
  onWsEvent('wifi/testResult', handleTestResult)
  await getWifiStatus()
  startAutoRefresh()
})

onUnmounted(() => {
  stopAutoRefresh()
  offWsEvent('wifi/scanResult', handleScanResult)
  offWsEvent('wifi/testResult', handleTestResult)
  clearScanTimeout()
  stopTestPolling()
})

function clearScanTimeout() {
  if (scanTimeoutTimer) {
    clearTimeout(scanTimeoutTimer)
    scanTimeoutTimer = null
  }
}

function stopTestPolling() {
  if (testPollTimer) {
    clearInterval(testPollTimer)
    testPollTimer = null
  }
}

async function getWifiStatus() {
  try {
    wifiStatus.value = await sendWsRequest<WifiStatus>('wifi/getInfo')
  } catch (error) {
    console.error('获取 WiFi 状态失败:', error)
  } finally {
    statusLoading.value = false
  }
}

function startAutoRefresh() {
  stopAutoRefresh()
  autoRefreshTimer = window.setInterval(() => {
    getWifiStatus()
  }, 5000)
}

function stopAutoRefresh() {
  if (autoRefreshTimer) {
    clearInterval(autoRefreshTimer)
    autoRefreshTimer = null
  }
}

async function scanWifi() {
  if (scanning.value) return

  scanning.value = true
  networks.value = []
  selectedNetwork.value = null

  try {
    // 立即返回 {status:"started"}，结果通过 wifi/scanResult 事件推送
    await sendWsRequest('wifi/scan')
    clearScanTimeout()
    scanTimeoutTimer = window.setTimeout(() => {
      scanning.value = false
      scanTimeoutTimer = null
      showAlert('扫描超时，请重试', { type: 'error' })
    }, SCAN_TIMEOUT_MS)
  } catch (error) {
    scanning.value = false
    showAlert('扫描 WiFi 网络失败: ' + errText(error), { type: 'error' })
  }
}

function handleScanResult(payload: any) {
  clearScanTimeout()
  scanning.value = false

  if (Array.isArray(payload)) {
    // 按信号强度排序
    networks.value = [...payload].sort((a, b) => b.rssi - a.rssi)
    if (networks.value.length === 0) {
      toast.info('未发现 WiFi 网络')
    }
  } else if (payload?.message) {
    showAlert('扫描失败: ' + payload.message, { type: 'error' })
  }
}

function selectNetwork(ssid: string) {
  selectedNetwork.value = ssid
  password.value = ''
  testState.value = 'idle'
  testMessage.value = ''
  showConnectDialog.value = true
}

function closeConnectDialog() {
  if (testState.value === 'testing' || saving.value) return
  showConnectDialog.value = false
  selectedNetwork.value = null
  password.value = ''
}

function validateInput(): string | null {
  if (!selectedNetwork.value) return '请选择一个 WiFi 网络'
  if (requiresPassword.value && !password.value) return '请输入密码'
  return null
}

/** 只测试连通性，不保存配置 */
async function testConnection() {
  const invalid = validateInput()
  if (invalid) {
    toast.warning(invalid)
    return
  }

  testState.value = 'testing'
  testMessage.value = '正在测试连接，设备会短暂断开当前网络...'

  try {
    await sendWsRequest('wifi/test', {
      ssid: selectedNetwork.value,
      password: password.value
    })
    startTestPolling()
  } catch (error) {
    const message = errText(error)
    // 设备可能已经因为上一次测试在切换网络：转为轮询结果
    if (message.includes('已在进行中')) {
      startTestPolling()
      return
    }
    finishTest(false, undefined, message)
  }
}

// 测试期间设备会断开 WiFi，事件可能收不到，所以同时轮询 wifi/testStatus
function startTestPolling() {
  stopTestPolling()
  testPollTimer = window.setInterval(async () => {
    try {
      const status = await sendWsRequest<any>('wifi/testStatus')
      if (!status?.running && status?.hasResult) {
        finishTest(!!status.success, status.ip, status.errorMessage)
      }
    } catch {
      /* 设备正在切换网络，忽略本轮 */
    }
  }, 3000)

  window.setTimeout(() => {
    if (testState.value === 'testing') {
      finishTest(false, undefined, '测试超时，请确认密码后重试')
    }
  }, TEST_TIMEOUT_MS)
}

function handleTestResult(payload: any) {
  if (testState.value !== 'testing') return
  finishTest(!!payload?.success, payload?.ip, payload?.errorMessage)
}

function finishTest(success: boolean, ip?: string, errorMessage?: string) {
  stopTestPolling()
  testState.value = success ? 'success' : 'error'
  testMessage.value = success
    ? `连接成功${ip ? `，设备获取到 IP ${ip}` : ''}`
    : `连接失败：${errorMessage || '未知错误'}`
}

/** 保存配置到设备（不测试；测试请用“测试连接”） */
async function saveConnection() {
  const invalid = validateInput()
  if (invalid) {
    toast.warning(invalid)
    return
  }

  const targetSsid = selectedNetwork.value!
  saving.value = true

  try {
    await sendWsRequest('wifi/saveConfig', {
      ssid: targetSsid,
      password: password.value
    })
  } catch (error) {
    saving.value = false
    showAlert('保存配置失败: ' + errText(error), { type: 'error' })
    return
  }

  saving.value = false
  const tested = testState.value === 'success'
  showConnectDialog.value = false
  selectedNetwork.value = null
  password.value = ''
  toast.success(`已保存 ${targetSsid} 的配置`)

  const restart = await showConfirm(
    `WiFi 配置已保存${tested ? '（已通过连接测试）' : '（未测试连接）'}。` +
      '需要重启设备才会用新网络连接，是否立即重启？',
    { title: '重启以生效', type: 'warning', confirmText: '立即重启', cancelText: '稍后' }
  )
  if (restart) {
    try {
      await fetch('/api/system/restart', { method: 'POST' })
      toast.info('设备正在重启...')
    } catch (error) {
      showAlert('重启请求失败: ' + errText(error), { type: 'error' })
    }
  }
}

function errText(error: unknown): string {
  return error instanceof Error ? error.message : String(error)
}


function getSignalStrength(rssi: number): string {
  if (rssi >= -50) return 'excellent'
  if (rssi >= -60) return 'good'
  if (rssi >= -70) return 'fair'
  return 'weak'
}

// 信号强度用同一枚 signal 图标，颜色由 .excellent/.good/.fair/.weak 控制
function getEncryptionIcon(encryption: number): 'lock' | 'unlock' {
  return encryption === 0 ? 'unlock' : 'lock'
}

function getStatusClass(): string {
  if (statusLoading.value) return 'state-muted'
  if (!wifiStatus.value) return 'state-muted'
  if (wifiStatus.value.mode === 'AP') return 'state-warning'
  if (wifiStatus.value.ssid) return 'state-success'
  return 'state-muted'
}

function getStatusText(): string {
  if (statusLoading.value) return '加载中...'
  if (!wifiStatus.value) return '未连接'
  if (wifiStatus.value.mode === 'AP') {
    return `AP模式: ${wifiStatus.value.ssid}`
  }
  if (wifiStatus.value.ssid) {
    return `已连接: ${wifiStatus.value.ssid} (${wifiStatus.value.ip})`
  }
  return '未连接'
}
</script>

<template>
  <main>
    <div class="container">
      <div class="page-header">
        <h1>WiFi 配置</h1>
        <div class="status-badge" :class="getStatusClass()">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
            <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
            <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
            <line x1="12" y1="20" x2="12.01" y2="20"></line>
          </svg>
          <span>{{ getStatusText() }}</span>
        </div>
      </div>

      <!-- 扫描区域 -->
      <div class="section">
        <div class="section-header">
          <h3>WiFi 网络</h3>
          <button @click="scanWifi" :disabled="scanning" class="btn btn-primary">
            <svg v-if="!scanning" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
              <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
              <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
              <line x1="12" y1="20" x2="12.01" y2="20"></line>
            </svg>
            <span class="spinner-icon" v-else></span>
            {{ scanning ? '扫描中...' : '扫描网络' }}
          </button>
        </div>

        <!-- 网络列表 -->
        <div class="wifi-list" v-if="networks.length > 0">
          <div 
            v-for="network in networks" 
            :key="network.ssid"
            class="wifi-item"
            :class="{ selected: selectedNetwork === network.ssid }"
            @click="selectNetwork(network.ssid)"
          >
            <div class="wifi-info">
              <div class="wifi-ssid">
                <Icon :name="getEncryptionIcon(network.encryption)" :size="14" />
                <span>{{ network.ssid }}</span>
              </div>
              <div class="wifi-meta">
                <span class="signal" :class="getSignalStrength(network.rssi)">
                  {{ network.rssi }} dBm
                </span>
                <span class="channel">CH {{ network.channel }}</span>
              </div>
            </div>
            <div class="wifi-signal" :class="getSignalStrength(network.rssi)">
              <Icon name="signal" :size="20" />
            </div>
          </div>
        </div>

        <div v-else-if="scanning" class="scan-progress">
          <div class="spinner"></div>
          <p>正在扫描 WiFi 网络...</p>
        </div>

        <div v-else class="empty-state">
          <div class="empty-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
              <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
              <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
              <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
              <line x1="12" y1="20" x2="12.01" y2="20"></line>
            </svg>
          </div>
          <p class="empty-title">未扫描网络</p>
          <p class="empty-hint">点击上方"扫描网络"按钮来查找附近的 WiFi</p>
        </div>
      </div>

      <!-- 连接弹窗：测试连接 / 保存配置分开 -->
      <Dialog
        :show="showConnectDialog"
        :title="`连接到 ${selectedNetwork ?? ''}`"
        type="confirm"
        extra-text="测试连接"
        :extra-disabled="testState === 'testing' || saving"
        :confirm-disabled="testState === 'testing' || saving"
        :show-cancel="true"
        :cancel-text="'取消'"
        :confirm-text="saving ? '保存中...' : '保存配置'"
        @extra="testConnection"
        @confirm="saveConnection"
        @cancel="closeConnectDialog"
      >
        <div class="connect-dialog">
          <div class="connect-meta">
            <span class="connect-meta-item">
              <Icon :name="getEncryptionIcon(selectedNetworkInfo?.encryption ?? 1)" :size="14" />
              {{ selectedNetworkInfo?.encryption === 0 ? '开放网络' : '加密网络' }}
            </span>
            <span v-if="selectedNetworkInfo" class="connect-meta-item">
              <Icon name="signal" :size="14" />
              {{ selectedNetworkInfo.rssi }} dBm
            </span>
            <span v-if="selectedNetworkInfo" class="connect-meta-item">
              CH {{ selectedNetworkInfo.channel }}
            </span>
          </div>

          <div class="input-group" v-if="requiresPassword">
            <label for="wifi-password">WiFi 密码</label>
            <div class="password-input-wrapper">
              <input
                id="wifi-password"
                :type="showPassword ? 'text' : 'password'"
                v-model="password"
                placeholder="请输入 WiFi 密码"
                :disabled="testState === 'testing' || saving"
                @keyup.enter="testConnection"
              />
              <button
                type="button"
                class="password-toggle"
                @click="showPassword = !showPassword"
              >
                <Icon :name="showPassword ? 'eyeOff' : 'eye'" :size="16" />
              </button>
            </div>
          </div>
          <p v-else class="connect-hint">开放网络，无需密码</p>

          <!-- 测试结果 -->
          <div v-if="testState !== 'idle'" class="test-result" :class="testState">
            <div v-if="testState === 'testing'" class="spinner small"></div>
            <Icon v-else :name="testState === 'success' ? 'success' : 'error'" :size="16" />
            <span>{{ testMessage }}</span>
          </div>

          <p class="connect-hint">
            测试连接只验证密码，不会保存；保存后需重启设备才会使用新网络。
          </p>
        </div>
      </Dialog>

      <!-- 返回按钮 -->
      <div class="back-to-home">
        <router-link to="/" class="back-link">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <line x1="19" y1="12" x2="5" y2="12"></line>
            <polyline points="12 19 5 12 12 5"></polyline>
          </svg>
          返回首页
        </router-link>
      </div>
    </div>
  </main>
</template>

<style scoped>
/* 页头的状态徽标：内联小药丸，图标必须显式给尺寸，
   否则 SVG 会按容器宽度撑开（表现为一个占满整屏的巨大 WiFi 图标）。 */
.status-badge {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  max-width: 100%;
  padding: 0.375rem 0.75rem;
  border: 1px solid var(--color-border);
  border-radius: var(--radius-full);
  background: var(--color-bg-elevated);
  color: var(--color-text-secondary);
  font-size: 0.8125rem;
  font-weight: 600;
}

.status-badge svg {
  width: 1rem;
  height: 1rem;
  flex-shrink: 0;
}

.status-badge span {
  overflow: hidden;
  white-space: nowrap;
  text-overflow: ellipsis;
}

.status-badge.state-success {
  background: var(--color-success-light);
  color: var(--color-success);
  border-color: transparent;
}

.status-badge.state-warning {
  background: var(--color-warning-light);
  color: var(--color-warning);
  border-color: transparent;
}

.status-badge.state-muted {
  background: var(--color-bg-elevated);
  color: var(--color-text-muted);
}

.section {
  margin: var(--spacing-xl) 0;
  padding: var(--spacing-xl);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
}

.section h3 {
  margin-bottom: var(--spacing-lg);
  position: relative;
  padding-left: var(--spacing-md);
}

.section h3::before {
  content: '';
  position: absolute;
  left: 0;
  top: 4px;
  bottom: 4px;
  width: 3px;
  border-radius: var(--radius-full);
  background: var(--color-primary);
}



.wifi-list {
  margin-top: var(--spacing-lg);
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.wifi-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: var(--spacing-md) var(--spacing-lg);
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-md);
  cursor: pointer;
  transition: all 0.2s ease;
}

.wifi-item:hover {
  background: var(--color-bg-hover);
  border-color: var(--color-border-hover);
  box-shadow: var(--shadow-sm);
}

.wifi-item.selected {
  background: var(--color-primary-light);
  border-color: var(--color-primary);
}

.wifi-info {
  flex: 1;
}

.wifi-ssid {
  display: flex;
  align-items: center;
  gap: 0.375rem;
  font-weight: 600;
  color: var(--color-text);
  margin-bottom: 0.25rem;
}

.wifi-ssid .icon {
  color: var(--color-text-muted);
}

.wifi-meta {
  display: flex;
  gap: var(--spacing-md);
  font-size: 0.8125rem;
  color: var(--color-text-muted);
}

.signal.excellent { color: var(--color-success); }
.signal.good { color: var(--color-success); }
.signal.fair { color: var(--color-warning); }
.signal.weak { color: var(--color-danger); }

.wifi-signal {
  display: flex;
  align-items: center;
  color: var(--color-text-muted);
}

.scan-progress {
  text-align: center;
  padding: var(--spacing-xl);
  color: var(--color-text-secondary);
}

.scan-progress p {
  color: var(--color-text);
  font-size: 0.9375rem;
}

.empty-state {
  padding: var(--spacing-lg);
  text-align: center;
}

.empty-icon {
  width: 3.5rem;
  height: 3.5rem;
  margin: 0 auto var(--spacing-sm);
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(6, 182, 212, 0.1);
  border-radius: var(--radius-lg);
}

.empty-icon svg {
  width: 1.75rem;
  height: 1.75rem;
  color: var(--color-primary);
}

.empty-title {
  font-size: 0.9375rem;
  font-weight: 600;
  color: var(--color-text);
  margin: 0 0 var(--spacing-xs);
}

.empty-hint {
  font-size: 0.875rem;
  color: var(--color-text-secondary);
  margin: 0;
}

.spinner {
  width: 3rem;
  height: 3rem;
  margin: 0 auto var(--spacing-md);
  border: 3px solid var(--color-border);
  border-top-color: var(--color-primary);
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

.password-input-wrapper {
  position: relative;
}

.password-toggle {
  position: absolute;
  right: var(--spacing-sm);
  top: 50%;
  transform: translateY(-50%);
  background: none;
  border: none;
  cursor: pointer;
  font-size: 1.25rem;
  padding: var(--spacing-xs);
}

.back-to-home {
  text-align: center;
  margin-top: var(--spacing-2xl);
}

.back-link {
  display: inline-flex;
  align-items: center;
  gap: var(--spacing-sm);
  color: var(--color-primary);
  text-decoration: none;
  font-weight: 500;
  padding: var(--spacing-sm) var(--spacing-md);
  border-radius: var(--radius-sm);
  transition: var(--transition-fast);
}

.back-link:hover {
  background: var(--color-bg-hover);
  text-decoration: none;
}

.back-link svg {
  width: 1rem;
  height: 1rem;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* 连接弹窗 */
.connect-dialog {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
}

.connect-dialog .input-group {
  display: flex;
  flex-direction: column;
  gap: 0.375rem;
}

.connect-dialog .input-group label {
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-text-secondary);
}

.connect-meta {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: var(--spacing-sm);
  color: var(--color-text-muted);
  font-size: 0.8125rem;
}

.connect-meta-item {
  display: inline-flex;
  align-items: center;
  gap: 0.25rem;
}

/* 项之间用间隔点分隔，避免看起来像两行无关文字 */
.connect-meta-item + .connect-meta-item::before {
  content: '';
  width: 3px;
  height: 3px;
  margin-right: var(--spacing-sm);
  border-radius: 50%;
  background: currentColor;
  opacity: 0.5;
}

/* 密码框：右侧给眼睛按钮留位置 */
.connect-dialog .password-input-wrapper input {
  padding-right: 2.5rem;
}

.connect-hint {
  color: var(--color-text-muted);
  font-size: 0.8125rem;
  line-height: 1.5;
  margin: 0;
}

.test-result {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm) var(--spacing-md);
  border-radius: var(--radius-sm);
  font-size: 0.875rem;
}

.test-result.success {
  background: var(--color-success-light);
  color: var(--color-success);
}

.test-result.error {
  background: var(--color-danger-light);
  color: var(--color-danger);
}

.test-result.testing {
  background: var(--color-bg-elevated);
  color: var(--color-text-secondary);
}

.spinner.small {
  width: 14px;
  height: 14px;
  margin: 0;
  border-width: 2px;
}

.btn-group.horizontal {
  margin-top: var(--spacing-xl);
}
</style>
