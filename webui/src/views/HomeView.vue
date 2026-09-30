<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { useWebSocket } from '@/composables/useWebSocket'
import { showAlert, showConfirm } from '@/composables/useDialog'
import { useToast } from '@/composables/useToast'

const router = useRouter()
const toast = useToast()
const { sendWsRequest } = useWebSocket()

interface SystemStatus {
  wifi: {
    connected: boolean
    ssid: string
    ip: string
    rssi: number
  }
  battery: {
    voltage: number
    percentage: number
  }
}

const status = ref<SystemStatus>({
  wifi: {
    connected: false,
    ssid: '未连接',
    ip: '',
    rssi: 0
  },
  battery: {
    voltage: 0,
    percentage: 0
  }
})

const loading = ref(true)

onMounted(async () => {
  await loadStatus()
})

async function loadStatus() {
  try {
    // WiFi 状态走 WebSocket（与 WiFi 配置页同一份实现，REST 版本已移除）
    try {
      const wifiData = await sendWsRequest<any>('wifi/getInfo')
      status.value.wifi = {
        connected: !!wifiData?.connected,
        ssid: wifiData?.ssid || '未连接',
        ip: wifiData?.ip || '',
        rssi: wifiData?.rssi ?? 0
      }
    } catch (error) {
      console.error('获取 WiFi 状态失败:', error)
    }

    // 加载电池状态
    const batteryRes = await fetch('/api/battery')
    if (batteryRes.ok) {
      const batteryData = await batteryRes.json()
      status.value.battery = batteryData
    }
  } catch (error) {
    console.error('加载状态失败:', error)
  } finally {
    loading.value = false
  }
}

async function restartSystem() {
  const confirmed = await showConfirm('确定要重启系统吗？设备将断开连接约 10 秒。', {
    title: '重启设备',
    type: 'warning',
    confirmText: '重启'
  })
  if (!confirmed) return
  
  try {
    await fetch('/api/system/restart', { method: 'POST' })
    toast.info('系统正在重启...')
  } catch (error) {
    showAlert('重启失败: ' + error, { type: 'error' })
  }
}

function getWifiStateClass() {
  if (loading.value) return 'state-loading'
  return status.value.wifi.connected ? 'state-success' : 'state-danger'
}

function getBatteryStateClass() {
  if (loading.value) return 'state-loading'
  const voltage = status.value.battery.voltage
  if (voltage > 3.4) return 'state-success'
  if (voltage > 3.2) return 'state-warning'
  return 'state-danger'
}

function getWifiStatusText() {
  if (loading.value) return '加载中...'
  return status.value.wifi.connected 
    ? `${status.value.wifi.ssid} (${status.value.wifi.rssi}dBm)` 
    : '未连接'
}

function getBatteryStatusText() {
  if (loading.value) return '加载中...'
  return `${status.value.battery.voltage.toFixed(2)}V (${status.value.battery.percentage}%)`
}
</script>

<template>
  <main>
    <div class="container narrow">
      <!-- Hero 区域 -->
      <header class="hero">
        <div class="hero-content">
          <h1>NFC门禁</h1>
          <p class="hero-subtitle">门禁控制中心 · ESP32-C3</p>
        </div>
      </header>

      <!-- 状态卡片 -->
      <div class="status-grid">
        <div class="status-card" :class="getWifiStateClass()">
          <div class="status-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
              <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
              <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
              <line x1="12" y1="20" x2="12.01" y2="20"></line>
            </svg>
          </div>
          <div class="status-body">
            <div class="status-label">WiFi</div>
            <div class="status-value">{{ getWifiStatusText() }}</div>
          </div>
        </div>

        <div class="status-card" :class="getBatteryStateClass()">
          <div class="status-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <rect x="2" y="7" width="16" height="10" rx="2"></rect>
              <line x1="22" y1="11" x2="22" y2="13"></line>
            </svg>
          </div>
          <div class="status-body">
            <div class="status-label">电池</div>
            <div class="status-value">{{ getBatteryStatusText() }}</div>
          </div>
        </div>
      </div>

      <!-- 主要功能 -->
      <div class="nav-primary">
        <RouterLink to="/wifi" class="nav-card-large">
          <div class="nav-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M5 12.55a11 11 0 0 1 14.08 0"></path>
              <path d="M1.42 9a16 16 0 0 1 21.16 0"></path>
              <path d="M8.53 16.11a6 6 0 0 1 6.95 0"></path>
              <line x1="12" y1="20" x2="12.01" y2="20"></line>
            </svg>
          </div>
          <div class="nav-content">
            <div class="nav-title">WiFi配置</div>
            <div class="nav-desc">网络连接设置</div>
          </div>
        </RouterLink>

        <RouterLink to="/cards" class="nav-card-large">
          <div class="nav-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <rect x="2" y="5" width="20" height="14" rx="2"></rect>
              <line x1="2" y1="10" x2="22" y2="10"></line>
            </svg>
          </div>
          <div class="nav-content">
            <div class="nav-title">卡片管理</div>
            <div class="nav-desc">添加和管理NFC卡</div>
          </div>
        </RouterLink>
      </div>

      <!-- 次要功能 -->
      <div class="nav-secondary">
        <RouterLink to="/servo" class="nav-card">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <circle cx="12" cy="12" r="3"></circle>
            <path d="M12 1v6m0 6v6M5.6 5.6l4.2 4.2m4.4 4.4l4.2 4.2m-12.8 0l4.2-4.2m4.4-4.4l4.2-4.2"></path>
          </svg>
          <span>舵机控制</span>
        </RouterLink>

        <RouterLink to="/files" class="nav-card">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path>
          </svg>
          <span>文件管理</span>
        </RouterLink>

        <RouterLink to="/ota" class="nav-card">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <polyline points="16 16 12 12 8 16"></polyline>
            <line x1="12" y1="12" x2="12" y2="21"></line>
            <path d="M20.39 18.39A5 5 0 0 0 18 9h-1.26A8 8 0 1 0 3 16.3"></path>
            <polyline points="16 16 12 12 8 16"></polyline>
          </svg>
          <span>OTA升级</span>
        </RouterLink>

        <button @click="restartSystem" class="nav-card">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M23 4v6h-6"></path>
            <path d="M20.49 15a9 9 0 1 1-2.12-9.36L23 10"></path>
          </svg>
          <span>重启系统</span>
        </button>
      </div>
    </div>
  </main>
</template>

<style scoped>
.hero {
  position: relative;
  overflow: hidden;
  padding: var(--spacing-xl) var(--spacing-lg);
  border-radius: var(--radius-lg);
  background: linear-gradient(135deg, #0891b2 0%, #06b6d4 50%, #22d3ee 100%);
  box-shadow: 0 10px 30px -10px rgba(6, 182, 212, 0.35);
  color: white;
  margin-bottom: var(--spacing-2xl);
}

.hero::before {
  content: '';
  position: absolute;
  top: -60px;
  right: -50px;
  width: 200px;
  height: 200px;
  background: radial-gradient(circle, rgba(255, 255, 255, 0.28), rgba(255, 255, 255, 0.06) 55%, transparent 70%);
  pointer-events: none;
}

.hero::after {
  content: '';
  position: absolute;
  top: -34px;
  right: -34px;
  width: 104px;
  height: 104px;
  border: 2px solid rgba(255, 255, 255, 0.5);
  border-radius: 50%;
  background: rgba(255, 255, 255, 0.08);
  box-shadow:
    0 0 0 14px rgba(255, 255, 255, 0.1),
    0 0 0 30px rgba(255, 255, 255, 0.06);
  pointer-events: none;
}

.hero-content {
  position: relative;
  z-index: 1;
}

.hero h1 {
  margin: 0 0 var(--spacing-xs);
  color: white;
  font-size: 2rem;
  font-weight: 700;
  letter-spacing: -0.03em;
  line-height: 1.2;
}

.hero-subtitle {
  margin: 0;
  color: rgba(255, 255, 255, 0.92);
  font-size: 0.9375rem;
  font-weight: 500;
  letter-spacing: 0.01em;
}

.status-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
  gap: var(--spacing-lg);
  margin-bottom: var(--spacing-2xl);
}

.status-card {
  display: flex;
  align-items: flex-start;
  gap: var(--spacing-md);
  padding: var(--spacing-lg);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-left: 3px solid var(--color-border);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
  transition: all 0.2s ease;
}

.status-card:hover {
  box-shadow: var(--shadow-md);
  border-color: var(--color-border-hover);
}

.status-card.state-success {
  border-left-color: var(--color-success);
}

.status-card.state-warning {
  border-left-color: var(--color-warning);
}

.status-card.state-danger {
  border-left-color: var(--color-danger);
}

.status-icon {
  flex-shrink: 0;
  width: 3rem;
  height: 3rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-md);
  background: var(--color-bg-elevated);
  color: var(--color-text-muted);
  transition: background 0.2s;
}

.status-card.state-success .status-icon {
  background: rgba(16, 185, 129, 0.12);
  color: var(--color-success);
}

.status-card.state-warning .status-icon {
  background: rgba(245, 158, 11, 0.12);
  color: var(--color-warning);
}

.status-card.state-danger .status-icon {
  background: rgba(239, 68, 68, 0.12);
  color: var(--color-danger);
}

.status-icon svg {
  width: 1.375rem;
  height: 1.375rem;
}

.status-icon svg {
  width: 1.25rem;
  height: 1.25rem;
}

.status-body {
  flex: 1;
  min-width: 0;
}

.status-label {
  font-size: 0.6875rem;
  font-weight: 700;
  color: var(--color-text-muted);
  text-transform: uppercase;
  letter-spacing: 0.08em;
  margin-bottom: 0.375rem;
}

.status-value {
  font-size: 0.9375rem;
  font-weight: 600;
  color: var(--color-text);
  line-height: 1.4;
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
}

.nav-primary {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: var(--spacing-lg);
  margin-bottom: var(--spacing-xl);
}

.nav-card-large {
  display: flex;
  align-items: center;
  gap: var(--spacing-lg);
  padding: var(--spacing-xl);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-lg);
  text-decoration: none;
  box-shadow: var(--shadow-sm);
  transition: all 0.2s;
}

.nav-card-large:hover {
  border-color: var(--color-primary);
  box-shadow: 0 4px 12px rgba(6, 182, 212, 0.15);
  transform: translateY(-2px);
}

.nav-icon {
  flex-shrink: 0;
  width: 3.5rem;
  height: 3.5rem;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--color-primary-light);
  border-radius: var(--radius-lg);
  color: var(--color-primary);
}

.nav-icon svg {
  width: 1.75rem;
  height: 1.75rem;
}

.nav-content {
  flex: 1;
}

.nav-title {
  font-size: 1.125rem;
  font-weight: 600;
  color: var(--color-text);
  margin-bottom: 0.25rem;
}

.nav-desc {
  font-size: 0.875rem;
  color: var(--color-text-muted);
}

.nav-secondary {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(140px, 1fr));
  gap: var(--spacing-md);
}

.nav-card {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-lg);
  min-height: 110px;
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  color: var(--color-text-secondary);
  font-weight: 600;
  font-size: 0.9375rem;
  text-decoration: none;
  transition: all 0.2s ease;
  box-shadow: var(--shadow-xs);
  cursor: pointer;
}

.nav-card:hover {
  background: var(--color-primary-light);
  border-color: var(--color-primary);
  color: var(--color-primary);
  box-shadow: var(--shadow-md);
  transform: translateY(-2px);
  text-decoration: none;
}

.nav-card:active {
  transform: translateY(0) scale(0.98);
}

.nav-card svg {
  width: 2rem;
  height: 2rem;
  stroke-width: 2;
  transition: transform 0.2s ease;
}

.nav-card:hover svg {
  transform: scale(1.1);
}

@media (max-width: 640px) {
  .status-grid {
    grid-template-columns: 1fr;
    gap: var(--spacing-md);
  }
  
  .nav-grid {
    grid-template-columns: repeat(2, 1fr);
    gap: var(--spacing-md);
  }
  
  .hero {
    padding: var(--spacing-lg);
  }
  
  .hero h1 {
    font-size: 1.625rem;
  }
  
  .hero-subtitle {
    font-size: 0.875rem;
  }
  
  .nav-card {
    min-height: 120px;
    padding: var(--spacing-lg) var(--spacing-md);
  }
  
  .nav-card svg {
    width: 1.75rem;
    height: 1.75rem;
  }
}

@media (min-width: 768px) {
  .nav-grid {
    grid-template-columns: repeat(3, 1fr);
  }
}

@media (min-width: 1024px) {
  .container {
    max-width: 56rem;
  }
}
</style>
