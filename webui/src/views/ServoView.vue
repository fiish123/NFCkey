<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { useRouter } from 'vue-router'
import { showAlert } from '@/composables/useDialog'
import { useToast } from '@/composables/useToast'

const router = useRouter()
const toast = useToast()

interface ServoConfig {
  unlockPosition: number
  lockPosition: number
  currentPosition: number
}

const config = ref<ServoConfig>({
  unlockPosition: 800,
  lockPosition: 1180,
  currentPosition: 0
})

const testPosition = ref(900)
const loading = ref(true)
const testing = ref(false)
const saving = ref(false)
const activeTab = ref<'test' | 'config'>('test')

onMounted(async () => {
  await loadConfig()
})

async function loadConfig() {
  loading.value = true
  try {
    const response = await fetch('/api/servo')
    if (response.ok) {
      const data = await response.json()
      config.value = {
        unlockPosition: data.unlock || 800,
        lockPosition: data.lock || 1180,
        currentPosition: data.current || 0
      }
      testPosition.value = Math.floor((config.value.unlockPosition + config.value.lockPosition) / 2)
    }
  } catch (error) {
    console.error('加载配置失败:', error)
  } finally {
    loading.value = false
  }
}

async function testServo() {
  testing.value = true
  try {
    const response = await fetch('/api/servo/test', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        position: testPosition.value
      })
    })

    if (response.ok) {
      config.value.currentPosition = testPosition.value
    } else {
      showAlert('测试失败', { type: 'error' })
    }
  } catch (error) {
    console.error('测试失败:', error)
    showAlert('测试失败，请重试', { type: 'error' })
  } finally {
    testing.value = false
  }
}

async function setUnlock() {
  config.value.unlockPosition = testPosition.value
}

async function setLock() {
  config.value.lockPosition = testPosition.value
}

async function saveConfig() {
  saving.value = true
  try {
    const response = await fetch('/api/servo', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        unlock: config.value.unlockPosition,
        lock: config.value.lockPosition
      })
    })

    if (response.ok) {
      toast.success('配置保存成功')
    } else {
      showAlert('保存失败', { type: 'error' })
    }
  } catch (error) {
    console.error('保存失败:', error)
    showAlert('保存失败，请重试', { type: 'error' })
  } finally {
    saving.value = false
  }
}

async function quickAction(action: 'unlock' | 'lock') {
  const position = action === 'unlock' ? config.value.unlockPosition : config.value.lockPosition
  
  try {
    const response = await fetch('/api/servo/test', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ position })
    })

    if (response.ok) {
      config.value.currentPosition = position
    }
  } catch (error) {
    console.error('执行失败:', error)
  }
}
</script>

<template>
  <main>
    <div class="container">
      <h1>舵机控制</h1>

      <!-- 当前状态 -->
      <div class="status-card">
        <div class="status-header">
          <h3>当前位置</h3>
          <div class="position-display">{{ config.currentPosition }}</div>
        </div>
        <div class="position-bar">
          <div class="position-track">
            <div 
              class="position-marker unlock" 
              :style="{ left: (config.unlockPosition / 1280 * 100) + '%' }"
            >
              <span class="marker-label">{{ config.unlockPosition }}</span>
            </div>
            <div 
              class="position-marker lock" 
              :style="{ left: (config.lockPosition / 1280 * 100) + '%' }"
            >
              <span class="marker-label">{{ config.lockPosition }}</span>
            </div>
            <div 
              class="position-indicator" 
              :style="{ left: (config.currentPosition / 1280 * 100) + '%' }"
            ></div>
          </div>
          <div class="position-range">
            <span class="range-label">0</span>
            <span class="range-label">1280</span>
          </div>
        </div>
      </div>

      <!-- 控制区域 -->
      <div class="section">
        <div class="section-header">
          <h3>舵机控制</h3>
        </div>

        <!-- 标签页切换 -->
        <div class="tabs">
          <button 
            class="tab-btn" 
            :class="{ active: activeTab === 'test' }"
            @click="activeTab = 'test'"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <circle cx="12" cy="12" r="10"></circle>
              <polyline points="12 6 12 12 16 14"></polyline>
            </svg>
            快速测试
          </button>
          <button 
            class="tab-btn" 
            :class="{ active: activeTab === 'config' }"
            @click="activeTab = 'config'"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <circle cx="12" cy="12" r="3"></circle>
              <path d="M12 1v6m0 6v6M5.6 5.6l4.2 4.2m4.4 4.4l4.2 4.2m-12.8 0l4.2-4.2m4.4-4.4l4.2-4.2"></path>
            </svg>
            精确配置
          </button>
        </div>

        <!-- 快速测试面板 -->
        <div v-show="activeTab === 'test'" class="tab-panel">
          <div class="test-controls">
            <div class="slider-group">
              <label>
                <span>测试位置</span>
                <span class="value-display">{{ testPosition }}</span>
              </label>
              <input
                type="range"
                v-model.number="testPosition"
                min="0"
                max="1280"
                step="10"
                class="position-slider"
              />
              <div class="slider-labels">
                <span>0</span>
                <span>640</span>
                <span>1280</span>
              </div>
            </div>

            <button 
              @click="testServo" 
              :disabled="testing"
              class="btn btn-primary"
            >
              {{ testing ? '执行中...' : '测试此位置' }}
            </button>
          </div>

          <div class="btn-group horizontal preset-actions">
            <button @click="setUnlock" class="btn btn-outline">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                <path d="M7 11V7a5 5 0 0 1 9.9-1"></path>
              </svg>
              设为开锁
            </button>
            <button @click="setLock" class="btn btn-outline">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
              </svg>
              设为关锁
            </button>
          </div>
        </div>

        <!-- 精确配置面板 -->
        <div v-show="activeTab === 'config'" class="tab-panel">
          <div class="config-grid">
            <div class="config-item">
              <div class="config-icon unlock">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                  <path d="M7 11V7a5 5 0 0 1 9.9-1"></path>
                </svg>
              </div>
              <div class="config-content">
                <div class="config-label">开锁位置</div>
                <input 
                  type="number" 
                  v-model.number="config.unlockPosition"
                  min="0"
                  max="1280"
                  class="config-input"
                />
              </div>
            </div>

            <div class="config-item">
              <div class="config-icon lock">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                  <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
                </svg>
              </div>
              <div class="config-content">
                <div class="config-label">关锁位置</div>
                <input 
                  type="number" 
                  v-model.number="config.lockPosition"
                  min="0"
                  max="1280"
                  class="config-input"
                />
              </div>
            </div>
          </div>

          <div class="save-btn-wrapper">
            <button 
              @click="saveConfig" 
              :disabled="saving"
              class="btn btn-primary save-btn"
            >
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"></path>
                <polyline points="17 21 17 13 7 13 7 21"></polyline>
                <polyline points="7 3 7 8 15 8"></polyline>
              </svg>
              {{ saving ? '保存中...' : '保存配置' }}
            </button>
          </div>
        </div>
      </div>

      <!-- 返回 -->
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
.tabs {
  display: flex;
  gap: var(--spacing-sm);
  margin-bottom: var(--spacing-lg);
  border-bottom: 2px solid var(--color-border-light);
}

.tab-btn {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-md) var(--spacing-lg);
  background: none;
  border: none;
  border-bottom: 2px solid transparent;
  margin-bottom: -2px;
  color: var(--color-text-muted);
  font-size: 0.9375rem;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.tab-btn svg {
  width: 1.125rem;
  height: 1.125rem;
}

.tab-btn:hover {
  color: var(--color-text);
  background: var(--color-bg-elevated);
}

.tab-btn.active {
  color: var(--color-primary);
  border-bottom-color: var(--color-primary);
}

.tab-panel {
  animation: fadeIn 0.2s ease-in;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(-4px); }
  to { opacity: 1; transform: translateY(0); }
}

.status-card {
  padding: var(--spacing-xl);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
  margin-bottom: var(--spacing-xl);
}

.status-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: var(--spacing-lg);
}

.status-header h3 {
  margin: 0;
  padding: 0;
}

.status-header h3::before {
  display: none;
}

.position-display {
  font-size: 2rem;
  font-weight: 700;
  font-family: var(--font-mono);
  color: var(--color-primary);
}

.position-track {
  position: relative;
  height: 3rem;
  background: var(--color-bg-elevated);
  border-radius: var(--radius-md);
  border: 1px solid var(--color-border);
  margin-bottom: var(--spacing-sm);
}

.position-range {
  display: flex;
  justify-content: space-between;
  font-size: 0.75rem;
  color: var(--color-text-muted);
  font-family: var(--font-mono);
}

.position-marker {
  position: absolute;
  top: 50%;
  transform: translate(-50%, -50%);
  width: 3px;
  height: 2rem;
  background: currentColor;
  border-radius: var(--radius-full);
}

.position-marker.unlock {
  color: var(--color-success);
}

.position-marker.lock {
  color: var(--color-danger);
}

.marker-label {
  position: absolute;
  top: -1.5rem;
  left: 50%;
  transform: translateX(-50%);
  font-size: 0.6875rem;
  font-weight: 600;
  white-space: nowrap;
  color: currentColor;
}

.position-indicator {
  position: absolute;
  top: 50%;
  transform: translate(-50%, -50%);
  width: 1rem;
  height: 1rem;
  background: var(--color-primary);
  border: 3px solid var(--color-bg-card);
  border-radius: 50%;
  box-shadow: 0 0 0 2px var(--color-primary);
  transition: left 0.3s ease;
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

.test-controls {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-lg);
}

.slider-group label {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: var(--spacing-sm);
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-text-secondary);
}

.value-display {
  font-family: var(--font-mono);
  font-size: 1.125rem;
  color: var(--color-primary);
}

.position-slider {
  width: 100%;
  height: 0.5rem;
  background: var(--color-bg-elevated);
  border-radius: var(--radius-full);
  outline: none;
  -webkit-appearance: none;
  appearance: none;
}

.position-slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 1.5rem;
  height: 1.5rem;
  background: var(--color-primary);
  border: 3px solid var(--color-bg-card);
  border-radius: 50%;
  cursor: pointer;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
}

.position-slider::-moz-range-thumb {
  width: 1.5rem;
  height: 1.5rem;
  background: var(--color-primary);
  border: 3px solid var(--color-bg-card);
  border-radius: 50%;
  cursor: pointer;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.2);
}

.slider-labels {
  display: flex;
  justify-content: space-between;
  margin-top: var(--spacing-xs);
  font-size: 0.75rem;
  color: var(--color-text-muted);
  font-family: var(--font-mono);
}

.preset-actions {
  margin-top: var(--spacing-xl);
}

.config-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: var(--spacing-lg);
  margin-bottom: var(--spacing-lg);
}

.config-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-lg);
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
}

.config-icon {
  width: 3rem;
  height: 3rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-md);
}

.config-icon.unlock {
  background: var(--color-success-light);
  color: var(--color-success);
}

.config-icon.lock {
  background: var(--color-danger-light);
  color: var(--color-danger);
}

.config-icon svg {
  width: 1.5rem;
  height: 1.5rem;
}

.config-content {
  flex: 1;
}

.config-label {
  font-size: 0.8125rem;
  font-weight: 600;
  color: var(--color-text-muted);
  margin-bottom: 0.5rem;
}

.config-input {
  width: 100%;
  padding: var(--spacing-sm) var(--spacing-md);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-sm);
  background: var(--color-bg-card);
  color: var(--color-text);
  font-family: var(--font-mono);
  font-size: 1.125rem;
  font-weight: 600;
  text-align: center;
}

.save-btn-wrapper {
  display: flex;
  justify-content: center;
  margin-top: var(--spacing-xl);
}

.save-btn {
  min-width: 200px;
  font-size: 1rem;
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

@media (max-width: 768px) {
  .quick-actions,
  .config-grid,
  .preset-actions {
    grid-template-columns: 1fr;
  }
}
</style>
