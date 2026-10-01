<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useRouter } from 'vue-router'
import { useWebSocket } from '../composables/useWebSocket'
import { useToast } from '../composables/useToast'
import { showConfirm } from '@/composables/useDialog'
import Icon from '@/components/Icon.vue'
import Dialog from '../components/Dialog.vue'
import {
  parseUpdatePackage,
  hashPackage,
  syncCheck,
  buildUploadPlan,
  pruneStalePackageFiles,
  devicePath,
  uploadDataFile,
  uploadFirmware,
  UploadCancelled,
  type DataFile,
  type UpdatePackage,
  type UploadPlan
} from '@/utils/updatePackage'

const router = useRouter()
const { onWsEvent, offWsEvent } = useWebSocket()
const toast = useToast()

interface SystemInfo {
  version: string
  chipModel: string
  flashSize: number
  freeHeap: number
  sketchSize: number
  freeSketchSpace: number
}

interface OTAStage {
  name: string
  label: string
  progress: number
  status: 'pending' | 'active' | 'completed' | 'error'
}

const systemInfo = ref<SystemInfo | null>(null)
const loading = ref(true)
const selectedFile = ref<File | null>(null)
const updatePackage = ref<UpdatePackage | null>(null)
/** 与设备比对后真正需要上传的文件 */
const uploadPlan = ref<UploadPlan | null>(null)
const parsing = ref(false)
const uploading = ref(false)
const showConfirmDialog = ref(false)
const showSuccessDialog = ref(false)

// OTA 进度阶段：先数据、后固件（固件上传完设备会立即重启，必须最后做）
const stages = ref<OTAStage[]>([
  { name: 'data', label: '更新数据文件', progress: 0, status: 'pending' },
  // 上传与写入是同时进行的（设备边收边写 Flash），合成一个进度条
  { name: 'firmware', label: '上传并写入固件', progress: 0, status: 'pending' },
  { name: 'verify', label: '校验固件', progress: 0, status: 'pending' },
  { name: 'reboot', label: '重启设备', progress: 0, status: 'pending' }
])

/** 设备端阶段名 -> 界面阶段名（write 与上传合并） */
const DEVICE_STAGE_MAP: Record<string, string> = {
  write: 'firmware',
  verify: 'verify',
  reboot: 'reboot'
}

const currentStage = ref<string>('')
/** 当前正在处理的文件名（数据文件逐个上传时显示） */
const currentFile = ref('')
const errorMessage = ref<string>('')
let cancelled = false
let finishTimer: number | null = null

/** 升级收尾：设备写入/校验/重启完成后收起进度卡片并提示 */
function finishUpgrade() {
  if (!uploading.value) return
  if (finishTimer) {
    clearTimeout(finishTimer)
    finishTimer = null
  }
  completeStage('firmware')
  completeStage('verify')
  completeStage('reboot')
  // 保留进度卡片，让用户看到进度条走到 100%（关掉成功弹窗后再收起）
  showSuccessDialog.value = true
}

function stage(name: string): OTAStage {
  return stages.value.find(item => item.name === name)!
}

function activateStage(name: string) {
  stage(name).status = 'active'
  currentStage.value = name
}

function completeStage(name: string) {
  const target = stage(name)
  target.status = 'completed'
  target.progress = 100
}

function failStage(name: string) {
  stage(name).status = 'error'
}

onMounted(async () => {
  await loadSystemInfo()
  // 监听 OTA 进度事件
  onWsEvent('ota/progress', handleOtaProgress)
})

onUnmounted(() => {
  offWsEvent('ota/progress', handleOtaProgress)
})

async function loadSystemInfo() {
  loading.value = true
  try {
    const response = await fetch('/api/system/info')
    if (response.ok) {
      systemInfo.value = await response.json()
    }
  } catch (error) {
    console.error('加载系统信息失败:', error)
    toast.error('加载系统信息失败')
  } finally {
    loading.value = false
  }
}

async function selectFile(event: Event) {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file) return

  selectedFile.value = null
  updatePackage.value = null

  if (!file.name.toLowerCase().endsWith('.zip')) {
    toast.error('请选择升级包（.zip，用 scripts/pack_ota.py 生成）')
    input.value = ''
    return
  }

  // 整包上限：固件最大 2MB + 数据文件
  const maxSize = 6 * 1024 * 1024
  if (file.size > maxSize) {
    toast.error(`升级包过大（最大支持 6MB）\n当前文件: ${formatSize(file.size)}`)
    input.value = ''
    return
  }

  parsing.value = true
  try {
    const pkg = await parseUpdatePackage(file)
    // 浏览器算 SHA-256（http 下没有 crypto.subtle），再让设备比对已有文件
    await hashPackage(pkg)
    updatePackage.value = pkg

    try {
      const actions = await syncCheck(pkg.dataFiles)
      uploadPlan.value = buildUploadPlan(pkg, actions)
      if (uploadPlan.value.skipped > 0) {
        toast.info(
          `设备上已有 ${uploadPlan.value.skipped}/${pkg.dataFiles.length} 个文件内容相同，将跳过上传`
        )
      }
    } catch (error) {
      // 比对失败就退回“全部上传”，不影响升级
      console.error('比对文件失败:', error)
      uploadPlan.value = {
        pending: pkg.dataFiles,
        skipped: 0,
        pendingBytes: pkg.dataTotalSize
      }
      toast.warning('无法与设备比对文件，将上传全部数据文件')
    }

    selectedFile.value = file
  } catch (error) {
    toast.error(error instanceof Error ? error.message : '解析升级包失败')
    updatePackage.value = null
    uploadPlan.value = null
    input.value = ''
  } finally {
    parsing.value = false
  }
}

function confirmUpload() {
  if (!updatePackage.value) {
    toast.warning('请先选择升级包')
    return
  }
  showConfirmDialog.value = true
}

async function startUpload() {
  showConfirmDialog.value = false

  const pkg = updatePackage.value
  if (!pkg) return

  uploading.value = true
  cancelled = false
  errorMessage.value = ''
  stages.value.forEach(item => {
    item.progress = 0
    item.status = 'pending'
  })

  try {
    // 1) 数据文件（网页资源、提示音）
    activateStage('data')
    const pendingFiles: DataFile[] = uploadPlan.value?.pending ?? pkg.dataFiles
    if (pendingFiles.length === 0) {
      currentFile.value = '数据文件均与设备一致，已全部跳过'
    } else {
      let doneBytes = 0
      const totalBytes = pendingFiles.reduce((sum, item) => sum + item.size, 0) || 1
      for (const [index, dataFile] of pendingFiles.entries()) {
        currentFile.value = `${devicePath(dataFile)} (${index + 1}/${pendingFiles.length})`
        await uploadDataFile(dataFile, {
          isCancelled: () => cancelled,
          onProgress: loaded => {
            stage('data').progress = Math.max(
              stage('data').progress,
              Math.min(99, Math.round(((doneBytes + loaded) / totalBytes) * 100))
            )
          }
        })
        doneBytes += dataFile.size
        stage('data').progress = Math.round((doneBytes / totalBytes) * 100)
      }
      currentFile.value = ''
    }

    // 2) 清掉设备上不属于本次升级包的旧文件：sync-check 只上传有变化的文件，
    //    换过哈希名的旧产物（上次构建的 index-xxxx.js.gz）不会自己消失
    currentFile.value = '清理设备上的旧文件…'
    try {
      const pruned = await pruneStalePackageFiles(pkg)
      if (pruned.removed.length > 0) {
        toast.info(`已清理 ${pruned.removed.length} 个设备上的旧文件`)
      }
      if (pruned.failed.length > 0) {
        console.warn('部分旧文件清理失败:', pruned.failed)
      }
    } catch (error) {
      console.error('清理旧文件失败:', error)
      toast.warning('清理设备上的旧文件失败，升级继续')
    }
    currentFile.value = ''
    stage('data').progress = 100
    completeStage('data')

    // 3) 固件（设备边收边写 Flash，写完会自动重启，必须最后上传）
    activateStage('firmware')
    currentFile.value = pkg.firmware.name
    await uploadFirmware(pkg.firmware, {
      isCancelled: () => cancelled,
      onProgress: (loaded, total) => {
        if (!total) return
        stage('firmware').progress = Math.max(
          stage('firmware').progress,
          Math.round((loaded / total) * 100)
        )
      }
    })
    completeStage('firmware')

    // 设备端继续推送 write/verify/reboot 进度，进度卡片保持显示；
    // 兜底：万一事件收不到（WebSocket 已断），也要把流程收尾
    finishTimer = window.setTimeout(finishUpgrade, 3000)
  } catch (error) {
    uploading.value = false
    currentFile.value = ''
    if (finishTimer) {
      clearTimeout(finishTimer)
      finishTimer = null
    }
    if (error instanceof UploadCancelled) {
      toast.info('升级已取消')
      stages.value.forEach(item => {
        if (item.status === 'active') item.status = 'pending'
      })
      currentStage.value = ''
      return
    }
    handleUploadError(error instanceof Error ? error.message : '升级失败')
  }
}

function handleOtaProgress(data: any) {
  const { stage: deviceStage, progress, message } = data

  if (!deviceStage) return

  const stageName = DEVICE_STAGE_MAP[deviceStage]
  if (!stageName) return

  const stageObj = stages.value.find(item => item.name === stageName)
  if (!stageObj) return

  const value = progress || 0
  // 上传进度与设备写入进度是并行的，取较大值，进度条只前进不后退
  stageObj.progress = Math.max(stageObj.progress, value)

  if (stageObj.progress >= 100) {
    stageObj.status = 'completed'

    const order = ['firmware', 'verify', 'reboot']
    const currentIndex = order.indexOf(stageName)
    if (currentIndex >= 0 && currentIndex < order.length - 1) {
      const nextStage = stages.value.find(item => item.name === order[currentIndex + 1])
      if (nextStage) {
        nextStage.status = 'active'
        currentStage.value = nextStage.name
      }
    } else if (stageName === 'reboot') {
      finishUpgrade()
    }
  } else {
    stageObj.status = 'active'
    currentStage.value = stageName
  }

  if (message) {
    console.log(`[OTA] ${message}`)
  }
}

function handleUploadError(message: string) {
  uploading.value = false
  errorMessage.value = message

  const active = stages.value.find(item => item.status === 'active')
  if (active) active.status = 'error'

  let errorDetail = message
  if (message.includes('空间') || message.toLowerCase().includes('space')) {
    errorDetail = '设备存储空间不足\n\n请先删除一些文件再重试'
  } else if (message.includes('哈希')) {
    errorDetail = '文件校验失败\n\n升级包可能已损坏，请重新生成'
  } else if (message.includes('大小超限')) {
    errorDetail = '固件过大\n\n请确认升级包来自本项目的 pack_ota.py'
  }

  toast.error(errorDetail, 5000)
}

function handleRebootConfirm() {
  showSuccessDialog.value = false
  uploading.value = false
  // 不自动刷新：未开启 webdebug 时设备重启后不会再启动 Web 服务，
  // 刷新只会得到连接失败页面，交给用户按设备既定方式重新开启
  toast.info('设备正在重启，请稍候重新连接', 4000)
}

async function cancelUpload() {
  if (!uploading.value) return

  const confirmed = await showConfirm('确定要取消升级吗？', {
    title: '取消升级',
    type: 'warning',
    confirmText: '取消升级',
    cancelText: '继续升级'
  })
  if (confirmed) {
    cancelled = true
  }
}

// 计算属性
const totalProgress = computed(() => {
  const total = stages.value.length || 1
  const unit = 100 / total
  const completed = stages.value.filter(item => item.status === 'completed').length
  const active = stages.value.find(item => item.status === 'active')
  const activePart = active ? (active.progress / 100) * unit : 0
  return Math.min(100, Math.round(completed * unit + activePart))
})

const usedPercentage = computed(() => {
  if (!systemInfo.value) return 0
  const total = systemInfo.value.sketchSize + systemInfo.value.freeSketchSpace
  return Math.round((systemInfo.value.sketchSize / total) * 100)
})

function formatSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(2)} KB`
  return `${(bytes / (1024 * 1024)).toFixed(2)} MB`
}
</script>

<template>
  <main>
    <div class="container">
      <h1>OTA 升级</h1>

      <!-- 系统信息 -->
      <div v-if="!loading && systemInfo" class="system-card">
        <div class="system-header">
          <div class="system-icon">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <rect x="4" y="4" width="16" height="16" rx="2" ry="2"></rect>
              <rect x="9" y="9" width="6" height="6"></rect>
              <line x1="9" y1="1" x2="9" y2="4"></line>
              <line x1="15" y1="1" x2="15" y2="4"></line>
              <line x1="9" y1="20" x2="9" y2="23"></line>
              <line x1="15" y1="20" x2="15" y2="23"></line>
              <line x1="20" y1="9" x2="23" y2="9"></line>
              <line x1="20" y1="14" x2="23" y2="14"></line>
              <line x1="1" y1="9" x2="4" y2="9"></line>
              <line x1="1" y1="14" x2="4" y2="14"></line>
            </svg>
          </div>
          <div class="system-title">
            <h2>系统信息</h2>
            <p>当前固件版本: {{ systemInfo.version }}</p>
          </div>
        </div>
        
        <div class="system-details">
          <div class="detail-item">
            <span class="detail-label">芯片型号</span>
            <span class="detail-value">{{ systemInfo.chipModel }}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">Flash 大小</span>
            <span class="detail-value">{{ formatSize(systemInfo.flashSize) }}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">可用内存</span>
            <span class="detail-value">{{ formatSize(systemInfo.freeHeap) }}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">固件大小</span>
            <span class="detail-value">{{ formatSize(systemInfo.sketchSize) }}</span>
          </div>
          <div class="detail-item">
            <span class="detail-label">可用空间</span>
            <span class="detail-value">{{ formatSize(systemInfo.freeSketchSpace) }}</span>
          </div>
          <div class="detail-item full-width">
            <span class="detail-label">固件占用</span>
            <div class="progress-bar-wrapper">
              <div class="progress-bar-track">
                <div class="progress-bar-fill" :style="{ width: `${usedPercentage}%` }"></div>
              </div>
              <span class="progress-text">{{ usedPercentage }}%</span>
            </div>
          </div>
        </div>
      </div>

      <!-- 升级包区域：升级过程中由下方进度卡片顶替，页面不会变长 -->
      <div v-if="!uploading" class="upload-card">
        <h2>上传升级包</h2>

        <div class="upload-area">
          <input
            type="file"
            id="update-package"
            accept=".zip,application/zip"
            @change="selectFile"
            :disabled="uploading"
            class="file-input"
          />
          <label for="update-package" class="file-label" :class="{ disabled: uploading }">
            <Icon name="file" :size="48" />
            <span v-if="parsing">正在解析升级包...</span>
            <span v-else-if="!selectedFile">点击选择升级包（.zip）</span>
            <span v-else>{{ selectedFile.name }} ({{ formatSize(selectedFile.size) }})</span>
          </label>
        </div>

        <!-- 升级包内容 -->
        <div v-if="updatePackage" class="package-summary">
          <div class="package-row">
            <span class="package-label">固件</span>
            <span class="package-value">
              {{ updatePackage.firmware.name }} · {{ formatSize(updatePackage.firmware.size) }}
            </span>
          </div>
          <div class="package-row">
            <span class="package-label">数据文件</span>
            <span class="package-value">
              {{ updatePackage.dataFiles.length }} 个 · {{ formatSize(updatePackage.dataTotalSize) }}
              <template v-if="uploadPlan">
                <template v-if="uploadPlan.pending.length === 0">
                  · 已是最新，无需上传
                </template>
                <template v-else>
                  · 需更新 {{ uploadPlan.pending.length }} 个
                  <template v-if="uploadPlan.skipped > 0">（跳过 {{ uploadPlan.skipped }} 个）</template>
                </template>
              </template>
            </span>
          </div>
        </div>

        <div class="upload-hint">
          <p>• 升级包为 scripts/pack_ota.py 生成的标准包（含 firmware.bin 与 data/）</p>
          <p>• 数据文件先更新，固件最后写入并自动重启</p>
          <p>• 升级过程中请勿断电或关闭页面</p>
        </div>

        <button
          type="button"
          class="btn btn-primary btn-large"
          :disabled="!updatePackage || uploading || parsing"
          @click="confirmUpload"
        >
          <span v-if="!uploading">开始升级</span>
          <span v-else>升级中...</span>
        </button>
      </div>

      <!-- OTA 进度：顶替升级包卡片的位置 -->
      <div v-else class="progress-card">
        <h2>升级进度</h2>
        
        <!-- 总进度条 -->
        <div class="total-progress">
          <div class="progress-bar-wrapper">
            <div class="progress-bar-track large">
              <div class="progress-bar-fill" :style="{ width: `${totalProgress}%` }"></div>
            </div>
            <span class="progress-text large">{{ totalProgress }}%</span>
          </div>
        </div>

        <!-- 分阶段进度 -->
        <div class="stages">
          <div
            v-for="stage in stages"
            :key="stage.name"
            class="stage"
            :class="stage.status"
          >
            <div class="stage-icon">
              <svg v-if="stage.status === 'completed'" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3">
                <polyline points="20 6 9 17 4 12"></polyline>
              </svg>
              <svg v-else-if="stage.status === 'error'" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
              <div v-else-if="stage.status === 'active'" class="spinner"></div>
              <div v-else class="stage-number">{{ stages.indexOf(stage) + 1 }}</div>
            </div>
            
            <div class="stage-content">
              <div class="stage-header">
                <span class="stage-label">{{ stage.label }}</span>
                <span v-if="stage.status === 'active'" class="stage-progress">{{ stage.progress }}%</span>
              </div>
              
              <div v-if="stage.status === 'active'" class="stage-bar">
                <div class="stage-bar-fill" :style="{ width: `${stage.progress}%` }"></div>
              </div>

              <!-- 正在处理的文件名 -->
              <div
                v-if="stage.status === 'active' && currentFile"
                class="stage-file"
                :title="currentFile"
              >
                {{ currentFile }}
              </div>
            </div>
          </div>
        </div>

        <!-- 取消按钮 -->
        <button
          v-if="currentStage === 'data' || currentStage === 'firmware'"
          type="button"
          class="btn btn-secondary"
          @click="cancelUpload"
        >
          取消升级
        </button>
      </div>

      <!-- 返回首页 -->
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

    <!-- 确认对话框 -->
    <Dialog
      :show="showConfirmDialog"
      title="确认升级"
      type="warning"
      confirm-text="开始升级"
      cancel-text="取消"
      :show-cancel="true"
      @confirm="startUpload"
      @cancel="showConfirmDialog = false"
    >
      <p>确定要升级固件吗？</p>
      <p style="margin-top: 8px; color: var(--color-danger);">
        <strong>警告：</strong>升级过程中设备将重启，请勿断电或关闭页面！
      </p>
    </Dialog>

    <!-- 成功对话框 -->
    <Dialog
      :show="showSuccessDialog"
      title="升级成功"
      type="success"
      confirm-text="确定"
      @confirm="handleRebootConfirm"
    >
      <p>升级包写入成功，设备正在重启...</p>
    </Dialog>
  </main>
</template>

<style scoped>
/* 系统信息卡片 */
.system-card {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  padding: var(--spacing-lg);
  margin-bottom: var(--spacing-xl);
}

.system-header {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  margin-bottom: var(--spacing-lg);
}

.system-icon {
  width: 64px;
  height: 64px;
  background: var(--color-primary-light);
  border-radius: var(--radius-md);
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.system-icon svg {
  width: 32px;
  height: 32px;
  color: var(--color-primary);
}

.system-title h2 {
  font-size: 1.25rem;
  font-weight: 600;
  color: var(--color-text);
  margin: 0 0 var(--spacing-xs);
}

.system-title p {
  font-size: 0.875rem;
  color: var(--color-text-muted);
  margin: 0;
}

.system-details {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: var(--spacing-md);
}

.detail-item {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-xs);
}

.detail-item.full-width {
  grid-column: 1 / -1;
}

.detail-label {
  font-size: 0.75rem;
  color: var(--color-text-muted);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.detail-value {
  font-size: 0.9375rem;
  font-weight: 600;
  color: var(--color-text);
}

/* 上传区域 */
.upload-card {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  padding: var(--spacing-xl);
  margin-bottom: var(--spacing-xl);
}

.upload-card h2 {
  font-size: 1.125rem;
  font-weight: 600;
  color: var(--color-text);
  margin: 0 0 var(--spacing-lg);
}

.upload-area {
  margin-bottom: var(--spacing-lg);
}

.file-input {
  display: none;
}

.file-label {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-xl);
  border: 2px dashed var(--color-border);
  border-radius: var(--radius-md);
  cursor: pointer;
  transition: var(--transition-fast);
  text-align: center;
}

.file-label:hover:not(.disabled) {
  border-color: var(--color-primary);
  background: var(--color-primary-light);
}

.file-label.disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.file-label .icon {
  color: var(--color-text-muted);
}

.file-label span {
  font-size: 0.9375rem;
  color: var(--color-text-secondary);
}

.upload-hint {
  position: relative;
  background: var(--color-bg-elevated);
  padding: var(--spacing-md);
  border-radius: var(--radius-sm);
  overflow: hidden;
  margin-bottom: var(--spacing-lg);
}

/* 左侧色条：两端内缩，画成一条直线，不跟着圆角拐弯 */
.upload-hint::before {
  content: '';
  position: absolute;
  left: 0;
  top: var(--radius-sm);
  bottom: var(--radius-sm);
  width: 3px;
  background: var(--color-warning);
}

.upload-hint p {
  font-size: 0.875rem;
  color: var(--color-text-secondary);
  margin: var(--spacing-xs) 0;
}

.stage-file {
  margin-top: 0.25rem;
  color: var(--color-text-muted);
  font-size: 0.75rem;
  font-family: var(--font-heading);
  overflow: hidden;
  white-space: nowrap;
  text-overflow: ellipsis;
}

.package-summary {
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  padding: var(--spacing-md);
  margin-bottom: var(--spacing-md);
}

.package-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: var(--spacing-md);
  font-size: 0.875rem;
}

.package-row + .package-row {
  margin-top: var(--spacing-xs);
  padding-top: var(--spacing-xs);
  border-top: 1px dashed var(--color-border);
}

.package-label {
  color: var(--color-text-muted);
  flex-shrink: 0;
}

.package-value {
  color: var(--color-text);
  font-weight: 600;
  text-align: right;
  word-break: break-all;
}

.btn-large {
  width: 100%;
  padding: var(--spacing-md) var(--spacing-xl);
  font-size: 1rem;
}

/* 进度卡片 */
.progress-card {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  padding: var(--spacing-xl);
  margin-bottom: var(--spacing-xl);
}

.progress-card h2 {
  font-size: 1.125rem;
  font-weight: 600;
  color: var(--color-text);
  margin: 0 0 var(--spacing-lg);
}

.total-progress {
  margin-bottom: var(--spacing-xl);
}

.progress-bar-wrapper {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.progress-bar-track {
  flex: 1;
  height: 8px;
  background: var(--color-bg-elevated);
  border-radius: var(--radius-full);
  overflow: hidden;
}

.progress-bar-track.large {
  height: 12px;
}

.progress-bar-fill {
  height: 100%;
  background: var(--color-primary);
  border-radius: var(--radius-full);
  transition: width 0.3s ease;
}

.progress-text {
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-text-secondary);
  min-width: 48px;
  text-align: right;
}

.progress-text.large {
  font-size: 1rem;
  color: var(--color-text);
}

/* 阶段进度 */
.stages {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
  margin-bottom: var(--spacing-lg);
}

.stage {
  position: relative;
  display: flex;
  align-items: flex-start;
  gap: var(--spacing-md);
  padding: var(--spacing-md);
  background: var(--color-bg-elevated);
  border-radius: var(--radius-md);
  overflow: hidden;
}

/* 左侧色条：两端内缩，画成一条直线，不跟着圆角拐弯 */
.stage::before {
  content: '';
  position: absolute;
  left: 0;
  top: var(--radius-md);
  bottom: var(--radius-md);
  width: 3px;
  background: var(--accent-color, var(--color-border));
}

.stage.active {
  --accent-color: var(--color-primary);
  background: var(--color-primary-light);
}

.stage.completed {
  --accent-color: var(--color-success);
}

.stage.error {
  --accent-color: var(--color-danger);
  background: var(--color-danger-light);
}

.stage-icon {
  width: 36px;
  height: 36px;
  border-radius: var(--radius-full);
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--color-bg-card);
  border: 2px solid var(--color-border);
  flex-shrink: 0;
}

.stage.active .stage-icon {
  border-color: var(--color-primary);
  background: var(--color-primary-light);
}

.stage.completed .stage-icon {
  border-color: var(--color-success);
  background: var(--color-success);
}

.stage.completed .stage-icon svg {
  color: white;
  width: 20px;
  height: 20px;
}

.stage.error .stage-icon {
  border-color: var(--color-danger);
  background: var(--color-danger);
}

.stage.error .stage-icon svg {
  color: white;
  width: 16px;
  height: 16px;
}

.stage-number {
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-text-muted);
}

.spinner {
  width: 20px;
  height: 20px;
  border: 2px solid var(--color-border);
  border-top-color: var(--color-primary);
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.stage-content {
  flex: 1;
}

.stage-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: var(--spacing-xs);
}

.stage-label {
  font-size: 0.9375rem;
  font-weight: 600;
  color: var(--color-text);
}

.stage-progress {
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-primary);
}

.stage-bar {
  height: 6px;
  background: var(--color-bg-card);
  border-radius: var(--radius-full);
  overflow: hidden;
}

.stage-bar-fill {
  height: 100%;
  background: var(--color-primary);
  border-radius: var(--radius-full);
  transition: width 0.3s ease;
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

@media (max-width: 640px) {
  .system-details {
    grid-template-columns: 1fr;
  }
}
</style>
