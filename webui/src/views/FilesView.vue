<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { useRouter } from 'vue-router'
import Icon from '@/components/Icon.vue'
import { showAlert, showConfirm } from '@/composables/useDialog'
import { useToast } from '@/composables/useToast'

const router = useRouter()
const toast = useToast()

interface FileItem {
  name: string
  size: number
  isDirectory: boolean
}

interface StorageInfo {
  total: number
  used: number
  free: number
}

const currentPath = ref('/')
const files = ref<FileItem[]>([])
const storage = ref<StorageInfo>({ total: 0, used: 0, free: 0 })
const loading = ref(true)
const uploading = ref(false)
const uploadProgress = ref(0)
const selectedFile = ref<File | null>(null)
const showUploadArea = ref(false)

onMounted(async () => {
  await loadFiles()
  await loadStorage()
})

async function loadFiles(path: string = '/') {
  loading.value = true
  currentPath.value = path
  
  try {
    const response = await fetch(`/api/files?path=${encodeURIComponent(path)}`)
    if (response.ok) {
      const data = await response.json()
      files.value = data.files || []
    }
  } catch (error) {
    console.error('加载文件列表失败:', error)
  } finally {
    loading.value = false
  }
}

async function loadStorage() {
  try {
    const response = await fetch('/api/files/storage')
    if (response.ok) {
      storage.value = await response.json()
    }
  } catch (error) {
    console.error('加载存储信息失败:', error)
  }
}

function navigateToParent() {
  const parts = currentPath.value.split('/').filter(Boolean)
  parts.pop()
  const newPath = '/' + parts.join('/')
  loadFiles(newPath)
}

function navigateTo(item: FileItem) {
  if (item.isDirectory) {
    const newPath = currentPath.value === '/' 
      ? `/${item.name}` 
      : `${currentPath.value}/${item.name}`
    loadFiles(newPath)
  }
}

async function deleteFile(item: FileItem) {
  const fullPath = currentPath.value === '/' 
    ? `/${item.name}` 
    : `${currentPath.value}/${item.name}`
  
  const confirmed = await showConfirm(`确定要删除 "${item.name}" 吗？`, {
    title: '删除文件',
    type: 'warning',
    confirmText: '删除'
  })
  if (!confirmed) {
    return
  }

  try {
    const response = await fetch(`/api/files${fullPath}`, {
      method: 'DELETE'
    })

    if (response.ok) {
      toast.success(`已删除 ${item.name}`)
      await loadFiles(currentPath.value)
      await loadStorage()
    } else {
      const message = await response
        .json()
        .then((body: any) => body?.message)
        .catch(() => null)
      showAlert(message || '删除失败', { type: 'error' })
    }
  } catch (error) {
    console.error('删除失败:', error)
    showAlert('删除失败，请重试', { type: 'error' })
  }
}

function downloadFile(item: FileItem) {
  const fullPath = currentPath.value === '/' 
    ? `/${item.name}` 
    : `${currentPath.value}/${item.name}`
  window.open(`/api/files${fullPath}?download=1`, '_blank')
}

function selectFile(event: Event) {
  const input = event.target as HTMLInputElement
  if (input.files && input.files[0]) {
    selectedFile.value = input.files[0]
    showUploadArea.value = true
  }
}

async function uploadFile() {
  if (!selectedFile.value) return

  uploading.value = true
  uploadProgress.value = 0

  try {
    const formData = new FormData()
    // path 必须排在 file 前面：固件是在收到文件第一块数据时读取目标目录的，
    // 而 multipart 字段要等该字段自己的结束边界解析完才可见。
    formData.append('path', currentPath.value)
    formData.append('file', selectedFile.value)

    const xhr = new XMLHttpRequest()
    
    xhr.upload.addEventListener('progress', (e) => {
      if (e.lengthComputable) {
        uploadProgress.value = Math.round((e.loaded / e.total) * 100)
      }
    })

    xhr.addEventListener('load', async () => {
      if (xhr.status === 200) {
        const name = selectedFile.value?.name ?? ''
        selectedFile.value = null
        showUploadArea.value = false
        toast.success(`已上传 ${name}`)
        await loadFiles(currentPath.value)
        await loadStorage()
      } else {
        let message = '上传失败'
        try {
          message = JSON.parse(xhr.responseText)?.message || message
        } catch {
          /* 非 JSON 响应，保持默认文案 */
        }
        showAlert(message, { type: 'error' })
      }
      uploading.value = false
    })

    xhr.addEventListener('error', () => {
      showAlert('上传失败', { type: 'error' })
      uploading.value = false
    })

    xhr.open('POST', '/api/files/upload')
    xhr.send(formData)
  } catch (error) {
    console.error('上传失败:', error)
    showAlert('上传失败，请重试', { type: 'error' })
    uploading.value = false
  }
}

function formatSize(bytes: number): string {
  if (bytes === 0) return '0 B'
  const k = 1024
  const sizes = ['B', 'KB', 'MB', 'GB']
  const i = Math.floor(Math.log(bytes) / Math.log(k))
  return Math.round(bytes / Math.pow(k, i) * 100) / 100 + ' ' + sizes[i]
}

const pathParts = computed(() => {
  return currentPath.value.split('/').filter(Boolean)
})

const usagePercent = computed(() => {
  if (storage.value.total === 0) return 0
  return Math.round((storage.value.used / storage.value.total) * 100)
})

const sortedFiles = computed(() => {
  return [...files.value].sort((a, b) => {
    // 目录优先
    if (a.isDirectory && !b.isDirectory) return -1
    if (!a.isDirectory && b.isDirectory) return 1
    // 名称排序
    return a.name.localeCompare(b.name)
  })
})
</script>

<template>
  <main>
    <div class="container">
      <h1>文件管理</h1>

      <!-- 工具栏：路径+存储+上传 -->
      <div class="toolbar">
        <div class="toolbar-main">
          <!-- 返回按钮 -->
          <button 
            v-if="currentPath !== '/'"
            @click="navigateToParent" 
            class="toolbar-btn"
            title="返回上级"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <polyline points="15 18 9 12 15 6"></polyline>
            </svg>
          </button>
          
          <!-- 路径显示 -->
          <div class="path-display">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path>
            </svg>
            <span v-if="pathParts.length === 0">/</span>
            <template v-else>
              <span>/ </span>
              <span v-for="(part, index) in pathParts" :key="index">
                {{ part }}{{ index < pathParts.length - 1 ? ' / ' : '' }}
              </span>
            </template>
          </div>
        </div>

        <div class="toolbar-actions">
          <!-- 存储信息（紧凑版） -->
          <div class="storage-compact" :class="{ warning: usagePercent > 80, danger: usagePercent > 90 }">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <ellipse cx="12" cy="5" rx="9" ry="3"></ellipse>
              <path d="M21 12c0 1.66-4 3-9 3s-9-1.34-9-3"></path>
              <path d="M3 5v14c0 1.66 4 3 9 3s9-1.34 9-3V5"></path>
            </svg>
            <span class="storage-text">{{ formatSize(storage.used) }} / {{ formatSize(storage.total) }}</span>
            <span class="storage-percent">{{ usagePercent }}%</span>
          </div>

          <!-- 上传按钮 -->
          <label class="btn btn-primary upload-btn">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path>
              <polyline points="17 8 12 3 7 8"></polyline>
              <line x1="12" y1="3" x2="12" y2="15"></line>
            </svg>
            上传文件
            <input type="file" @change="selectFile" style="display: none" />
          </label>
        </div>
      </div>

      <!-- 上传区域 -->
      <div v-if="showUploadArea" class="upload-area">
        <div class="upload-info">
          <div class="file-icon"><Icon name="file" :size="20" /></div>
          <div class="file-details">
            <div class="file-name">{{ selectedFile?.name }}</div>
            <div class="file-size">{{ formatSize(selectedFile?.size || 0) }}</div>
          </div>
        </div>
        
        <div v-if="uploading" class="upload-progress">
          <div class="progress-bar">
            <div class="progress-fill" :style="{ width: uploadProgress + '%' }"></div>
          </div>
          <div class="progress-text">{{ uploadProgress }}%</div>
        </div>

        <div class="upload-actions">
          <div class="btn-group horizontal">
            <button 
              @click="showUploadArea = false; selectedFile = null" 
              class="btn btn-secondary"
              :disabled="uploading"
            >
              取消
            </button>
            <button 
              @click="uploadFile" 
              class="btn btn-primary"
              :disabled="uploading"
            >
              {{ uploading ? '上传中...' : '开始上传' }}
            </button>
          </div>
        </div>
      </div>

      <!-- 文件列表 -->
      <div class="section">
        <div class="section-header">
          <h3>文件列表 ({{ sortedFiles.length }})</h3>
        </div>

        <div v-if="loading" class="loading-state">
          <div class="spinner"></div>
          <p>加载中...</p>
        </div>

        <div v-else-if="sortedFiles.length === 0" class="empty-state">
          <p>此目录为空</p>
        </div>

        <div v-else class="file-list">
          <div 
            v-for="item in sortedFiles" 
            :key="item.name"
            class="file-item"
            :class="{ directory: item.isDirectory }"
            @click="navigateTo(item)"
          >
            <div class="file-icon">
              <svg v-if="item.isDirectory" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M22 19a2 2 0 0 1-2 2H4a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h5l2 3h9a2 2 0 0 1 2 2z"></path>
              </svg>
              <svg v-else viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M13 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V9z"></path>
                <polyline points="13 2 13 9 20 9"></polyline>
              </svg>
            </div>
            <div class="file-info">
              <div class="file-name">{{ item.name }}</div>
              <div class="file-meta" v-if="!item.isDirectory">{{ formatSize(item.size) }}</div>
            </div>
            <div class="file-actions" @click.stop>
              <button 
                v-if="!item.isDirectory"
                @click="downloadFile(item)" 
                class="action-btn"
                title="下载"
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"></path>
                  <polyline points="7 10 12 15 17 10"></polyline>
                  <line x1="12" y1="15" x2="12" y2="3"></line>
                </svg>
              </button>
              <button 
                @click="deleteFile(item)" 
                class="action-btn delete"
                title="删除"
              >
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polyline points="3 6 5 6 21 6"></polyline>
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                </svg>
              </button>
            </div>
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
.toolbar {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: var(--spacing-lg);
  padding: var(--spacing-md) var(--spacing-lg);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-sm);
  margin-bottom: var(--spacing-xl);
  flex-wrap: wrap;
}

.toolbar-main {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  flex: 1;
  min-width: 0;
}

.toolbar-actions {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
}

.toolbar-btn {
  width: 2.5rem;
  height: 2.5rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  background: var(--color-bg-elevated);
  cursor: pointer;
  transition: var(--transition-fast);
}

.toolbar-btn:hover {
  background: var(--color-bg-hover);
  border-color: var(--color-border-hover);
}

.toolbar-btn svg {
  width: 1.25rem;
  height: 1.25rem;
}

.storage-compact {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm) var(--spacing-md);
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-md);
  font-size: 0.8125rem;
  color: var(--color-text-secondary);
}

.storage-compact svg {
  width: 1rem;
  height: 1rem;
  color: var(--color-primary);
}

.storage-compact.warning {
  border-color: var(--color-warning);
}

.storage-compact.warning svg {
  color: var(--color-warning);
}

.storage-compact.danger {
  border-color: var(--color-danger);
}

.storage-compact.danger svg {
  color: var(--color-danger);
}

.storage-text {
  font-family: var(--font-mono);
  font-weight: 500;
}

.storage-percent {
  font-family: var(--font-mono);
  font-weight: 700;
  color: var(--color-primary);
}

.storage-compact.warning .storage-percent {
  color: var(--color-warning);
}

.storage-compact.danger .storage-percent {
  color: var(--color-danger);
}

.path-display {
  flex: 1;
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  font-family: var(--font-mono);
  font-size: 0.875rem;
  color: var(--color-text);
}

.path-display svg {
  width: 1.25rem;
  height: 1.25rem;
  color: var(--color-primary);
  flex-shrink: 0;
}

.upload-btn {
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  padding: var(--spacing-sm) var(--spacing-lg);
  background: var(--color-primary);
  color: white;
  border-radius: var(--radius-md);
  font-weight: 600;
  cursor: pointer;
  transition: var(--transition-fast);
}

.upload-btn:hover {
  background: var(--color-primary-hover);
}

.upload-btn svg {
  width: 1.125rem;
  height: 1.125rem;
}

.upload-area {
  padding: var(--spacing-xl);
  background: var(--color-bg-card);
  border: 2px dashed var(--color-primary);
  border-radius: var(--spacing-lg);
  margin-bottom: var(--spacing-xl);
}

.upload-info {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  margin-bottom: var(--spacing-lg);
}

.file-icon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 2.5rem;
  height: 2.5rem;
  color: var(--color-primary);
}

.file-details {
  flex: 1;
}

.file-name {
  font-weight: 600;
  color: var(--color-text);
  margin-bottom: 0.25rem;
  word-break: break-all;
}

.file-size {
  font-size: 0.875rem;
  color: var(--color-text-muted);
}

.upload-progress {
  margin-bottom: var(--spacing-lg);
}

.progress-bar {
  height: 0.75rem;
  background: var(--color-bg-elevated);
  border-radius: var(--radius-full);
  overflow: hidden;
  margin-bottom: var(--spacing-sm);
}

.progress-fill {
  height: 100%;
  background: var(--color-primary);
  border-radius: var(--radius-full);
  transition: width 0.3s ease;
}

.progress-text {
  text-align: right;
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-primary);
}

.upload-actions {
  display: flex;
  gap: var(--spacing-sm);
  justify-content: flex-end;
}

.section {
  margin: var(--spacing-xl) 0;
  padding: var(--spacing-xl);
  background: var(--color-bg-card);
  border: 1px solid var(--color-border-light);
  border-radius: var(--spacing-lg);
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

.file-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.file-item {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-md) var(--spacing-lg);
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-md);
  transition: all 0.2s ease;
}

.file-item:hover {
  background: var(--color-bg-hover);
  box-shadow: var(--shadow-sm);
}

.file-item.directory {
  cursor: pointer;
  border-left: 3px solid var(--color-primary);
}

.file-item .file-icon {
  width: 2.5rem;
  height: 2.5rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-md);
  background: var(--color-bg-card);
  color: var(--color-text-muted);
}

.file-item.directory .file-icon {
  color: var(--color-primary);
}

.file-item .file-icon svg {
  width: 1.25rem;
  height: 1.25rem;
}

.file-info {
  flex: 1;
  min-width: 0;
}

.file-meta {
  font-size: 0.8125rem;
  color: var(--color-text-muted);
  font-family: var(--font-mono);
}

.file-actions {
  display: flex;
  gap: var(--spacing-xs);
}

.action-btn {
  width: 2rem;
  height: 2rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border: 1px solid var(--color-border);
  border-radius: var(--radius-sm);
  background: transparent;
  color: var(--color-text-muted);
  cursor: pointer;
  transition: var(--transition-fast);
}

.action-btn:hover {
  background: var(--color-bg-hover);
  border-color: var(--color-border-hover);
  color: var(--color-text);
}

.action-btn.delete:hover {
  background: var(--color-danger-light);
  border-color: var(--color-danger);
  color: var(--color-danger);
}

.action-btn svg {
  width: 1rem;
  height: 1rem;
}

.loading-state,
.empty-state {
  text-align: center;
  padding: var(--spacing-2xl);
  color: var(--color-text-muted);
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
</style>
