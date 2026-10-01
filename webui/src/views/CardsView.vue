<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { useRouter } from 'vue-router'
import { showAlert, showConfirm } from '@/composables/useDialog'
import { useToast } from '@/composables/useToast'

const router = useRouter()
const toast = useToast()

interface Card {
  uid: string
  name: string
  type: number
}

const cards = ref<Card[]>([])
const loading = ref(true)
const selectedCard = ref<string | null>(null)
const showAddDialog = ref(false)
const showRenameDialog = ref(false)
const newCardName = ref('')
const renameCardName = ref('')
const waitingForCard = ref(false)

onMounted(async () => {
  await loadCards()
})

async function loadCards() {
  loading.value = true
  try {
    const response = await fetch('/api/cards')
    if (response.ok) {
      const data = await response.json()
      cards.value = data.cards || []
    }
  } catch (error) {
    console.error('加载卡片失败:', error)
  } finally {
    loading.value = false
  }
}

function openAddDialog() {
  newCardName.value = ''
  showAddDialog.value = true
  waitingForCard.value = false
}

function closeAddDialog() {
  showAddDialog.value = false
  waitingForCard.value = false
  newCardName.value = ''
}

async function startAddCard() {
  if (!newCardName.value.trim()) {
    toast.warning('请输入卡片名称')
    return
  }

  waitingForCard.value = true

  try {
    const response = await fetch('/api/cards/read', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      }
    })

    if (response.ok) {
      const data = await response.json()
      
      // 保存卡片
      await saveCard(data.uid, newCardName.value, data.type || 0)
    } else {
      showAlert('读取卡片失败，请重试', { type: 'error' })
      waitingForCard.value = false
    }
  } catch (error) {
    console.error('添加卡片失败:', error)
    showAlert('添加卡片失败，请重试', { type: 'error' })
    waitingForCard.value = false
  }
}

async function saveCard(uid: string, name: string, type: number) {
  try {
    const response = await fetch('/api/cards', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        uid,
        name,
        type
      })
    })

    if (response.ok) {
      toast.success('卡片添加成功')
      closeAddDialog()
      await loadCards()
    } else {
      showAlert('保存卡片失败', { type: 'error' })
      waitingForCard.value = false
    }
  } catch (error) {
    console.error('保存卡片失败:', error)
    showAlert('保存卡片失败', { type: 'error' })
    waitingForCard.value = false
  }
}

function openRenameDialog(card: Card) {
  selectedCard.value = card.uid
  renameCardName.value = card.name
  showRenameDialog.value = true
}

function closeRenameDialog() {
  showRenameDialog.value = false
  selectedCard.value = null
  renameCardName.value = ''
}

async function renameCard() {
  if (!selectedCard.value || !renameCardName.value.trim()) {
    toast.warning('请输入新名称')
    return
  }

  try {
    const response = await fetch(`/api/cards/${selectedCard.value}`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        name: renameCardName.value
      })
    })

    if (response.ok) {
      closeRenameDialog()
      await loadCards()
    } else {
      showAlert('重命名失败', { type: 'error' })
    }
  } catch (error) {
    console.error('重命名失败:', error)
    showAlert('重命名失败', { type: 'error' })
  }
}

async function deleteCard(uid: string, name: string) {
  const confirmed = await showConfirm(`确定要删除卡片"${name}"吗？`, {
    title: '删除卡片',
    type: 'warning',
    confirmText: '删除'
  })
  if (!confirmed) {
    return
  }

  try {
    const response = await fetch(`/api/cards/${uid}`, {
      method: 'DELETE'
    })

    if (response.ok) {
      await loadCards()
    } else {
      showAlert('删除失败', { type: 'error' })
    }
  } catch (error) {
    console.error('删除失败:', error)
    showAlert('删除失败', { type: 'error' })
  }
}

function getCardTypeLabel(type: number): string {
  const types: { [key: number]: string } = {
    0: 'MIFARE',
    1: 'UID',
    2: 'NTAG',
    3: '其他'
  }
  return types[type] || '未知'
}

const cardCount = computed(() => cards.value.length)
</script>

<template>
  <main>
    <div class="container">
      <div class="page-header">
        <h1>卡片管理 <span class="badge">{{ cardCount }}</span></h1>
      </div>

      <!-- 操作按钮 -->
      <div class="section">
        <div class="section-header">
          <h3>卡片管理</h3>
          <button @click="openAddDialog" class="btn btn-primary">
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <line x1="12" y1="5" x2="12" y2="19"></line>
              <line x1="5" y1="12" x2="19" y2="12"></line>
            </svg>
            添加卡片
          </button>
        </div>
      </div>

      <!-- 卡片列表 -->
      <div class="section">
        <div class="section-header">
          <h3>卡片列表 ({{ cardCount }})</h3>
        </div>

        <div v-if="loading" class="loading-state">
          <div class="spinner"></div>
          <p>加载中...</p>
        </div>

        <div v-else-if="cards.length === 0" class="empty-state">
          <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <rect x="2" y="5" width="20" height="14" rx="2"></rect>
            <line x1="2" y1="10" x2="22" y2="10"></line>
          </svg>
          <p>还没有注册任何卡片</p>
          <p class="empty-hint">点击"添加新卡片"开始添加</p>
        </div>

        <div v-else class="card-list">
          <div v-for="card in cards" :key="card.uid" class="card-item">
            <div class="card-info">
              <div class="card-name">{{ card.name }}</div>
              <div class="card-meta">
                <span class="card-uid">{{ card.uid }}</span>
                <span class="card-type">{{ getCardTypeLabel(card.type) }}</span>
              </div>
            </div>
            <div class="card-actions">
              <button @click="openRenameDialog(card)" class="btn btn-sm" title="重命名">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M11 4H4a2 2 0 0 0-2 2v14a2 2 0 0 0 2 2h14a2 2 0 0 0 2-2v-7"></path>
                  <path d="M18.5 2.5a2.121 2.121 0 0 1 3 3L12 15l-4 1 1-4 9.5-9.5z"></path>
                </svg>
              </button>
              <button @click="deleteCard(card.uid, card.name)" class="btn btn-sm btn-danger" title="删除">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <polyline points="3 6 5 6 21 6"></polyline>
                  <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"></path>
                </svg>
              </button>
            </div>
          </div>
        </div>
      </div>

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

    <!-- 添加卡片对话框 -->
    <Transition name="modal">
      <div v-if="showAddDialog" class="modal-overlay" @click.self="closeAddDialog">
        <div class="modal">
          <div class="modal-header">
            <h3>添加新卡片</h3>
            <button @click="closeAddDialog" class="modal-close">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
          <div class="modal-body">
            <div v-if="!waitingForCard">
              <div class="input-group">
                <label for="cardName">卡片名称</label>
                <input
                  id="cardName"
                  type="text"
                  v-model="newCardName"
                  placeholder="例如：张三的门禁卡"
                  @keyup.enter="startAddCard"
                />
              </div>
            </div>
            <div v-else class="waiting-state">
              <div class="nfc-animation">
                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <rect x="2" y="5" width="20" height="14" rx="2"></rect>
                  <line x1="2" y1="10" x2="22" y2="10"></line>
                </svg>
              </div>
              <p>请将卡片靠近读卡器...</p>
            </div>
          </div>
          <div class="modal-footer">
            <div class="btn-group horizontal">
              <button @click="closeAddDialog" class="btn btn-secondary">取消</button>
              <button 
                v-if="!waitingForCard"
                @click="startAddCard" 
                :disabled="!newCardName.trim()"
                class="btn btn-primary"
              >
                开始读卡
              </button>
            </div>
          </div>
        </div>
      </div>
    </Transition>

    <!-- 重命名对话框 -->
    <Transition name="modal">
      <div v-if="showRenameDialog" class="modal-overlay" @click.self="closeRenameDialog">
        <div class="modal">
          <div class="modal-header">
            <h3>重命名卡片</h3>
            <button @click="closeRenameDialog" class="modal-close">
              <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"></line>
                <line x1="6" y1="6" x2="18" y2="18"></line>
              </svg>
            </button>
          </div>
          <div class="modal-body">
            <div class="input-group">
              <label for="renameName">新名称</label>
              <input
                id="renameName"
                type="text"
                v-model="renameCardName"
                @keyup.enter="renameCard"
              />
            </div>
          </div>
          <div class="modal-footer">
            <div class="btn-group horizontal">
              <button @click="closeRenameDialog" class="btn btn-secondary">取消</button>
              <button 
                @click="renameCard" 
                :disabled="!renameCardName.trim()"
                class="btn btn-primary"
              >
                确定
              </button>
            </div>
          </div>
        </div>
      </div>
    </Transition>
  </main>
</template>

<style scoped>
.page-header {
  display: flex;
  align-items: center;
  margin-bottom: var(--spacing-2xl);
}

.page-header h1 {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  margin: 0;
}

.badge {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-width: 2rem;
  height: 2rem;
  padding: 0 var(--spacing-sm);
  background: var(--color-primary);
  color: white;
  font-size: 0.875rem;
  font-weight: 600;
  border-radius: var(--radius-full);
  font-family: var(--font-mono);
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

.card-list {
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
}

.card-item {
  position: relative;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: var(--spacing-md) var(--spacing-lg);
  background: var(--color-bg-elevated);
  border: 1px solid var(--color-border-light);
  border-radius: var(--radius-md);
  overflow: hidden;
  transition: all 0.2s ease;
}

/* 左侧色条：两端内缩，画成一条直线，不跟着卡片圆角拐弯 */
.card-item::before {
  content: '';
  position: absolute;
  left: 0;
  top: var(--radius-md);
  bottom: var(--radius-md);
  width: 3px;
  background: var(--color-primary);
}

.card-item:hover {
  background: var(--color-bg-hover);
  box-shadow: var(--shadow-sm);
}

.card-info {
  flex: 1;
}

.card-name {
  font-weight: 600;
  color: var(--color-text);
  margin-bottom: 0.25rem;
}

.card-meta {
  display: flex;
  gap: var(--spacing-md);
  font-size: 0.8125rem;
  color: var(--color-text-muted);
}

.card-uid {
  font-family: var(--font-mono);
}

.card-type {
  padding: 0.125rem 0.5rem;
  background: var(--color-primary-light);
  color: var(--color-primary);
  border-radius: var(--radius-sm);
  font-size: 0.75rem;
  font-weight: 600;
}

.card-actions {
  display: flex;
  gap: var(--spacing-xs);
}

.loading-state,
.empty-state,
.waiting-state {
  text-align: center;
  padding: var(--spacing-xl);
  color: var(--color-text-muted);
}

.empty-state svg {
  width: 4rem;
  height: 4rem;
  margin: 0 auto var(--spacing-md);
  color: var(--color-border);
}

.empty-state p {
  margin: 0.5rem 0;
  font-size: 0.9375rem;
}

.empty-hint {
  font-size: 0.875rem;
  margin-top: 0.5rem;
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

.nfc-animation {
  width: 4rem;
  height: 4rem;
  margin: 0 auto var(--spacing-md);
  color: var(--color-primary);
  animation: pulse 2s ease-in-out infinite;
}

.nfc-animation svg {
  width: 100%;
  height: 100%;
}

@keyframes pulse {
  0%, 100% { opacity: 0.6; transform: scale(1); }
  50% { opacity: 1; transform: scale(1.1); }
}

/* 模态框样式 */
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(15, 23, 42, 0.6);
  display: flex;
  align-items: center;
  justify-content: center;
  padding: var(--spacing-lg);
  z-index: 1000;
}

.modal {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-xl);
  max-width: 28rem;
  width: 100%;
}

/* 进入/退出动画：遮罩淡入淡出，卡片轻微上浮 + 缩放 */
.modal-enter-active,
.modal-leave-active {
  transition: opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.modal-enter-active .modal,
.modal-leave-active .modal {
  transition: opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1),
    transform 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.modal-enter-from,
.modal-leave-to {
  opacity: 0;
}

.modal-enter-from .modal,
.modal-leave-to .modal {
  opacity: 0;
  transform: scale(0.98) translateY(0.5rem);
}

/* 退场时遮罩已经在淡出，不该再拦截点击 */
.modal-leave-active {
  pointer-events: none;
}

.modal-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: var(--spacing-lg);
  border-bottom: 1px solid var(--color-border-light);
}

.modal-header h3 {
  margin: 0;
  padding: 0;
}

.modal-header h3::before {
  display: none;
}

.modal-close {
  width: 2rem;
  height: 2rem;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-sm);
  transition: var(--transition-fast);
}

.modal-close:hover {
  background: var(--color-bg-hover);
}

.modal-close svg {
  width: 1.25rem;
  height: 1.25rem;
}

.modal-body {
  padding: var(--spacing-xl);
}

.modal-footer {
  padding: var(--spacing-lg);
  border-top: 1px solid var(--color-border-light);
}

.modal-footer .btn-group {
  width: 100%;
}

.input-group {
  margin: 0;
}

.input-group label {
  display: block;
  margin-bottom: var(--spacing-sm);
  font-size: 0.875rem;
  font-weight: 600;
  color: var(--color-text-secondary);
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
