<script setup lang="ts">
import { computed } from 'vue'
import Icon from './Icon.vue'
import type { IconName } from './icons'

interface Props {
  show: boolean
  title?: string
  type?: 'info' | 'success' | 'warning' | 'error' | 'confirm'
  confirmText?: string
  cancelText?: string
  showCancel?: boolean
  /** 可选的第三个按钮（例如 WiFi 弹窗里的“测试连接”），点击只触发 extra，不关闭弹窗 */
  extraText?: string
  extraDisabled?: boolean
  confirmDisabled?: boolean
}

const props = withDefaults(defineProps<Props>(), {
  title: '提示',
  type: 'info',
  confirmText: '确定',
  cancelText: '取消',
  showCancel: false,
  extraText: '',
  extraDisabled: false,
  confirmDisabled: false
})

const emit = defineEmits<{
  confirm: []
  cancel: []
  extra: []
  close: []
}>()

const iconName = computed<IconName>(() => {
  const icons: Record<string, IconName> = {
    info: 'info',
    success: 'success',
    warning: 'warning',
    error: 'error',
    confirm: 'question'
  }
  return icons[props.type] ?? 'info'
})

const typeClass = computed(() => {
  return `dialog-${props.type}`
})

function handleConfirm() {
  emit('confirm')
  emit('close')
}

function handleCancel() {
  emit('cancel')
  emit('close')
}

function handleBackdropClick(event: MouseEvent) {
  if (event.target === event.currentTarget) {
    handleCancel()
  }
}
</script>

<template>
  <Teleport to="body">
    <Transition name="dialog">
      <div
        v-if="show"
        class="dialog-backdrop"
        @click="handleBackdropClick"
      >
        <div class="dialog" :class="typeClass" role="dialog" aria-modal="true">
          <div class="dialog-header">
            <div class="dialog-icon"><Icon :name="iconName" :size="24" /></div>
            <h3 class="dialog-title">{{ title }}</h3>
          </div>
          
          <div class="dialog-body">
            <slot></slot>
          </div>
          
          <!-- 按钮顺序（国内习惯）：附加操作 → 主操作 → 取消（取消固定在最右） -->
          <div class="dialog-footer">
            <button
              v-if="extraText"
              type="button"
              class="btn btn-secondary"
              :disabled="extraDisabled"
              @click="emit('extra')"
            >
              {{ extraText }}
            </button>
            <!-- 有取消按钮时，青色高亮给取消；主操作降为普通按钮（单按钮提示框仍用青色） -->
            <button
              type="button"
              class="btn"
              :class="showCancel ? 'btn-secondary' : 'btn-primary'"
              :disabled="confirmDisabled"
              @click="handleConfirm"
            >
              {{ confirmText }}
            </button>
            <button
              v-if="showCancel"
              type="button"
              class="btn btn-primary"
              @click="handleCancel"
            >
              {{ cancelText }}
            </button>
          </div>
        </div>
      </div>
    </Transition>
  </Teleport>
</template>

<style scoped>
.dialog-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(15, 23, 42, 0.5);
  backdrop-filter: blur(4px);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 10000;
  padding: var(--spacing-lg);
}

.dialog {
  background: var(--color-bg-card);
  border-radius: var(--radius-lg);
  box-shadow: var(--shadow-lg);
  max-width: 480px;
  width: 100%;
  overflow: hidden;
}

.dialog-header {
  display: flex;
  align-items: center;
  gap: var(--spacing-md);
  padding: var(--spacing-lg);
  border-bottom: 1px solid var(--color-border);
}

.dialog-icon {
  width: 48px;
  height: 48px;
  border-radius: var(--radius-full);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 24px;
  flex-shrink: 0;
}

.dialog-info .dialog-icon {
  background: var(--color-primary-light);
  color: var(--color-primary);
}

.dialog-success .dialog-icon {
  background: var(--color-success-light);
  color: var(--color-success);
}

.dialog-warning .dialog-icon {
  background: var(--color-warning-light);
  color: var(--color-warning);
}

.dialog-error .dialog-icon {
  background: var(--color-danger-light);
  color: var(--color-danger);
}

.dialog-confirm .dialog-icon {
  background: var(--color-primary-light);
  color: var(--color-primary);
}

.dialog-title {
  font-size: 1.125rem;
  font-weight: 600;
  color: var(--color-text);
  margin: 0;
}

.dialog-body {
  padding: var(--spacing-lg);
  color: var(--color-text-secondary);
  line-height: 1.6;
}

.dialog-footer {
  display: flex;
  flex-wrap: wrap;
  gap: var(--spacing-sm);
  padding: var(--spacing-lg);
  border-top: 1px solid var(--color-border);
  justify-content: flex-end;
}

.dialog-footer .btn {
  min-width: 80px;
}

/* 动画 */
.dialog-enter-active,
.dialog-leave-active {
  transition: opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.dialog-enter-active .dialog,
.dialog-leave-active .dialog {
  transition: transform 0.2s cubic-bezier(0.4, 0, 0.2, 1), opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.dialog-enter-from,
.dialog-leave-to {
  opacity: 0;
}

.dialog-enter-from .dialog,
.dialog-leave-to .dialog {
  transform: scale(0.98) translateY(0.5rem);
  opacity: 0;
}

@media (max-width: 640px) {
  .dialog-backdrop {
    padding: var(--spacing-md);
  }
  
  .dialog {
    max-width: 100%;
  }
}
</style>
