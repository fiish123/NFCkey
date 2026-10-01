<script setup lang="ts">
import Icon from './Icon.vue'
import type { IconName } from './icons'
import { useToast } from '../composables/useToast'

const { toasts, remove } = useToast()

const TOAST_ICONS: Record<string, IconName> = {
  info: 'info',
  success: 'success',
  warning: 'warning',
  error: 'error'
}

function getIcon(type: string): IconName {
  return TOAST_ICONS[type] ?? 'info'
}
</script>

<template>
  <Teleport to="body">
    <div class="toast-container">
      <TransitionGroup name="toast">
        <div
          v-for="toast in toasts"
          :key="toast.id"
          class="toast"
          :class="`toast-${toast.type}`"
          @click="remove(toast.id)"
        >
          <div class="toast-icon"><Icon :name="getIcon(toast.type)" :size="18" /></div>
          <div class="toast-message">{{ toast.message }}</div>
          <button
            type="button"
            class="toast-close"
            @click.stop="remove(toast.id)"
            aria-label="关闭"
          >
            <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
              <line x1="18" y1="6" x2="6" y2="18"></line>
              <line x1="6" y1="6" x2="18" y2="18"></line>
            </svg>
          </button>
        </div>
      </TransitionGroup>
    </div>
  </Teleport>
</template>

<style>
.toast-container {
  position: fixed;
  top: var(--spacing-lg);
  right: var(--spacing-lg);
  z-index: 10001;
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
  pointer-events: none;
}

.toast {
  position: relative;
  display: flex;
  align-items: center;
  gap: var(--spacing-sm);
  min-width: 300px;
  max-width: 480px;
  padding: var(--spacing-md);
  background: var(--color-bg-card);
  border-radius: var(--radius-md);
  box-shadow: var(--shadow-lg);
  overflow: hidden;
  pointer-events: auto;
  cursor: pointer;
}

/* 左侧色条：两端内缩，画成一条直线，不跟着圆角拐弯 */
.toast::before {
  content: '';
  position: absolute;
  left: 0;
  top: var(--radius-md);
  bottom: var(--radius-md);
  width: 4px;
  background: var(--accent-color, var(--color-primary));
}

.toast-info {
  --accent-color: var(--color-primary);
}

.toast-success {
  --accent-color: var(--color-success);
}

.toast-warning {
  --accent-color: var(--color-warning);
}

.toast-error {
  --accent-color: var(--color-danger);
}

.toast-icon {
  width: 32px;
  height: 32px;
  border-radius: var(--radius-full);
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 18px;
  flex-shrink: 0;
}

.toast-info .toast-icon {
  background: var(--color-primary-light);
  color: var(--color-primary);
}

.toast-success .toast-icon {
  background: var(--color-success-light);
  color: var(--color-success);
}

.toast-warning .toast-icon {
  background: var(--color-warning-light);
  color: var(--color-warning);
}

.toast-error .toast-icon {
  background: var(--color-danger-light);
  color: var(--color-danger);
}

.toast-message {
  flex: 1;
  color: var(--color-text);
  font-size: 0.875rem;
  line-height: 1.5;
}

.toast-close {
  width: 24px;
  height: 24px;
  padding: 0;
  border: none;
  background: none;
  color: var(--color-text-muted);
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-sm);
  transition: var(--transition-fast);
  flex-shrink: 0;
}

.toast-close:hover {
  background: var(--color-bg-hover);
  color: var(--color-text);
}

.toast-close svg {
  width: 14px;
  height: 14px;
}

/* 动画 */
.toast-enter-active,
.toast-leave-active {
  transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
}

.toast-enter-from {
  opacity: 0;
  transform: translateX(1.5rem);
}

.toast-leave-to {
  opacity: 0;
  transform: translateX(1.5rem);
}

.toast-move {
  transition: transform 0.25s cubic-bezier(0.4, 0, 0.2, 1);
}

@media (max-width: 640px) {
  .toast-container {
    top: var(--spacing-md);
    right: var(--spacing-md);
    left: var(--spacing-md);
  }
  
  .toast {
    min-width: auto;
    max-width: none;
  }
}
</style>
