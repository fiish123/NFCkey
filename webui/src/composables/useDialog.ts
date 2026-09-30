import { ref } from 'vue'

/**
 * 全局对话框服务：用应用内样式的 Dialog 取代原生 alert()/confirm()。
 *
 * 用法：
 *   await showAlert('删除失败', { type: 'error' })
 *   if (!(await showConfirm('确定要删除该文件吗？', { type: 'warning' }))) return
 *
 * 由 App.vue 里挂载的 <DialogHost /> 负责渲染（Dialog.vue 是受控组件）。
 */

export type DialogType = 'info' | 'success' | 'warning' | 'error' | 'confirm'

export interface DialogOptions {
  title?: string
  type?: DialogType
  confirmText?: string
  cancelText?: string
  showCancel?: boolean
}

export interface DialogState extends DialogOptions {
  message: string
}

// 全局单例：同一时刻只显示一个对话框
const current = ref<DialogState | null>(null)
let resolver: ((value: boolean) => void) | null = null

function open(state: DialogState): Promise<boolean> {
  // 如果上一个还没结束（例如用户连续触发），按“取消”收尾，避免 Promise 悬挂
  if (resolver) {
    const previous = resolver
    resolver = null
    previous(false)
  }
  current.value = state
  return new Promise<boolean>((resolve) => {
    resolver = resolve
  })
}

/** 确认框：返回用户是否点了“确定” */
export function showConfirm(message: string, options: DialogOptions = {}): Promise<boolean> {
  return open({
    message,
    title: options.title ?? '请确认',
    type: options.type ?? 'confirm',
    confirmText: options.confirmText ?? '确定',
    cancelText: options.cancelText ?? '取消',
    showCancel: true,
    ...options
  })
}

/** 提示框：只有一个确定按钮 */
export function showAlert(message: string, options: DialogOptions = {}): Promise<void> {
  return open({
    message,
    title: options.title ?? '提示',
    type: options.type ?? 'info',
    confirmText: options.confirmText ?? '确定',
    showCancel: false,
    ...options
  }).then(() => undefined)
}

/** 供 DialogHost 使用：读取当前状态并结束对话框 */
export function useDialog() {
  function settle(value: boolean) {
    const resolve = resolver
    resolver = null
    current.value = null
    if (resolve) resolve(value)
  }

  return { current, settle }
}
