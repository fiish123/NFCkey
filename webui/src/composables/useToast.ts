import { ref } from 'vue'

interface ToastOptions {
  message: string
  type?: 'info' | 'success' | 'warning' | 'error'
  duration?: number
}

interface Toast extends ToastOptions {
  id: number
  type: 'info' | 'success' | 'warning' | 'error'
  duration: number
}

const toasts = ref<Toast[]>([])
let nextId = 0

export function useToast() {
  function show(options: ToastOptions) {
    const toast: Toast = {
      id: nextId++,
      message: options.message,
      type: options.type || 'info',
      duration: options.duration || 3000
    }
    
    toasts.value.push(toast)
    
    if (toast.duration > 0) {
      setTimeout(() => {
        remove(toast.id)
      }, toast.duration)
    }
    
    return toast.id
  }
  
  function remove(id: number) {
    const index = toasts.value.findIndex(t => t.id === id)
    if (index > -1) {
      toasts.value.splice(index, 1)
    }
  }
  
  function success(message: string, duration?: number) {
    return show({ message, type: 'success', duration })
  }
  
  function error(message: string, duration?: number) {
    return show({ message, type: 'error', duration })
  }
  
  function warning(message: string, duration?: number) {
    return show({ message, type: 'warning', duration })
  }
  
  function info(message: string, duration?: number) {
    return show({ message, type: 'info', duration })
  }
  
  return {
    toasts,
    show,
    remove,
    success,
    error,
    warning,
    info
  }
}
