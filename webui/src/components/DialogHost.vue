<script setup lang="ts">
import Dialog from './Dialog.vue'
import { useDialog } from '../composables/useDialog'

// 全局唯一的对话框宿主：各页面通过 showAlert/showConfirm 触发
const { current, settle } = useDialog()
</script>

<template>
  <Dialog
    :show="!!current"
    :title="current?.title ?? '提示'"
    :type="current?.type ?? 'info'"
    :confirm-text="current?.confirmText ?? '确定'"
    :cancel-text="current?.cancelText ?? '取消'"
    :show-cancel="current?.showCancel ?? false"
    @confirm="settle(true)"
    @cancel="settle(false)"
  >
    {{ current?.message }}
  </Dialog>
</template>
