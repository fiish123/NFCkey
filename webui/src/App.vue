<script setup lang="ts">
import { onUnmounted } from 'vue'
import { RouterView } from 'vue-router'
import DialogHost from './components/DialogHost.vue'
import LogWindow from './components/LogWindow.vue'
import Toast from './components/Toast.vue'
import { useWebSocket } from './composables/useWebSocket'
import { useLogger } from './composables/useLogger'
import { sweepStaleWebAssets } from './utils/deviceFiles'

const { connectWebSocket, disconnectWebSocket } = useWebSocket()
const { initLogger, cleanupLogger } = useLogger()

// 在 setup 阶段就发起连接：子组件的 onMounted 早于父组件，
// 这样它们首次 sendWsRequest 时连接已经在建立中
connectWebSocket()
initLogger()

// 旧构建产物自检：上一次升级若由旧版网页执行，换过哈希名的 /web/assets/*.gz
// 会留在 flash 里；本页引用的资源才是该留的那一份，其余同名前缀的旧文件删掉
sweepStaleWebAssets()
  .then(removed => {
    if (removed.length > 0) {
      console.info('已清理设备上的旧网页资源:', removed)
    }
  })
  .catch(() => {
    /* 自检失败不影响使用 */
  })

onUnmounted(() => {
  cleanupLogger()
  disconnectWebSocket()
})
</script>

<template>
  <div class="app-shell">
    <RouterView v-slot="{ Component, route }">
      <Transition name="page" mode="out-in">
        <component :is="Component" :key="route.path" />
      </Transition>
    </RouterView>
    
    <!-- 日志浮窗 -->
    <LogWindow />
    
    <!-- Toast 通知 -->
    <Toast />
    
    <!-- 应用内对话框（取代原生 alert/confirm） -->
    <DialogHost />
  </div>
</template>

<style>
@import './assets/style.css';

.app-shell {
  min-height: 100vh;
}

.page-enter-active,
.page-leave-active {
  transition: opacity 0.2s cubic-bezier(0.4, 0, 0.2, 1), transform 0.2s cubic-bezier(0.4, 0, 0.2, 1);
}

.page-enter-from {
  opacity: 0;
  transform: translateX(0.5rem);
}

.page-leave-to {
  opacity: 0;
  transform: translateX(-0.5rem);
}
</style>
