<script setup lang="ts">
import { computed } from 'vue'
import { ICON_PATHS, type IconName } from './icons'

// 统一的图标组件：内联 SVG，颜色跟随 currentColor，尺寸可控
const props = withDefaults(
  defineProps<{
    name: IconName
    size?: number | string
    strokeWidth?: number | string
  }>(),
  { size: 16, strokeWidth: 1.8 }
)

const paths = computed(() => ICON_PATHS[props.name] as readonly string[])
</script>

<template>
  <svg
    class="icon"
    :width="size"
    :height="size"
    viewBox="0 0 24 24"
    fill="none"
    stroke="currentColor"
    :stroke-width="strokeWidth"
    stroke-linecap="round"
    stroke-linejoin="round"
    aria-hidden="true"
    focusable="false"
  >
    <path v-for="(d, i) in paths" :key="i" :d="d" />
  </svg>
</template>

<style scoped>
.icon {
  display: block;
  flex-shrink: 0;
}
</style>
