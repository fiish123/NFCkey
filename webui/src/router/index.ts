import { createRouter, createWebHistory } from 'vue-router'
import HomeView from '@/views/HomeView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'home',
      component: HomeView
    },
    {
      path: '/wifi',
      name: 'wifi',
      component: () => import('@/views/WiFiView.vue')
    },
    {
      path: '/cards',
      name: 'cards',
      component: () => import('@/views/CardsView.vue')
    },
    {
      path: '/servo',
      name: 'servo',
      component: () => import('@/views/ServoView.vue')
    },
    {
      path: '/files',
      name: 'files',
      component: () => import('@/views/FilesView.vue')
    },
    {
      path: '/ota',
      name: 'ota',
      component: () => import('@/views/OTAView.vue')
    }
  ],
})

export default router
