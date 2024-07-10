import { createRouter, createWebHistory, RouteRecordRaw } from 'vue-router';
import Home from '../views/Home.vue';
import Register from '../views/Register.vue';
import Control from '../views/Control.vue';
import Cadastre from '../views/Cadastre.vue';
import Login from '../views/Login.vue'

const routes: Array<RouteRecordRaw> = [
  {
    path: '/',
    name: 'Cadastre',
    component: Cadastre,
    meta: { hideSidebar: true }
  },

  {
    path: '/login',
    name: 'Login',
    component: Login,
    meta: { hideSidebar: true }
  },

  {
    path: '/register',
    name: 'Register',
    component: Register,
    
  },
  {
    path: '/control',
    name: 'Control',
    component: Control
  },
  {
    path: '/home',
    name: 'Home',
    component: Home
  }
];

const router = createRouter({
  history: createWebHistory(),
  routes
});

export default router;
