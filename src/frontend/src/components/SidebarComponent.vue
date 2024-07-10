<template>
  <div class="sidebar" 
  :class="{ open: !collapsed }" 
  :style="{ width: sidebarWidth }" 
  @mouseenter="openSidebar"
  @mouseleave="closeSidebar"
  >
    <div class="whitecanna" :class="{ open: !collapsed }">
      <button class="whitecanna" @click="goToHome">
        <svg width="53" height="42" viewBox="0 0 53 42" fill="none" xmlns="http://www.w3.org/2000/svg">
        <path d="M0 11.1008C5.31857 15.1487 17.8105 25.9724 25.2297 36.884C24.7263 21.9375 15.435 9.68091 0 11.1008Z" fill="url(#paint0_linear_177_161)"/>
        <path d="M41.4585 1.44849C40.1775 8.33931 36.0542 25.0554 29.8085 36.7933C42.2631 30.2618 48.8921 16.1307 41.4585 1.44849Z" fill="url(#paint1_linear_177_161)"/>
        <path d="M18.772 0C21.3561 6.45021 26.7901 22.7232 27.8532 36.2133C34.6387 23.1765 32.6274 7.48409 18.772 0Z" fill="url(#paint2_linear_177_161)"/>
        <defs>
        <linearGradient id="paint0_linear_177_161" x1="5.48109" y1="7.48188" x2="24.996" y2="37.0383" gradientUnits="userSpaceOnUse">
        <stop stop-color="white"/>
        <stop offset="1" stop-color="#999999"/>
        </linearGradient>
        <linearGradient id="paint1_linear_177_161" x1="46.736" y1="5.40484" x2="25.5811" y2="33.6242" gradientUnits="userSpaceOnUse">
        <stop stop-color="white"/>
        <stop offset="1" stop-color="#999999"/>
        </linearGradient>
        <linearGradient id="paint2_linear_177_161" x1="25.1957" y1="0" x2="25.1957" y2="36.2133" gradientUnits="userSpaceOnUse">
        <stop stop-color="white"/>
        <stop offset="1" stop-color="#999999"/>
        </linearGradient>
        </defs>
        </svg>
      </button>
    </div>
    <div :class="['line', { 'line-collapsed': collapsed }]"></div>
    <div class="nav">
      <button class="nav-button" @click="goToControl">
        <i class="fas fa-signal icon"></i>
        <span v-if="!collapsed">Central de Controle</span>
      </button>
      <button class="nav-button" @click="goToRegister">
        <i class="fas fa-users icon"></i>
        <span v-if="!collapsed">Registro de Uso</span>
      </button>
      <button class="nav-button" @click="goToHome">
        <i class="fas fa-folder-open icon"></i>
        <span v-if="!collapsed">Histórico de Dados</span>
      </button>
      <button v-if="!collapsed" class="out-button" @click="goToLogin">
        <i class="fa-solid fa-right-from-bracket fa-rotate-270"></i>
        <span class="button-text"> Sair </span>
      </button>
    </div>
    <span
      class="collapse-icon"
      :class="{ 'rotate-180': collapsed }"
      @click="toggleSidebar"
    >
      <i class="fas fa-angle-double-left"></i>
    </span>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import { collapsed, sidebarWidth, toggleSidebar } from '../state/state';
import { useRouter } from 'vue-router';
import WhiteCanna from '../assets/whitecanna.svg';

export default defineComponent({
  name: 'SidebarComponent',
  setup() {
    const router = useRouter();

    function goToControl() {
      var url = window.location.href;
      var parsedUrl = new URL(url);
      var id = parsedUrl.searchParams.get("id");

      router.push('/control?id=' + id);
      toggleSidebar();
    }

    function goToRegister() {
      var url = window.location.href;
      var parsedUrl = new URL(url);
      var id = parsedUrl.searchParams.get("id");

      router.push('/register?id=' + id);
      toggleSidebar();
    }

    function goToHome() {
      var url = window.location.href;
      var parsedUrl = new URL(url);
      var id = parsedUrl.searchParams.get("id");

      router.push('/home?id=' + id);
      toggleSidebar();
    }

    function goToLogin(){
      router.push('/login')
      toggleSidebar();
    }

    function openSidebar() {
      if (collapsed.value) {
        toggleSidebar();
      }
    }

    function closeSidebar() {
      if (!collapsed.value) {
        toggleSidebar();
      }
    }

    return {
      collapsed,
      toggleSidebar,
      sidebarWidth,
      goToControl,
      goToRegister,
      goToHome,
      goToLogin,
      WhiteCanna,
      openSidebar,
      closeSidebar
    };
  }
});

</script>

<style>
:root {
  --sidebar-bg-color: #077336;
  --sidebar-item-hover: #C3C1C1;
}
</style>

<style scoped>
:root {
  --sidebar-bg-color: #077336;
  --sidebar-item-hover: #C3C1C1;
}

.sidebar {
  color: #077336;
  background-color: var(--sidebar-bg-color);
  float: left;
  position: fixed;
  z-index: 1;
  top: 0;
  left: 0;
  bottom: 0;
  padding: 0.5em;
  transition: 0.4s ease;
  display: flex;
  flex-direction: column;
}

.sidebar.open .whitecanna {
  flex-grow: 1
}

.whitecanna {
  display: flex;
  justify-content: center;
  align-items: center;
  width: 100%;
  margin-bottom: 0.5rem;
}

.line {
  height: 0.12rem;
  background-color: white;
  align-self: center;
  margin-bottom: 0.4rem;
  transition: width 0.4s ease;
  width: 16rem;
}

.line-collapsed {
  width: 3rem;
}

.sidebar {
  color: #077336;
  background-color: var(--sidebar-bg-color);
  float: left;
  position: fixed;
  z-index: 1;
  top: 0;
  left: 0;
  bottom: 0;
  padding: 0.5em;
  transition: 0.4s ease;
  display: flex;
  flex-direction: column;
}

.collapse-icon {
  position: absolute;
  bottom: 0;
  padding: 1.28em;
  color: aliceblue;
  transition: 0.2s linear;
}

.rotate-180 {
  transform: rotate(180deg);
  transition: 0.2s linear;
}

.out-button{
  color: black;
  border: none;
  background: transparent;
  display: flex;
  align-items: center;
  padding: 0.8em;
  margin: 0.25em ;
  cursor: pointer;
  white-space: nowrap; 
  margin-top: 24em;
  margin-bottom: 2em;
}

.button-text {
  margin-left: 0.5em;
  font-size: 1.1em;
  font-weight:bold;
}

.nav-button {
  color: white;
  border: none;
  background: transparent;
  display: flex;
  align-items: center;
  padding: 0.75em;
  margin: 0.25em 0;
  cursor: pointer;
  white-space: nowrap; 
}

.nav-button i {
  margin-right: 0.5em;
}

.icon {
  margin: 0.75rem 0.25rem 0.5rem;
}

.whitecanna{
  padding-top: 10;
}

</style>
