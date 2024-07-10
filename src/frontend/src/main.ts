import { createApp } from 'vue';
import App from './App.vue';
import router from './router';
import Notifications from '@kyvg/vue3-notification';
import '@fortawesome/fontawesome-free/css/all.css';
import './style.css';

const app = createApp(App);
app.use(router);
app.use(Notifications);
app.mount('#app');
