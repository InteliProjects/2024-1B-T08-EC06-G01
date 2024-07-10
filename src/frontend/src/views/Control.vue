<template>
  <div id="app" class="flex flex-col h-screen overflow-hidden p-4">
    <h1 class="title mb-8">Central de Controle</h1>
    <div class="container flex flex-1 overflow-hidden space-x-4 pl-4"> <!-- Ajuste de padding-left aqui -->
      <div class="left-half flex-2 flex items-center justify-center p-4">
        <CameraComponent />
      </div>
      <div class="right-half flex-1 flex flex-col space-y-4">
        <div class="top-half flex-1 flex items-center justify-center p-4">
          <TeleopComponent />
        </div>
        <div class="bottom-half flex-1 flex items-center justify-center p-4">
          <SensorComponent />
        </div>
      </div>
    </div>
    <div class="flex p-4 bg-white border-gray-300 mt-4 ml-40"> <!-- Removi a classe border-t -->
      <EmergencyButton />
      <div class="flex-grow"></div>
      <button @click="logIa" class="bg-green-500 text-white py-2 px-8 rounded mr-12 hover:bg-green-600">
        Verificação de Resíduos
      </button>
      <button class="bg-green-500 text-white py-2 px-4 rounded mr-12 hover:bg-green-600">
        Modo Manual
      </button>
    </div>
  </div>
</template>

<script>
import CameraComponent from '../components/CameraComponent.vue'
import TeleopComponent from '../components/TeleopComponent.vue'
import SensorComponent from '../components/SensorComponent.vue'
import EmergencyButton from '../components/EmergencyButton.vue'
import axios from 'axios';

const websocket = new WebSocket(import.meta.env.VITE_CONTROL_WEBSOCKET);

console.log(import.meta.env.VITE_CONTROL_WEBSOCKET);
console.log("control");

websocket.onopen = () => {
  console.log('Connected to the control websocket');
};

websocket.onclose = () => {
  console.log('Disconnected from the control websocket');
};

websocket.onerror = (error) => {
   console.error('Error:', error);
};

export default {
  name: 'App',
  components: {
    CameraComponent,
    TeleopComponent,
    SensorComponent,
    EmergencyButton
  },
  methods: {
    async logEmergencyStop() {
      console.log('Emergency Stop');
      await this.searchIdLog();
    },
    async logIa() {
      console.log('IA');
      await this.searchIdLog2();
    },
    async searchIdLog() {
      console.log('searchIdLog');
      var url = window.location.href;
      var parsedUrl = new URL(url);
      var userId = parsedUrl.searchParams.get("id");
      this.userId = userId;

      console.log(this.userId);

      await axios.get('http://localhost:8000/logs/list')
        .then(res => {
          console.log(res.data, "toaki");

          const logs = res.data.data;
          const userId = this.userId;

          const userLogs = logs.filter(log => log.user_id && log.user_id.id == userId);
          console.log(userLogs, "userLogs");

          if (userLogs.length === 0) {
            console.log('Usuário não encontrado ou sem logs');
          } else {
            const latestLog = userLogs.sort((a, b) => new Date(b.date) - new Date(a.date))[0];
            console.log(latestLog, "last log");

            console.log(latestLog.id);

            this.sendLog(latestLog.id);
          }
        })
        .catch(error => {
          console.log(error);
        });
    },
    async searchIdLog2() {
      console.log('searchIdLog');
      var url = window.location.href;
      var parsedUrl = new URL(url);
      var userId = parsedUrl.searchParams.get("id");
      this.userId = userId;

      console.log(this.userId);

      await axios.get('http://localhost:8000/logs/list')
        .then(res => {
          console.log(res.data, "toaki");

          const logs = res.data.data;
          const userId = this.userId;

          const userLogs = logs.filter(log => log.user_id && log.user_id.id == userId);
          console.log(userLogs, "userLogs");

          if (userLogs.length === 0) {
            console.log('Usuário não encontrado ou sem logs');
          } else {
            const latestLog = userLogs.sort((a, b) => new Date(b.date) - new Date(a.date))[0];
            console.log(latestLog, "last log");

            console.log(latestLog.id);

            this.sendLogIa(latestLog.id);
          }
        })
        .catch(error => {
          console.log(error);
        });
    },
    async sendLog(id) { 
      const data = {
        emergency_button: true
      };

      await axios.put(`http://localhost:8000/logs/update/${id}`, data)
        .then(res => {
          console.log(res.data);
          alert('Modo de emergência ativado');
        })
        .catch(error => {
          console.log(error);
        });
    },
    async sendLogIa(id) { 
      const data = {
        ia_request: true
      };

      await axios.put(`http://localhost:8000/logs/update/${id}`, data)
        .then(res => {
          console.log(res.data);
          alert('Verificação de resíduos ativada');
        })
        .catch(error => {
          console.log(error);
        });
    }
  }
}
</script>


<style scoped>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: #2c3e50;
}

.title {
  font-family: 'Poppins';
  text-align: left;
  font-size: 2.3rem;
  padding-left: 11rem;
}

.container {
  display: flex;
  height: 100%;
  padding-left: 9rem;
}

.left-half {
  flex: 2;
  display: flex;
  align-items: center;
  justify-content: center;
}

.right-half {
  flex: 1;
  display: flex;
  flex-direction: column;
}

.top-half, .bottom-half {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: #fff;
}
</style>
