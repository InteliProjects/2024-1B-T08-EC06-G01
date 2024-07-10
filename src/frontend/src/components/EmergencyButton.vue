<template>
  <div>
    <button class="bg-red-500 text-white py-2 px-8 rounded hover:bg-red-700" @click="emergencyStop">
      Modo de Emergência
    </button>
  </div>
</template>

<script>
import { useNotification } from '@kyvg/vue3-notification';

export default {
  name: 'EmergencyButton',
  data() {
    return {
      websocket: null,
    };
  },
  created() {
    this.websocket = new WebSocket(import.meta.env.VITE_CONTROL_WEBSOCKET);

    this.websocket.onopen = () => {
      console.log('Connected to the control websocket');
    };

    this.websocket.onclose = () => {
      console.log('Disconnected from the control websocket');
    };

    this.websocket.onerror = (error) => {
      console.error('Error:', error);
    };
  },
  methods: {
    emergencyStop() {
      const { notify } = useNotification();
      console.log('Emergency Stop');

      if (this.websocket.readyState === WebSocket.OPEN) {
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "emergency"
          }
        }));
        notify({
          group: 'foo',
          type: 'error',
          title: 'Emergência',
          text: 'A parada de emergência foi acionada. Para continuar movimentando o robô, reinicie o serviço responsável pelo controle do robô.'
        });
      } else {
        console.error('WebSocket is not open.');
        notify({
          group: 'foo',
          type: 'error',
          title: 'Erro',
          text: 'Não foi possível enviar a parada de emergência. O WebSocket não está conectado.'
        });
      }
    }
  }
};
</script>

<style scoped>
/* Adicione seu estilo aqui, se necessário */
</style>
