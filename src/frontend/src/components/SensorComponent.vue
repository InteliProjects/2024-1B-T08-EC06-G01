<template>
        <!-- Temperature Display -->
        <div class="flex items-center">
          <div class="w-20 h-20 bg-gradient-to-r from-green-400 to-yellow-400 rounded-full flex items-center justify-center">
            <span class="text-2xl" id="temperature-data">45º</span>
          </div>
          <span class="ml-2 text-xl">Temperatura Atual</span>
        </div>
  </template>

  <script>
  export default {
    name: 'SensorComponent',
  }

  const websocket = new WebSocket(import.meta.env.VITE_CONTROL_WEBSOCKET);

  console.log(import.meta.env.VITE_CONTROL_WEBSOCKET)

  websocket.onopen = () => {
    console.log('Connected to the control websocket');
  };

  websocket.onclose = () => {
    console.log('Disconnected from the control websocket');
  };

  websocket.onerror = (error) => {
    console.error('Error:', error);
  };

  websocket.onmessage = (event) => {
    const json = JSON.parse(event.data);

    if ("temperature" in json) {
      console.log('Temperature:', json.temperature);
      document.getElementById('temperature-data').innerText = `${Number(json.temperature).toFixed(2)}º`;
    }
  }

  </script>

  <style scoped>
  .bottom {
    background-color: #d4edda;
    height: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 20px;
  }
  </style>
