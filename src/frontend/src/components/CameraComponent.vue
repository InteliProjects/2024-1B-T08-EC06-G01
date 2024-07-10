<template>
      <div class="flex-1 flex flex-col items-center justify-center bg-white p-4">
        <!-- Real-time Image Box -->
        <div class="flex-1 flex items-center justify-center bg-gray-200 w-full mb-4">
          <!-- <span>Imagem em tempo real</span> -->
          <img id="videoStream" alt="Imagem em tempo real" style="width: 640px; height: 480px;" />

        </div>
          <span id="latency">Latência: 0ms</span>
        </div>
  </template>
z
  <script>
  const websocket = new WebSocket(import.meta.env.VITE_VIDEO_WEBSOCKET);
  let latencies = [];

  console.log(import.meta.env.VITE_VIDEO_WEBSOCKET)

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
    // console.log('Message:', event.data);
    const json = JSON.parse(event.data);
    document.getElementById('videoStream').src = `data:image/jpeg;base64,${json.bytes}`;

    if ("timestamp" in json)
      latencies.push(new Date() - new Date(json.timestamp));

    if (latencies.length > 10) {
      const sum = latencies.reduce((a, b) => a + b, 0);
      const avg = sum / latencies.length;
      document.getElementById('latency').innerText = `Latência: ${Math.round(avg)}ms`;
      latencies = [];
    }
  };


  export default {
    name: 'CameraComponent'
  }
  </script>

  <style scoped>
  .box {
    background-color: #f0f0f0;
    height: 100%;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 20px;
  }
  </style>
