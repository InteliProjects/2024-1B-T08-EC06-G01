<template>
  <div class="flex justify-center items-center w-full">
    <div class="flex flex-col items-center space-y-2">
      <div>
        <button @mouseover="mouseOverFn" @mouseleave="mouseLeaveFn" :class="{ 'clicked': isForwardClicked }"><ArrowUp /></button>

      </div>
      <div class="flex space-x-2">
        <button @mouseover="mouseOverFn" @mouseleave="mouseLeaveFn" :class="{ 'clicked': isLeftClicked }"><ArrowLeft /></button>
        <button @mouseover="mouseOverFn" @mouseleave="mouseLeaveFn" :class="{ 'clicked': isBackwardClicked }"><ArrowDown /></button>
        <button @mouseover="mouseOverFn" @mouseleave="mouseLeaveFn" :class="{ 'clicked': isRightClicked }"><ArrowRight /></button>
      </div>
      <p v-show="showMessage">
        {{ message }} 
        </p>
    </div>
  </div>
</template>

<script>
  // Importando o SVG das setinhas
  import ArrowUp from '../assets/arrow-up.svg';
  import ArrowDown from '../assets/arrow-down.svg';
  import ArrowRight from '../assets/arrow-right.svg';
  import ArrowLeft from '../assets/arrow-left.svg';

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


  export default {
    name: 'TeleopComponent',

    // Adicionando o SVG das setinhas ao componente
    components: {
      ArrowUp,
      ArrowDown,
      ArrowRight,
      ArrowLeft
    },

    // Data é onde você define as variáveis que serão utilizadas no template
    data() {
      return {
        isForwardClicked: false,
        isBackwardClicked: false,
        isLeftClicked: false,
        isRightClicked: false,
        websocket: websocket,
        message: 'Use as setas do teclado para movimentar o robô',
        showMessage: false
      }
    },


    // Em métodos vocÊ define as funções que serão chamadas no template
    methods: {
      mouseOverFn() {
        this.showMessage = true;
        console.log('Mouse Over')
      },

      mouseLeaveFn() {
        this.showMessage = false;
        console.log('Mouse Leave')
      },

      emergencyStop() {
        console.log('Emergency Stop')
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "emergency"
          }
        }))

        return this.$notify({
            title: 'Emergência',
            text:'A parada de emergência foi acionada, para continuar movimentando o robô, reinicie o serviço responsável pelo controle do robô',
            type: 'error'
        });
      },

      dirtyInfo() {
        console.log('Dirty Info')
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "dirty"
          }
        }))

        return this.$notify({
            title: 'Dirty',
            text:'O botão que notifica se a imagem apresenta sujeira ou não foi ativado',
            type: 'error'
        });
      },

      moveForward() {
        this.isForwardClicked = true;
        console.log('Moving Forward');
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "forward"
          }
        }))

      },
      moveBackward() {
        this.isBackwardClicked = true;
        console.log('Moving Backward')
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "backward"
          }
        }))
      },
      moveLeft() {
        this.isLeftClicked = true;
        console.log('Moving Left')
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "left"
          }
        }))
      },
      moveRight() {
        this.isRightClicked = true;
        console.log('Moving Right')
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "right"
          }
        }))
      },

      // As funções abaixo são utilizadas para detectar quando as teclas foram pressionadas
      handleKeypress(event) {
      if (event.key === 'w' || event.key === 'ArrowUp' || event.key === 'W') {
        this.moveForward();
      } else if (event.key === 's' || event.key === 'ArrowDown' || event.key === 'S') {
        this.moveBackward();
      } else if (event.key === 'a' || event.key === 'ArrowLeft' || event.key === 'A') {
        this.moveLeft();
      } else if (event.key === 'd' || event.key === 'ArrowRight' || event.key === 'D') {
        this.moveRight();
      } else if (event.key === 'q' || event.key === 'Q') {
        this.emergencyStop();
      } else if (event.key === 'm' || event.key === 'M') {
        this.dirtyInfo();
      }
    },

    handleKeyup(event) {
      if (event.key === 'w' || event.key === 'ArrowUp' || event.key === 'W') {
        this.isForwardClicked = false;
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "stopped"
          }
        }))
      }
      else if (event.key === 's' || event.key === 'ArrowDown' || event.key === 'S') {
        this.isBackwardClicked = false;
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "stopped"
          }
        }))
      }
      else if (event.key === 'a' || event.key === 'ArrowLeft' || event.key === 'A') {
        this.isLeftClicked = false;
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "stopped"
          }
        }))
      }
      else if (event.key === 'd' || event.key === 'ArrowRight' || event.key === 'D') {
        this.isRightClicked = false;
        this.websocket.send(JSON.stringify({
          type: "CPacketControl",
          data: {
            state: "stopped"
          }
        }))
      }
    }
  },

  mounted() {
    window.addEventListener('keydown', this.handleKeypress);
    window.addEventListener('keyup', this.handleKeyup);
    this.websocket.onmessage = (event) => {
      console.log('Message:', event.data);
      const json = JSON.parse(event.data);


      if ("obstacle" in json) {
        const obstacle = json.obstacle;

        if (obstacle != 'none')
          return this.$notify({
            title: 'Obstaculo detectado',
            text:`Obstaculo detectado na direção "${obstacle}"`,
            type: 'warn'
          });
      }

      if ("type" in json) {
        switch (json.type) {
          case "SPacketError":
            return this.$notify({
              title: 'Erro',
              text: json.data.message,
              type: 'error'
            });
          case "SPacketInfo":
            return this.$notify({
              title: 'Notificação',
              text: json.data.message,
              type: 'info'
            });
        }
      }

      if ("dirty" in json) {
        const dirty = json.dirty;

        if (dirty == true)
          return this.$notify({
            title: 'Sujeira detectada',
            text:`A sujeira foi detectada pelo robô`,
            type: 'warn'
          });
      }

      console.log(event)
    };
  },

  beforeDestroy() {
    window.removeEventListener('keydown', this.handleKeypress);
    window.removeEventListener('keyup', this.handleKeyup);
  }
};
  </script>

  <style scoped>
  .top {
    background-color: #cce5ff;
    height: 50%;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 20px;
  }

  button {
    background-color: transparent;
    border: none;
    cursor: pointer;
  }

  .clicked {
    filter: invert(48%) sepia(79%) saturate(276%) hue-rotate(86deg) brightness(118%) contrast(119%);
  }



  </style>
