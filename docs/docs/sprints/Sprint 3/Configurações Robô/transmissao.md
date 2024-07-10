---
title: "Transmissão de imagens"
sidebar_position: 1
description: Nessa seção, iremos abordar a transmissão de imagens do robô Turtlebot 3, que é um dos principais componentes do projeto.
---

# Transmissão de imagens

## Introdução

&emsp;Dentre as funcionalidades implementadas no robô na sprint 3, uma delas é a implementação de uma câmera para a realização da transmissão de vídeo em tempo real. A câmera embutida no robô captura imagens que são transmitidas para um computador. Isso permite ao operador visualizar em tempo real o que o robô está vendo, o que proporciona dados valiosos para a Atvos sobre a limpeza dos tubos do reboiler.

### Câmera do robô

<div align="center"> 

![](../../../../..\docs\static\img\sprint3\robocamera.png)

Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;A câmera utilizada é a mesma presente no robô Dobot Magician Lite. Ela é conectada ao Raspberry Pi 4 presente no turtlebot, e assim é configurada para realizar a transmissão das imagens. Ela está posicionada na parte da frente do robô, onde foi fixada com um suporte feito numa impressora 3D, para garantir a qualidade das gravações.
    
### Transmissão do vídeo

<div align="center"> 

![](../../../../..\docs\static\img\sprint3\robovideo.png)

Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;Nesta imagem, é possível ver a transmissão do vídeo em que a câmera embutida no robô transmite em tempo real. A partir do momento que o código é inicializado, a câmera começa e funcionar e enviar as imagens para o computador. Essa transmissão de imagens é feita a partir do websockets, uma tecnologia de comunicação bidirecional que permite a transmissão de dados entre um navegador web e um servidor de maneira eficiente e em tempo real.

### Latência

<div align="center"> 

![](../../../../..\docs\static\img\sprint3\latencia.png)

Fonte: Elaborado pelo grupo Cannabot
</div>

&emsp;A partir desta outra imagem, é possível ver novamente a tela em que as imagens estão sendo transmitidas. Abaixo do quadro com o vídeo da transmissão, há uma estimativa da latência do processo de aquisição, processamento e envio da imagem. 

&emsp;Na imagem acima, a latência se encontra na casa dos 20 mil milissegundos. Essa é uma estimativa ruim, mas vale ressaltar que no momento desta captura de tela a internet da faculdade estava relativamente fraca. Durante a maioria dos testes o robô apresentou uma latência de aproximadamente 500 milissegundos. 

&emsp;Dessa forma, a equipe conseguiu implementar a transmissão de imagens em tempo real no robô, para fazer o monitoramento e controle dos processos de limpeza dos tubos do reboiler.
Esta funcionalidade não só melhora a capacidade de supervisão do operador, mas também proporciona à Atvos dados visuais que podem ser analisados para otimizar ainda mais os processos de limpeza dos reboilers.

### Código

&emsp;Para o funionamento da transmissão de imagens do robô, nós criamos um websocket, que está rodando no robô, que é responsável por enviar as imagens capturadas pela câmera do robô para o nosso frontend. O código do websocket pode ser encontrado no repositório do projeto, através desse caminho`(src/package/camera/main.py)`

A seguir é apresentado o código do websocket:

```python

class WebSocketServer:
    def __init__(self, host='localhost', port=8765, framerate: int = 50):
        self.host = host
        self.port = port
        self.clients = set()
        self.loop = None
        self.thread = None
        self.capture = None
        self.sleep_time = 1 / framerate
        self.framerate = framerate
        self.frame_count = 0

    async def register_client(self, websocket):
        self.clients.add(websocket)
        try:
            async for msg in websocket:
                pass  # Ignore incoming messages
        finally:
            self.clients.remove(websocket)

    async def _broadcast(self, message):
        if self.clients:
            await asyncio.gather(*[client.send(message) for client in self.clients])

    def broadcast(self, message):
        """Synchronous wrapper for the asynchronous _broadcast function."""
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self.loop)

    async def websocket_handler(self, websocket, path):
        await self.register_client(websocket)

    def start_server(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        server_coro = websockets.serve(self.websocket_handler, self.host, self.port)
        self.loop.run_until_complete(server_coro)

        self.loop.create_task(self.broadcast_image_forever())

        print(f"WebSocket server started on ws://{self.host}:{self.port}")
        self.loop.run_forever()

    # def run_in_thread(self):
    #     self.thread = threading.Thread(target=self.start_server)
    #     self.thread.start()

    async def broadcast_image(self):
        if not self.capture:
            self.capture = cv2.VideoCapture(0)

        ret, frame = self.capture.read()
        self.frame_count += 1

        timestamp = None
        if self.frame_count > (self.framerate / 20): # send timestamp every 0.5 seconds
            timestamp = datetime.datetime.now()
            self.frame_count = 0

        if ret:
            _, buffer = cv2.imencode('.jpg', frame)

            broadcast = {
                "bytes": base64.b64encode(buffer).decode('utf-8'),
            }

            if timestamp:
                broadcast["timestamp"] = str(timestamp)

            self.broadcast(json.dumps(broadcast))

    async def broadcast_image_forever(self):
        while not self.loop:
            await asyncio.sleep(0.1)

        while True:
            if len(self.clients) == 0:
                if self.capture:
                    self.capture.release()
                    self.capture = None

                await asyncio.sleep(0.1)
                continue

            self.loop.create_task(self.broadcast_image())
            await asyncio.sleep(self.sleep_time) # 50Hz

```

A classe `WebSocketServer` é responsável por gerenciar a configuração e execução do servidor WebSocket, a captura de imagens da câmera e a transmissão dessas imagens para clientes conectados. A seguir a descrição de cada método da classe:

- `__init__`: Inicializa a instância da classe com os parâmetros de host, porta e framerate.
- `register_client`: Adiciona um novo cliente à lista de clientes conectados.
- `_broadcast`: Método assíncrono que envia uma mensagem para todos os clientes conectados.
- `broadcast`: Método síncrono que chama o método `_broadcast` de forma assíncrona. (Wrapper síncrono)
- `websocket_handler`: Método assíncrono que gerencia a conexão de um novo cliente.
- `start_server`: Inicializa o servidor WebSocket e inicia o loop de eventos.
- `broadcast_image`: Captura uma imagem da câmera, codifica em base64 e envia para os clientes conectados.
- `broadcast_image_forever`: Método assíncrono que executa o método `broadcast_image` em loop enquanto houver clientes conectados.

Na próxima, sprint pretendemos buscar forma de melhorar a latência da transmissão de imagens e otimzar o envio de imagens do robô, já que a latência é um fator crítico para a operação do robô, especialmente em ambientes remotos e com conexões de internet instáveis. Além disso, queremos otimizar para que o programa não sobrecarregue o processamento do Raspberry Pi 4.