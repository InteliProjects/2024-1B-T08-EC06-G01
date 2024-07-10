---
title: "Teleoperação"
sidebar_position: 1
description: Essa secção detalha o sistema da teleoperação presente no projeto, indicando suas características e mudanças feitas ao longo do projeto.
---

## Introdução

&emsp;Para projetos que envolvem a operação de um robô, torna-se essencial utilizar a teleoperação para controlar o robô a partir de uma distância. A teleoperação permite que operadores direcionem as ações do robô remotamente, oferecendo uma flexibilidade crucial em diversos cenários onde a presença física direta pode ser inviável ou arriscada. A partir desse pressuposto, o projeto foi desenvolvido com a utilização da teleoperação do robô, permitindo que ele execute comandos predeterminados pela aplicação com precisão e eficiência.

## Teleoperação

&emsp;A comunicação com o robô neste projeto foi feita a partir do WebSocket, uma tecnologia de comunicação bidirecional entre cliente e servidor que permite a troca contínua de informações em tempo real. Diferente dos métodos tradicionais de comunicação HTTP, que são unidirecionais e requerem que o cliente faça solicitações individuais ao servidor, o WebSocket estabelece uma conexão persistente que facilita uma interação mais dinâmica e responsiva. Isso é vantajoso em aplicações que demandam atualização constante e imediata, como no caso do controle de robôs.

&emsp;Para implementar essa tecnologia, utilizou-se o framework FastAPI, conhecido por sua eficiência e capacidade de lidar com altas demandas de tráfego. O FastAPI proporciona toda a funcionalidade da API necessária para a comunicação com o robô. A escolha dele se justifica por sua performance superior, fácil integração com outras ferramentas e bibliotecas, e suporte a assinaturas assíncronas, o que é crucial para o manuseio eficiente de WebSockets.

### WebSocket

&emsp;Para utilizar o WebSocket para controlar o robô do projeto, foi necessário configurar as rotas da comunicação entre o cliente e o servidor. A implementação do WebSocket foi realizada no arquivo `websocket.py`, onde esta configuração foi estabelecida. O código a seguir ilustra como essas rotas foram configuradas:

```python
@router.websocket("/control")
async def control_robot(websocket: WebSocket):
	await websocket.accept()
	await robot.add_client(websocket)
	await camera.add_client(websocket)

	try:
		while True:
			raw = await websocket.receive_text()

			if not robot.websocket:
				continue

			try: data = json.loads(raw)
			except json.JSONDecodeError:
				await websocket.send_json({ "type": "SPacketError", "data": {
					"message": "JSON inválido"
				}})
				continue

			try: packet = ControlPacket(**data)
			except pydantic.ValidationError as e:
				await websocket.send_json({ "type": "SPacketError", "data": {
					"message": str(e).replace('\n', '')
				}})
				continue

			await robot.send(json.dumps({ 'control': packet.data.state }))

	except WebSocketDisconnect:
		await robot.remove_client(websocket)
		await camera.remove_client(websocket)
		await websocket.close()
	except ConnectionClosedError:
		print("Connection to robot has been lost.")
		await robot.reconnect()
		await websocket.send_json({ "type": "SPacketError", "data": {
			"message": "Conexão com o robô foi perdida"
		}})
		return
```

&emsp;Segue a documentação desta outra rota, que também foi necessária para implementar o controle do robô:

```python
@app.websocket("/ws_control")
    async def websocket_endpoint(websocket: WebSocket):
        await websocket.accept()

        clients.add(websocket)

        try:
            while True:
                # Receber o comando de movimento do robô
                data = await websocket.receive_text()

                print(f"Recebido: {data}")

                # Parse do JSON recebido
                message_data = json.loads(data)
                command = message_data['control']  # Comando de movimento
                print(f"Comando: {command}")
                # Atualizar o estado do robô

                if command not in ['stopped', 'forward', 'left', 'right', 'backward', 'emergency']:
                    # Enviar mensagem de erro pelo WebSocket
                    await websocket.send_text(json.dumps({'error': 'Comando inválido'}))
                    continue

                if command == "emergency":
                    robot.emergency()
                    break

                robot.state = command

        except Exception as e:
            print(f"Erro: {e}")
            await websocket.close()

    return app
```
### Controle do robô

&emsp;Para desenvolver o controle do robô e gerenciar todos os serviços, tópicos e ações necessários, foi criada uma classe chamada Robot. Essa classe é responsável por operar a comunicação e o comportamento do robô, integrando-se com diferentes tópicos ROS e serviços para permitir uma operação eficiente e segura. Segue a implementação da classe:

```python
class Robot(Node):
    def __init__(self):
        super().__init__('ros_turtlebot_teleop') # type: ignore

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.odom_counter: int = 19
        self.create_subscription(String, '/sensor_data', self.temp_callback, 10)
        self.create_service(Empty, '/emergency_stop_teleop', self.emergency_stop_external)
        self.reported_speed = Twist()

        self.state = 'stopped'
        self.lidar_data = 'none'
        self.ready = False

        # Subscrever no tópico do Lidar
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.safe_distance = 0.2 # Distância de segurança em metros

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Aguardando o estado de prontidão do robô...')

         # Cliente de serviço
        self.cli = self.create_client(Empty, '/emergency_stop_teleop')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Serviço /emergency_stop_teleop não disponível, esperando...')
        self.req = Empty.Request()

    def emergency_stop_external(self, request, response):
        self.get_logger().info('Recebido pedido de parada de emergência externa')
        self.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        self.stop()
        return response

    def scan_callback(self, data):
        # Obtém os dados do Lidar
        ranges = data.ranges

        # Verifica se há obstáculos dentro da distância de segurança e enviar o índice da array
        if min(ranges) <= self.safe_distance:
            min_index = ranges.index(min(ranges))
            numero_indices = len(ranges)
            # print('Número de índices:', numero_indices)

            # Calcular os índices que representam a frente e as traseiras
            valor_A = numero_indices // 4
            valor_B = valor_A * 3

            # print('Valor A:', valor_A)
            # print('Valor B:', valor_B)
            # print('Índice:', min_index)

            # Dividir o array de distâncias em frente (de valor_A até valor_B) e trás (de valor_B até o final mais de 0 até valor_A)
            if valor_A < min_index < valor_B:
                # print("Obstáculo átras")
                # return {'obstacle': 'back'}

                if self.lidar_data != 'back':
                    print('Obstáculo detectado atrás')
                    self.lidar_data = 'back'
                    broadcast(json.dumps({'obstacle': 'back'}))
            else:
                # print("Obstáculo na frente")
                # return {'obstacle': 'front'}

                if self.lidar_data != 'front':
                    print('Obstáculo detectado à frente')
                    self.lidar_data = 'front'
                    broadcast(json.dumps({'obstacle': 'front'}))
        else:
            # Se não houver obstáculos próximos, continue em frente
            # print('Nenhum obstáculo detectado')
            # return {'obstacle': 'none'}

            if self.lidar_data != 'none':
                print('Nenhum obstáculo detectado')
                self.lidar_data = 'none'
                broadcast(json.dumps({'obstacle': 'none'}))

    def call_emergency_stop_service(self):
        self.get_logger().info('Chamando serviço de parada de emergência...')
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Parada de emergência realizada com sucesso')
        else:
            self.get_logger().error('Falha ao chamar o serviço de parada de emergência')

    def timer_callback(self):
        twist = Twist()

        match self.state:
            case 'stopped':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            case 'forward':
                if self.lidar_data == 'none' or self.lidar_data == 'back':
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0
            case 'left':
                twist.linear.x = 0.0
                twist.angular.z = 1.0
            case 'right':
                twist.linear.x = 0.0
                twist.angular.z = -1.0
            case 'backward':
                if self.lidar_data == 'none' or self.lidar_data == 'front':
                    twist.linear.x = -0.2
                    twist.angular.z = 0.0
            case 'emergency':
                self.emergency()
            case _:
                self.get_logger().warn(f'Invalid state: {self.state}')

        self.publisher.publish(twist)

    def odometry_callback(self, msg: Odometry):
        self.odom_counter += 1
        self.reported_speed = msg.twist.twist

        if type(msg.pose.pose) == Pose and self.odom_counter % 20 == 0:
            self.odom_counter = 0
            current_position = msg.pose.pose.position
            broadcast(json.dumps({'position': {'x': current_position.x, 'y': current_position.y}}))

        if not self.ready:
            self.ready = True
            self.get_logger().info('Robô disponível! Iniciando teleoperação...')


    def temp_callback(self, msg):
        jsonified = json.loads(msg.data)
        self.get_logger().info(f'Temperatura: {jsonified}')
        broadcast(json.dumps({
            'temperature': jsonified['temperature_celsius']
        }))


    def emergency(self):
        self.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        self.stop()

    def stop(self):
        self.publisher.publish(Twist())
        self.get_logger().info('Parando o robô...')
        rclpy.shutdown()
```

## Conclusão

&emsp;Dessa forma, é possível concluir que a implementação do controle de robôs via teleoperação, utilizando tecnologias como WebSocket e FastAPI, permite uma comunicação eficiente e em tempo real entre o operador e o robô. A utilização do WebSocket facilita a comunicação bidirecional entre o cliente e o servidor, permitindo o envio e recebimento de comandos em tempo real. 

&emsp;Para melhorar ainda mais a teleoperação do robô, é possível considerar alguns próximos passos, como por exemplo a redução da latência existente na comunicação, pois quanto mais rápido for a resposta do robô em relação aos comandos do usuário, melhor será a operação do robô. A partir desta melhoria, será possível melhorar a funcionalidade e a segurança do sistema de teleoperação do robô.