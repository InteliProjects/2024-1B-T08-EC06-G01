import asyncio
import json
import threading

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

clients = set()

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

def ws_app(robot):
    app = FastAPI()

    # Rota para receber comandos de movimento do robô
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

                # Enviar aviso de obstaculo
                # await websocket.send_text(json.dumps(robot.scan_callback()))

                # Verificar se o comando é de parada de emergência
                #if command == 'emergency_stop':
                 #   robot.call_emergency_stop_service()
                #else:
                    # Atualizar o estado do robô
                  #  robot.state = command


        except Exception as e:
            print(f"Erro: {e}")
            await websocket.close()

    return app

async def _broadcast(message):
    asyncio.gather(*[client.send_text(message) for client in clients])

def broadcast(message):
    asyncio.run(_broadcast(message))

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    # Iniciar o servidor WebSocket em uma thread separada
    ws_thread = threading.Thread(target=lambda: uvicorn.run(ws_app(robot), host="0.0.0.0", port=3000))
    ws_thread.start()

    rclpy.spin(robot)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
