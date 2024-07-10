---
title: "Sistema de segurança atualizado"
sidebar_position: 1
description: Nessa secção iremos atualizar em relação ao sistema de segurança que atua sobre o robô.
---

## Botão de parada de emergência

Durante a Sprint 2, a equipe Cannabot apresentou a primeira versão da funcionalidade de parada de emergência do robô, inicialmente implementada através da interface de linha de comando (CLI). Nesta sprint, com a introdução da interface de usuário, foram realizadas alterações significativas na forma de ativação dessa função.

Agora, ao clicar no botão de parada de emergência na tela "Central de Controle" do frontend, o usuário aciona o serviço "emergency_stop_teleop". Este serviço, ao ser requisitado, executa a função de parada do robô.

A seguir, está o código de configuração do serviço:

```Python
class Robot(Node):
    def __init__(self):
        super().__init__('ros_turtlebot_teleop')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.create_service(Empty, '/emergency_stop_teleop', self.emergency_stop_external)
        self.reported_speed = Twist()

        self.state = 'stopped'
        self.ready = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Aguardando o estado de prontidão do robô...')

    def emergency_stop_external(self, request, response):
        self.get_logger().info('Recebido pedido de parada de emergência externa')
        self.get_logger().info('PARADA DE EMERGÊNCIA ATIVADA')
        self.stop()
        return response
```

Neste código, primeiro é criado o serviço de parada de emergência através da função create_service, que associa o serviço ao tópico /emergency_stop_teleop. Quando esse serviço é requisitado, a função emergency_stop_external é executada, registrando a solicitação de parada de emergência nos logs e chamando a função stop().

A função stop() é crucial para a segurança do sistema, pois é responsável por interromper qualquer movimento do robô. Ela publica uma mensagem vazia no tópico de velocidade, o que efetivamente para o robô, e em seguida, encerra o nó ROS, garantindo que nenhuma outra operação seja executada. O código da função stop() é mostrado abaixo:

```Python 
 def stop(self):
    self.publisher.publish(Twist())
    self.get_logger().info('Parando o robô...')
    rclpy.shutdown() 
```
Essa implementação garante que, ao receber o comando de parada de emergência, o robô interrompa imediatamente todas as suas operações, proporcionando uma camada adicional de segurança essencial em situações críticas. O uso de logs detalhados permite o monitoramento eficaz do processo, assegurando que cada etapa da parada de emergência seja devidamente registrada e verificada.

## Detecção de obstáculo e parada imediata

Uma abordagem crucial para garantir a segurança do sistema foi implementada através da detecção de obstáculos à frente ou atrás do robô. Essa funcionalidade desempenha um papel fundamental na prevenção de colisões e na garantia da integridade do equipamento e do ambiente ao seu redor.

A detecção de obstáculos permite que o robô reaja de forma proativa, impedindo qualquer movimento em direção a esses obstáculos, mesmo que o usuário tente comandar essa ação. Essa capacidade é especialmente vital em ambientes dinâmicos e imprevisíveis.

Para incorporar essa funcionalidade, o primeiro passo foi utilizar o Lidar. O LIDAR (Light Detection and Ranging) é um sensor que mede distâncias e detecta objetos ao redor do robô. Esse sensor emite um feixe de laser que varre o ambiente, calculando o tempo que leva para o feixe refletir de volta ao sensor após atingir um objeto. No modelo do Turtlebot3, o robô utilizado nesta prova de conceito, o Lidar já está integrado.

Com base nos dados fornecidos pelo Lidar, foi possível implementar a lógica a seguir, que impede a colisão do robô:

```Python
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
```

No código apresentado, a variável "ranges" recebe uma lista contendo todos os registros do lidar em uma varredura de 360 graus. Como essa lista é dinâmica, foi criada uma outra variável chamada "numero_indices", que armazena o tamanho dessa lista. A partir dessa informação, é possível implementar a lógica para determinar se há obstáculos à frente ou atrás do robô.

A lógica empregada consiste em dividir a lista em quatro partes para definir os quadrantes que compõem a circunferência do lidar. A partir disso, são criadas as variáveis A e B, que representam, respectivamente, as delimitações do primeiro e do último quadrante, partindo do ângulo 0. Para detectar se um obstáculo está atrás do robô, basta verificar se seu índice na lista é maior que A e menor que B, ou seja, se ele está no segundo ou terceiro quadrante. Caso contrário, o obstáculo está à frente do robô.

É importante ressaltar que consideramos uma distância de 0.2 metros como critério para determinar a presença ou não de um obstáculo.

No código a seguir, as condições para a movimentação do robô são definidas. No caso "forward" (avançar), o robô só pode se locomover se o sensor lidar indicar que há um obstáculo atrás ou se não houver obstáculo. Essa mesma lógica é aplicada quando o movimento é para trás.

```Python
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
```
Essa abordagem dinâmica permite que o robô ajuste seu comportamento de acordo com a presença de obstáculos, garantindo uma navegação segura e eficiente em ambientes desafiadores. O monitoramento constante do Lidar e a adaptação contínua do comportamento do robô contribuem significativamente para a segurança e a confiabilidade do sistema como um todo.