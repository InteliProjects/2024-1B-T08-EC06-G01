---
title: Classe Robô
sidebar_position: 2
---

# Classe Robô

Nesse tópico, vamos apresentar como foi construido a classe do robô que é responsável por fazer o conrole do robô e das suas funções. Para realizar isso, estamos utilizando a blibioteca RCLPY para fazer a criação dos nós do ROS. Nessa sprint, focamos em fazer o controle básico do robô, onde a cada a ação vai fazer o robô se mover na direção pedida por um certo periodo de tempo e em uma certa velocidade.

```python

class TurtleBot(Node):
    def __init__(self):
        super().__init__('turtlebot')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    # Função para enviar comandos de movimento para o robô
    def move(self, linear: Vector3, angular: Vector3, duration: float):
        msg = Twist()
        msg.linear = linear
        msg.angular = angular
        self.publisher_.publish(msg)

        # Esperar o tempo de duração antes de para o movimento
        time.sleep(duration)  # Simples delay para esperar antes de parar o movimento
        self.stop()

    def stop(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def move_forward(self, speed: float, duration: float):
        # Criar um Vector3 com a velocidade linear
        self.move(Vector3(x=speed, y=0.0, z=0.0), Vector3(), duration)

    def move_backward(self, speed: float, duration: float):
        self.move(Vector3(x=-speed, y=0.0, z=0.0), Vector3(), duration)

    def rotate_left(self, speed: float, duration: float):
        # Criar um Vector3 com a velocidade angular
        self.move(Vector3(), Vector3(z=speed), duration)

    def rotate_right(self, speed: float, duration: float):
        self.move(Vector3(), Vector3(z=-speed), duration)

    # Seta todas as velocidades para 0 para parar o robô imediatamente
    def emergency_stop(self):
        self.move(Vector3(x=0.0, y=0.0, z=0.0), Vector3(), 0.0)

```

A seguir será descrito o que cada função faz:

- `move`: Função para enviar comandos de movimento para o robô. Recebe como parâmetros a velocidade linear, a velocidade angular e a duração do movimento.
- `stop`: Função para parar o robô.
- `move_forward`: Função para fazer o robô se mover para frente. Recebe como parâmetros a velocidade e a duração do movimento.
- `move_backward`: Função para fazer o robô se mover para trás. Recebe como parâmetros a velocidade e a duração do movimento.
- `rotate_left`: Função para fazer o robô girar para a esquerda. Recebe como parâmetros a velocidade e a duração do movimento.
- `rotate_right`: Função para fazer o robô girar para a direita. Recebe como parâmetros a velocidade e a duração do movimento.
- `emergency_stop`: Função para parar o robô imediatamente.
- `__init__`: Função construtora da classe, onde é inicializado o nó do ROS e o publisher para enviar comandos de movimento para o robô.
- `publisher_`: Publisher para enviar comandos de movimento para o tópico de velocidade do robô.

Nas próximas sprints, pretendemos fazer melhorias na movimentação do robô, por exemplo, o giro por graus e o movimento autônomo.