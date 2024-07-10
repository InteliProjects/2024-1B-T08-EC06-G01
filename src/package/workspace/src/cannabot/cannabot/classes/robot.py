import inquirer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

# from turtlebot3_teleop import


class TurtleBot(Node):
	def __init__(self):
		# super().__init__('turtlebot')

		self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

		# self.subscription = self.create_subscription(String, 'turtlebot_commands', self.callback, 10)
		# self.subscription

	# Callback para receber comandos de movimento da CLI
	# def callback(self, msg):
	# 	mov_command = msg.data
	# 	# speed = 0.1

	# 	if mov_command == 'front':
	# 		self.move_forward(0.1)
	# 	elif mov_command == 'back':
	# 		self.move_backward(0.1)
	# 	elif mov_command == 'left':
	# 		self.rotate_left(0.1)
	# 	elif mov_command == 'right':
	# 		self.rotate_right(0.1)

	# 	self.get_logger().info('Comando de movimento recebido: "%s"' % msg.data)

	# Função para enviar comandos de movimento para o robô
	def move(self, linear: Vector3, angular: Vector3):
		msg = Twist()
		msg.linear = linear
		msg.angular = angular
		self.publisher_.publish(msg)

	def move_forward(self, speed: float):
		# Criar um Vector3 com a velocidade linear
		self.move(Vector3(x=speed, y=0.0, z=0.0), Vector3())

	def move_backward(self, speed: float):
		self.move(Vector3(x=-speed, y=0.0, z=0.0), Vector3())

	def rotate_left(self, speed: float):
		# Criar um Vector3 com a velocidade angular
		self.move(Vector3(), Vector3(z=speed))

	def rotate_right(self, speed: float):
		self.move(Vector3(), Vector3(z=-speed))


# def main(args=None):
# 	rclpy.init(args=args)

# 	turtlebot = TurtleBot()

# 	rclpy.spin(turtlebot)

	# turtlebot.destroy_node()
	# rclpy.shutdown()