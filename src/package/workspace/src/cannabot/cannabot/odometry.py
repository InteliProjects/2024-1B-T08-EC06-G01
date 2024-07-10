import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.subscription import Subscription


class OdometrySubscriber:
	def __init__(self):
		self.node: rclpy.Node = rclpy.create_node('odometry_subscriber') # type: ignore
		self.subscription: Subscription = self.node.create_subscription(
			Odometry, '/odom', self.odometry_callback, 10
		)
		self.odom_counter: int = 19

	def odometry_callback(self, msg: Odometry):
		self.odom_counter += 1
		if type(msg.pose.pose) == Pose and self.odom_counter % 20 == 0:
			self.odom_counter = 0
			current_position = msg.pose.pose.position
			self.node.get_logger().info(f'Current position: {current_position}')


def main():
	rclpy.init()
	odometry_subscriber = OdometrySubscriber().node
	odometry_subscriber.get_logger().info('Odometry subscriber started')
	try:
		rclpy.spin(odometry_subscriber)
	finally:
		odometry_subscriber.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()