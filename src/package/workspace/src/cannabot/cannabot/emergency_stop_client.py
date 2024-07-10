import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Importar o serviço padrão Trigger

class EmergencyButtonClient(Node):
    def __init__(self):
        super().__init__('emergency_button_client')
        self.client = self.create_client(Trigger, 'stop_robot')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service available.')

    def send_stop_request(self):
        req = Trigger.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = EmergencyButtonClient()

    response = client.send_stop_request()
    client.get_logger().info(f'Response: success={response.success}, message="{response.message}"')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
