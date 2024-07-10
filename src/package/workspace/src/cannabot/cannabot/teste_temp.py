import json

import bme280
import rclpy
import smbus2
from rclpy.node import Node
from std_msgs.msg import String


class BME280Publisher(Node):
    def __init__(self):
        super().__init__('bme280_publisher') # type: ignore
        self.publisher_ = self.create_publisher(String, '/sensor_data', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

        # BME280 sensor address (default address)
        self.address = 0x76
        # Initialize I2C bus
        self.bus = smbus2.SMBus(1)
        # Load calibration parameters
        self.calibration_params = bme280.load_calibration_params(self.bus, self.address)

    def celsius_to_fahrenheit(self, celsius):
        return (celsius * 9/5) + 32

    def timer_callback(self):
        try:
            # Read sensor data
            data = bme280.sample(self.bus, self.address, self.calibration_params)

            # Extract temperature, pressure, and humidity
            temperature_celsius = data.temperature
            pressure = data.pressure
            humidity = data.humidity

            # Convert temperature to Fahrenheit
            temperature_fahrenheit = self.celsius_to_fahrenheit(temperature_celsius)

            # Create the JSON message
            sensor_data = {
                'temperature_celsius': temperature_celsius,
                'temperature_fahrenheit': temperature_fahrenheit,
                'pressure': pressure,
                'humidity': humidity
            }
            msg = String()
            msg.data = json.dumps(sensor_data)

            # Publish the message
            self.publisher_.publish(msg)

            # Log the readings
            self.get_logger().info(f'Publishing: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = BME280Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
