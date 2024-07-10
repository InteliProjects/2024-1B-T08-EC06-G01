#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        qos = QoSProfile(depth=10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.safe_distance = 0.2 # Defina a distância de segurança em metros

    def scan_callback(self, data):
        # Obtém os dados do Lidar
        ranges = data.ranges

        # Verifica se há obstáculos dentro da distância de segurança e enviar o índice da array
        if min(ranges) <= self.safe_distance:
            min_index = ranges.index(min(ranges))
            numero_indices = len(ranges)
            print('Número de índices:', numero_indices)

            # Calcule os índices que representam a frente e as traseiras
            valor_A = numero_indices // 4
            valor_B = valor_A * 3

            print('Valor A:', valor_A)
            print('Valor B:', valor_B)
            print('Índice:', min_index)

            # Dividir o array de distâncias em frente (de valor_A até valor_B) e trás (de valor_B até o final mais de 0 até valor_A)
            if valor_A < min_index < valor_B:
                print("Obstáculo átras")
            else:
                print("Obstáculo na frente")
        else:
            # Se não houver obstáculos próximos, continue em frente
            print('Nenhum obstáculo detectado')

def main(args=None):
    print('Starting scan listener')
    rclpy.init(args=args)
    avoidance = CollisionAvoidance()
    rclpy.spin(avoidance)
    rclpy.shutdown()
    print('done.')

if __name__ == '__main__':
    main()
