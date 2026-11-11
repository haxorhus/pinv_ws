"""Muestra en vivo el rumbo de /imu/data (ENU) para verificar la brújula.

Uso: python3 imu_yaw.py   (con ROS 2 Jazzy sourceado; Ctrl+C para salir)

ENU: 0 = este, 90 = norte, 180/-180 = oeste, -90 = sur; girar la placa
en sentido antihorario (vista desde arriba) tiene que AUMENTAR el yaw.
El rumbo es magnético: puede diferir de la brújula del celular en
"norte verdadero" por la declinación del lugar.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

POINTS = [(0, 'E'), (90, 'N'), (180, 'O'), (-180, 'O'), (-90, 'S')]


class YawPrinter(Node):
    def __init__(self):
        super().__init__('imu_yaw')
        self.count = 0
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.count += 1
        if self.count % 10:          # 5 Hz en pantalla
            return
        q = m.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y ** 2 + q.z ** 2)))
        roll = math.degrees(math.atan2(2 * (q.w * q.x + q.y * q.z),
                                       1 - 2 * (q.x ** 2 + q.y ** 2)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        nearest = min(POINTS, key=lambda p: abs(p[0] - yaw))[1]
        yaw_std = math.degrees(math.sqrt(m.orientation_covariance[8]))
        print(f'yaw ENU {yaw:7.1f} deg (~{nearest})   roll {roll:6.1f}  pitch {pitch:6.1f}'
              f'   sigma_yaw {yaw_std:5.1f} deg', flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(YawPrinter())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
