"""Estadísticas de /imu/data durante N segundos, suscripto best-effort como el EKF.

Uso: python3 imu_stats.py [segundos]   (con ROS 2 Jazzy sourceado)
Esperado con el firmware corregido: 50 Hz, stamps no crecientes = 0,
retraso stamp->recepción de pocos ms, frame imu_link.
"""
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Stats(Node):
    def __init__(self):
        super().__init__('imu_stats')
        self.msgs = []
        self.create_subscription(Imu, '/imu/data', self.cb, qos_profile_sensor_data)

    def cb(self, m):
        self.msgs.append((time.time(), m))


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    n = Stats()
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    msgs = n.msgs
    if len(msgs) < 2:
        print('INCONCLUSIVE: menos de 2 mensajes')
        return 2
    t = [a for a, _ in msgs]
    dts = [b - a for a, b in zip(t, t[1:])]
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in msgs]
    sdts = [b - a for a, b in zip(stamps, stamps[1:])]
    lag = [rx - st for (rx, _), st in zip(msgs, stamps)]
    q = [m.orientation for _, m in msgs]
    norms = [math.sqrt(o.x ** 2 + o.y ** 2 + o.z ** 2 + o.w ** 2) for o in q]
    yaws = [yaw_deg(o) for o in q]
    acc = [m.linear_acceleration for _, m in msgs]
    gz = [a.z for a in acc]
    last = msgs[-1][1]
    print(f'mensajes: {len(msgs)} en {secs:.0f} s')
    print(f'frecuencia recepción: {1 / statistics.mean(dts):.1f} Hz '
          f'(dt max {max(dts) * 1000:.0f} ms)')
    print(f'frecuencia según stamps: {1 / statistics.mean(sdts):.1f} Hz '
          f'(stamps no crecientes: {sum(d <= 0 for d in sdts)})')
    print(f'retraso stamp->recepción: medio {statistics.mean(lag) * 1000:.0f} ms, '
          f'min {min(lag) * 1000:.0f}, max {max(lag) * 1000:.0f} ms')
    print(f'frame_id: {sorted({m.header.frame_id for _, m in msgs})}')
    print(f'norma cuaternión: {min(norms):.4f} .. {max(norms):.4f}')
    print(f'yaw: medio {statistics.mean(yaws):.1f} deg, rango {min(yaws):.1f} .. {max(yaws):.1f}')
    print(f'acc z: medio {statistics.mean(gz):.2f} m/s2; acc x,y último: '
          f'{acc[-1].x:.2f}, {acc[-1].y:.2f}')
    print('covarianzas diag (orient, gyro, acc):',
          [round(last.orientation_covariance[i], 5) for i in (0, 4, 8)],
          [round(last.angular_velocity_covariance[i], 5) for i in (0, 4, 8)],
          [round(last.linear_acceleration_covariance[i], 5) for i in (0, 4, 8)])
    return 0


if __name__ == '__main__':
    sys.exit(main())
