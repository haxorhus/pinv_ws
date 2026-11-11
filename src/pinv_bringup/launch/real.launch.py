"""
Lanza todo el GNC en el bote real (Raspberry Pi 5).

    ros2 launch pinv_bringup real.launch.py
    ros2 launch pinv_bringup real.launch.py serial_port:=/dev/ttyACM1
    ros2 launch pinv_bringup real.launch.py microros:=false   # sin ESP32

Contrato con el hardware (todo lo de arriba no cambia si se cambia el firmware):
    /imu/data  sensor_msgs/Imu          ESP32 -> Pi   imu_link, 50 Hz, best-effort
    /gps/fix   sensor_msgs/NavSatFix    driver RTK    gps_link, 5-10 Hz
    /cmd_vel   geometry_msgs/Twist      Pi -> ESP32   >= 10 Hz, watchdog 500 ms

PENDIENTE: el driver del GPS RTK (y su cliente NTRIP) se agrega acá cuando se
defina el receptor.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        path, launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items())


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('microros', default_value='true',
                              description='Lanzar el agente micro-ROS para el ESP32'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Puerto USB del ESP32-S3'),
        DeclareLaunchArgument('baudrate', default_value='115200'),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', LaunchConfiguration('serial_port'),
                       '-b', LaunchConfiguration('baudrate')],
            condition=IfCondition(LaunchConfiguration('microros')),
        ),

        include('pinv_description', 'description.launch.py'),
        include('pinv_localization', 'localization.launch.py'),
        include('pinv_navigation', 'navigation.launch.py'),
    ])
