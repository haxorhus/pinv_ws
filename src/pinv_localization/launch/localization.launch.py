"""Localización: EKF (IMU + GPS), navsat_transform y la TF fija map -> odom."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('pinv_localization'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # map -> odom identidad: el origen del mapa es donde arranca el bote.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(config, 'ekf.yaml'),
                        {'use_sim_time': use_sim_time}],
        ),

        # Suscribe imu, gps/fix y odometry/filtered; publica odometry/gps.
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(config, 'navsat.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('imu', '/imu/data')],
        ),
    ])
