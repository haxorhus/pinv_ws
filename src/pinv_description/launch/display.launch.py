"""Muestra el modelo del catamarán en RViz (para revisar el URDF)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pinv_description')

    return LaunchDescription([
        DeclareLaunchArgument('rvizconfig',
                              default_value=os.path.join(share, 'rviz', 'config.rviz')),
        IncludeLaunchDescription(os.path.join(share, 'launch', 'description.launch.py')),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
    ])
