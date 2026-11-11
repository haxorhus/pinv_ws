"""
Nav2 para el catamarán: map_server con mapa vacío, planner, RPP y BT sin reversa.

Escucha /odometry/filtered y la TF map -> odom -> base_link (pinv_localization).
Publica /cmd_vel (geometry_msgs/Twist) directo desde controller_server.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAV2_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
]


def generate_launch_description():
    share = get_package_share_directory('pinv_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    common = {'use_sim_time': use_sim_time}
    bt_xml = {
        'default_nav_to_pose_bt_xml':
            os.path.join(share, 'behavior_trees', 'navigate_to_pose_asv.xml'),
        'default_nav_through_poses_bt_xml':
            os.path.join(share, 'behavior_trees', 'navigate_through_poses_asv.xml'),
    }

    map_server = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, common,
                        {'yaml_filename': LaunchConfiguration('map')}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[common, {'autostart': True, 'node_names': ['map_server']}],
        ),
    ]

    nav2 = [
        Node(
            package=package,
            executable=name,
            name=name,
            output='screen',
            parameters=[params_file, common] + ([bt_xml] if name == 'bt_navigator' else []),
        )
        for package, name in NAV2_NODES
    ]
    nav2.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[common, {'autostart': True,
                             'node_names': [name for _, name in NAV2_NODES]}],
    ))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(share, 'maps', 'empty_400m.yaml'),
            description='Mapa vacío; el origen (0, 0) es la posición inicial del bote'),
        *map_server,
        *nav2,
    ])
