from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Rutas relativas al paquete instalado (independiente del usuario / máquina)
CONFIG_DIR = os.path.join(get_package_share_directory('sim_utilities'), 'config')
SENSOR_CONFIG_YAML = os.path.join(CONFIG_DIR, 'sensor_config.yaml')
EKF_YAML = os.path.join(CONFIG_DIR, 'ekf.yaml')
NAVSAT_TRANSFORM_YAML = os.path.join(CONFIG_DIR, 'navsat_transform.yaml')
NAV2_PARAMS_YAML = os.path.join(CONFIG_DIR, 'nav2_params.yaml')

def generate_launch_description():
    return LaunchDescription([
        #sensor republisher node
        Node(
            package='sim_utilities',
            executable='republisher',
            parameters=[SENSOR_CONFIG_YAML]
        ),
        #cmd translator node
        Node(
            package='sim_utilities',
            executable='cmd_translator_node',
            name='cmd_translator',
            output='screen',
        ),
        #cmd node
        Node( 
            package='sim_utilities',
            executable='cmd_node',
            name='cmd_node',
            output='screen',            
        ),
        # empty map publisher
        Node(
            package='sim_utilities',
            executable='empty_map_pub',
            name='empty_map_pub',
            output='screen'
        ),

        #EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[EKF_YAML],
            remappings=[
                ("/odometry/filtered", "/localization/odometry"),
                ("/odometry/gps", "/odometry/gps2")]        
        ),

        # NavSat Transform Node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            respawn=True,
            parameters=[NAVSAT_TRANSFORM_YAML],
            remappings=[
            ('/imu', '/navsat/imu'),
            ('/gps/fix', '/navsat/gps'),
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
            ],
        ),
    ])
    