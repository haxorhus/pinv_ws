from launch import LaunchDescription
from launch.actions import TimerAction
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

    utilities = [

        Node(
            package='sim_utilities',
            executable='republisher',
            parameters=[SENSOR_CONFIG_YAML],
            output='screen'
        ),

        Node(
            package='sim_utilities',
            executable='empty_map_pub',
            name='empty_map_pub',
            output='screen'
        ),

        Node( 
            package='sim_utilities',
            executable='cmd_node',
            name='cmd_node',
            output='screen',            
        ),

        #Node(
        #    package='sim_utilities',
        #    executable='transform_pub',
        #    name='transform_pub',
        #    output='screen',
        #),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[EKF_YAML],
            remappings=[
                ("/odometry/filtered", "/localization/odometry"),
                ("/odometry/gps", "/odometry/gps2")
            ]        
        ),

        # NavSat
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            respawn=True,
            parameters=[NAVSAT_TRANSFORM_YAML],
            remappings=[
                ('/imu', '/navsat/imu'),
                ('/gps/fix', '/navsat/gps'),
                ("/odometry/filtered", "/localization/odometry"),
                ("/odometry/gps", "/odometry/gps2")
            ],
        ),
    ]



    nav2_nodes = TimerAction(
        period=4.0,
        actions=[

            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output='screen',
                parameters=[NAV2_PARAMS_YAML],
            ),

            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output='screen',
                parameters=[NAV2_PARAMS_YAML],
            ),

            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output='screen',
                parameters=[NAV2_PARAMS_YAML],
            ),

            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',   
                output='screen',
                parameters=[NAV2_PARAMS_YAML],
            ),

            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output='screen',
                parameters=[{
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "bt_navigator",
                        "behavior_server",  
                    ]
                }]
            ),
        ]
    )


    return LaunchDescription(
        utilities + [nav2_nodes]
    )