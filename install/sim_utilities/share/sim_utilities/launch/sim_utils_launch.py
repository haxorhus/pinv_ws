from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #sensor republisher node
        Node(
            package='sim_utilities',
            executable='republisher',
            parameters=['/home/jose/pinv_ws/src/sim_utilities/config/sensor_config.yaml']
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
            parameters=['/home/jose/pinv_ws/src/sim_utilities/config/ekf.yaml'],
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
            parameters=['/home/jose/pinv_ws/src/sim_utilities/config/navsat_transform.yaml'],
            remappings=[
            ('/imu', '/navsat/imu'),
            ('/gps/fix', '/navsat/gps'),
            ("/odometry/filtered", "/localization/odometry"),
            ("/odometry/gps", "/odometry/gps2")
            ],
        ),
    ])
    