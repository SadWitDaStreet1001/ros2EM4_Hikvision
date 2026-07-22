"""启动 EM4 雷达（Robosense RSEM4）。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration(
        'config_path',
        default='/root/ros2_ws/src/rslidar_sdk/config/config_em4.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='/root/ros2_ws/src/rslidar_sdk/config/config_em4.yaml'
        ),

        Node(
            package='rslidar_sdk', executable='rslidar_sdk_node',
            name='rslidar_sdk', output='screen',
            parameters=[{'config_path': config_path}],
        ),

        # 静态 TF：base_link -> rslidar（粗值，标定后用真值覆盖）
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '--x', '0.10', '--y', '0.0', '--z', '-0.30',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'rslidar',
            ],
        ),
    ])
