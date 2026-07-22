"""启动 Hikvision 网络相机节点。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rtsp_url',
            default_value='rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101'),
        DeclareLaunchArgument('image_topic',       default_value='/hikvision/image_raw'),
        DeclareLaunchArgument('frame_id',          default_value='hikvision_camera'),
        DeclareLaunchArgument('encoding',          default_value='bgr8'),
        DeclareLaunchArgument('capture_backend',   default_value='ffmpeg'),
        DeclareLaunchArgument('target_fps',        default_value='10.0'),
        DeclareLaunchArgument('show_local',        default_value='false'),
        DeclareLaunchArgument('reconnect_delay',   default_value='2.0'),
        DeclareLaunchArgument('read_fail_limit',   default_value='25'),
        DeclareLaunchArgument('publisher_queue_size', default_value='1'),

        Node(
            package='camera_driver', executable='hikvision_node',
            name='hikvision_node', output='screen',
            prefix='python3 -u',
            parameters=[{
                'rtsp_url':     LaunchConfiguration('rtsp_url'),
                'image_topic':  LaunchConfiguration('image_topic'),
                'frame_id':     LaunchConfiguration('frame_frame_id') if False else LaunchConfiguration('frame_id'),
                'encoding':     LaunchConfiguration('encoding'),
                'capture_backend':     LaunchConfiguration('capture_backend'),
                'target_fps':   LaunchConfiguration('target_fps'),
                'show_local':   LaunchConfiguration('show_local'),
                'reconnect_delay': LaunchConfiguration('reconnect_delay'),
                'read_fail_limit': LaunchConfiguration('read_fail_limit'),
                'publisher_queue_size': LaunchConfiguration('publisher_queue_size'),
            }],
        ),

        # 静态 TF：base_link -> hikvision_camera（粗值）
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'rslidar',          # 用雷达做 base_link
                '--child-frame-id', 'hikvision_camera',
            ],
        ),
    ])
