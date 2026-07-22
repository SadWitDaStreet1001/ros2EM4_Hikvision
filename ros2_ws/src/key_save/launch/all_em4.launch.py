"""一键启动：EM4 雷达 + Hikvision 相机 + 按键保存。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rslidar_share = FindPackageShare('rslidar_sdk')
    camera_share  = FindPackageShare('camera_driver')
    keysave_share = FindPackageShare('key_save')

    return LaunchDescription([
        # 1) EM4 雷达
        Node(
            package='rslidar_sdk', executable='rslidar_sdk_node',
            name='rslidar_sdk', output='screen',
            parameters=[{
                'config_path': PathJoinSubstitution([rslidar_share, 'config', 'config_em4.yaml']),
            }],
        ),

        # 2) Hikvision 相机
        Node(
            package='camera_driver', executable='hikvision_node',
            name='hikvision_node', output='screen',
            prefix='python3 -u',
            parameters=[{
                'rtsp_url':    'rtsp://admin:hyzx_hyzx@192.168.1.64:554/Streaming/Channels/101',
                'image_topic': '/hikvision/image_raw',
                'frame_id':    'hikvision_camera',
                'target_fps':  10.0,
            }],
        ),

        # 3) 静态 TF：rslidar -> hikvision_camera（粗值，标定后会覆盖）
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='rslidar_to_hikvision_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'rslidar',
                '--child-frame-id', 'hikvision_camera',
            ],
        ),

        # 4) 按键保存节点
        Node(
            package='key_save', executable='key_save',
            name='key_save', output='screen',
            prefix='python3 -u',
            parameters=[{
                'camera_topic':   '/hikvision/image_raw',
                'lidar_topic':    '/rslidar_points',
                'save_dir':       LaunchConfiguration('save_dir'),
                'image_encoding': 'bgr8',
                'max_dt_ms':      100.0,
                'auto_session':   True,
            }],
        ),
    ])
