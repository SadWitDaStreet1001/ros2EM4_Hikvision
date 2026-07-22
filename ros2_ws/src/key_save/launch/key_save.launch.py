"""启动按键保存节点（默认匹配 EM4 + Hikvision 的真实话题）。

按 s / 空格 / 回车 → 保存当前缓存的最新一帧
按 q → 退出
按 c → 显示已保存数
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic',  default_value='/hikvision/image_raw'),
        DeclareLaunchArgument('lidar_topic',   default_value='/rslidar_points'),
        DeclareLaunchArgument('save_dir',      default_value='/root/bags/save'),
        DeclareLaunchArgument('image_encoding', default_value='bgr8'),
        DeclareLaunchArgument('max_dt_ms',     default_value='100.0'),

        Node(
            package='key_save', executable='key_save',
            name='key_save', output='screen',
            prefix='python3 -u',   # 日志立刻刷出
            parameters=[{
                'camera_topic':   LaunchConfiguration('camera_topic'),
                'lidar_topic':    LaunchConfiguration('lidar_topic'),
                'save_dir':       LaunchConfiguration('save_dir'),
                'image_encoding': LaunchConfiguration('image_encoding'),
                'max_dt_ms':      LaunchConfiguration('max_dt_ms'),
                'auto_session':   True,
            }],
        ),
    ])
