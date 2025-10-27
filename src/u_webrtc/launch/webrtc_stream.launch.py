#!/usr/bin/env python3
"""
WebRTC 视频流传输启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取配置文件路径
    pkg_share_dir = get_package_share_directory('u_webrtc')
    config_file = os.path.join(pkg_share_dir, 'config', 'webrtc_config.yaml')
    
    # 声明启动参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='WebRTC 配置文件的完整路径'
    )
    
    declare_image_topic = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='要订阅的图像话题名称'
    )
    
    declare_signaling_server = DeclareLaunchArgument(
        'signaling_server_url',
        default_value='ws://localhost:8080',
        description='WebRTC 信令服务器地址'
    )
    
    declare_codec = DeclareLaunchArgument(
        'codec',
        default_value='VP8',
        description='视频编码格式 (VP8, VP9, H264)'
    )
    
    # 创建 WebRTC 流节点
    webrtc_streamer_node = Node(
        package='u_webrtc',
        executable='webrtc_streamer_node',
        name='webrtc_streamer',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'signaling_server_url': LaunchConfiguration('signaling_server_url'),
                'codec': LaunchConfiguration('codec'),
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=False,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_image_topic,
        declare_signaling_server,
        declare_codec,
        webrtc_streamer_node,
    ])



