#!/usr/bin/env python3
"""
WebRTC 视频流传输简单启动文件（无参数）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取配置文件路径
    pkg_share_dir = get_package_share_directory('u_webrtc')
    config_file = os.path.join(pkg_share_dir, 'config', 'webrtc_config.yaml')
    
    # 创建 WebRTC 流节点
    webrtc_streamer_node = Node(
        package='u_webrtc',
        executable='webrtc_streamer_node',
        name='webrtc_streamer',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        webrtc_streamer_node,
    ])



