#!/bin/bash
# u_webrtc 依赖安装脚本

set -e

echo "=========================================="
echo "u_webrtc 依赖安装脚本"
echo "=========================================="

# 检查是否为 root 用户
if [ "$EUID" -eq 0 ]; then 
    echo "请不要使用 sudo 运行此脚本"
    echo "脚本会在需要时自动请求 sudo 权限"
    exit 1
fi

# 更新软件包列表
echo ""
echo "更新软件包列表..."
sudo apt-get update

# 安装 OpenCV
echo ""
echo "安装 OpenCV..."
sudo apt-get install -y libopencv-dev

# 安装 JSON 库
echo ""
echo "安装 nlohmann-json..."
sudo apt-get install -y nlohmann-json3-dev

# 安装 ROS2 cv_bridge
echo ""
echo "安装 ROS2 cv_bridge..."
sudo apt-get install -y ros-humble-cv-bridge

# 安装 image_transport（可选，用于图像传输优化）
echo ""
echo "安装 ROS2 image_transport..."
sudo apt-get install -y ros-humble-image-transport

# 安装 WebSocket 库（可选）
echo ""
echo "安装 WebSocket++ 库..."
sudo apt-get install -y libwebsocketpp-dev

# 安装 Boost（WebSocket++ 依赖）
echo ""
echo "安装 Boost 库..."
sudo apt-get install -y libboost-all-dev

# 检查安装
echo ""
echo "=========================================="
echo "检查依赖安装情况..."
echo "=========================================="

check_package() {
    if dpkg -l | grep -q "^ii  $1"; then
        echo "✓ $1 已安装"
    else
        echo "✗ $1 未安装"
    fi
}

check_package "libopencv-dev"
check_package "nlohmann-json3-dev"
check_package "ros-humble-cv-bridge"
check_package "libwebsocketpp-dev"
check_package "libboost-all-dev"

echo ""
echo "=========================================="
echo "依赖安装完成！"
echo "=========================================="
echo ""
echo "注意事项："
echo "1. 当前实现提供框架代码，WebRTC 功能需要集成实际的库"
echo "2. 推荐使用 libdatachannel 作为 WebRTC 实现"
echo "3. 需要部署 WebSocket 信令服务器"
echo ""
echo "下一步："
echo "1. 编译项目：colcon build --packages-select u_webrtc"
echo "2. Source 环境：source install/setup.bash"
echo "3. 启动节点：ros2 launch u_webrtc webrtc_stream_simple.launch.py"
echo ""



