#!/bin/bash
# 测试编译脚本

set -e

echo "==========================================="
echo "    u_webrtc 编译测试"
echo "==========================================="
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误：ROS2 环境未加载"
    echo "请先运行：source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS2 发行版: $ROS_DISTRO"
echo ""

# 进入工作空间根目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_ROOT="$SCRIPT_DIR/../../.."

cd "$WS_ROOT"
echo "工作空间: $(pwd)"
echo ""

# 清理旧的编译文件（可选）
read -p "是否清理旧的编译文件？(y/n) " clean_choice
if [ "$clean_choice" = "y" ]; then
    echo "清理编译文件..."
    rm -rf build/u_webrtc install/u_webrtc log/u_webrtc
    echo "清理完成"
    echo ""
fi

# 编译
echo "开始编译 u_webrtc..."
echo ""

colcon build --packages-select u_webrtc \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

if [ $? -eq 0 ]; then
    echo ""
    echo "==========================================="
    echo "✓ 编译成功！"
    echo "==========================================="
    echo ""
    echo "下一步："
    echo "1. source install/setup.bash"
    echo "2. ros2 launch u_webrtc webrtc_stream_simple.launch.py"
    echo ""
else
    echo ""
    echo "==========================================="
    echo "✗ 编译失败"
    echo "==========================================="
    echo ""
    echo "请检查错误信息并修复"
    exit 1
fi



