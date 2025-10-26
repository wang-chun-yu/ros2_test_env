#!/bin/bash
# u_webrtc 快速启动脚本

set -e

echo "==========================================="
echo "    u_webrtc 快速启动助手"
echo "==========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在正确的目录
if [ ! -f "package.xml" ]; then
    echo -e "${RED}错误：请在 u_webrtc 包目录中运行此脚本${NC}"
    exit 1
fi

# 显示菜单
echo "请选择要启动的组件："
echo ""
echo "  1) 安装依赖"
echo "  2) 编译项目"
echo "  3) 启动信令服务器"
echo "  4) 启动 ROS2 WebRTC 节点"
echo "  5) 启动 Web 客户端"
echo "  6) 显示项目信息"
echo "  0) 退出"
echo ""
read -p "请输入选项 [0-6]: " choice

case $choice in
    1)
        echo -e "${GREEN}安装依赖...${NC}"
        ./scripts/install_dependencies.sh
        ;;
    
    2)
        echo -e "${GREEN}编译项目...${NC}"
        cd ../../..
        colcon build --packages-select u_webrtc
        source install/setup.bash
        echo -e "${GREEN}编译完成！${NC}"
        echo "请运行：source install/setup.bash"
        ;;
    
    3)
        echo -e "${GREEN}启动信令服务器...${NC}"
        cd signaling_server
        
        # 检查是否安装了 websockets
        if ! python3 -c "import websockets" 2>/dev/null; then
            echo -e "${YELLOW}安装 Python 依赖...${NC}"
            pip install -r requirements.txt
        fi
        
        echo -e "${GREEN}信令服务器运行在 ws://localhost:8080${NC}"
        python3 server.py
        ;;
    
    4)
        echo -e "${GREEN}启动 ROS2 WebRTC 节点...${NC}"
        
        # 检查 ROS2 环境
        if [ -z "$ROS_DISTRO" ]; then
            echo -e "${RED}错误：ROS2 环境未加载${NC}"
            echo "请先运行：source /opt/ros/humble/setup.bash"
            exit 1
        fi
        
        # 检查工作空间
        if [ ! -f "../../../install/setup.bash" ]; then
            echo -e "${YELLOW}项目未编译，正在编译...${NC}"
            cd ../../..
            colcon build --packages-select u_webrtc
            source install/setup.bash
            cd -
        fi
        
        # 检查是否有图像话题
        echo -e "${YELLOW}检查图像话题...${NC}"
        if ! ros2 topic list | grep -q "image"; then
            echo -e "${YELLOW}警告：未检测到图像话题${NC}"
            echo "请确保相机节点正在运行"
            echo ""
            read -p "是否继续启动？(y/n) " continue_choice
            if [ "$continue_choice" != "y" ]; then
                exit 0
            fi
        fi
        
        echo -e "${GREEN}启动 WebRTC 节点...${NC}"
        source ../../../install/setup.bash
        ros2 launch u_webrtc webrtc_stream_simple.launch.py
        ;;
    
    5)
        echo -e "${GREEN}启动 Web 客户端...${NC}"
        cd web_client
        echo -e "${GREEN}Web 客户端运行在 http://localhost:8000${NC}"
        echo -e "${YELLOW}请在浏览器中打开此地址${NC}"
        python3 -m http.server 8000
        ;;
    
    6)
        echo ""
        echo "==========================================="
        echo "    u_webrtc 项目信息"
        echo "==========================================="
        echo ""
        echo "项目名称: u_webrtc"
        echo "版本: 1.0.0"
        echo "描述: ROS2 到 WebRTC 视频流传输桥接"
        echo ""
        echo "目录结构:"
        tree -L 2 .
        echo ""
        echo "文档:"
        echo "  - README_USAGE.md: 使用说明"
        echo "  - PROJECT_OVERVIEW.md: 项目概览"
        echo "  - readme.md: 技术方案"
        echo ""
        ;;
    
    0)
        echo "退出"
        exit 0
        ;;
    
    *)
        echo -e "${RED}无效选项${NC}"
        exit 1
        ;;
esac



