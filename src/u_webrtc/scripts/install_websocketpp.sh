#!/bin/bash
# 安装 WebSocket++ 和 Boost 依赖

set -e

echo "=========================================="
echo "  安装 WebSocket++ 和 Boost"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否已安装
if dpkg -l | grep -q libwebsocketpp-dev; then
    echo -e "${GREEN}WebSocket++ 已安装${NC}"
    echo ""
    read -p "是否重新安装？(y/n) " reinstall
    if [ "$reinstall" != "y" ]; then
        echo "退出安装"
        exit 0
    fi
fi

# 安装 Boost（WebSocket++ 的依赖）
echo ""
echo -e "${GREEN}步骤 1: 安装 Boost 库...${NC}"
sudo apt-get update
sudo apt-get install -y \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-chrono-dev \
    libboost-random-dev

# 安装 WebSocket++
echo ""
echo -e "${GREEN}步骤 2: 安装 WebSocket++...${NC}"
sudo apt-get install -y libwebsocketpp-dev

# 安装 OpenSSL（WSS 支持）
echo ""
echo -e "${GREEN}步骤 3: 安装 OpenSSL...${NC}"
sudo apt-get install -y libssl-dev

# 验证安装
echo ""
echo "=========================================="
echo -e "${GREEN}✅ WebSocket++ 安装完成！${NC}"
echo "=========================================="
echo ""

echo "验证安装:"
echo ""

# 检查 WebSocket++
if [ -f "/usr/include/websocketpp/config/asio_client.hpp" ]; then
    echo -e "${GREEN}✓ WebSocket++ 头文件已安装${NC}"
    echo "  路径: /usr/include/websocketpp/"
else
    echo -e "${RED}✗ WebSocket++ 头文件未找到${NC}"
fi

# 检查 Boost
if pkg-config --exists boost 2>/dev/null || [ -d "/usr/include/boost" ]; then
    echo -e "${GREEN}✓ Boost 库已安装${NC}"
    if [ -d "/usr/include/boost" ]; then
        echo "  路径: /usr/include/boost/"
    fi
else
    echo -e "${YELLOW}⚠ Boost 路径未找到，但可能已安装${NC}"
fi

# 检查 OpenSSL
if pkg-config --exists openssl; then
    echo -e "${GREEN}✓ OpenSSL 已安装${NC}"
    openssl_version=$(pkg-config --modversion openssl)
    echo "  版本: $openssl_version"
else
    echo -e "${YELLOW}⚠ OpenSSL pkg-config 未找到${NC}"
fi

echo ""
echo "下一步:"
echo "1. 重新编译 u_webrtc:"
echo "   cd ~/work"
echo "   rm -rf build/u_webrtc"
echo "   colcon build --packages-select u_webrtc"
echo ""
echo "2. WebSocket 信令客户端将自动启用"
echo ""

