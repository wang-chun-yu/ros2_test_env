#!/bin/bash
# 安装 libvpx 视频编码库

set -e

echo "=========================================="
echo "    安装 libvpx (VP8/VP9 编码器)"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否已安装
if pkg-config --exists vpx 2>/dev/null; then
    echo -e "${GREEN}libvpx 已安装${NC}"
    vpx_version=$(pkg-config --modversion vpx)
    echo "版本: $vpx_version"
    echo ""
    read -p "是否重新安装？(y/n) " reinstall
    if [ "$reinstall" != "y" ]; then
        echo "退出安装"
        exit 0
    fi
fi

# 安装 libvpx
echo ""
echo -e "${GREEN}安装 libvpx...${NC}"
sudo apt-get update
sudo apt-get install -y libvpx-dev

# 验证安装
echo ""
echo "=========================================="
echo -e "${GREEN}✅ libvpx 安装完成！${NC}"
echo "=========================================="
echo ""

if pkg-config --exists vpx; then
    echo "版本信息:"
    pkg-config --modversion vpx
    echo ""
    echo "库路径:"
    pkg-config --libs vpx
    echo ""
    echo "头文件路径:"
    pkg-config --cflags vpx
else
    echo -e "${RED}⚠️  安装可能失败，请检查${NC}"
fi

echo ""
echo "下一步:"
echo "1. 重新编译 u_webrtc:"
echo "   cd ~/work"
echo "   colcon build --packages-select u_webrtc"
echo ""
echo "2. VP8 编码器将自动启用"
echo ""

