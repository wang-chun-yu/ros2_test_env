#!/bin/bash
# 安装 libdatachannel 脚本

set -e

echo "=========================================="
echo "    安装 libdatachannel"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否已安装
if ldconfig -p 2>/dev/null | grep -q libdatachannel; then
    echo -e "${GREEN}libdatachannel 已安装${NC}"
    echo ""
    read -p "是否重新安装？(y/n) " reinstall
    if [ "$reinstall" != "y" ]; then
        echo "退出安装"
        exit 0
    fi
fi

# 1. 安装系统依赖
echo ""
echo -e "${GREEN}步骤 1: 安装系统依赖...${NC}"
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    libssl-dev

# 2. 创建临时目录
TEMP_DIR=$(mktemp -d)
echo ""
echo "临时目录: $TEMP_DIR"

cd "$TEMP_DIR"

# 3. 克隆仓库（带子模块）
echo ""
echo -e "${GREEN}步骤 2: 克隆 libdatachannel (带子模块)...${NC}"
echo "这可能需要几分钟..."

git clone --recursive --depth 1 https://github.com/paullouisageneau/libdatachannel.git

cd libdatachannel

# 4. 验证子模块
echo ""
echo -e "${GREEN}步骤 3: 验证子模块...${NC}"
git submodule status

# 检查关键子模块是否存在
if [ ! -f "deps/plog/CMakeLists.txt" ]; then
    echo -e "${RED}错误: 子模块未正确初始化${NC}"
    echo "尝试手动初始化..."
    git submodule update --init --recursive
fi

# 5. 配置编译
echo ""
echo -e "${GREEN}步骤 4: 配置 CMake...${NC}"
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DUSE_GNUTLS=0 \
    -DUSE_NICE=0 \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 6. 编译
echo ""
echo -e "${GREEN}步骤 5: 编译 (这可能需要几分钟)...${NC}"
cmake --build build -j$(nproc)

# 7. 安装
echo ""
echo -e "${GREEN}步骤 6: 安装...${NC}"
sudo cmake --install build

# 8. 更新动态链接库缓存
echo ""
echo "更新动态链接库缓存..."
sudo ldconfig

# 9. 清理
echo ""
echo "清理临时文件..."
cd ~
rm -rf "$TEMP_DIR"

# 10. 验证安装
echo ""
echo "=========================================="
echo -e "${GREEN}✅ libdatachannel 安装完成！${NC}"
echo "=========================================="
echo ""
echo "验证安装:"
if ldconfig -p | grep -q libdatachannel; then
    echo -e "${GREEN}✓ libdatachannel 库已在系统中${NC}"
    ldconfig -p | grep libdatachannel
else
    echo -e "${YELLOW}⚠ 未找到 libdatachannel 库，可能需要重启终端${NC}"
fi

echo ""
echo "下一步:"
echo "1. 重新编译 u_webrtc:"
echo "   cd ~/work"
echo "   colcon build --packages-select u_webrtc"
echo ""
echo "2. 查看编译输出，应该看到:"
echo "   ✅ Found libdatachannel - WebRTC functionality enabled"
echo ""

