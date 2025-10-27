# u_webrtc 故障排查指南

## 🔧 编译错误：找不到 libdatachannel

### 问题描述

```
CMake Error: Could not find a package configuration file provided by "libdatachannel"
```

### 原因分析

u_webrtc 是一个**框架实现**，设计为支持可选的 libdatachannel 集成：

1. **不安装 libdatachannel**：可以编译并运行框架代码（当前状态）
2. **安装 libdatachannel**：启用完整的 WebRTC 功能

### ✅ 解决方案

#### 方案 1：仅编译框架（推荐新手）

框架代码已经可以独立编译，无需 libdatachannel：

```bash
cd ~/work
colcon build --packages-select u_webrtc
```

编译时会看到警告：
```
⚠️  libdatachannel not found - using framework implementation only
```

这是**正常的**！框架代码可以正常编译和运行。

#### 方案 2：安装 libdatachannel（启用完整功能）

如果你想启用真实的 WebRTC 功能：

```bash
# 1. 运行安装脚本
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/install_libdatachannel.sh

# 2. 重新编译
cd ~/work
colcon build --packages-select u_webrtc --cmake-clean-cache

# 3. 查看编译输出
# 应该看到: ✅ Found libdatachannel - WebRTC functionality enabled
```

### 编译输出说明

#### 没有 libdatachannel

```
Starting >>> u_webrtc
⚠️  libdatachannel not found - using framework implementation only
To enable full WebRTC support, install libdatachannel:
  See: scripts/install_libdatachannel.sh
Finished <<< u_webrtc [0.5s]
```

**状态**：✅ 编译成功，但只有框架功能

#### 有 libdatachannel

```
Starting >>> u_webrtc
✅ Found libdatachannel - WebRTC functionality enabled
Finished <<< u_webrtc [0.8s]
```

**状态**：✅ 编译成功，WebRTC 功能已启用

## 🔍 其他常见问题

### 问题 1：找不到 nlohmann_json

**错误**：
```
nlohmann/json.hpp: No such file or directory
```

**解决**：
```bash
sudo apt-get install nlohmann-json3-dev
```

### 问题 2：找不到 cv_bridge

**错误**：
```
Could not find a package configuration file provided by "cv_bridge"
```

**解决**：
```bash
sudo apt-get install ros-humble-cv-bridge
```

### 问题 3：OpenCV 错误

**错误**：
```
Could not find a package configuration file provided by "OpenCV"
```

**解决**：
```bash
sudo apt-get install libopencv-dev
```

### 问题 4：编译后运行节点报错

**错误**：
```
error while loading shared libraries: libdatachannel.so
```

**解决**：
```bash
# 更新动态链接库缓存
sudo ldconfig

# 如果还不行，添加库路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

## 📊 依赖关系图

```
u_webrtc
├── [必需] ROS2 Humble
├── [必需] rclcpp
├── [必需] sensor_msgs
├── [必需] cv_bridge
├── [必需] OpenCV
├── [必需] nlohmann_json
└── [可选] libdatachannel ← 用于真实 WebRTC 功能
```

## 🔄 清理重新编译

如果遇到奇怪的编译问题：

```bash
cd ~/work

# 清理 u_webrtc 的编译文件
rm -rf build/u_webrtc install/u_webrtc log/u_webrtc

# 重新编译
colcon build --packages-select u_webrtc
```

## 🚀 验证安装

### 1. 检查节点是否可用

```bash
source ~/work/install/setup.bash
ros2 pkg list | grep u_webrtc
```

应该看到：`u_webrtc`

### 2. 检查可执行文件

```bash
ros2 pkg executables u_webrtc
```

应该看到：`u_webrtc webrtc_streamer_node`

### 3. 测试运行

```bash
ros2 run u_webrtc webrtc_streamer_node
```

应该看到节点启动（可能会警告找不到图像话题，这是正常的）。

## 📖 相关文档

- **QUICK_START.md** - 快速开始指南
- **README_USAGE.md** - 详细使用说明
- **PROJECT_OVERVIEW.md** - 项目概览
- **ARCHITECTURE.md** - 架构设计

## 💡 开发建议

### 阶段 1：熟悉框架（当前）

1. ✅ 编译框架代码（不需要 libdatachannel）
2. ✅ 理解各个模块的作用
3. ✅ 运行信令服务器和 Web 客户端
4. ✅ 测试 ROS2 图像订阅和转换

### 阶段 2：集成 WebRTC（高级）

1. 🔧 安装 libdatachannel
2. 🔧 修改 PeerConnectionWrapper 实现
3. 🔧 修改 SignalingClient 实现
4. 🔧 集成视频编码器
5. 🔧 端到端测试

## 🆘 获取帮助

如果问题仍未解决：

1. 查看完整的编译日志：
   ```bash
   colcon build --packages-select u_webrtc --event-handlers console_direct+
   ```

2. 检查系统环境：
   ```bash
   echo $ROS_DISTRO
   pkg-config --list-all | grep opencv
   ldconfig -p | grep datachannel
   ```

3. 查看详细文档：
   ```bash
   cd ~/Path/work/ros2_test_env/src/u_webrtc
   ls *.md
   ```

---

**提示**：框架实现可以完全独立于 libdatachannel 运行，专注于学习和理解系统架构。等熟悉后再集成真实的 WebRTC 库。

