# u_webrtc - ROS2 WebRTC 视频流系统

## 📚 完整文档导航

欢迎使用 u_webrtc！我们提供了完整的文档体系。请根据您的需求选择阅读：

### 🚀 快速开始（新手推荐）

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| **[QUICK_START.md](QUICK_START.md)** | 快速上手指南 | 第一次使用，想快速体验功能 |
| **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** | 命令速查手册 | 快速查找常用命令和配置 |

### 📖 部署指南（生产环境）

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| **[COMPLETE_DEPLOYMENT.md](COMPLETE_DEPLOYMENT.md)** | 完整部署流程 | 从零开始部署到生产环境 |
| **[INTEGRATION_STATUS.md](INTEGRATION_STATUS.md)** | 集成状态总览 | 查看系统完整性和功能状态 |

### 🔧 技术文档（深入理解）

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| **[readme.md](readme.md)** | 技术方案和架构 | 了解系统设计思路和技术选型 |
| **[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)** | 项目概览 | 了解项目结构和开发计划 |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | 架构设计详解 | 深入理解系统架构和数据流 |

### 🎯 模块文档（专项功能）

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| **[VP8_ENCODER.md](VP8_ENCODER.md)** | VP8 编码器详解 | 使用和配置视频编码功能 |
| **[WEBSOCKET_CLIENT.md](WEBSOCKET_CLIENT.md)** | WebSocket 客户端 | 配置和使用信令连接 |
| **[RTP_PACKETIZATION.md](RTP_PACKETIZATION.md)** | RTP 封装说明 | 了解 RTP 封装的实现细节 |

### 🐛 故障排查

| 文档 | 说明 | 适用场景 |
|------|------|----------|
| **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** | 故障排查手册 | 遇到编译、运行问题时查阅 |

---

## 📖 推荐阅读路径

### 路径 1: 快速体验（30 分钟）⚡
```
1. 阅读 QUICK_START.md
2. 运行示例程序
3. 在浏览器查看视频流
```

### 路径 2: 完整部署（2 小时）🚀
```
1. 阅读 COMPLETE_DEPLOYMENT.md
2. 按步骤安装所有依赖
3. 编译并部署系统
4. 进行端到端测试
```

### 路径 3: 深入学习（1 天）📚
```
1. readme.md - 了解技术方案
2. PROJECT_OVERVIEW.md - 了解项目结构
3. ARCHITECTURE.md - 理解系统架构
4. VP8_ENCODER.md - 学习编码器
5. WEBSOCKET_CLIENT.md - 学习信令
6. RTP_PACKETIZATION.md - 学习 RTP
```

### 路径 4: 问题解决（按需）🔧
```
遇到问题 → TROUBLESHOOTING.md → QUICK_REFERENCE.md → 解决
```

---

## 🎯 系统概览

### 核心功能

✅ **ROS2 图像订阅** - 订阅 ROS2 相机话题  
✅ **图像格式转换** - 使用 OpenCV 转换为 I420  
✅ **VP8 视频编码** - 使用 libvpx 进行真实编码  
✅ **RTP 封装** - 符合 RFC 3550 标准  
✅ **WebRTC 传输** - 使用 libdatachannel  
✅ **WebSocket 信令** - 使用 websocketpp  
✅ **Web 客户端** - HTML5 + WebRTC API  

### 技术栈

- **ROS2**: Humble
- **视频编码**: libvpx (VP8)
- **WebRTC**: libdatachannel
- **信令**: websocketpp + Boost
- **图像处理**: OpenCV
- **构建系统**: CMake + colcon

### 性能指标

- **延迟**: 70-150ms（端到端）
- **分辨率**: 最高 1920×1080 @ 30fps
- **比特率**: 500 Kbps - 8 Mbps
- **CPU 使用**: 15-40%（取决于配置）

---

## 🚀 快速开始

### 一键安装（完整功能）

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc

# 安装所有依赖
./scripts/install_libdatachannel.sh
./scripts/install_libvpx.sh
./scripts/install_websocketpp.sh

# 编译
cd ~/work
colcon build --packages-select u_webrtc
source install/setup.bash
```

### 运行示例

```bash
# 终端 1: 信令服务器
cd ~/Path/work/ros2_test_env/src/u_webrtc/signaling_server
python3 server.py

# 终端 2: WebRTC 节点
ros2 launch u_webrtc webrtc_stream_simple.launch.py

# 终端 3: Web 客户端
cd ~/Path/work/ros2_test_env/src/u_webrtc/web_client
python3 -m http.server 8000

# 浏览器: http://localhost:8000
```

详细步骤请查看 [QUICK_START.md](QUICK_START.md)

---

## 📊 项目结构

```
u_webrtc/
├── include/u_webrtc/          # 头文件
│   ├── image_subscriber.hpp
│   ├── image_converter.hpp
│   ├── video_encoder.hpp
│   ├── rtp_packetizer.hpp
│   ├── peer_connection_wrapper.hpp
│   ├── signaling_client.hpp
│   ├── webrtc_streamer.hpp
│   └── config_manager.hpp
│
├── src/                       # 源文件
│   ├── main.cpp
│   ├── image_subscriber.cpp
│   ├── image_converter.cpp
│   ├── video_encoder.cpp
│   ├── rtp_packetizer.cpp
│   ├── peer_connection_wrapper.cpp
│   ├── signaling_client.cpp
│   ├── webrtc_streamer.cpp
│   └── config_manager.cpp
│
├── scripts/                   # 安装脚本
│   ├── install_dependencies.sh
│   ├── install_libdatachannel.sh
│   ├── install_libvpx.sh
│   ├── install_websocketpp.sh
│   ├── quick_start.sh
│   └── test_build.sh
│
├── config/                    # 配置文件
│   └── webrtc_config.yaml
│
├── launch/                    # ROS2 启动文件
│   ├── webrtc_stream.launch.py
│   └── webrtc_stream_simple.launch.py
│
├── signaling_server/          # 信令服务器
│   ├── server.py
│   ├── requirements.txt
│   └── README.md
│
├── web_client/                # Web 客户端
│   ├── index.html
│   ├── client.js
│   └── README.md
│
└── docs/                      # 文档（10 份）
    ├── README.md              # 本文档
    ├── readme.md              # 技术方案
    ├── QUICK_START.md
    ├── QUICK_REFERENCE.md
    ├── COMPLETE_DEPLOYMENT.md
    ├── VP8_ENCODER.md
    ├── WEBSOCKET_CLIENT.md
    ├── RTP_PACKETIZATION.md
    ├── INTEGRATION_STATUS.md
    └── TROUBLESHOOTING.md
```

---

## 🎉 特色功能

### 1. 双模式架构

系统支持两种运行模式，自动检测和切换：

- **真实模式**：使用 libvpx、libdatachannel、websocketpp（生产环境）
- **框架模式**：使用模拟实现（开发测试）

### 2. 完整的 WebRTC 实现

- ✅ ICE 协商和穿透
- ✅ DTLS 加密
- ✅ SRTP 安全传输
- ✅ STUN/TURN 支持

### 3. 灵活的配置

通过 YAML 配置文件或 ROS2 参数轻松调整：
- 视频分辨率和帧率
- 编码比特率
- 信令服务器地址
- STUN/TURN 服务器

### 4. 详尽的文档

10 份完整文档，涵盖：
- 快速开始
- 完整部署
- 技术细节
- 故障排查

---

## 🔧 系统要求

### 最低要求
- Ubuntu 22.04 LTS
- ROS2 Humble
- 4 核 CPU，4GB RAM
- 1Gbps 网络（局域网）

### 推荐配置
- Ubuntu 22.04 LTS
- ROS2 Humble
- 8 核 CPU，8GB RAM
- 1Gbps 网络

---

## 📞 获取帮助

### 文档资源

- **快速问题**：查看 [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- **编译问题**：查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **配置问题**：查看各模块文档

### 社区支持

- 提交 Issue
- 查看现有文档
- 参考示例代码

---

## 📝 许可证

请查看 LICENSE 文件

---

## 🌟 致谢

本项目使用了以下优秀的开源库：

- [ROS2](https://docs.ros.org/) - 机器人操作系统
- [libdatachannel](https://github.com/paullouisageneau/libdatachannel) - WebRTC 库
- [libvpx](https://github.com/webmproject/libvpx) - VP8/VP9 编码器
- [websocketpp](https://github.com/zaphoyd/websocketpp) - WebSocket 库
- [OpenCV](https://opencv.org/) - 计算机视觉库

---

**开始您的 WebRTC 之旅！** 🚀

从 [QUICK_START.md](QUICK_START.md) 开始，或查看 [COMPLETE_DEPLOYMENT.md](COMPLETE_DEPLOYMENT.md) 了解完整部署流程。

