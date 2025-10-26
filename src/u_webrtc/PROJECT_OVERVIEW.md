# u_webrtc 项目概览

## 📋 项目简介

**u_webrtc** 是一个完整的 ROS2 到 WebRTC 视频流传输系统，实现了从机器人传感器到 Web 浏览器的低延迟实时视频传输。

### 核心功能

- ✅ 订阅 ROS2 `sensor_msgs/msg/Image` 消息
- ✅ 自动图像格式转换（支持多种编码格式）
- ✅ 视频编码（VP8 框架实现）
- ✅ WebRTC 实时传输（框架实现）
- ✅ WebSocket 信令交换
- ✅ 完整的 Web 客户端示例

## 📁 项目结构

```
u_webrtc/
├── include/u_webrtc/          # 头文件
│   ├── image_subscriber.hpp
│   ├── image_converter.hpp
│   ├── video_encoder.hpp
│   ├── webrtc_streamer.hpp
│   ├── peer_connection_wrapper.hpp
│   ├── signaling_client.hpp
│   └── config_manager.hpp
│
├── src/                        # 源文件
│   ├── image_subscriber.cpp
│   ├── image_converter.cpp
│   ├── video_encoder.cpp
│   ├── webrtc_streamer.cpp
│   ├── peer_connection_wrapper.cpp
│   ├── signaling_client.cpp
│   ├── config_manager.cpp
│   └── main.cpp
│
├── config/                     # 配置文件
│   └── webrtc_config.yaml
│
├── launch/                     # 启动文件
│   ├── webrtc_stream.launch.py
│   └── webrtc_stream_simple.launch.py
│
├── scripts/                    # 工具脚本
│   └── install_dependencies.sh
│
├── signaling_server/           # WebSocket 信令服务器
│   ├── server.py
│   ├── requirements.txt
│   └── README.md
│
├── web_client/                 # HTML5 Web 客户端
│   ├── index.html
│   ├── client.js
│   └── README.md
│
├── CMakeLists.txt             # CMake 构建配置
├── package.xml                # ROS2 包配置
├── readme.md                  # 技术方案文档
├── README_USAGE.md            # 使用说明
└── PROJECT_OVERVIEW.md        # 本文件
```

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                      ROS2 环境                                │
│                                                               │
│  相机节点 → Image Topic → u_webrtc 节点                       │
│                              ↓                                │
│                     ImageSubscriber                           │
│                              ↓                                │
│                     ImageConverter                            │
│                              ↓                                │
│                      VideoEncoder                             │
│                              ↓                                │
│                    WebRTCStreamer                             │
│                              ↓                                │
└──────────────────────────────┼───────────────────────────────┘
                               ↓
                    ┌──────────────────┐
                    │ 信令服务器         │
                    │ (WebSocket)       │
                    └──────────────────┘
                               ↓
                    ┌──────────────────┐
                    │ Web 浏览器客户端  │
                    │ (HTML5+WebRTC)   │
                    └──────────────────┘
```

## 🚀 快速开始

### 1. 安装依赖

```bash
cd src/u_webrtc
./scripts/install_dependencies.sh
```

### 2. 编译项目

```bash
cd ~/Path/work/ros2_test_env
colcon build --packages-select u_webrtc
source install/setup.bash
```

### 3. 启动信令服务器

```bash
cd src/u_webrtc/signaling_server
pip install -r requirements.txt
python3 server.py
```

### 4. 启动 ROS2 节点

```bash
# 在新终端中
source install/setup.bash
ros2 launch u_webrtc webrtc_stream_simple.launch.py
```

### 5. 打开 Web 客户端

```bash
# 在新终端中
cd src/u_webrtc/web_client
python3 -m http.server 8000
```

然后在浏览器中访问：`http://localhost:8000`

## 🔧 配置

编辑 `config/webrtc_config.yaml` 来调整参数：

```yaml
webrtc_streamer:
  ros__parameters:
    image_topic: "/camera/image_raw"
    signaling_server_url: "ws://localhost:8080"
    codec: "VP8"
    target_bitrate: 2000000
    max_framerate: 30
    width: 1280
    height: 720
```

## 📚 核心模块说明

### ImageSubscriber
- 订阅 ROS2 图像话题
- 使用 SensorDataQoS 确保实时性

### ImageConverter
- 支持多种 ROS 图像编码格式
- 转换为 WebRTC 标准的 I420 格式
- 使用 OpenCV 进行格式转换

### VideoEncoder
- 提供编码器接口
- VP8Encoder 实现（框架）
- 支持关键帧请求

### WebRTCStreamer
- 核心流管理器
- 协调各个模块
- 管理连接生命周期

### SignalingClient
- WebSocket 信令客户端
- 处理 Offer/Answer/ICE 交换

### PeerConnectionWrapper
- WebRTC PeerConnection 封装
- 简化 WebRTC API 使用

## ⚙️ 技术栈

- **ROS2 Humble**: 机器人操作系统
- **C++17**: 主要开发语言
- **OpenCV**: 图像处理
- **nlohmann/json**: JSON 解析
- **WebRTC**: 实时通信协议（框架）
- **WebSocket**: 信令通信
- **Python**: 信令服务器
- **HTML5/JavaScript**: Web 客户端

## 🔍 当前状态

### ✅ 已实现

1. ✅ 完整的项目结构
2. ✅ ROS2 图像订阅和转换
3. ✅ 配置管理系统
4. ✅ 模块化设计
5. ✅ 信令服务器（Python）
6. ✅ Web 客户端（HTML5）
7. ✅ 启动文件和配置
8. ✅ 完整的文档

### 🚧 需要集成

当前实现提供了完整的**框架代码**，以下功能需要集成实际的库：

1. **WebRTC 实现**
   - 推荐：libdatachannel
   - 备选：libwebrtc, Janus Gateway

2. **视频编码器**
   - VP8: libvpx
   - H.264: x264
   - VP9: libvpx

3. **WebSocket 客户端**
   - websocketpp
   - Boost.Beast

## 📖 详细文档

- **readme.md** - 完整的技术方案和设计文档
- **README_USAGE.md** - 详细的使用说明和故障排查
- **signaling_server/README.md** - 信令服务器文档
- **web_client/README.md** - Web 客户端文档

## 🛠️ 下一步开发建议

### 阶段 1：集成 libdatachannel

```bash
# 安装 libdatachannel
git clone https://github.com/paullouisageneau/libdatachannel.git
cd libdatachannel
cmake -B build
cmake --build build
sudo cmake --install build
```

在 `peer_connection_wrapper.cpp` 中集成实际实现。

### 阶段 2：集成视频编码器

```bash
sudo apt-get install libvpx-dev
```

在 `video_encoder.cpp` 中使用 libvpx 实现 VP8 编码。

### 阶段 3：实现 WebSocket 客户端

```bash
sudo apt-get install libwebsocketpp-dev
```

在 `signaling_client.cpp` 中实现真实的 WebSocket 连接。

### 阶段 4：性能优化

- 使用硬件编码（NVENC/VAAPI）
- 实现帧缓冲池
- 添加自适应比特率控制

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

Apache-2.0

## 📧 联系方式

如有问题或建议，请创建 Issue。

---

**注意**：这是一个框架实现，提供了完整的架构和接口设计。要使其完全工作，需要集成上述提到的实际 WebRTC 和编码库。框架设计遵循现代 C++ 最佳实践，便于扩展和集成。



