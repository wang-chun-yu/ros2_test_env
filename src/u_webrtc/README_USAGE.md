# u_webrtc 使用说明

## 项目简介

u_webrtc 是一个 ROS2 到 WebRTC 的视频流传输桥接节点，用于将 ROS2 的 `sensor_msgs/msg/Image` 消息通过 WebRTC 协议实现低延迟的视频流传输。

## 功能特性

- ✅ 订阅 ROS2 图像话题
- ✅ 自动图像格式转换（支持多种 ROS 图像编码）
- ✅ VP8 视频编码（框架实现）
- ✅ WebRTC 信令交换（框架实现）
- ✅ 可配置的视频参数（分辨率、比特率、帧率）
- ✅ 自动重连机制

## 依赖安装

### 系统依赖

```bash
# ROS2 Humble 必须已安装
# OpenCV
sudo apt-get install libopencv-dev

# JSON 库
sudo apt-get install nlohmann-json3-dev

# cv_bridge
sudo apt-get install ros-humble-cv-bridge
```

### WebRTC 库（可选）

**注意**：当前实现提供了框架代码，WebRTC 相关功能需要集成实际的 WebRTC 库：

- **libwebrtc**：Google 官方 WebRTC 库（推荐但编译复杂）
- **libdatachannel**：轻量级 WebRTC 库
- **Janus Gateway**：可作为中转服务器

## 编译

```bash
# 进入工作空间
cd ~/Path/work/ros2_test_env

# 编译
colcon build --packages-select u_webrtc

# Source 环境
source install/setup.bash
```

## 配置

编辑配置文件 `config/webrtc_config.yaml`：

```yaml
webrtc_streamer:
  ros__parameters:
    # 图像话题
    image_topic: "/camera/image_raw"
    
    # 信令服务器地址
    signaling_server_url: "ws://localhost:8080"
    
    # 视频编码参数
    codec: "VP8"
    target_bitrate: 2000000  # 2 Mbps
    max_framerate: 30
    width: 1280
    height: 720
```

## 启动

### 方式 1：使用启动文件（推荐）

```bash
# 使用默认配置
ros2 launch u_webrtc webrtc_stream_simple.launch.py

# 使用自定义参数
ros2 launch u_webrtc webrtc_stream.launch.py \
    image_topic:=/my_camera/image \
    signaling_server_url:=ws://192.168.1.100:8080
```

### 方式 2：直接运行节点

```bash
ros2 run u_webrtc webrtc_streamer_node \
    --ros-args \
    -p image_topic:=/camera/image_raw \
    -p signaling_server_url:=ws://localhost:8080
```

## 测试

### 1. 启动模拟相机

```bash
# 使用 usb_cam 或其他相机节点
ros2 run usb_cam usb_cam_node_exe

# 或使用 image_publisher 发布测试图像
ros2 run image_publisher image_publisher_node test_image.jpg
```

### 2. 查看话题

```bash
# 查看可用的图像话题
ros2 topic list | grep image

# 查看图像话题信息
ros2 topic info /camera/image_raw
```

### 3. 启动 WebRTC 流节点

```bash
ros2 launch u_webrtc webrtc_stream_simple.launch.py
```

## 架构说明

```
ROS2 Image Topic → ImageSubscriber → ImageConverter → VideoEncoder → WebRTCStreamer → 客户端
                                                           ↕
                                                   SignalingClient
```

### 核心模块

- **ImageSubscriber**：订阅 ROS2 图像话题
- **ImageConverter**：将 ROS Image 转换为 WebRTC 格式（I420）
- **VideoEncoder**：视频编码（VP8/VP9/H264）
- **WebRTCStreamer**：WebRTC 流管理
- **SignalingClient**：WebSocket 信令客户端
- **PeerConnectionWrapper**：WebRTC PeerConnection 封装

## 下一步开发

当前实现提供了完整的框架代码，要使其完全工作，需要：

### 1. 集成真实的 WebRTC 库

选择以下方案之一：

#### 方案 A：使用 libdatachannel（推荐）

```bash
# 安装 libdatachannel
git clone https://github.com/paullouisageneau/libdatachannel.git
cd libdatachannel
cmake -B build
cmake --build build
sudo cmake --install build
```

在 `CMakeLists.txt` 中添加：
```cmake
find_package(LibDataChannel REQUIRED)
target_link_libraries(webrtc_streamer_node LibDataChannel::LibDataChannel)
```

#### 方案 B：使用 Janus Gateway

部署 Janus 作为 WebRTC 服务器，节点作为客户端连接。

### 2. 实现 WebSocket 客户端

使用 `websocketpp` 或 `Boost.Beast`：

```bash
sudo apt-get install libwebsocketpp-dev
```

### 3. 集成真实的视频编码器

使用 `libvpx`（VP8/VP9）或 `x264`（H.264）：

```bash
sudo apt-get install libvpx-dev libx264-dev
```

### 4. 开发 Web 客户端

创建 HTML5 播放器接收 WebRTC 流。

## 故障排查

### 问题：无法连接到信令服务器

**解决**：
- 检查信令服务器是否运行
- 验证 `signaling_server_url` 配置是否正确
- 检查网络防火墙设置

### 问题：图像格式转换失败

**解决**：
- 确认输入图像话题的编码格式
- 检查 OpenCV 版本是否支持该格式
- 查看日志输出获取详细错误信息

### 问题：编译错误

**解决**：
- 确认所有依赖已安装
- 检查 ROS2 环境是否正确 source
- 尝试清理并重新编译：
  ```bash
  rm -rf build install log
  colcon build --packages-select u_webrtc
  ```

## 性能调优

1. **降低延迟**：
   - 减小 `qos_depth`
   - 增加 `target_bitrate`
   - 使用硬件编码

2. **提高质量**：
   - 增加分辨率和比特率
   - 使用 H.264 编码

3. **降低带宽**：
   - 减小分辨率
   - 降低比特率和帧率
   - 使用 VP9 编码

## 许可证

Apache-2.0

## 参考资料

- [ROS2 文档](https://docs.ros.org/)
- [WebRTC 标准](https://webrtc.org/)
- [OpenCV 文档](https://docs.opencv.org/)



