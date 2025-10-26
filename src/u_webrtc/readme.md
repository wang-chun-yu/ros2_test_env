# 功能简介
传输ros2 sensor_msgs/msg/Image类型的数据通过webrtc进行推流，实现视频传输

# u_webrtc 技术方案

## 1. 项目概述

**目标**：构建一个 ROS2 节点，订阅 `sensor_msgs/msg/Image` 消息，通过 WebRTC 协议实现低延迟的视频流传输。

## 2. 技术架构

### 2.1 核心技术栈
- **ROS2 Humble**: 机器人操作系统框架
- **WebRTC (libwebrtc/webrtc-native)**: 实时通信协议
- **C++17/20**: 主要开发语言
- **OpenCV**: 图像处理和格式转换
- **VP8/VP9/H.264**: 视频编码
- **WebSocket**: 信令服务器通信
- **JSON (nlohmann/json)**: 信令数据交换

### 2.2 系统架构

```
┌─────────────────────────────────────────────────────────┐
│                    ROS2 Environment                      │
│  ┌──────────────┐         ┌──────────────┐             │
│  │ Camera Node  │────────▶│ Image Topics │             │
│  └──────────────┘         └──────┬───────┘             │
│                                   │                      │
│                    ┌──────────────▼────────────────┐    │
│                    │   u_webrtc Node               │    │
│                    │  ┌─────────────────────────┐  │    │
│                    │  │ ImageSubscriber         │  │    │
│                    │  └───────┬─────────────────┘  │    │
│                    │          │                     │    │
│                    │  ┌───────▼─────────────────┐  │    │
│                    │  │ ImageConverter          │  │    │
│                    │  └───────┬─────────────────┘  │    │
│                    │          │                     │    │
│                    │  ┌───────▼─────────────────┐  │    │
│                    │  │ WebRTCStreamer          │  │    │
│                    │  │  - PeerConnection       │  │    │
│                    │  │  - VideoEncoder         │  │    │
│                    │  │  - SignalingClient      │  │    │
│                    │  └───────┬─────────────────┘  │    │
│                    └──────────┼─────────────────────┘    │
└───────────────────────────────┼──────────────────────────┘
                                │ WebRTC/WebSocket
                    ┌───────────▼────────────┐
                    │  Signaling Server      │
                    │  (WebSocket Server)    │
                    └───────────┬────────────┘
                                │
                    ┌───────────▼────────────┐
                    │  Web Client/Receiver   │
                    │  (HTML5 + JavaScript)  │
                    └────────────────────────┘
```

## 3. 模块设计

### 3.1 核心模块划分

```cpp
src/u_webrtc/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── u_webrtc/
│       ├── image_subscriber.hpp      // ROS2 图像订阅器
│       ├── image_converter.hpp       // 图像格式转换器
│       ├── video_encoder.hpp         // 视频编码器接口
│       ├── webrtc_streamer.hpp       // WebRTC 流管理器
│       ├── peer_connection_wrapper.hpp // PeerConnection 封装
│       ├── signaling_client.hpp      // 信令客户端
│       └── config_manager.hpp        // 配置管理器
├── src/
│   ├── image_subscriber.cpp
│   ├── image_converter.cpp
│   ├── video_encoder.cpp
│   ├── webrtc_streamer.cpp
│   ├── peer_connection_wrapper.cpp
│   ├── signaling_client.cpp
│   ├── config_manager.cpp
│   └── main.cpp                      // 节点主入口
├── config/
│   └── webrtc_config.yaml           // 配置文件
├── launch/
│   └── webrtc_stream.launch.py      // 启动文件
└── test/
    ├── test_image_converter.cpp
    └── test_webrtc_streamer.cpp
```

### 3.2 类设计

#### 3.2.1 ImageSubscriber（图像订阅器）

```cpp
namespace u_webrtc {

class ImageSubscriber : public rclcpp::Node {
public:
    using ImageCallback = std::function<void(const sensor_msgs::msg::Image::SharedPtr)>;
    
    explicit ImageSubscriber(const std::string& nodeName, 
                            const std::string& topicName);
    
    void setImageCallback(ImageCallback callback);
    
private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
    ImageCallback _userCallback;
    std::string _topicName;
};

} // namespace u_webrtc
```

#### 3.2.2 ImageConverter（图像转换器）

```cpp
namespace u_webrtc {

class ImageConverter {
public:
    struct ConvertedFrame {
        std::vector<uint8_t> data;
        int width;
        int height;
        std::string format; // "I420", "RGB", etc.
    };
    
    ImageConverter() = default;
    ~ImageConverter() = default;
    
    // 将 ROS Image 转换为 WebRTC 可用格式
    std::optional<ConvertedFrame> convertToWebRTCFormat(
        const sensor_msgs::msg::Image::SharedPtr& rosImage);
    
    // 转换为 I420 格式（WebRTC 常用）
    std::optional<ConvertedFrame> convertToI420(
        const cv::Mat& image);
    
private:
    cv::Mat rosImageToMat(const sensor_msgs::msg::Image::SharedPtr& rosImage);
};

} // namespace u_webrtc
```

#### 3.2.3 WebRTCStreamer（核心流管理器）

```cpp
namespace u_webrtc {

class WebRTCStreamer {
public:
    struct StreamConfig {
        std::string signalingServerUrl;
        int targetBitrate;
        int maxFrameRate;
        std::string codecName; // "VP8", "VP9", "H264"
    };
    
    explicit WebRTCStreamer(const StreamConfig& config);
    ~WebRTCStreamer();
    
    // 初始化 WebRTC 连接
    bool initialize();
    
    // 推送帧数据
    bool pushFrame(const ImageConverter::ConvertedFrame& frame);
    
    // 连接到信令服务器
    bool connectToSignalingServer();
    
    // 创建 Offer/Answer
    void createOffer();
    void handleAnswer(const std::string& sdp);
    
    // ICE 候选处理
    void addIceCandidate(const std::string& candidate);
    
private:
    void onIceCandidate(const std::string& candidate);
    void onConnectionStateChange(const std::string& state);
    
    std::unique_ptr<PeerConnectionWrapper> _peerConnection;
    std::unique_ptr<SignalingClient> _signalingClient;
    std::unique_ptr<VideoEncoder> _videoEncoder;
    StreamConfig _config;
    std::atomic<bool> _isConnected{false};
};

} // namespace u_webrtc
```

#### 3.2.4 SignalingClient（信令客户端）

```cpp
namespace u_webrtc {

class SignalingClient {
public:
    using MessageCallback = std::function<void(const std::string&)>;
    
    explicit SignalingClient(const std::string& serverUrl);
    ~SignalingClient();
    
    bool connect();
    void disconnect();
    
    // 发送 SDP
    void sendOffer(const std::string& sdp);
    void sendAnswer(const std::string& sdp);
    
    // 发送 ICE 候选
    void sendIceCandidate(const std::string& candidate);
    
    // 设置回调
    void onMessage(MessageCallback callback);
    
private:
    void handleWebSocketMessage(const std::string& message);
    
    std::string _serverUrl;
    MessageCallback _messageCallback;
    // WebSocket 连接对象（使用 websocketpp 或 Boost.Beast）
};

} // namespace u_webrtc
```

## 4. 数据流设计

### 4.1 端到端数据流

```
ROS2 Image Topic
    │
    ├─▶ ImageSubscriber::imageCallback()
    │       │
    │       ├─▶ ImageConverter::convertToWebRTCFormat()
    │       │       │
    │       │       └─▶ cv::cvtColor() (格式转换)
    │       │
    │       └─▶ WebRTCStreamer::pushFrame()
    │               │
    │               ├─▶ VideoEncoder::encode() (编码)
    │               │
    │               └─▶ PeerConnection::sendEncodedFrame()
    │                       │
    │                       └─▶ 通过 WebRTC 传输到客户端
    │
    └─▶ 信令流程
            │
            ├─▶ SignalingClient::connect()
            ├─▶ SignalingClient::sendOffer()
            ├─▶ SignalingClient::handleAnswer()
            └─▶ SignalingClient::sendIceCandidate()
```

## 5. 依赖库集成

### 5.1 CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.8)
project(u_webrtc)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# WebRTC 库（需要预先编译或使用系统安装）
pkg_check_modules(WEBRTC REQUIRED libwebrtc)

# WebSocket 库
find_package(websocketpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${WEBRTC_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(webrtc_streamer_node
  src/main.cpp
  src/image_subscriber.cpp
  src/image_converter.cpp
  src/webrtc_streamer.cpp
  src/peer_connection_wrapper.cpp
  src/signaling_client.cpp
  src/config_manager.cpp
)

# 链接依赖
ament_target_dependencies(webrtc_streamer_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(webrtc_streamer_node
  ${OpenCV_LIBS}
  ${WEBRTC_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 安装
install(TARGETS webrtc_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_image_converter test/test_image_converter.cpp)
  target_link_libraries(test_image_converter ${OpenCV_LIBS})
endif()

ament_package()
```

### 5.2 package.xml 配置

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>u_webrtc</name>
  <version>1.0.0</version>
  <description>ROS2 to WebRTC video streaming bridge</description>
  
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>libopencv-dev</depend>
  
  <test_depend>ament_cmake_gtest</test_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 6. 配置文件设计

### 6.1 webrtc_config.yaml

```yaml
webrtc_streamer:
  ros__parameters:
    # ROS2 配置
    image_topic: "/camera/image_raw"
    qos_depth: 10
    
    # WebRTC 配置
    signaling_server_url: "ws://localhost:8080"
    stun_server: "stun:stun.l.google.com:19302"
    turn_server: ""
    
    # 视频编码配置
    codec: "VP8"  # VP8, VP9, H264
    target_bitrate: 2000000  # 2 Mbps
    max_framerate: 30
    resolution:
      width: 1280
      height: 720
    
    # 性能配置
    encoding_threads: 4
    use_hardware_encoding: true
```

## 7. 性能优化策略

### 7.1 内存管理
- 使用 `std::shared_ptr` 和 `std::unique_ptr` 管理资源
- 使用对象池复用帧缓冲区
- 避免不必要的图像拷贝，使用 `std::move` 语义

### 7.2 并发处理
```cpp
class WebRTCStreamer {
private:
    // 使用线程池处理编码任务
    std::unique_ptr<ThreadPool> _encodingThreadPool;
    
    // 无锁队列缓存待编码帧
    LockFreeQueue<ConvertedFrame> _frameQueue;
    
    void encodingWorker() {
        while (_isRunning) {
            if (auto frame = _frameQueue.tryPop()) {
                encodeAndSend(*frame);
            }
        }
    }
};
```

### 7.3 编码优化
- 优先使用硬件编码（NVENC, VAAPI, QSV）
- 动态调整比特率适应网络状况
- 实现关键帧请求机制

## 8. 启动文件示例

### 8.1 webrtc_stream.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('u_webrtc'),
        'config',
        'webrtc_config.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='u_webrtc',
            executable='webrtc_streamer_node',
            name='webrtc_streamer',
            parameters=[config_file],
            output='screen',
            emulate_tty=True
        )
    ])
```

## 9. 测试方案

### 9.1 单元测试
- 测试图像格式转换正确性
- 测试 WebRTC 连接建立
- 测试信令消息解析

### 9.2 集成测试
- 端到端视频流传输测试
- 网络抖动下的稳定性测试
- 多客户端连接测试

### 9.3 性能测试
- 延迟测试（端到端 < 500ms）
- 帧率稳定性测试
- CPU/内存占用监控

## 10. 实施步骤

1. **第一阶段**：基础框架搭建
   - 实现 ImageSubscriber 和 ImageConverter
   - 集成 OpenCV 进行图像处理
   
2. **第二阶段**：WebRTC 集成
   - 实现 PeerConnectionWrapper
   - 集成 libwebrtc 或使用原生 WebRTC 库
   
3. **第三阶段**：信令服务器
   - 实现 SignalingClient
   - 开发简单的 WebSocket 信令服务器（Node.js/Python）
   
4. **第四阶段**：优化与测试
   - 性能优化
   - 完善错误处理
   - 编写单元测试和集成测试
   
5. **第五阶段**：Web 客户端
   - 开发 HTML5 播放器
   - 实现交互控制界面

## 11. 注意事项

1. **WebRTC 库选择**：
   - Google 官方 libwebrtc（复杂但功能完整）
   - PiWebRTC（轻量级替代）
   - Janus Gateway（作为中转服务器）

2. **编码格式兼容性**：
   - VP8：浏览器支持最好
   - H.264：需要授权但延迟低
   - VP9：压缩率高但编码慢

3. **网络 NAT 穿透**：
   - 必须配置 STUN/TURN 服务器
   - 建议使用 coturn 自建 TURN 服务器

4. **安全性考虑**：
   - 使用 DTLS 加密数据传输
   - WebSocket 使用 WSS（TLS）
   - 实现客户端认证机制

这个技术方案提供了一个完整的、符合现代 C++ 规范的 ROS2-WebRTC 视频流传输系统架构。你可以根据实际需求进行调整和扩展。