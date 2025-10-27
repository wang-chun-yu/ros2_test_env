# u_webrtc 快速参考卡

## 🚀 一键安装（完整功能）

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc

# 1. 安装所有依赖
./scripts/install_dependencies.sh        # 基础依赖
./scripts/install_libdatachannel.sh      # WebRTC 传输
./scripts/install_libvpx.sh              # VP8 编码
./scripts/install_websocketpp.sh         # WebSocket 信令

# 2. 编译
cd ~/work
colcon build --packages-select u_webrtc
source install/setup.bash
```

## 📦 分阶段安装

### 最小安装（框架模式）
```bash
sudo apt-get install -y libopencv-dev nlohmann-json3-dev ros-humble-cv-bridge
cd ~/work && colcon build --packages-select u_webrtc
```

### 标准安装（WebRTC + 编码）
```bash
# 最小安装 +
./scripts/install_libdatachannel.sh
./scripts/install_libvpx.sh
cd ~/work && rm -rf build/u_webrtc && colcon build --packages-select u_webrtc
```

### 完整安装（生产就绪）
```bash
# 标准安装 +
./scripts/install_websocketpp.sh
cd ~/work && rm -rf build/u_webrtc && colcon build --packages-select u_webrtc
```

## 🎬 快速启动

### 完整系统（4 个终端）

```bash
# 终端 1: 信令服务器
cd ~/Path/work/ros2_test_env/src/u_webrtc/signaling_server
python3 server.py

# 终端 2: 相机（选择一种）
ros2 run usb_cam usb_cam_node_exe                    # USB 摄像头
ros2 run v4l2_camera v4l2_camera_node                # v4l2 摄像头
ros2 run image_publisher image_publisher_node image.jpg  # 测试图片

# 终端 3: WebRTC 流节点
cd ~/work && source install/setup.bash
ros2 launch u_webrtc webrtc_stream_simple.launch.py

# 终端 4: Web 客户端
cd ~/Path/work/ros2_test_env/src/u_webrtc/web_client
python3 -m http.server 8000

# 浏览器: http://localhost:8000
```

### 单命令启动（使用脚本）

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/quick_start.sh
```

## 🔧 常用命令

### 编译相关
```bash
# 完整编译
cd ~/work && colcon build --packages-select u_webrtc

# 清理后编译
cd ~/work && rm -rf build/u_webrtc install/u_webrtc && colcon build --packages-select u_webrtc

# 查看详细输出
colcon build --packages-select u_webrtc --event-handlers console_direct+

# 仅编译（不安装）
colcon build --packages-select u_webrtc --cmake-target all
```

### 节点相关
```bash
# 运行节点
ros2 run u_webrtc webrtc_streamer_node

# 使用参数运行
ros2 run u_webrtc webrtc_streamer_node --ros-args \
    -p image_topic:=/my_camera/image \
    -p width:=640 \
    -p height:=480

# 使用配置文件
ros2 launch u_webrtc webrtc_stream.launch.py

# 调试模式
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level debug
```

### 话题相关
```bash
# 列出话题
ros2 topic list

# 查看话题频率
ros2 topic hz /camera/image_raw

# 查看话题带宽
ros2 topic bw /camera/image_raw

# 查看话题信息
ros2 topic info /camera/image_raw

# 显示图像（需要 rqt）
ros2 run rqt_image_view rqt_image_view
```

### 参数相关
```bash
# 列出参数
ros2 param list /webrtc_streamer

# 获取参数
ros2 param get /webrtc_streamer width

# 设置参数
ros2 param set /webrtc_streamer target_bitrate 3000000

# 导出参数
ros2 param dump /webrtc_streamer
```

## 📊 验证命令

### 检查依赖
```bash
# libdatachannel
pkg-config --modversion libdatachannel

# libvpx
pkg-config --modversion vpx

# websocketpp
ls /usr/include/websocketpp/

# OpenCV
pkg-config --modversion opencv4

# Boost
dpkg -l | grep libboost
```

### 检查编译状态
```bash
cd ~/work
colcon list -n u_webrtc
colcon build --packages-select u_webrtc 2>&1 | grep -E "Found|Linked"
```

### 检查运行状态
```bash
# 节点列表
ros2 node list

# 节点信息
ros2 node info /webrtc_streamer

# WebSocket 连接测试
telnet localhost 8080
curl http://localhost:8080

# Web 服务器测试
curl http://localhost:8000
```

## 🐛 故障排查

### 编译问题
```bash
# 找不到 libdatachannel
./scripts/install_libdatachannel.sh
rm -rf build/u_webrtc && colcon build --packages-select u_webrtc

# 找不到 libvpx
sudo apt-get install libvpx-dev
rm -rf build/u_webrtc && colcon build --packages-select u_webrtc

# 找不到 websocketpp
sudo apt-get install libwebsocketpp-dev libboost-dev
rm -rf build/u_webrtc && colcon build --packages-select u_webrtc

# 清理所有并重新编译
cd ~/work && rm -rf build install log && colcon build
```

### 运行时问题
```bash
# 找不到相机话题
ros2 topic list | grep image

# WebSocket 连接失败
ps aux | grep server.py
netstat -tulpn | grep 8080

# 无视频显示
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level debug

# 延迟过高
# 降低分辨率和比特率在配置文件中
```

## 📝 配置快速修改

### 修改相机话题
```yaml
# config/webrtc_config.yaml
image_topic: "/my_camera/image_raw"
```

### 修改视频参数
```yaml
width: 640
height: 480
target_bitrate: 1000000  # 1 Mbps
max_framerate: 30
```

### 修改信令服务器
```yaml
signaling_server_url: "ws://192.168.1.100:8080"
```

### 修改 STUN/TURN
```yaml
stun_server: "stun:stun.l.google.com:19302"
turn_server: "turn:your-turn-server:3478"
turn_username: "user"
turn_password: "pass"
```

## 🔍 性能调优

### 低延迟配置
```yaml
width: 640
height: 480
target_bitrate: 1000000
max_framerate: 60
encoding_threads: 2
```

### 高质量配置
```yaml
width: 1920
height: 1080
target_bitrate: 8000000
max_framerate: 30
encoding_threads: 8
```

### 节能配置
```yaml
width: 320
height: 240
target_bitrate: 500000
max_framerate: 15
encoding_threads: 1
```

## 📚 文档速查

| 文档 | 用途 |
|------|------|
| **readme.md** | 技术方案和设计 |
| **QUICK_START.md** | 快速上手指南 |
| **COMPLETE_DEPLOYMENT.md** | 完整部署流程 |
| **VP8_ENCODER.md** | VP8 编码器详解 |
| **WEBSOCKET_CLIENT.md** | WebSocket 客户端 |
| **RTP_PACKETIZATION.md** | RTP 封装说明 |
| **INTEGRATION_STATUS.md** | 集成状态总览 |
| **TROUBLESHOOTING.md** | 故障排查指南 |
| **QUICK_REFERENCE.md** | 本文档 |

## 🎯 常见使用场景

### 场景 1: 本地测试
```bash
# 使用测试图片
ros2 run image_publisher image_publisher_node test.jpg
ros2 run u_webrtc webrtc_streamer_node

# 使用 USB 摄像头
ros2 run usb_cam usb_cam_node_exe
ros2 run u_webrtc webrtc_streamer_node
```

### 场景 2: 远程监控
```yaml
# 配置公网 TURN 服务器
turn_server: "turn:turn.example.com:3478"
turn_username: "user"
turn_password: "pass"

# 降低比特率以适应网络
target_bitrate: 500000
```

### 场景 3: 多路流
```bash
# 启动多个实例（不同端口）
ros2 run u_webrtc webrtc_streamer_node --ros-args \
    -p image_topic:=/camera1/image \
    -p signaling_server_url:=ws://localhost:8081

ros2 run u_webrtc webrtc_streamer_node --ros-args \
    -p image_topic:=/camera2/image \
    -p signaling_server_url:=ws://localhost:8082
```

## 🚨 紧急修复

### 完全重置
```bash
# 停止所有服务
pkill -f server.py
pkill -f webrtc_streamer

# 清理编译
cd ~/work
rm -rf build install log

# 重新编译
colcon build

# 重启
# ... 按正常流程启动
```

### 重新安装依赖
```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc

# 卸载
sudo rm -rf /usr/local/lib/libdatachannel*
sudo rm -rf /usr/local/include/rtc/

# 重新安装
./scripts/install_libdatachannel.sh
./scripts/install_libvpx.sh
./scripts/install_websocketpp.sh

# 重新编译
cd ~/work
rm -rf build/u_webrtc
colcon build --packages-select u_webrtc
```

## 📞 获取帮助

### 日志位置
```bash
# ROS2 日志
~/.ros/log/

# 信令服务器日志
~/Path/work/ros2_test_env/src/u_webrtc/signaling_server/signaling.log

# 系统日志
journalctl -u webrtc-streamer
journalctl -u webrtc-signaling
```

### 诊断信息收集
```bash
# 系统信息
uname -a
lsb_release -a

# ROS2 环境
printenv | grep ROS

# 依赖版本
pkg-config --list-all | grep -E "libdatachannel|vpx|opencv"

# 节点状态
ros2 node list
ros2 topic list
ros2 param list
```

---

**保存此页面以便快速查阅！** 📌

