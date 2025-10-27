# 完整部署指南

## 🎯 u_webrtc 完整生产环境部署

本指南涵盖从零开始部署完整的 ROS2-WebRTC 视频流系统的所有步骤。

## 📋 系统要求

### 硬件要求

- **CPU**: 4 核心或更多（推荐 8 核心）
- **内存**: 最小 4GB RAM（推荐 8GB+）
- **网络**: 1Gbps 以太网（对于高清流）
- **摄像头**: USB 摄像头或 ROS2 兼容的相机

### 软件要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble
- **编译器**: GCC 11+ 或 Clang 14+
- **CMake**: 3.16+
- **Python**: 3.10+

## 🚀 完整部署流程

### 阶段 0: 准备基础环境

```bash
# 更新系统
sudo apt-get update
sudo apt-get upgrade -y

# 安装 ROS2 Humble（如果尚未安装）
# 参考: https://docs.ros.org/en/humble/Installation.html

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 安装基础开发工具
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-pip
```

### 阶段 1: 安装必需依赖（必须）

```bash
# 1. 安装 OpenCV
sudo apt-get install -y libopencv-dev

# 2. 安装 nlohmann-json
sudo apt-get install -y nlohmann-json3-dev

# 3. 安装 ROS2 cv_bridge
sudo apt-get install -y ros-humble-cv-bridge

# 4. 安装 ROS2 image-transport
sudo apt-get install -y ros-humble-image-transport

# 验证安装
pkg-config --modversion opencv4
dpkg -l | grep nlohmann-json3-dev
```

**此时可以编译基础框架，但不包含真实功能。**

### 阶段 2: 安装 WebRTC 传输层（核心）

```bash
# 安装 libdatachannel
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/install_libdatachannel.sh

# 等待编译完成（可能需要 10-20 分钟）
# 验证安装
pkg-config --modversion libdatachannel
```

**此时可以进行 WebRTC 传输，但视频编码和信令是框架实现。**

### 阶段 3: 安装视频编码器（关键）

```bash
# 安装 libvpx
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/install_libvpx.sh

# 验证安装
pkg-config --modversion vpx
```

**此时可以进行真实的 VP8 视频编码，但信令仍是框架实现。**

### 阶段 4: 安装 WebSocket 信令（完整）

```bash
# 安装 websocketpp 和 Boost
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/install_websocketpp.sh

# 验证安装
ls /usr/include/websocketpp/
dpkg -l | grep libboost
```

**此时所有核心功能都是真实实现，生产环境完全就绪。**

### 阶段 5: 编译 u_webrtc

```bash
# 进入工作空间
cd ~/work

# 清理旧的编译文件
rm -rf build/u_webrtc install/u_webrtc log/u_webrtc

# 编译
colcon build --packages-select u_webrtc

# 查看编译输出，应该看到:
# ✅ Found LibDataChannel - WebRTC functionality enabled
# ✅ Found libvpx - VP8/VP9 encoding enabled
# ✅ Found WebSocket++ - Real WebSocket signaling enabled

# 如果编译失败，查看错误信息
colcon build --packages-select u_webrtc --event-handlers console_direct+

# 设置环境
source install/setup.bash
```

### 阶段 6: 部署信令服务器

```bash
# 进入信令服务器目录
cd ~/Path/work/ros2_test_env/src/u_webrtc/signaling_server

# 安装 Python 依赖
pip3 install -r requirements.txt

# 测试运行
python3 server.py

# 应该看到:
# WebSocket 信令服务器运行在 ws://0.0.0.0:8080

# 后台运行（生产环境）
nohup python3 server.py > signaling.log 2>&1 &

# 检查进程
ps aux | grep server.py
```

### 阶段 7: 部署 Web 客户端

```bash
# 进入 Web 客户端目录
cd ~/Path/work/ros2_test_env/src/u_webrtc/web_client

# 方式 1: 使用 Python HTTP 服务器（开发/测试）
python3 -m http.server 8000

# 方式 2: 使用 Nginx（生产环境）
sudo apt-get install -y nginx

# 复制文件到 Nginx 目录
sudo cp -r . /var/www/html/webrtc/

# 配置 Nginx
sudo nano /etc/nginx/sites-available/webrtc

# 添加配置:
# server {
#     listen 80;
#     server_name your-domain.com;
#     
#     location /webrtc/ {
#         root /var/www/html;
#         index index.html;
#     }
# }

# 启用配置
sudo ln -s /etc/nginx/sites-available/webrtc /etc/nginx/sites-enabled/
sudo nginx -t
sudo systemctl reload nginx
```

### 阶段 8: 配置相机源

```bash
# 方式 1: USB 摄像头
ros2 run usb_cam usb_cam_node_exe

# 方式 2: 使用 v4l2_camera
sudo apt-get install -y ros-humble-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node

# 方式 3: 使用模拟相机（测试）
sudo apt-get install -y ros-humble-image-publisher
ros2 run image_publisher image_publisher_node /path/to/image.jpg

# 验证相机话题
ros2 topic list | grep image
ros2 topic hz /camera/image_raw
```

### 阶段 9: 配置 u_webrtc 参数

```bash
# 编辑配置文件
cd ~/Path/work/ros2_test_env/src/u_webrtc/config
nano webrtc_config.yaml

# 关键配置项:
webrtc_streamer:
  ros__parameters:
    # 输入话题
    image_topic: "/camera/image_raw"
    
    # 视频参数
    width: 1280
    height: 720
    target_bitrate: 2000000  # 2 Mbps
    max_framerate: 30
    
    # 信令服务器
    signaling_server_url: "ws://localhost:8080"
    
    # STUN/TURN 服务器
    stun_server: "stun:stun.l.google.com:19302"
    # turn_server: "turn:your-turn-server:3478"
    # turn_username: "username"
    # turn_password: "password"
```

### 阶段 10: 启动完整系统

```bash
# 终端 1: 信令服务器
cd ~/Path/work/ros2_test_env/src/u_webrtc/signaling_server
python3 server.py

# 终端 2: 相机节点（根据实际情况选择）
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe

# 终端 3: u_webrtc 节点
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py

# 或使用简化启动:
ros2 launch u_webrtc webrtc_stream_simple.launch.py

# 终端 4: Web 服务器
cd ~/Path/work/ros2_test_env/src/u_webrtc/web_client
python3 -m http.server 8000

# 浏览器: 打开 http://localhost:8000
# 点击"连接"按钮
```

## 🔍 验证部署

### 1. 验证依赖安装

```bash
# 检查 libdatachannel
pkg-config --modversion libdatachannel || echo "未安装"

# 检查 libvpx
pkg-config --modversion vpx || echo "未安装"

# 检查 websocketpp
ls /usr/include/websocketpp/ || echo "未安装"

# 检查 Boost
dpkg -l | grep libboost || echo "未安装"
```

### 2. 验证编译状态

```bash
cd ~/work
colcon build --packages-select u_webrtc 2>&1 | grep "Found"

# 应该看到:
# ✅ Found LibDataChannel
# ✅ Found libvpx
# ✅ Found WebSocket++
```

### 3. 验证节点运行

```bash
# 检查节点
ros2 node list
# 应该看到: /webrtc_streamer

# 检查话题
ros2 topic list | grep webrtc

# 检查参数
ros2 param list /webrtc_streamer
```

### 4. 验证 WebSocket 连接

```bash
# 使用 websocat 测试（可选）
sudo apt-get install -y websocat
websocat ws://localhost:8080

# 应该看到连接成功
```

### 5. 验证端到端流

- **浏览器**: 打开开发者工具 (F12)
- **Console**: 应该看到 "WebSocket connected"
- **Network**: 应该看到 WebSocket 连接和数据流
- **Video**: 应该看到视频流播放

## 🐛 常见问题排查

### 问题 1: 编译失败

```bash
# 查看详细错误
colcon build --packages-select u_webrtc --event-handlers console_direct+

# 检查依赖
./scripts/install_dependencies.sh

# 清理后重新编译
cd ~/work
rm -rf build install log
colcon build
```

### 问题 2: 找不到相机话题

```bash
# 列出所有话题
ros2 topic list

# 检查相机节点
ros2 node list

# 测试相机
ros2 run rqt_image_view rqt_image_view
```

### 问题 3: WebSocket 连接失败

```bash
# 检查信令服务器
curl http://localhost:8080
telnet localhost 8080

# 检查防火墙
sudo ufw status
sudo ufw allow 8080

# 查看日志
cat signaling.log
```

### 问题 4: 浏览器无视频

- 检查浏览器控制台错误
- 验证 ICE 候选交换
- 检查 STUN/TURN 配置
- 查看 ROS2 节点日志

```bash
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level debug
```

### 问题 5: 延迟过高

```yaml
# 调整配置参数
width: 640           # 降低分辨率
height: 480
target_bitrate: 1000000  # 降低比特率
max_framerate: 30
encoding_threads: 2  # 减少线程
```

## 📊 性能优化

### 1. 网络优化

```yaml
# 使用 TURN 服务器（穿透 NAT）
turn_server: "turn:your-server:3478"
turn_username: "user"
turn_password: "pass"

# 调整比特率
target_bitrate: 2000000  # 2 Mbps（局域网）
# target_bitrate: 500000   # 500 Kbps（互联网）
```

### 2. 编码优化

```yaml
# 低延迟模式
width: 640
height: 480
max_framerate: 60
encoding_threads: 2

# 高质量模式
width: 1920
height: 1080
max_framerate: 30
encoding_threads: 8
```

### 3. 系统优化

```bash
# 提高进程优先级
sudo nice -n -10 ros2 launch u_webrtc webrtc_stream.launch.py

# 使用实时调度
sudo chrt -f 50 ros2 launch u_webrtc webrtc_stream.launch.py

# 禁用 CPU 节能
sudo cpupower frequency-set -g performance
```

## 🔐 安全配置

### 1. 使用 HTTPS（生产环境）

```nginx
server {
    listen 443 ssl;
    server_name your-domain.com;
    
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;
    
    location /webrtc/ {
        root /var/www/html;
    }
}
```

### 2. 使用 WSS（加密 WebSocket）

```python
# signaling_server/server.py
# 修改为使用 SSL
import ssl

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain('cert.pem', 'key.pem')

# 在 websockets.serve 中使用 ssl_context
```

### 3. 添加认证

```yaml
# 在配置中添加认证 token
webrtc_streamer:
  ros__parameters:
    auth_token: "your-secret-token"
```

## 🎯 生产环境检查清单

- [ ] 所有依赖已安装并验证
- [ ] 编译输出显示所有库已找到
- [ ] 相机节点正常运行
- [ ] 信令服务器正常运行
- [ ] Web 客户端可访问
- [ ] WebSocket 连接成功
- [ ] 视频流正常显示
- [ ] 延迟在可接受范围内（<200ms）
- [ ] CPU 使用率正常（<50%）
- [ ] 内存使用正常（<2GB）
- [ ] 网络带宽充足
- [ ] 使用 HTTPS/WSS（生产环境）
- [ ] 配置了 TURN 服务器（公网环境）
- [ ] 添加了认证机制
- [ ] 配置了日志和监控
- [ ] 设置了自动重启机制

## 🔄 自动化部署

### systemd 服务配置

```bash
# 创建信令服务
sudo nano /etc/systemd/system/webrtc-signaling.service

[Unit]
Description=WebRTC Signaling Server
After=network.target

[Service]
Type=simple
User=ros
WorkingDirectory=/home/ros/Path/work/ros2_test_env/src/u_webrtc/signaling_server
ExecStart=/usr/bin/python3 server.py
Restart=always

[Install]
WantedBy=multi-user.target

# 创建 ROS2 服务
sudo nano /etc/systemd/system/webrtc-streamer.service

[Unit]
Description=WebRTC Streamer Node
After=network.target webrtc-signaling.service

[Service]
Type=simple
User=ros
Environment="ROS_DOMAIN_ID=0"
WorkingDirectory=/home/ros/work
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch u_webrtc webrtc_stream.launch.py'
Restart=always

[Install]
WantedBy=multi-user.target

# 启用服务
sudo systemctl daemon-reload
sudo systemctl enable webrtc-signaling
sudo systemctl enable webrtc-streamer

# 启动服务
sudo systemctl start webrtc-signaling
sudo systemctl start webrtc-streamer

# 查看状态
sudo systemctl status webrtc-signaling
sudo systemctl status webrtc-streamer
```

## 📈 监控和日志

```bash
# 查看 ROS2 日志
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level info

# 查看系统资源
htop
iotop
nethogs

# 实时监控话题
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw

# 导出日志
ros2 bag record -a  # 记录所有话题
```

## 🎉 完成！

恭喜！您已成功部署完整的 ROS2-WebRTC 视频流系统。

**下一步**:
1. 根据实际需求调整配置参数
2. 进行性能测试和优化
3. 配置监控和告警
4. 准备备份和恢复方案

**需要帮助？**
- 查看 TROUBLESHOOTING.md
- 查看各模块的详细文档
- 提交 Issue

---

**祝您使用愉快！** 🚀

