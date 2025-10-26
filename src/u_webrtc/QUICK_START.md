# u_webrtc 快速开始指南

## 🎯 5 分钟快速体验

### 前提条件

- ✅ Ubuntu 22.04
- ✅ ROS2 Humble 已安装
- ✅ Python 3.x

### 步骤 1：安装依赖（约 2 分钟）

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/install_dependencies.sh
```

### 步骤 2：编译项目（约 1 分钟）

```bash
cd ~/Path/work/ros2_test_env
source /opt/ros/humble/setup.bash
colcon build --packages-select u_webrtc
source install/setup.bash
```

或使用测试脚本：

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/test_build.sh
```

### 步骤 3：启动系统（3 个终端）

#### 终端 1：启动信令服务器

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc/signaling_server
pip install -r requirements.txt
python3 server.py
```

您应该看到：
```
WebRTC 信令服务器启动中...
监听地址: ws://0.0.0.0:8080
服务器已启动，等待客户端连接...
```

#### 终端 2：启动模拟相机（可选，如果没有真实相机）

```bash
# 安装 image_publisher
sudo apt-get install ros-humble-image-publisher

# 发布测试图像
ros2 run image_publisher image_publisher_node \
    --ros-args \
    -p filename:=/usr/share/pixmaps/debian-logo.png \
    -r image_raw:=/camera/image_raw
```

#### 终端 3：启动 WebRTC 节点

```bash
cd ~/Path/work/ros2_test_env
source install/setup.bash
ros2 launch u_webrtc webrtc_stream_simple.launch.py
```

您应该看到：
```
[INFO] [webrtc_streamer]: WebRTC 视频流节点启动成功
[INFO] [webrtc_streamer]: 已连接到信令服务器
```

### 步骤 4：打开 Web 客户端

#### 方式 A：使用 Python HTTP 服务器（推荐）

在新终端中：

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc/web_client
python3 -m http.server 8000
```

然后在浏览器中打开：`http://localhost:8000`

#### 方式 B：直接打开 HTML 文件

```bash
# 使用默认浏览器打开
xdg-open ~/Path/work/ros2_test_env/src/u_webrtc/web_client/index.html
```

### 步骤 5：连接并查看视频

1. 在 Web 客户端中，确认服务器地址为 `ws://localhost:8080`
2. 点击"连接"按钮
3. 等待视频流显示

## 🔍 验证安装

### 检查 ROS2 节点

```bash
# 查看节点
ros2 node list | grep webrtc

# 查看节点信息
ros2 node info /webrtc_streamer

# 查看订阅的话题
ros2 topic info /camera/image_raw
```

### 检查日志

```bash
# 实时查看日志
ros2 topic echo /rosout | grep webrtc
```

## 🛠️ 使用快速启动脚本

```bash
cd ~/Path/work/ros2_test_env/src/u_webrtc
./scripts/quick_start.sh
```

然后选择相应的选项：

- **1** - 安装依赖
- **2** - 编译项目
- **3** - 启动信令服务器
- **4** - 启动 ROS2 节点
- **5** - 启动 Web 客户端
- **6** - 显示项目信息

## ⚙️ 自定义配置

### 修改图像话题

```bash
# 方式 1：使用启动参数
ros2 launch u_webrtc webrtc_stream.launch.py \
    image_topic:=/my_camera/image

# 方式 2：修改配置文件
nano config/webrtc_config.yaml
```

### 调整视频质量

编辑 `config/webrtc_config.yaml`：

```yaml
# 高质量（高带宽）
target_bitrate: 5000000  # 5 Mbps
max_framerate: 60
width: 1920
height: 1080

# 低延迟（低带宽）
target_bitrate: 1000000  # 1 Mbps
max_framerate: 15
width: 640
height: 480
```

### 修改信令服务器地址

```bash
ros2 launch u_webrtc webrtc_stream.launch.py \
    signaling_server_url:=ws://192.168.1.100:8080
```

## 🐛 常见问题

### 问题 1：编译失败

**错误**：`nlohmann/json.hpp: No such file or directory`

**解决**：
```bash
sudo apt-get install nlohmann-json3-dev
```

### 问题 2：找不到图像话题

**错误**：节点启动但无视频输出

**解决**：
```bash
# 检查话题列表
ros2 topic list | grep image

# 确认话题是否正确
ros2 topic info /camera/image_raw

# 发布测试图像
ros2 run image_publisher image_publisher_node test_image.jpg
```

### 问题 3：信令服务器连接失败

**错误**：`连接到信令服务器失败`

**解决**：
1. 确认信令服务器正在运行
2. 检查端口 8080 是否被占用：`netstat -tuln | grep 8080`
3. 检查防火墙设置

### 问题 4：Web 客户端无法连接

**错误**：浏览器控制台显示 WebSocket 连接错误

**解决**：
1. 确认信令服务器地址正确
2. 检查浏览器是否支持 WebRTC
3. 尝试使用 Chrome 或 Firefox

## 📊 性能监控

### 查看节点状态

```bash
# CPU 和内存使用
top -p $(pgrep -f webrtc_streamer_node)

# 话题频率
ros2 topic hz /camera/image_raw

# 话题带宽
ros2 topic bw /camera/image_raw
```

### Web 客户端统计

在 Web 客户端中查看实时统计信息，包括：
- 接收帧数
- 连接状态
- 比特率

## 🎯 下一步

1. **阅读完整文档**：`README_USAGE.md`
2. **查看技术方案**：`readme.md`
3. **项目概览**：`PROJECT_OVERVIEW.md`
4. **集成真实 WebRTC 库**：参考文档中的集成指南

## 💡 提示

- 使用 `Ctrl+C` 停止运行中的程序
- 建议使用 `tmux` 或 `screen` 管理多个终端
- 查看日志文件了解详细错误信息
- 首次运行建议使用测试图像而非真实相机

## 🔗 相关资源

- [ROS2 文档](https://docs.ros.org/en/humble/)
- [WebRTC 文档](https://webrtc.org/)
- [OpenCV 文档](https://docs.opencv.org/)

---

**祝您使用愉快！** 🚀

如有问题，请查看 `README_USAGE.md` 中的故障排查部分。



