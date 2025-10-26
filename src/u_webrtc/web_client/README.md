# WebRTC 视频流 Web 客户端

这是一个简单的 HTML5 WebRTC 客户端，用于接收来自 ROS2 节点的视频流。

## 使用方法

### 方式 1：使用 Python HTTP 服务器

```bash
cd web_client
python3 -m http.server 8000
```

然后在浏览器中访问：`http://localhost:8000`

### 方式 2：直接用浏览器打开

直接双击 `index.html` 文件，用浏览器打开。

## 操作步骤

1. 启动信令服务器（在另一个终端）：
   ```bash
   cd ../signaling_server
   python3 server.py
   ```

2. 启动 ROS2 WebRTC 节点：
   ```bash
   ros2 launch u_webrtc webrtc_stream_simple.launch.py
   ```

3. 在浏览器中：
   - 输入信令服务器地址（默认：`ws://localhost:8080`）
   - 点击"连接"按钮
   - 等待视频流显示

## 功能特性

- ✅ WebRTC 视频接收
- ✅ 自动处理 ICE 协商
- ✅ 实时统计信息显示
- ✅ 美观的现代化 UI
- ✅ 连接状态监控

## 浏览器兼容性

- Chrome/Edge：完全支持
- Firefox：完全支持
- Safari：需要 HTTPS（本地开发可能需要额外配置）

## 故障排查

### 问题：视频无法显示

**可能原因**：
1. 信令服务器未启动
2. ROS2 节点未运行
3. 浏览器不支持 WebRTC

**解决方法**：
- 检查浏览器控制台错误信息（F12）
- 确认所有服务都已启动
- 尝试使用 Chrome/Firefox 浏览器

### 问题：连接一直处于 "connecting" 状态

**可能原因**：
1. STUN/TURN 服务器配置问题
2. 防火墙阻止 WebRTC 连接
3. NAT 穿透失败

**解决方法**：
- 在本地网络测试
- 配置 TURN 服务器
- 检查防火墙设置



