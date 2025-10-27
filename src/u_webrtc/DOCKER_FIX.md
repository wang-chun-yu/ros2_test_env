# Docker 环境运行问题修复

## 🐛 问题描述

在 Docker 容器中运行 u_webrtc 时遇到以下错误：

### 错误 1: TLS 握手失败
```
[ERROR] [SignalingClient]: WebSocket 连接失败: TLS handshake failed
```

### 错误 2: 程序崩溃
```
terminate called without an active exception
[ERROR] [webrtc_streamer_node-1]: process has died [pid 5909, exit code -6]
```

## 🔍 根本原因

### 原因 1: TLS 配置不匹配

**问题**：旧的 `signaling_client.cpp` 使用了 `asio_tls_client` 配置，强制所有连接使用 TLS。

```cpp
// 旧代码（有问题）
typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
```

但信令服务器运行在普通的 `ws://localhost:8080`（非加密），导致 TLS 握手失败。

**WebSocket 协议区别**：
- `ws://` - 普通 WebSocket（非加密）
- `wss://` - 安全 WebSocket（TLS 加密）

### 原因 2: 连接失败后资源清理不当

WebSocket 连接失败后，某些资源未正确清理，导致程序异常退出。

## ✅ 解决方案

### 方案：自动检测 URL 类型

修改 `signaling_client.cpp`，根据 URL scheme 自动选择客户端类型：

```cpp
// 新代码（已修复）
typedef websocketpp::client<websocketpp::config::asio_client> ws_client;      // ws://
typedef websocketpp::client<websocketpp::config::asio_tls_client> wss_client; // wss://

// 自动检测
_useTls = (_serverUrl.find("wss://") == 0);

if (_useTls) {
    // 使用 WSS 客户端（TLS 加密）
    _wssClient = std::make_unique<wss_client>();
} else {
    // 使用 WS 客户端（普通连接）
    _wsClient = std::make_unique<ws_client>();
}
```

### 改进点

1. **双客户端支持**
   - `ws_client`: 用于 `ws://` URL
   - `wss_client`: 用于 `wss://` URL

2. **自动检测**
   - 根据 URL 前缀自动选择
   - 无需手动配置

3. **更好的错误处理**
   - 分别处理 WS 和 WSS 的错误
   - 改进资源清理

## 🔧 应用修复

### 步骤 1: 更新代码（已完成）

代码已自动更新到 `src/u_webrtc/src/signaling_client.cpp`

### 步骤 2: 重新编译

```bash
# 在 Docker 容器中执行
cd ~/work
rm -rf build/u_webrtc install/u_webrtc
colcon build --packages-select u_webrtc
source install/setup.bash
```

### 步骤 3: 验证修复

```bash
# 终端 1: 启动信令服务器
cd ~/work/src/u_webrtc/signaling_server
python3 server.py

# 终端 2: 启动 ROS2 节点
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py
```

**期望输出**：

```
[INFO] [SignalingClient]: WebSocket 客户端初始化完成 (WS 模式)
[INFO] [SignalingClient]: ✅ WebSocket 连接已建立: ws://localhost:8080
```

## 📊 验证清单

### ✅ 编译验证
```bash
cd ~/work
colcon build --packages-select u_webrtc 2>&1 | grep -i "finished"
# 应该看到: Finished <<< u_webrtc
```

### ✅ 运行验证
```bash
# 1. 检查节点是否启动
ros2 node list | grep webrtc_streamer
# 应该看到: /webrtc_streamer

# 2. 检查日志
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level info
# 应该看到: WebSocket 客户端初始化完成 (WS 模式)
# 应该看到: ✅ WebSocket 连接已建立
```

### ✅ 连接验证
```bash
# 在另一个终端测试 WebSocket 连接
telnet localhost 8080
# 或
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" http://localhost:8080
```

## 🎯 URL 配置指南

### 开发/测试环境（本地）
```yaml
# config/webrtc_config.yaml
signaling_server_url: "ws://localhost:8080"
```

### 生产环境（局域网）
```yaml
signaling_server_url: "ws://192.168.1.100:8080"
```

### 生产环境（互联网 + TLS）
```yaml
signaling_server_url: "wss://your-domain.com:443"
```

**注意**：
- 使用 `wss://` 需要配置信令服务器的 SSL 证书
- 详见 `WEBSOCKET_CLIENT.md` 中的 TLS 配置说明

## 🐳 Docker 特定注意事项

### 1. 网络配置

如果信令服务器在 Docker 外部运行：

```yaml
# 使用主机 IP 而不是 localhost
signaling_server_url: "ws://172.17.0.1:8080"  # Docker 默认网关
# 或
signaling_server_url: "ws://192.168.1.100:8080"  # 主机局域网 IP
```

### 2. 端口映射

确保 Docker 容器端口正确映射：

```bash
docker run -p 8080:8080 your-image
```

### 3. 防火墙

确保防火墙允许 8080 端口：

```bash
sudo ufw allow 8080
```

## 🔍 调试技巧

### 1. 检查 WebSocket 服务器

```bash
# 查看进程
ps aux | grep server.py

# 查看端口
netstat -tulpn | grep 8080

# 测试连接
curl http://localhost:8080
```

### 2. 启用详细日志

```bash
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level debug
```

### 3. 检查 WebSocket 握手

```python
# 在信令服务器添加调试日志
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 📚 相关文档

- **[WEBSOCKET_CLIENT.md](WEBSOCKET_CLIENT.md)** - WebSocket 客户端详细说明
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 故障排查指南
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - 命令速查

## 🎉 总结

### 修复内容
✅ 修复 TLS 握手失败问题  
✅ 支持 `ws://` 和 `wss://` 自动检测  
✅ 改进错误处理和资源清理  
✅ 添加详细的日志输出  

### 下一步
1. 重新编译项目
2. 测试 WebSocket 连接
3. 验证视频流传输

如果问题仍然存在，请查看 `TROUBLESHOOTING.md` 或启用详细日志进行调试。

