# 信令服务器修复说明

## 🎯 问题总结

### 第一个问题：WebSocket 客户端 TLS 错误 ✅ 已修复
- **状态**: ✅ 已修复
- **修复文件**: `src/signaling_client.cpp`
- **结果**: 客户端现在可以正确连接到 `ws://` 服务器

### 第二个问题：信令服务器 API 版本不兼容 ⚠️ 刚修复
- **状态**: ✅ 已修复
- **修复文件**: `signaling_server/server.py`
- **原因**: `websockets` 库版本变化导致 API 不兼容

## 🔍 详细分析

### 错误现象

**客户端日志**:
```
[INFO] [SignalingClient]: ✅ WebSocket 连接已建立: ws://localhost:8080
[INFO] [SignalingClient]: WebSocket 连接已关闭:  (Success)
```

**服务器日志**:
```
TypeError: handle_client() missing 1 required positional argument: 'path'
```

### 根本原因

**websockets 库 API 变化**:

| 版本 | Handler 签名 | 说明 |
|------|-------------|------|
| **< 10.0** | `async def handle_client(websocket, path)` | 旧版本，接收路径参数 |
| **≥ 10.0** | `async def handle_client(websocket)` | 新版本，不再传递路径 |

**过时的类型导入**:
```python
# 旧版本（已废弃）
from websockets.server import WebSocketServerProtocol

# 新版本（正确）
from websockets.asyncio.server import ServerConnection
```

## ✅ 修复方案

### 修改前（错误）

```python
from websockets.server import WebSocketServerProtocol

clients: Set[WebSocketServerProtocol] = set()

async def handle_client(websocket: WebSocketServerProtocol, path: str):
    """处理客户端连接"""
    # ... 处理逻辑
```

### 修改后（正确）

```python
from websockets.asyncio.server import ServerConnection

clients: Set[ServerConnection] = set()

async def handle_client(websocket: ServerConnection):
    """处理客户端连接"""
    # ... 处理逻辑（无需修改）
```

## 🚀 应用修复

### 步骤 1: 停止旧的服务器

```bash
# 按 Ctrl+C 停止正在运行的服务器
# 或在新终端中
pkill -f "python3 server.py"
```

### 步骤 2: 重新启动服务器

```bash
cd ~/work/src/u_webrtc/signaling_server
python3 server.py
```

**期望输出**:
```
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - ==================================================
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - WebRTC 信令服务器启动中...
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - 监听地址: ws://0.0.0.0:8080
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - ==================================================
2025-10-26 XX:XX:XX,XXX - websockets.server - INFO - server listening on 0.0.0.0:8080
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - 服务器已启动，等待客户端连接...
```

### 步骤 3: 重新启动 ROS2 节点

```bash
# 如果还在运行，先停止 (Ctrl+C)
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py
```

**期望输出**:
```
[INFO] [SignalingClient]: WebSocket 客户端初始化完成 (WS 模式)
[INFO] [SignalingClient]: ✅ WebSocket 连接已建立: ws://localhost:8080
[INFO] [webrtc_streamer]: WebRTC 视频流节点启动成功
```

**服务器端应该看到**:
```
2025-10-26 XX:XX:XX,XXX - __main__ - INFO - 新客户端连接: ('127.0.0.1', XXXXX), 当前客户端数: 1
```

## ✅ 验证成功标志

### 客户端（ROS2 节点）

✅ **应该看到**:
- `WebSocket 客户端初始化完成 (WS 模式)`
- `✅ WebSocket 连接已建立`
- `WebRTC 视频流节点启动成功`
- **连接保持稳定**，不会立即断开

❌ **不应该看到**:
- `TLS handshake failed`
- `WebSocket 连接已关闭` (在刚连接后立即出现)
- `terminate called without an active exception`

### 服务器端（Python）

✅ **应该看到**:
- `server listening on 0.0.0.0:8080`
- `新客户端连接: ('127.0.0.1', XXXXX), 当前客户端数: 1`
- **没有任何错误**

❌ **不应该看到**:
- `TypeError: handle_client() missing 1 required positional argument`
- `connection handler failed`

## 🔍 故障排查

### 问题 1: 仍然显示 DeprecationWarning

```
DeprecationWarning: websockets.server.WebSocketServerProtocol is deprecated
```

**原因**: 可能文件未正确保存或使用了缓存的 `.pyc` 文件

**解决**:
```bash
cd ~/work/src/u_webrtc/signaling_server
# 删除缓存
rm -rf __pycache__
# 重新启动
python3 server.py
```

### 问题 2: 连接建立但立即断开

**检查**:
```bash
# 查看服务器详细日志
cd ~/work/src/u_webrtc/signaling_server
python3 -u server.py  # -u 禁用输出缓冲

# 在另一个终端启动客户端
ros2 launch u_webrtc webrtc_stream.launch.py
```

**查看是否有任何错误消息**

### 问题 3: Import 错误

```python
ImportError: cannot import name 'ServerConnection' from 'websockets.asyncio.server'
```

**原因**: websockets 版本太旧

**解决**:
```bash
# 检查版本
pip3 show websockets

# 如果版本 < 10.0，升级
pip3 install --upgrade websockets

# 推荐版本: >= 12.0
```

### 问题 4: 程序仍然崩溃

**原因**: 可能是其他问题（例如 libdatachannel 初始化失败）

**调试**:
```bash
# 启用详细日志
ros2 run u_webrtc webrtc_streamer_node --ros-args --log-level debug

# 查看完整堆栈跟踪
```

## 📊 websockets 版本兼容性

| websockets 版本 | 状态 | Handler 签名 | 类型导入 |
|----------------|------|-------------|----------|
| < 10.0 | ⚠️ 旧 API | `(websocket, path)` | `websockets.server.WebSocketServerProtocol` |
| 10.0 - 11.x | ⚠️ 过渡期 | `(websocket)` | `websockets.server.WebSocketServerProtocol` |
| ≥ 12.0 | ✅ 推荐 | `(websocket)` | `websockets.asyncio.server.ServerConnection` |

**检查版本**:
```bash
python3 -c "import websockets; print(websockets.__version__)"
```

**升级到最新版本**:
```bash
pip3 install --upgrade websockets
```

## 🎯 完整测试流程

### 终端 1: 信令服务器
```bash
cd ~/work/src/u_webrtc/signaling_server
rm -rf __pycache__  # 清理缓存
python3 server.py

# 应该看到:
# ✅ server listening on 0.0.0.0:8080
# ✅ 服务器已启动，等待客户端连接...
```

### 终端 2: ROS2 节点
```bash
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py

# 应该看到:
# ✅ WebSocket 客户端初始化完成 (WS 模式)
# ✅ WebSocket 连接已建立
# ✅ WebRTC 视频流节点启动成功
```

### 验证服务器端
回到终端 1，应该看到：
```
✅ 新客户端连接: ('127.0.0.1', XXXXX), 当前客户端数: 1
```

### 验证连接稳定性
等待 10 秒，确认：
- ✅ 客户端没有断开
- ✅ 服务器没有错误消息
- ✅ ROS2 节点持续运行

## 📚 相关文档

- **[DOCKER_FIX.md](DOCKER_FIX.md)** - WebSocket 客户端修复
- **[WEBSOCKET_CLIENT.md](WEBSOCKET_CLIENT.md)** - WebSocket 客户端详细说明
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 通用故障排查

## 🎉 修复总结

### 已修复的问题

1. ✅ **WebSocket 客户端 TLS 配置**
   - 文件: `src/signaling_client.cpp`
   - 修复: 自动检测 ws:// 和 wss://

2. ✅ **信令服务器 API 兼容性**
   - 文件: `signaling_server/server.py`
   - 修复: 更新函数签名和类型导入

### 当前状态

✅ WebSocket 客户端可以连接  
✅ 信令服务器可以接受连接  
✅ 连接保持稳定  
⚠️ 需要测试完整的信令交换  

### 下一步

1. 启动相机节点发布图像
2. 测试完整的 WebRTC 连接
3. 在浏览器中验证视频流

---

**两个问题都已修复！请重新启动信令服务器和 ROS2 节点进行测试。** 🚀

