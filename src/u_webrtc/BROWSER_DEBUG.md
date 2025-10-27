# 浏览器客户端调试指南

## 🔍 问题诊断

从信令服务器日志可以看到：
```
✅ 浏览器已连接到信令服务器
✅ 服务器转发了 Offer 给浏览器
❌ 但浏览器没有发送 Answer
```

这表明**浏览器客户端没有正确响应**。

## 🎯 完整测试流程

### 步骤 1: 确认所有服务运行

```bash
# 终端 1: 信令服务器
cd ~/work/src/u_webrtc/signaling_server
python3 server.py
# 应该看到: server listening on 0.0.0.0:8080

# 终端 2: Web 服务器
cd ~/work/src/u_webrtc/web_client
python3 -m http.server 8000
# 应该看到: Serving HTTP on 0.0.0.0 port 8000
```

### 步骤 2: 打开浏览器并检查控制台

1. **打开浏览器**
   ```
   http://localhost:8000
   ```

2. **立即打开开发者工具**（在加载任何东西之前）
   - Chrome/Edge: 按 F12 或 Ctrl+Shift+I
   - Firefox: 按 F12
   - Safari: Cmd+Option+I (Mac)

3. **切换到 Console 标签**

4. **查看是否有任何错误**（红色文字）
   - 如果有错误，记录下来
   - 常见错误：
     ```
     Uncaught ReferenceError: xxx is not defined
     Failed to load resource
     WebSocket connection failed
     ```

### 步骤 3: 测试 WebSocket 连接

在浏览器控制台中运行：

```javascript
// 测试 WebSocket 连接
const ws = new WebSocket('ws://localhost:8080');
ws.onopen = () => console.log('✅ WebSocket 连接成功');
ws.onerror = (e) => console.error('❌ WebSocket 错误:', e);
ws.onmessage = (e) => console.log('📨 收到消息:', e.data);
```

**期望输出**:
```
✅ WebSocket 连接成功
```

### 步骤 4: 点击"连接"按钮（重要！）

页面上应该有一个**"连接"或"Connect"按钮**。

**必须点击这个按钮**才能：
1. 建立 WebSocket 连接
2. 创建 PeerConnection
3. 接收和处理 Offer
4. 发送 Answer

### 步骤 5: 启动 ROS2 节点

```bash
# 终端 3: ROS2 节点（等浏览器准备好后）
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py
```

### 步骤 6: 观察日志

**浏览器控制台应该显示**:
```
✅ WebSocket connected
📨 Received offer from peer
✅ Created answer
✅ Sent answer
✅ ICE gathering...
✅ ICE candidate: xxx
✅ WebRTC connected
🎥 Video playing
```

**信令服务器应该显示**:
```
✅ 新客户端连接: (浏览器)
✅ 新客户端连接: (ROS2)
✅ 收到 offer
✅ 消息已转发给 1 个客户端
✅ 收到 answer  ← 关键！
✅ 消息已转发给 1 个客户端
```

**ROS2 节点应该显示**:
```
✅ WebSocket 连接已建立
✅ 已发送 Offer
✅ 收到 Answer  ← 关键！
✅ WebRTC 连接已建立
```

## 🐛 常见问题

### 问题 1: 浏览器控制台显示"client.js 404"

**原因**: JavaScript 文件路径错误或文件不存在

**检查**:
```bash
ls ~/work/src/u_webrtc/web_client/
# 应该看到: index.html, client.js
```

**解决**:
```bash
# 确认文件存在
cat ~/work/src/u_webrtc/web_client/client.js | head -5
```

### 问题 2: 控制台显示"WebSocket connection to 'ws://localhost:8080' failed"

**原因**: 信令服务器未运行

**解决**:
```bash
# 检查服务器
ps aux | grep "python3 server.py"

# 重启服务器
cd ~/work/src/u_webrtc/signaling_server
python3 server.py
```

### 问题 3: 控制台显示"getUserMedia is not defined" 或 "RTCPeerConnection is not defined"

**原因**: 浏览器不支持 WebRTC 或需要 HTTPS

**解决**:
- 使用现代浏览器（Chrome 90+, Firefox 88+, Edge 90+）
- 对于 localhost，HTTP 是允许的
- 对于远程访问，需要 HTTPS

### 问题 4: 没有"连接"按钮

**检查 HTML**:
```bash
grep -i "button\|connect" ~/work/src/u_webrtc/web_client/index.html
```

**应该看到**:
```html
<button id="connectButton">连接</button>
或
<button id="connect">Connect</button>
```

### 问题 5: 点击按钮后没有反应

**在浏览器控制台测试**:
```javascript
// 手动触发连接
document.getElementById('connectButton').click();
// 或
document.getElementById('connect').click();
```

**查看是否有错误输出**

### 问题 6: 浏览器说"WebSocket is already in CLOSING or CLOSED state"

**原因**: 连接被过早关闭

**解决**: 不要重复点击连接按钮，刷新页面后再试

## 📋 完整测试清单

在测试之前，按顺序检查：

- [ ] 信令服务器运行在 8080 端口
- [ ] Web 服务器运行在 8000 端口
- [ ] 浏览器开发者工具已打开
- [ ] 浏览器控制台无错误
- [ ] 页面加载完成，显示"连接"按钮
- [ ] **已点击"连接"按钮** ← 最关键！
- [ ] 浏览器控制台显示 "WebSocket connected"
- [ ] 然后启动 ROS2 节点
- [ ] 观察浏览器是否收到 offer
- [ ] 观察浏览器是否发送 answer

## 🔍 手动测试信令流程

如果自动流程不工作，可以在浏览器控制台手动测试：

```javascript
// 1. 连接到信令服务器
const ws = new WebSocket('ws://localhost:8080');

ws.onopen = () => {
    console.log('✅ Connected to signaling server');
};

ws.onmessage = async (event) => {
    const message = JSON.parse(event.data);
    console.log('📨 Received:', message.type);
    
    if (message.type === 'offer') {
        console.log('📨 Got offer, should create answer...');
        // 这里应该创建 answer
        // 如果代码执行到这里但没有发送 answer，说明 PeerConnection 设置有问题
    }
};

ws.onerror = (error) => {
    console.error('❌ WebSocket error:', error);
};
```

## 📊 时序图

正确的流程应该是：

```
浏览器页面加载
    │
    ▼
用户点击"连接"按钮 ← 关键步骤！
    │
    ▼
创建 WebSocket 连接
    │
    ▼
创建 RTCPeerConnection
    │
    ▼
等待 Offer
    │
    ▼
收到 Offer
    │
    ▼
setRemoteDescription(offer)
    │
    ▼
createAnswer()
    │
    ▼
setLocalDescription(answer)
    │
    ▼
通过 WebSocket 发送 Answer
    │
    ▼
WebRTC 连接建立
```

## 💡 快速诊断命令

```bash
# 1. 检查所有服务是否运行
netstat -tulpn | grep -E "8080|8000"
# 应该看到两个端口都在监听

# 2. 测试信令服务器
curl http://localhost:8080
# 应该返回错误（因为是 WebSocket），但证明端口开放

# 3. 测试 Web 服务器
curl http://localhost:8000
# 应该返回 HTML 内容

# 4. 查看浏览器日志（如果使用 Chrome）
# 打开 chrome://webrtc-internals/
# 可以看到详细的 WebRTC 连接信息
```

## 🎯 如果一切都正确但仍然失败

可能需要检查 Web 客户端代码本身。请提供：

1. 浏览器控制台的完整输出（包括所有错误）
2. chrome://webrtc-internals/ 的信息（如果使用 Chrome）
3. Web 客户端的 JavaScript 代码（client.js）

---

**关键记住**：必须在浏览器中**点击"连接"按钮**后再启动 ROS2 节点！

