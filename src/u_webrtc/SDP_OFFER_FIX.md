# SDP Offer 生成问题修复

## 🔍 问题诊断

### 浏览器错误
```javascript
Failed to execute 'setRemoteDescription' on 'RTCPeerConnection': 
Failed to parse SessionDescription. Expect line: v=
```

### 根本原因

在 `peer_connection_wrapper.cpp` 的 `createOffer()` 函数中：

```cpp
// 旧代码（有问题）
std::string createOffer() {
    // libdatachannel 会自动创建 offer  ← 错误的假设！
    // 等待 onLocalDescription 回调
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return _localSdp;  // _localSdp 是空字符串！
}
```

**问题**：
1. libdatachannel 不会自动创建 offer
2. 需要主动调用 `setLocalDescription()` 来触发 SDP 生成
3. `_localSdp` 一直是空字符串
4. 发送给浏览器的是空的 SDP
5. 浏览器解析失败

## ✅ 修复方案

### 修改 1: 主动创建 Offer

```cpp
// 新代码（已修复）
std::string createOffer() {
    // 主动创建 offer
    _pc->setLocalDescription();  // ← 关键修复！
    
    // 等待 onLocalDescription 回调被触发
    int maxWait = 50; // 最多等待 500ms
    for (int i = 0; i < maxWait && _localSdp.empty(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    if (_localSdp.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("PeerConnection"), 
                    "创建 Offer 失败：超时");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("PeerConnection"), 
                   "Offer 创建成功，SDP 长度: %zu", _localSdp.length());
    }
    
    return _localSdp;
}
```

### 修改 2: 使用 VP8 编解码器

```cpp
// 旧代码
video.addH264Codec(96);  // ← 与 VP8Encoder 不匹配！

// 新代码
video.addVP8Codec(96);   // ← 与 VP8Encoder 匹配
```

## 🚀 应用修复

### 步骤 1: 代码已自动更新

修改已应用到：
```
src/u_webrtc/src/peer_connection_wrapper.cpp
```

### 步骤 2: 重新编译

```bash
cd ~/work
rm -rf build/u_webrtc install/u_webrtc
colcon build --packages-select u_webrtc
source install/setup.bash
```

### 步骤 3: 测试

```bash
# 终端 1: 信令服务器
cd ~/work/src/u_webrtc/signaling_server
python3 server.py

# 终端 2: 浏览器
cd ~/work/src/u_webrtc/web_client
python3 -m http.server 8000
# 打开浏览器 http://localhost:8000
# 按 F12，点击"连接"

# 终端 3: ROS2 节点（等浏览器连接后）
cd ~/work
source install/setup.bash
ros2 launch u_webrtc webrtc_stream.launch.py
```

## ✅ 验证成功标志

### ROS2 节点日志
```
✅ [INFO] [PeerConnection]: Offer 创建成功，SDP 长度: XXX
✅ [INFO] [SignalingClient]: 已发送 Offer
✅ [INFO] [WebRTCStreamer]: 收到 Answer
✅ [INFO] [PeerConnection]: Video track opened
✅ [INFO] [WebRTCStreamer]: 连接状态变化: connected
```

### 信令服务器日志
```
✅ 新客户端连接: (浏览器)
✅ 新客户端连接: (ROS2)
✅ 收到 offer
✅ 消息已转发给 1 个客户端
✅ 收到 answer  ← 关键！
✅ 消息已转发给 1 个客户端
```

### 浏览器控制台
```
✅ WebSocket connected
✅ Received offer (with valid SDP)
✅ Created answer
✅ Sent answer
✅ ICE gathering...
✅ WebRTC connected
✅ Video track received
```

### 不应该再出现的错误
```
❌ Failed to parse SessionDescription. Expect line: v=  ← 修复了！
❌ SDP is empty
❌ Invalid SDP format
```

## 🔍 SDP 格式说明

有效的 SDP 应该类似：
```
v=0
o=- 123456789 2 IN IP4 127.0.0.1
s=-
t=0 0
a=group:BUNDLE video
a=msid-semantic: WMS stream
m=video 9 UDP/TLS/RTP/SAVPF 96
c=IN IP4 0.0.0.0
a=rtcp:9 IN IP4 0.0.0.0
a=ice-ufrag:xxxx
a=ice-pwd:xxxx
a=fingerprint:sha-256 ...
a=setup:actpass
a=mid:video
a=sendonly
a=rtcp-mux
a=rtpmap:96 VP8/90000
```

**关键特征**：
- 第一行必须是 `v=0`（版本）
- 包含媒体描述 `m=video`
- 包含编解码器信息 `a=rtpmap:96 VP8/90000`
- 包含 ICE 参数

## 🐛 如果仍然失败

### 检查 1: Offer 是否生成
```bash
# 启动 ROS2 节点，查看日志
# 应该看到:
[INFO] [PeerConnection]: Offer 创建成功，SDP 长度: XXX
```

如果看到 "创建 Offer 失败：超时"：
- libdatachannel 可能未正确初始化
- 检查是否安装了 libdatachannel

### 检查 2: SDP 内容
在浏览器控制台运行：
```javascript
// 当收到 offer 时，打印 SDP
ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    if (msg.type === 'offer') {
        console.log('Received SDP:', msg.sdp);
        // 应该看到完整的 SDP 文本
    }
};
```

### 检查 3: libdatachannel 版本
```bash
pkg-config --modversion libdatachannel
```

如果版本太旧，可能不支持某些功能。推荐版本：≥ 0.18.0

### 检查 4: 编译日志
```bash
cd ~/work
colcon build --packages-select u_webrtc --event-handlers console_direct+
```

确保没有警告或错误。

## 📊 工作流程

修复后的完整流程：

```
ROS2 节点启动
    │
    ▼
PeerConnection 初始化
    │
    ▼
createOffer() 调用
    │
    ▼
_pc->setLocalDescription() ← 关键！
    │
    ▼
onLocalDescription 回调触发
    │
    ▼
_localSdp 被设置（包含有效 SDP）
    │
    ▼
返回 _localSdp
    │
    ▼
通过 WebSocket 发送给浏览器
    │
    ▼
浏览器 setRemoteDescription(sdp) ← 成功！
    │
    ▼
浏览器创建并发送 Answer
    │
    ▼
WebRTC 连接建立 ✅
```

## 📚 相关文档

- **DOCKER_FIX.md** - WebSocket 客户端修复
- **SIGNALING_SERVER_FIX.md** - 信令服务器修复
- **BROWSER_DEBUG.md** - 浏览器调试指南
- **WEBSOCKET_CLIENT.md** - WebSocket 使用说明

## 🎉 总结

### 修复的问题
1. ✅ SDP Offer 生成问题
2. ✅ 编解码器类型匹配（VP8）

### 下一步
1. 重新编译
2. 测试完整流程
3. 验证视频流传输

---

**这是最后一个核心问题！修复后系统应该可以完全工作。** 🚀

