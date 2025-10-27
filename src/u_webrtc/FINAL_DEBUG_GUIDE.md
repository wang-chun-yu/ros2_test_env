# 最终调试指南 - 视频无法显示

## 当前状态

从诊断结果：
```
readyState: 0 HAVE_NOTHING
videoWidth: 0
videoHeight: 0
轨道状态: live
PeerConnection 未创建  ← 这是问题！
```

## 立即修复

### 修复 1: 暴露 PeerConnection 到全局

已修复 `client.js`，添加了 `window.pc = pc;`

### 步骤 1: 刷新浏览器

**重要**：必须硬刷新浏览器以加载新的 JavaScript！

```
按 Ctrl+Shift+R (Windows/Linux)
或 Cmd+Shift+R (Mac)
```

### 步骤 2: 清除控制台并重新连接

1. 按 F12 打开开发者工具
2. 在 Console 标签中点击"清除"按钮（垃圾桶图标）
3. 输入服务器地址：`ws://localhost:8080`
4. 点击"连接"按钮

### 步骤 3: 查看完整的控制台日志

**必须按顺序看到以下日志**：

```
✅ WebSocket 已连接
✅ PeerConnection 已创建          ← 关键！
✅ 收到信令消息: candidate
✅ 收到信令消息: candidate
✅ 收到信令消息: offer
✅ 接收到媒体流, 轨道数: 1
✅ 视频元数据已加载              ← 关键！
✅ 视频开始播放                  ← 关键！
✅ 视频 play 事件触发
✅ 视频 playing 事件触发          ← 关键！
✅ 已发送 Answer
✅ 连接状态: connecting
✅ 连接状态: connected
```

### 步骤 4: 重新运行诊断

刷新后，在控制台中运行：

```javascript
const video = document.getElementById('videoElement');
const pc = window.pc;

console.log('=== 完整诊断 ===');

console.log('1. PeerConnection 状态:');
if (pc) {
    console.log('   存在: 是');
    console.log('   connectionState:', pc.connectionState);
    console.log('   iceConnectionState:', pc.iceConnectionState);
    console.log('   signalingState:', pc.signalingState);
} else {
    console.log('   存在: 否 ← 还是有问题！');
}

console.log('2. 视频元素状态:');
console.log('   readyState:', video.readyState, ['HAVE_NOTHING', 'HAVE_METADATA', 'HAVE_CURRENT_DATA', 'HAVE_FUTURE_DATA', 'HAVE_ENOUGH_DATA'][video.readyState]);
console.log('   videoWidth:', video.videoWidth);
console.log('   videoHeight:', video.videoHeight);
console.log('   paused:', video.paused);

console.log('3. 媒体流:');
if (video.srcObject) {
    const tracks = video.srcObject.getTracks();
    console.log('   轨道数:', tracks.length);
    tracks.forEach(t => {
        console.log('    ', t.kind, t.enabled, t.readyState, t.label);
    });
} else {
    console.log('   无媒体流');
}

console.log('4. 远端 SDP:');
if (pc && pc.remoteDescription) {
    console.log(pc.remoteDescription.sdp);
} else {
    console.log('   无');
}
```

## 问题诊断树

### 情况 1: PeerConnection 仍然未创建

**症状**：刷新后诊断仍然显示 "PeerConnection 未创建"

**原因**：浏览器缓存了旧的 JavaScript

**解决**：
1. 清除浏览器缓存（Ctrl+Shift+Delete）
2. 或者在开发者工具的 Network 标签中勾选 "Disable cache"
3. 关闭浏览器标签页，重新打开
4. 使用隐私/无痕模式测试

### 情况 2: PeerConnection 存在但 readyState 仍然是 0

**症状**：
```
PeerConnection 存在
readyState: 0 HAVE_NOTHING
videoWidth: 0
```

**可能原因**：

#### A. 浏览器不支持 VP8 编解码器

**检查**：
```javascript
RTCRtpReceiver.getCapabilities('video').codecs.forEach(codec => {
    console.log(codec.mimeType);
});
```

**应该看到**: `video/VP8`

**如果没有**: 您的浏览器不支持 VP8，需要改用 H.264

#### B. RTP Payload 格式仍然不正确

**检查 ROS2 日志**：
```
[INFO] RTP 统计: 已发送 100 帧...
```

**不应该看到**：
```
[ERROR] 发送帧失败: SRTP protect error
```

**如果还有错误**: RTP 格式问题未解决

#### C. SDP 中编解码器不匹配

**检查**：在浏览器控制台运行：
```javascript
console.log(window.pc.remoteDescription.sdp);
```

**查找**：
```
m=video ...
a=rtpmap:96 VP8/90000
```

**应该有**: `VP8/90000`

**如果没有**: SDP 协商失败

### 情况 3: 收不到 "视频元数据已加载" 日志

**症状**：连接成功但从未看到 "视频元数据已加载"

**原因**：`onloadedmetadata` 事件未触发

**可能的问题**：
1. 视频轨道格式不对
2. 浏览器无法解码数据
3. 没有关键帧

**检查 chrome://webrtc-internals**：

1. 在 Chrome 中打开：`chrome://webrtc-internals`
2. 找到 "getUserMedia" 或 "RTCPeerConnection" 部分
3. 查看统计：
   - `framesReceived`: 应该 > 0
   - `framesDecoded`: 应该 > 0 ← 关键！
   - `framesDropped`: 应该很少
   - `bytesReceived`: 应该持续增加

如果 `framesReceived > 0` 但 `framesDecoded = 0`：
→ **浏览器收到数据但无法解码**
→ **VP8 Payload Descriptor 格式可能还是不对**

## 高级调试：验证 VP8 Descriptor

如果上面所有检查都通过，但仍无法解码，需要验证 VP8 descriptor：

### 在 ROS2 端添加调试日志

修改 `peer_connection_wrapper.cpp`：

```cpp
void sendEncodedFrame(...) {
    // ... VP8 descriptor 代码 ...
    
    // 调试：打印前几个字节
    if (_frameSent < 5) {  // 只打印前 5 帧
        std::string hex;
        for (size_t i = 0; i < std::min(size_t(20), vp8Payload.size()); ++i) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", vp8Payload[i]);
            hex += buf;
        }
        RCLCPP_INFO(rclcpp::get_logger("PeerConnection"),
                   "帧 %lu VP8 Payload 前 20 字节: %s", 
                   _frameSent, hex.c_str());
    }
    
    // ... RTP packetizer 代码 ...
}
```

**应该看到**：
```
帧 0 VP8 Payload 前 20 字节: 10 XX XX XX ...
                              ↑
                          VP8 descriptor (0x10)
```

### 使用 Wireshark 抓包

```bash
# 捕获 localhost 流量
sudo tcpdump -i lo -w webrtc.pcap port 8080 or udp

# 在 Wireshark 中打开 webrtc.pcap
# 过滤: rtp
# 查看 RTP payload 的前几个字节
```

## 备用方案：切换到原始媒体流（测试）

如果所有方法都失败，可以测试浏览器是否能处理任何视频：

在 `client.js` 中临时添加测试代码：

```javascript
// 在 createPeerConnection() 之后添加
function testLocalVideo() {
    navigator.mediaDevices.getUserMedia({video: true})
        .then(stream => {
            videoElement.srcObject = stream;
            videoElement.play();
            console.log('本地摄像头测试成功');
        })
        .catch(err => {
            console.error('本地摄像头测试失败:', err);
        });
}

// 在连接按钮旁边添加测试按钮调用 testLocalVideo()
```

如果本地摄像头能显示：
→ **浏览器视频功能正常**
→ **问题在 WebRTC 数据传输/格式**

如果本地摄像头也不能显示：
→ **浏览器配置问题**
→ **硬件加速问题**

## 最后的检查清单

在请求进一步帮助前，请确认：

- [ ] 已硬刷新浏览器（Ctrl+Shift+R）
- [ ] 已清除浏览器缓存
- [ ] 控制台显示 "PeerConnection 已创建"
- [ ] 控制台显示 "接收到媒体流, 轨道数: 1"
- [ ] ROS2 日志无 "SRTP protect error"
- [ ] ROS2 日志显示 "RTP 统计: 已发送 XX 帧"
- [ ] 浏览器支持 VP8（运行编解码器检查）
- [ ] chrome://webrtc-internals 显示 `framesReceived > 0`
- [ ] 远端 SDP 包含 `a=rtpmap:96 VP8/90000`

## 提供诊断信息

如果问题仍然存在，请提供：

1. **浏览器类型和版本**
   - Chrome? Firefox? Edge? Safari?
   - 版本号？

2. **完整的控制台日志**（从"连接"到现在）

3. **运行诊断脚本的完整输出**

4. **ROS2 节点日志的最后 50 行**

5. **chrome://webrtc-internals 的截图**
   - 特别是 `framesReceived` 和 `framesDecoded`

6. **远端 SDP 内容**（运行 `console.log(window.pc.remoteDescription.sdp)`）

---

**下一步**：请刷新浏览器，重新连接，然后告诉我：
1. 控制台是否显示 "PeerConnection 已创建"？
2. 控制台是否显示 "视频元数据已加载"？
3. 运行完整诊断脚本的输出是什么？


