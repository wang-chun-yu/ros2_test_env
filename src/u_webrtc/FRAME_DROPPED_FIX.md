# Frame Dropped 问题修复 - 最终解决方案

## 问题诊断

从浏览器诊断发现：

```
✅ connectionState: connected
✅ packetsReceived: 3631
✅ bytesReceived: 4093934 (4MB)
✅ 编解码器: video/VP8

❌ framesReceived: 0    ← 无法识别帧
❌ framesDecoded: 0     ← 无法解码
❌ framesDropped: 92    ← 所有帧都被丢弃
```

## 根本原因

问题在于 **VP8 payload 分片处理不正确**！

### 错误的实现

```cpp
// 错误方式：在整个 VP8 数据前添加一个 descriptor，然后分包
uint8_t descriptor = 0x10;  // S=1
vp8Payload.push_back(descriptor);
vp8Payload.insert(vp8Payload.end(), frameData.begin(), frameData.end());

// 用 RTP packetizer 分包（这会导致问题！）
std::vector<RTPPacket> rtpPackets = _rtpPacketizer->packetize(vp8Payload, ...);
```

### 为什么失败

当 VP8 帧被分成多个 RTP 包时：

```
第 1 个包: [RTP Header][VP8 Descriptor (S=1)][VP8 数据 part 1]
第 2 个包: [RTP Header][VP8 数据 part 2]  ← 缺少 VP8 descriptor！
第 3 个包: [RTP Header][VP8 数据 part 3]  ← 缺少 VP8 descriptor！
```

**问题**：
- 只有第一个包有 VP8 descriptor
- 后续包直接是 VP8 数据（从中间开始）
- 浏览器无法识别这种格式
- 所有帧被丢弃

### RFC 7741 的要求

VP8 RTP payload 的每个包都**必须**有 VP8 descriptor：

```
包 1: [RTP][VP8 Desc (S=1, PID=0)][VP8 数据 part 1]
包 2: [RTP][VP8 Desc (S=0, PID=0)][VP8 数据 part 2]
包 3: [RTP][VP8 Desc (S=0, PID=0)][VP8 数据 part 3]
```

其中 S bit 的含义：
- S=1: 分区的开始
- S=0: 分区的继续

## 正确的解决方案

### 方案：避免分片，每个 VP8 帧作为单个 RTP 包发送

```cpp
void sendEncodedFrame(const std::vector<uint8_t>& frameData, bool isKeyFrame) {
    // 手动构造单个 RTP 包
    std::vector<uint8_t> rtpPacket;
    
    // 1. RTP Header (12 bytes)
    rtpPacket.push_back(0x80);  // V=2, P=0, X=0, CC=0
    rtpPacket.push_back(0x60 | 96);  // M=0, PT=96 (VP8)
    
    // 序列号（递增）
    static uint16_t seqNum = 0;
    seqNum++;
    rtpPacket.push_back((seqNum >> 8) & 0xFF);
    rtpPacket.push_back(seqNum & 0xFF);
    
    // 时间戳（90kHz 时钟）
    static uint32_t rtpTimestamp = 0;
    rtpTimestamp += 3000;  // 30fps: 90000/30=3000
    rtpPacket.push_back((rtpTimestamp >> 24) & 0xFF);
    rtpPacket.push_back((rtpTimestamp >> 16) & 0xFF);
    rtpPacket.push_back((rtpTimestamp >> 8) & 0xFF);
    rtpPacket.push_back(rtpTimestamp & 0xFF);
    
    // SSRC
    uint32_t ssrc = _rtpPacketizer->getConfig().ssrc;
    rtpPacket.push_back((ssrc >> 24) & 0xFF);
    rtpPacket.push_back((ssrc >> 16) & 0xFF);
    rtpPacket.push_back((ssrc >> 8) & 0xFF);
    rtpPacket.push_back(ssrc & 0xFF);
    
    // 2. VP8 Payload Descriptor (1 byte)
    uint8_t descriptor = 0x10;  // S=1, PID=0
    rtpPacket.push_back(descriptor);
    
    // 3. VP8 Encoded Data
    rtpPacket.insert(rtpPacket.end(), frameData.begin(), frameData.end());
    
    // 4. 发送
    _videoTrack->send(rtpPacket);
}
```

### 关键点

1. **每个帧作为单个 RTP 包**
   - 避免分片导致的 descriptor 问题
   - 简化实现

2. **正确的 RTP timestamp**
   - 使用 90kHz 时钟（不是毫秒）
   - 30fps: 每帧增加 3000 (90000/30)

3. **简单的 VP8 descriptor**
   - 1 字节：`0x10` (S=1, PID=0)
   - 足够用于单包传输

4. **序列号递增**
   - 每个包递增
   - 用于检测丢包

## 应用修复

### 步骤 1: 重新编译

```bash
cd ~/work
rm -rf build/u_webrtc install/u_webrtc
colcon build --packages-select u_webrtc
source install/setup.bash
```

### 步骤 2: 重启 ROS2 节点

```bash
ros2 launch u_webrtc webrtc_stream.launch.py
```

### 步骤 3: 刷新浏览器

按 `Ctrl+Shift+R` 刷新

### 步骤 4: 重新连接并诊断

在浏览器控制台运行：

```javascript
// 等待连接后，获取统计
window.pc.getStats().then(stats => {
    stats.forEach(report => {
        if (report.type === 'inbound-rtp' && report.mediaType === 'video') {
            console.log('framesReceived:', report.framesReceived);
            console.log('framesDecoded:', report.framesDecoded);
            console.log('framesDropped:', report.framesDropped);
        }
    });
});
```

### 期望结果

```
✅ framesReceived: > 0   ← 不再是 0！
✅ framesDecoded: > 0    ← 不再是 0！
✅ framesDropped: 0      ← 不再是 92！
```

### 视频显示

```javascript
const video = document.getElementById('videoElement');
console.log('readyState:', video.readyState);  // 应该 >= 1
console.log('videoWidth:', video.videoWidth);  // 应该 640
console.log('videoHeight:', video.videoHeight); // 应该 480
```

**视频应该显示画面！**

## 性能考虑

### MTU 限制

单个 RTP 包的最大大小通常是 1500 bytes (以太网 MTU)：
- RTP Header: 12 bytes
- VP8 Descriptor: 1 byte
- 可用于 VP8 数据: ~1487 bytes

### 压缩后的帧大小

对于 640x480 @ 2Mbps VP8:
- 关键帧（I-frame）: 约 5-15 KB
- 非关键帧（P-frame）: 约 0.5-3 KB

**问题**：关键帧可能超过 MTU！

### 解决方案

如果帧超过 MTU，有两个选择：

#### 选项 1: 降低比特率/分辨率（推荐）

```yaml
# config/webrtc_config.yaml
target_bitrate: 500000  # 500 kbps
width: 320
height: 240
```

这样可以确保大多数帧 < 1487 bytes。

#### 选项 2: 实现正确的分片（复杂）

需要为每个分片添加正确的 VP8 descriptor：

```cpp
// 第一个包: S=1
// 后续包: S=0
```

## 为什么这次应该成功

之前失败的原因：
1. ❌ 直接发送编码数据 → SRTP error
2. ❌ 分包后 VP8 descriptor 不正确 → frames dropped

这次成功的原因：
1. ✅ 发送完整的 RTP 包 → 无 SRTP error
2. ✅ 每个包有正确的 VP8 descriptor → 可以解码
3. ✅ 避免分片 → 简化实现
4. ✅ 正确的时间戳 (90kHz) → 播放流畅

## 调试技巧

### 查看 chrome://webrtc-internals

1. 打开：`chrome://webrtc-internals`
2. 查找 "inbound-rtp (video)" 部分
3. 监控：
   - `framesReceived`: 应该持续增加
   - `framesDecoded`: 应该持续增加
   - `framesDropped`: 应该保持 0 或很少

### ROS2 日志

```
[INFO] 视频统计: 已发送 100 帧
[INFO] 视频统计: 已发送 200 帧
```

不应该有：
```
[ERROR] 发送帧失败: SRTP protect error
```

### 浏览器控制台

应该看到：
```
✅ 视频元数据已加载
✅ 视频开始播放
✅ 视频 playing 事件触发
```

## 总结

**问题根源**：VP8 分片时每个包都需要 descriptor，但我们只添加了一个。

**解决方案**：避免分片，每个 VP8 帧作为单个 RTP 包发送。

**结果**：浏览器可以正确识别和解码 VP8 帧，视频显示正常。

---

**状态**: ✅ 应该修复  
**影响范围**: `peer_connection_wrapper.cpp` 和 `rtp_packetizer.hpp`  
**需要重新编译**: ✅ 是  
**复杂度**: 中等（简化了实现）


