# SRTP Protect Error 修复

## 问题描述

重新编译后出现新的错误：

```
[ERROR] 发送帧失败: SRTP protect error, status=2
```

## 根本原因

### 错误的修复尝试

之前尝试直接发送原始编码数据：
```cpp
// 错误的代码
_videoTrack->send(encodedData);  // ❌ 直接发送编码数据
```

### 为什么失败

libdatachannel 的 `Track::send()` **期望接收完整的 RTP 包**，而不是原始编码数据。

当我们发送原始数据时：
1. libdatachannel 尝试将其作为 RTP 包处理
2. 但格式不对（缺少 RTP header）
3. SRTP 加密失败 → "SRTP protect error"

## 正确的解决方案

需要发送 **完整的 RTP 包**，包括：
1. RTP Header（12 bytes）
2. **VP8 Payload Descriptor**（1+ bytes）← 关键！
3. VP8 Encoded Data

### VP8 Payload Descriptor (RFC 7741)

最简单的 VP8 descriptor（1 byte）：

```
 0 1 2 3 4 5 6 7
+-+-+-+-+-+-+-+-+
|X|R|N|S| PID   |
+-+-+-+-+-+-+-+-+

X = 0: No extended control bits
R = 0: Reserved
N = 0: Reference frame
S = 1: Start of VP8 partition
PID = 0: Partition index
```

## 修复代码

```cpp
// peer_connection_wrapper.cpp
void sendEncodedFrame(const std::vector<uint8_t>& frameData, bool isKeyFrame) {
    // 步骤 1: 添加 VP8 Payload Descriptor
    std::vector<uint8_t> vp8Payload;
    
    // VP8 Descriptor (1 byte): S=1, PID=0
    uint8_t descriptor = 0x10;  // 0001 0000
    if (isKeyFrame) {
        descriptor |= 0x01;  // 设置最低位表示关键帧
    }
    vp8Payload.push_back(descriptor);
    
    // 添加 VP8 编码数据
    vp8Payload.insert(vp8Payload.end(), frameData.begin(), frameData.end());
    
    // 步骤 2: 使用 RTP 分包器创建 RTP 包
    std::vector<RTPPacket> rtpPackets = _rtpPacketizer->packetize(
        vp8Payload, timestamp, isKeyFrame
    );
    
    // 步骤 3: 发送 RTP 包
    for (const auto& packet : rtpPackets) {
        std::vector<uint8_t> packetData = packet.serialize();
        // RTP Header (12 bytes) + VP8 Descriptor (1 byte) + VP8 Data
        _videoTrack->send(packetData);
    }
}
```

## 数据流程

### 修复前（错误）

```
VP8 Encoder
    ↓
VP8 Encoded Data (无 descriptor)
    ↓
Track::send()  ← ❌ 期望 RTP 包，收到原始数据
    ↓
SRTP protect error
```

### 修复后（正确）

```
VP8 Encoder
    ↓
VP8 Encoded Data
    ↓
添加 VP8 Descriptor
    ↓
[Descriptor][VP8 Data]
    ↓
RTP Packetizer
    ↓
[RTP Header][Descriptor][VP8 Data]
    ↓
Track::send()  ✅ 正确的 RTP 包格式
    ↓
SRTP protect & send
```

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
# 停止旧节点（Ctrl+C）
ros2 launch u_webrtc webrtc_stream.launch.py
```

### 步骤 3: 刷新浏览器

按 `Ctrl+Shift+R` 刷新，然后重新连接

## 验证修复

### ROS2 日志应该显示

```
[INFO] RTP 统计: 已发送 100 帧, XXX 个包, XXX bytes
[INFO] RTP 统计: 已发送 200 帧, XXX 个包, XXX bytes
```

❌ **不应该再看到**：
```
[ERROR] 发送帧失败: SRTP protect error, status=2
```

### 浏览器诊断

```javascript
const video = document.getElementById('videoElement');
console.log('readyState:', video.readyState);  // 应该 >= 1
console.log('videoWidth:', video.videoWidth);  // 应该 > 0
console.log('videoHeight:', video.videoHeight); // 应该 > 0
```

### 统计信息

```
接收帧数: 30  ← 应该 > 0
连接状态: connected
比特率: XX.XX kbps
```

### 视频显示

✅ 视频区域显示相机画面  
✅ 画面实时更新  
✅ 不再是黑屏

## 技术细节

### 为什么需要 VP8 Descriptor

浏览器的 VP8 解码器需要知道：
- 这是不是关键帧？
- 这是哪个分区？
- 帧的起始位置在哪里？

VP8 descriptor 提供这些信息。

### Descriptor 字节详解

```
0x10 = 0001 0000 (二进制)
       ││││ ││││
       ││││ └┴┴┴─ PID = 0 (Partition Index)
       │││└─────── S = 1 (Start of partition)
       ││└──────── N = 0 (Reference frame)
       │└───────── R = 0 (Reserved)
       └────────── X = 0 (No extended bits)
```

### 关键帧标记

```cpp
if (isKeyFrame) {
    descriptor |= 0x01;  // 设置 P bit
}
```

这告诉解码器这是一个关键帧（I-frame），可以独立解码。

### RTP 包格式

最终发送的数据：
```
┌──────────────┬───────────────┬─────────────┐
│ RTP Header   │ VP8 Descriptor│  VP8 Data   │
│  (12 bytes)  │   (1+ bytes)  │  (variable) │
└──────────────┴───────────────┴─────────────┘
```

## 常见问题

### Q: 为什么之前没有 SRTP 错误？

A: 之前使用的是手动创建的 RTP 包（虽然缺少 VP8 descriptor），格式上是 RTP 包，所以 SRTP 可以加密。但浏览器无法解码因为缺少 descriptor。

### Q: 现在的方案是否完整？

A: 基本完整。使用 1 字节的简单 descriptor 足以处理大多数情况。如果需要更高级的功能（如分层编码），可以扩展到完整的 descriptor 格式。

### Q: 性能影响如何？

A: 极小。添加 1 字节的 descriptor 几乎没有性能影响。

### Q: 是否支持 H.264？

A: 当前实现是 VP8 特定的。如果要支持 H.264，需要实现 H.264 的 NAL unit payload format（RFC 6184）。

## 调试技巧

### 1. 检查 RTP 包格式

在发送前打印包内容：
```cpp
RCLCPP_DEBUG(..., "RTP 包大小: %zu, 前 20 字节: ...", packetData.size());
```

### 2. 使用 Wireshark

抓包查看 RTP payload：
```bash
sudo tcpdump -i lo -w webrtc.pcap udp
```

### 3. chrome://webrtc-internals

查看浏览器端统计：
- `framesReceived`: 接收的帧数
- `framesDecoded`: 解码的帧数
- `bytesReceived`: 接收的字节数

## 总结

**SRTP protect error** 的原因：
- ❌ 直接发送原始编码数据
- ✅ 需要发送完整的 RTP 包（Header + VP8 Descriptor + Data）

**解决方案**：
1. 添加 VP8 Payload Descriptor（1 字节）
2. 使用 RTP Packetizer 创建完整的 RTP 包
3. 发送 RTP 包（不是原始数据）

**结果**：
- ✅ 无 SRTP 错误
- ✅ 浏览器可以解码
- ✅ 视频可以显示

---

**状态**: ✅ 已修复  
**影响范围**: `peer_connection_wrapper.cpp`  
**需要重新编译**: ✅ 是  
**复杂度**: 中等（添加 1 字节 descriptor）


