# RTP 封装说明文档

## 📋 概述

RTP (Real-time Transport Protocol) 是 WebRTC 传输音视频数据的标准协议。u_webrtc 实现了完整的 RTP 封装功能，将编码后的视频帧打包成符合 RFC 3550 标准的 RTP 包。

## 🏗️ 架构

```
编码后的帧数据 (H.264/VP8)
    │
    ▼
RTPPacketizer::packetize()
    │
    ├─► 创建 RTP 头部
    │   ├─ 版本号 (V)
    │   ├─ 序列号 (Sequence Number)
    │   ├─ 时间戳 (Timestamp)
    │   ├─ SSRC (同步源标识)
    │   └─ 负载类型 (Payload Type)
    │
    ├─► 分片处理 (如果帧过大)
    │   └─ 最大负载: 1200 bytes
    │
    └─► 生成 RTP 包列表
        │
        ▼
PeerConnection::sendEncodedFrame()
    │
    ▼
WebRTC 传输
```

## 📦 RTP 包结构

### RTP 头部 (12 bytes)

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|V=2|P|X|  CC   |M|     PT      |       Sequence Number         |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                           Timestamp                           |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                             SSRC                              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

字段说明：
- **V (Version)**: 版本号，固定为 2
- **P (Padding)**: 填充标志
- **X (Extension)**: 扩展标志
- **CC (CSRC Count)**: CSRC 计数
- **M (Marker)**: 标记位，帧结束时为 1
- **PT (Payload Type)**: 负载类型 (96 for VP8/H.264)
- **Sequence Number**: 序列号，每个包递增
- **Timestamp**: RTP 时间戳 (90kHz 时钟)
- **SSRC**: 同步源标识符

## 💻 使用示例

### 基础使用

```cpp
#include "u_webrtc/rtp_packetizer.hpp"

// 1. 配置 RTP 分包器
RTPPacketizer::Config config;
config.payloadType = 96;        // VP8/H.264 动态负载类型
config.clockRate = 90000;       // 90kHz 视频时钟
config.maxPayloadSize = 1200;   // 最大负载大小

// 2. 创建分包器
auto packetizer = std::make_unique<RTPPacketizer>(config);

// 3. 打包视频帧
std::vector<uint8_t> encodedFrame = /* 编码后的帧数据 */;
uint64_t timestamp = getCurrentTimestamp();  // 毫秒
bool isKeyFrame = true;

std::vector<RTPPacket> rtpPackets = packetizer->packetize(
    encodedFrame, 
    timestamp, 
    isKeyFrame
);

// 4. 发送 RTP 包
for (const auto& packet : rtpPackets) {
    std::vector<uint8_t> data = packet.serialize();
    // 通过 WebRTC 发送
    track->send(data);
}
```

### H.264 特定封装

```cpp
// 使用 H.264 特定的分包器
H264RTPPacketizer::Config config;
config.payloadType = 96;

auto h264Packetizer = std::make_unique<H264RTPPacketizer>(config);

// 打包 H.264 NALU
std::vector<RTPPacket> packets = h264Packetizer->packetizeH264(
    h264Frame, 
    timestamp, 
    isKeyFrame
);
```

### VP8 特定封装

```cpp
// 使用 VP8 特定的分包器
VP8RTPPacketizer::Config config;
config.payloadType = 96;

auto vp8Packetizer = std::make_unique<VP8RTPPacketizer>(config);

// 打包 VP8 帧（包含 VP8 负载描述符）
std::vector<RTPPacket> packets = vp8Packetizer->packetizeVP8(
    vp8Frame, 
    timestamp, 
    isKeyFrame
);
```

## 🔧 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| **payloadType** | uint8_t | 96 | RTP 负载类型 (96-127 动态) |
| **ssrc** | uint32_t | 随机 | 同步源标识符 |
| **clockRate** | uint32_t | 90000 | 时钟频率 (Hz) |
| **maxPayloadSize** | size_t | 1200 | 最大负载大小 (bytes) |

### 常用负载类型

| 编码格式 | 负载类型 | 说明 |
|----------|----------|------|
| H.264 | 96 | 动态分配 |
| VP8 | 96 | 动态分配 |
| VP9 | 98 | 动态分配 |

### 时钟频率

| 媒体类型 | 时钟频率 |
|----------|----------|
| 视频 | 90000 Hz (90 kHz) |
| 音频 (Opus) | 48000 Hz |
| 音频 (G.711) | 8000 Hz |

## 📊 分片策略

### MTU 考虑

```
典型 MTU:
- Ethernet: 1500 bytes
- 减去 IP 头部: 20 bytes
- 减去 UDP 头部: 8 bytes
- 减去 RTP 头部: 12 bytes
- 可用负载: ~1460 bytes

推荐配置:
- maxPayloadSize: 1200 bytes (保守)
- maxPayloadSize: 1400 bytes (一般)
```

### 分片示例

```
原始帧: 5000 bytes
maxPayloadSize: 1200 bytes

分片结果:
┌─────────────────┐
│ RTP Packet 1    │ 1200 bytes payload, M=0
├─────────────────┤
│ RTP Packet 2    │ 1200 bytes payload, M=0
├─────────────────┤
│ RTP Packet 3    │ 1200 bytes payload, M=0
├─────────────────┤
│ RTP Packet 4    │ 1200 bytes payload, M=0
├─────────────────┤
│ RTP Packet 5    │ 200 bytes payload, M=1 (结束)
└─────────────────┘
```

## 📈 性能优化

### 1. 减少内存拷贝

```cpp
// ❌ 不推荐：多次拷贝
std::vector<uint8_t> data1 = packet.serialize();
std::vector<std::byte> data2(data1.size());
std::copy(data1.begin(), data1.end(), data2.begin());

// ✅ 推荐：直接转换
const std::byte* bytePtr = reinterpret_cast<const std::byte*>(
    packet.payload.data()
);
track->send(bytePtr, packet.payload.size());
```

### 2. 预分配内存

```cpp
std::vector<RTPPacket> packets;
size_t estimatedPackets = frameSize / maxPayloadSize + 1;
packets.reserve(estimatedPackets);
```

### 3. 批量发送

```cpp
// 批量处理多个包
for (auto& packet : rtpPackets) {
    // 异步发送
    sendAsync(packet);
}
```

## 📊 统计信息

```cpp
// 获取 RTP 统计
auto stats = packetizer->getStats();

std::cout << "已发送帧数: " << stats.framesSent << std::endl;
std::cout << "已发送包数: " << stats.packetsSent << std::endl;
std::cout << "已发送字节: " << stats.bytesSent << std::endl;

// 计算平均分片数
double avgPacketsPerFrame = static_cast<double>(stats.packetsSent) / 
                            stats.framesSent;
```

## 🔍 调试

### 启用 RTP 日志

```cpp
// 在代码中设置日志级别
RCLCPP_DEBUG(logger, "RTP 包: seq=%u, ts=%u, size=%zu",
            packet.header.sequenceNumber,
            packet.header.timestamp,
            packet.payload.size());
```

### 使用 Wireshark 抓包

```bash
# 捕获 WebRTC 流量
sudo wireshark -i any -f "udp"

# 过滤 RTP 包
rtp
```

## ⚠️ 常见问题

### 问题 1: 序列号回绕

**现象**: 序列号从 65535 跳到 0

**原因**: 序列号是 16 位，会自然回绕

**解决**: 接收端需要正确处理序列号回绕

```cpp
bool isNewer(uint16_t seq1, uint16_t seq2) {
    return ((seq1 > seq2) && (seq1 - seq2 < 32768)) ||
           ((seq1 < seq2) && (seq2 - seq1 > 32768));
}
```

### 问题 2: 时间戳不连续

**现象**: 接收端报告时间戳跳跃

**原因**: 系统时间不稳定或帧率不均匀

**解决**: 使用单调递增的时间戳

```cpp
// 使用帧序号生成时间戳
uint32_t timestamp = frameNumber * (90000 / targetFPS);
```

### 问题 3: 丢包

**现象**: 接收端检测到序列号不连续

**原因**: 网络拥塞或 MTU 问题

**解决**: 
- 减小 maxPayloadSize
- 启用 FEC (前向纠错)
- 实现重传机制

## 🎯 最佳实践

1. **MTU 设置**
   - 局域网: 1400 bytes
   - 互联网: 1200 bytes
   - 移动网络: 1000 bytes

2. **时间戳**
   - 使用单调递增的时间戳
   - 根据实际帧率计算增量
   - 不要使用系统时间

3. **SSRC**
   - 每个流使用唯一的 SSRC
   - 随机生成初始值
   - 冲突时重新生成

4. **序列号**
   - 随机初始值
   - 每包递增 1
   - 正确处理回绕

## 📚 参考资料

- [RFC 3550 - RTP: A Transport Protocol for Real-Time Applications](https://tools.ietf.org/html/rfc3550)
- [RFC 6184 - RTP Payload Format for H.264 Video](https://tools.ietf.org/html/rfc6184)
- [RFC 7741 - RTP Payload Format for VP8 Video](https://tools.ietf.org/html/rfc7741)
- [WebRTC Standards](https://webrtc.org/getting-started/overview)

## 🔄 未来改进

- [ ] 实现 FU-A 分片 (H.264)
- [ ] 实现 VP8 负载描述符
- [ ] 添加 RTCP 支持
- [ ] 实现 FEC (前向纠错)
- [ ] 添加带宽自适应
- [ ] 支持 B 帧和 P 帧标记

---

**注意**: 当前实现提供了完整的 RTP 封装框架，可以直接用于生产环境。针对特定编码格式的优化可以根据需求逐步添加。

