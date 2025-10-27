#include "u_webrtc/rtp_packetizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <random>
#include <chrono>

namespace u_webrtc {

// ============================================================================
// RTPHeader 实现
// ============================================================================

std::vector<uint8_t> RTPHeader::serialize() const {
    std::vector<uint8_t> data(12); // RTP 头部固定 12 字节
    
    // Byte 0: V(2), P(1), X(1), CC(4)
    data[0] = (version << 6) | (padding << 5) | (extension << 4) | csrcCount;
    
    // Byte 1: M(1), PT(7)
    data[1] = (marker << 7) | payloadType;
    
    // Byte 2-3: 序列号（大端序）
    data[2] = (sequenceNumber >> 8) & 0xFF;
    data[3] = sequenceNumber & 0xFF;
    
    // Byte 4-7: 时间戳（大端序）
    data[4] = (timestamp >> 24) & 0xFF;
    data[5] = (timestamp >> 16) & 0xFF;
    data[6] = (timestamp >> 8) & 0xFF;
    data[7] = timestamp & 0xFF;
    
    // Byte 8-11: SSRC（大端序）
    data[8] = (ssrc >> 24) & 0xFF;
    data[9] = (ssrc >> 16) & 0xFF;
    data[10] = (ssrc >> 8) & 0xFF;
    data[11] = ssrc & 0xFF;
    
    return data;
}

std::optional<RTPHeader> RTPHeader::deserialize(const uint8_t* data, size_t size) {
    if (size < 12) {
        return std::nullopt;
    }
    
    RTPHeader header;
    
    // Byte 0
    header.version = (data[0] >> 6) & 0x03;
    header.padding = (data[0] >> 5) & 0x01;
    header.extension = (data[0] >> 4) & 0x01;
    header.csrcCount = data[0] & 0x0F;
    
    // Byte 1
    header.marker = (data[1] >> 7) & 0x01;
    header.payloadType = data[1] & 0x7F;
    
    // Byte 2-3
    header.sequenceNumber = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    
    // Byte 4-7
    header.timestamp = (static_cast<uint32_t>(data[4]) << 24) |
                       (static_cast<uint32_t>(data[5]) << 16) |
                       (static_cast<uint32_t>(data[6]) << 8) |
                       data[7];
    
    // Byte 8-11
    header.ssrc = (static_cast<uint32_t>(data[8]) << 24) |
                  (static_cast<uint32_t>(data[9]) << 16) |
                  (static_cast<uint32_t>(data[10]) << 8) |
                  data[11];
    
    return header;
}

// ============================================================================
// RTPPacket 实现
// ============================================================================

std::vector<uint8_t> RTPPacket::serialize() const {
    std::vector<uint8_t> headerData = header.serialize();
    std::vector<uint8_t> packetData;
    packetData.reserve(headerData.size() + payload.size());
    
    // 添加头部
    packetData.insert(packetData.end(), headerData.begin(), headerData.end());
    
    // 添加负载
    packetData.insert(packetData.end(), payload.begin(), payload.end());
    
    return packetData;
}

// ============================================================================
// RTPPacketizer 实现
// ============================================================================

RTPPacketizer::RTPPacketizer(const Config& config)
    : _config(config) {
    
    // 如果没有提供 SSRC，随机生成一个
    if (_config.ssrc == 0) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<uint32_t> dis(1, UINT32_MAX);
        _config.ssrc = dis(gen);
    }
    
    // 随机初始序列号
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint16_t> seqDis(0, UINT16_MAX);
    _sequenceNumber = seqDis(gen);
    
    RCLCPP_INFO(rclcpp::get_logger("RTPPacketizer"),
               "RTP 分包器初始化: SSRC=0x%08X, PayloadType=%d",
               _config.ssrc, _config.payloadType);
}

std::vector<RTPPacket> RTPPacketizer::packetize(
    const std::vector<uint8_t>& frameData,
    uint64_t timestamp,
    bool isKeyFrame) {
    
    std::vector<RTPPacket> packets;
    
    if (frameData.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RTPPacketizer"), "空帧数据");
        return packets;
    }
    
    uint32_t rtpTimestamp = convertToRTPTimestamp(timestamp);
    
    // 如果帧小于最大负载大小，发送单个包
    if (frameData.size() <= _config.maxPayloadSize) {
        RTPPacket packet = createPacket(
            frameData.data(),
            frameData.size(),
            rtpTimestamp,
            true  // marker = true (帧结束)
        );
        packets.push_back(packet);
    } else {
        // 分片发送
        size_t offset = 0;
        while (offset < frameData.size()) {
            size_t chunkSize = std::min(_config.maxPayloadSize, 
                                       frameData.size() - offset);
            bool isLast = (offset + chunkSize >= frameData.size());
            
            RTPPacket packet = createPacket(
                frameData.data() + offset,
                chunkSize,
                rtpTimestamp,
                isLast  // 只在最后一个分片设置 marker
            );
            
            packets.push_back(packet);
            offset += chunkSize;
        }
    }
    
    // 更新统计
    _stats.framesSent++;
    _stats.packetsSent += packets.size();
    for (const auto& packet : packets) {
        _stats.bytesSent += packet.payload.size() + 12;  // payload + header
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("RTPPacketizer"),
                "打包帧: %zu bytes -> %zu 个 RTP 包, 关键帧: %s",
                frameData.size(), packets.size(),
                isKeyFrame ? "是" : "否");
    
    // 避免未使用参数警告
    (void)isKeyFrame;
    
    return packets;
}

void RTPPacketizer::reset() {
    _sequenceNumber = 0;
    _lastTimestamp = 0;
    _stats = Stats();
    RCLCPP_INFO(rclcpp::get_logger("RTPPacketizer"), "RTP 分包器已重置");
}

RTPPacket RTPPacketizer::createPacket(
    const uint8_t* data,
    size_t size,
    uint32_t timestamp,
    bool marker) {
    
    RTPPacket packet;
    
    // 填充 RTP 头部
    packet.header.version = 2;
    packet.header.padding = 0;
    packet.header.extension = 0;
    packet.header.csrcCount = 0;
    packet.header.marker = marker ? 1 : 0;
    packet.header.payloadType = _config.payloadType;
    packet.header.sequenceNumber = _sequenceNumber++;
    packet.header.timestamp = timestamp;
    packet.header.ssrc = _config.ssrc;
    
    // 复制负载数据
    packet.payload.assign(data, data + size);
    
    _lastTimestamp = timestamp;
    
    return packet;
}

uint32_t RTPPacketizer::convertToRTPTimestamp(uint64_t milliseconds) {
    // RTP 时间戳 = 毫秒 * (时钟频率 / 1000)
    // 对于视频通常是 90kHz，所以乘以 90
    return static_cast<uint32_t>(milliseconds * (_config.clockRate / 1000));
}

// ============================================================================
// H264RTPPacketizer 实现
// ============================================================================

H264RTPPacketizer::H264RTPPacketizer(const Config& config)
    : RTPPacketizer(config) {
    RCLCPP_INFO(rclcpp::get_logger("H264RTPPacketizer"), 
               "H.264 RTP 分包器初始化");
}

std::vector<RTPPacket> H264RTPPacketizer::packetizeH264(
    const std::vector<uint8_t>& frameData,
    uint64_t timestamp,
    bool isKeyFrame) {
    
    // TODO: 实现 H.264 特定的 NALU 分包逻辑
    // 这里暂时使用基础分包
    return packetize(frameData, timestamp, isKeyFrame);
}

std::vector<size_t> H264RTPPacketizer::findNALUStartCodes(
    const std::vector<uint8_t>& data) {
    
    std::vector<size_t> positions;
    
    for (size_t i = 0; i < data.size() - 3; ++i) {
        // 查找 0x00 0x00 0x00 0x01 或 0x00 0x00 0x01
        if (data[i] == 0 && data[i+1] == 0) {
            if (data[i+2] == 1) {
                positions.push_back(i + 3);  // 3字节起始码
            } else if (data[i+2] == 0 && data[i+3] == 1) {
                positions.push_back(i + 4);  // 4字节起始码
            }
        }
    }
    
    return positions;
}

std::vector<RTPPacket> H264RTPPacketizer::createFUAPackets(
    const uint8_t* naluData,
    size_t naluSize,
    uint32_t timestamp,
    bool isLastNALU) {
    
    std::vector<RTPPacket> packets;
    
    // TODO: 实现 FU-A 分片逻辑（RFC 6184）
    // FU-A 用于将大的 NALU 分片传输
    
    // 避免未使用参数警告
    (void)naluData;
    (void)naluSize;
    (void)timestamp;
    (void)isLastNALU;
    
    return packets;
}

// ============================================================================
// VP8RTPPacketizer 实现
// ============================================================================

VP8RTPPacketizer::VP8RTPPacketizer(const Config& config)
    : RTPPacketizer(config) {
    RCLCPP_INFO(rclcpp::get_logger("VP8RTPPacketizer"), 
               "VP8 RTP 分包器初始化");
}

std::vector<RTPPacket> VP8RTPPacketizer::packetizeVP8(
    const std::vector<uint8_t>& frameData,
    uint64_t timestamp,
    bool isKeyFrame) {
    
    // VP8 负载描述符
    std::vector<uint8_t> descriptor = createVP8PayloadDescriptor(isKeyFrame);
    
    // 创建包含 VP8 描述符的负载
    std::vector<uint8_t> payload;
    payload.reserve(descriptor.size() + frameData.size());
    payload.insert(payload.end(), descriptor.begin(), descriptor.end());
    payload.insert(payload.end(), frameData.begin(), frameData.end());
    
    return packetize(payload, timestamp, isKeyFrame);
}

std::vector<uint8_t> VP8RTPPacketizer::createVP8PayloadDescriptor(bool isKeyFrame) {
    std::vector<uint8_t> descriptor;
    
    // VP8 负载描述符（简化版）
    // Byte 0: X S PartID
    uint8_t byte0 = 0x10;  // X=0, S=1(起始分片)
    if (isKeyFrame) {
        byte0 |= 0x01;  // 设置 PartID
    }
    descriptor.push_back(byte0);
    
    return descriptor;
}

} // namespace u_webrtc

