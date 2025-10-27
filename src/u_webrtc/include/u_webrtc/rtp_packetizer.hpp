#ifndef U_WEBRTC_RTP_PACKETIZER_HPP
#define U_WEBRTC_RTP_PACKETIZER_HPP

#include <vector>
#include <cstdint>
#include <memory>
#include <optional>

namespace u_webrtc {

/**
 * @brief RTP 头部结构（RFC 3550）
 */
struct RTPHeader {
    // Byte 0
    uint8_t version : 2;      // 版本号 (2)
    uint8_t padding : 1;      // 填充标志
    uint8_t extension : 1;    // 扩展标志
    uint8_t csrcCount : 4;    // CSRC 计数
    
    // Byte 1
    uint8_t marker : 1;       // 标记位（帧结束）
    uint8_t payloadType : 7;  // 负载类型
    
    // Byte 2-3
    uint16_t sequenceNumber;  // 序列号
    
    // Byte 4-7
    uint32_t timestamp;       // 时间戳
    
    // Byte 8-11
    uint32_t ssrc;           // 同步源标识符
    
    /**
     * @brief 序列化 RTP 头部到字节数组
     */
    std::vector<uint8_t> serialize() const;
    
    /**
     * @brief 从字节数组反序列化 RTP 头部
     */
    static std::optional<RTPHeader> deserialize(const uint8_t* data, size_t size);
} __attribute__((packed));

/**
 * @brief RTP 数据包结构
 */
struct RTPPacket {
    RTPHeader header;
    std::vector<uint8_t> payload;
    
    /**
     * @brief 序列化整个 RTP 包
     */
    std::vector<uint8_t> serialize() const;
};

/**
 * @brief RTP 分包器 - 将大的编码帧分割为 RTP 包
 */
class RTPPacketizer {
public:
    struct Config {
        uint8_t payloadType = 96;      // VP8/H.264 动态负载类型
        uint32_t ssrc;                 // 同步源标识符
        uint32_t clockRate = 90000;    // 视频时钟频率 (90kHz)
        size_t maxPayloadSize = 1200;  // 最大负载大小（避免 MTU 问题）
    };
    
    explicit RTPPacketizer(const Config& config);
    ~RTPPacketizer() = default;
    
    /**
     * @brief 将编码帧打包为 RTP 包
     * @param frameData 编码后的帧数据
     * @param timestamp 时间戳（毫秒）
     * @param isKeyFrame 是否为关键帧
     * @return RTP 包列表
     */
    std::vector<RTPPacket> packetize(
        const std::vector<uint8_t>& frameData,
        uint64_t timestamp,
        bool isKeyFrame
    );
    
    /**
     * @brief 重置序列号和时间戳
     */
    void reset();
    
    /**
     * @brief 获取当前序列号
     */
    uint16_t getSequenceNumber() const { return _sequenceNumber; }
    
    /**
     * @brief 获取统计信息
     */
    struct Stats {
        uint64_t packetsSent = 0;
        uint64_t bytesSent = 0;
        uint64_t framesSent = 0;
    };
    
    Stats getStats() const { return _stats; }
    
    /**
     * @brief 获取配置
     */
    const Config& getConfig() const { return _config; }
    
private:
    Config _config;
    uint16_t _sequenceNumber = 0;
    uint32_t _lastTimestamp = 0;
    Stats _stats;
    
    /**
     * @brief 创建 RTP 包
     */
    RTPPacket createPacket(
        const uint8_t* data,
        size_t size,
        uint32_t timestamp,
        bool marker
    );
    
    /**
     * @brief 将毫秒转换为 RTP 时间戳
     */
    uint32_t convertToRTPTimestamp(uint64_t milliseconds);
};

/**
 * @brief H.264 RTP 分包器（实现 RFC 6184）
 */
class H264RTPPacketizer : public RTPPacketizer {
public:
    explicit H264RTPPacketizer(const Config& config);
    
    /**
     * @brief H.264 特定的分包逻辑
     * 支持单 NALU 模式和 FU-A 分片
     */
    std::vector<RTPPacket> packetizeH264(
        const std::vector<uint8_t>& frameData,
        uint64_t timestamp,
        bool isKeyFrame
    );
    
private:
    /**
     * @brief 查找 H.264 NALU 起始码
     */
    std::vector<size_t> findNALUStartCodes(const std::vector<uint8_t>& data);
    
    /**
     * @brief 创建 FU-A 分片包
     */
    std::vector<RTPPacket> createFUAPackets(
        const uint8_t* naluData,
        size_t naluSize,
        uint32_t timestamp,
        bool isLastNALU
    );
};

/**
 * @brief VP8 RTP 分包器（实现 RFC 7741）
 */
class VP8RTPPacketizer : public RTPPacketizer {
public:
    explicit VP8RTPPacketizer(const Config& config);
    
    /**
     * @brief VP8 特定的分包逻辑
     */
    std::vector<RTPPacket> packetizeVP8(
        const std::vector<uint8_t>& frameData,
        uint64_t timestamp,
        bool isKeyFrame
    );
    
private:
    /**
     * @brief 创建 VP8 负载描述符
     */
    std::vector<uint8_t> createVP8PayloadDescriptor(bool isKeyFrame);
};

} // namespace u_webrtc

#endif // U_WEBRTC_RTP_PACKETIZER_HPP

