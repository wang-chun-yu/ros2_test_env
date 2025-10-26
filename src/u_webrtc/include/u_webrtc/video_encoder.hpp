#ifndef U_WEBRTC_VIDEO_ENCODER_HPP
#define U_WEBRTC_VIDEO_ENCODER_HPP

#include <vector>
#include <string>
#include <memory>
#include <optional>

namespace u_webrtc {

/**
 * @brief 视频编码器接口
 */
class VideoEncoder {
public:
    struct EncodedFrame {
        std::vector<uint8_t> data;
        bool isKeyFrame;
        int64_t timestamp;
    };
    
    struct EncoderConfig {
        std::string codec;      // "VP8", "VP9", "H264"
        int width;
        int height;
        int targetBitrate;
        int maxFramerate;
        int encodingThreads;
    };
    
    explicit VideoEncoder(const EncoderConfig& config);
    virtual ~VideoEncoder() = default;
    
    /**
     * @brief 初始化编码器
     * @return 成功返回 true
     */
    virtual bool initialize() = 0;
    
    /**
     * @brief 编码一帧图像
     * @param frameData 原始帧数据
     * @param width 图像宽度
     * @param height 图像高度
     * @return 编码后的帧数据
     */
    virtual std::optional<EncodedFrame> encode(
        const uint8_t* frameData, 
        int width, 
        int height) = 0;
    
    /**
     * @brief 请求生成关键帧
     */
    virtual void requestKeyFrame() = 0;
    
protected:
    EncoderConfig _config;
};

/**
 * @brief VP8 编码器实现
 */
class VP8Encoder : public VideoEncoder {
public:
    explicit VP8Encoder(const EncoderConfig& config);
    ~VP8Encoder() override;
    
    bool initialize() override;
    std::optional<EncodedFrame> encode(
        const uint8_t* frameData, 
        int width, 
        int height) override;
    void requestKeyFrame() override;
    
private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace u_webrtc

#endif // U_WEBRTC_VIDEO_ENCODER_HPP



