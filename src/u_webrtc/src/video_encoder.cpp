#include "u_webrtc/video_encoder.hpp"
#include <rclcpp/rclcpp.hpp>

#ifdef USE_LIBVPX
#include <vpx/vpx_encoder.h>
#include <vpx/vp8cx.h>
#endif

namespace u_webrtc {

VideoEncoder::VideoEncoder(const EncoderConfig& config)
    : _config(config) {
}

// ============================================================================
// VP8Encoder 实现
// ============================================================================

#ifdef USE_LIBVPX

class VP8Encoder::Impl {
public:
    Impl(const EncoderConfig& config) : _config(config) {}
    
    ~Impl() {
        if (_isInitialized) {
            vpx_codec_destroy(&_codec);
            vpx_img_free(&_rawImage);
        }
    }
    
    bool initialize() {
        // 配置 VP8 编码器
        vpx_codec_err_t res = vpx_codec_enc_config_default(
            vpx_codec_vp8_cx(), 
            &_cfg, 
            0
        );
        
        if (res != VPX_CODEC_OK) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), 
                        "获取默认配置失败: %s", vpx_codec_err_to_string(res));
            return false;
        }
        
        // 设置编码参数
        _cfg.g_w = _config.width;
        _cfg.g_h = _config.height;
        _cfg.g_timebase.num = 1;
        _cfg.g_timebase.den = _config.maxFramerate;
        _cfg.rc_target_bitrate = _config.targetBitrate / 1000;  // kbps
        _cfg.g_error_resilient = VPX_ERROR_RESILIENT_DEFAULT;
        _cfg.g_lag_in_frames = 0;  // 实时编码，无延迟
        _cfg.rc_end_usage = VPX_CBR;  // 恒定比特率
        _cfg.g_threads = _config.encodingThreads;
        
        // 初始化编码器
        res = vpx_codec_enc_init(
            &_codec, 
            vpx_codec_vp8_cx(), 
            &_cfg, 
            0
        );
        
        if (res != VPX_CODEC_OK) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), 
                        "初始化编码器失败: %s", vpx_codec_err_to_string(res));
            return false;
        }
        
        // 设置编码器控制参数
        vpx_codec_control(&_codec, VP8E_SET_CPUUSED, -6);  // 速度优先（-16到16）
        vpx_codec_control(&_codec, VP8E_SET_STATIC_THRESHOLD, 0);
        vpx_codec_control(&_codec, VP8E_SET_MAX_INTRA_BITRATE_PCT, 300);
        
        // 分配图像缓冲区
        if (!vpx_img_alloc(&_rawImage, VPX_IMG_FMT_I420, 
                          _config.width, _config.height, 1)) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), "分配图像缓冲区失败");
            vpx_codec_destroy(&_codec);
            return false;
        }
        
        _isInitialized = true;
        
        RCLCPP_INFO(rclcpp::get_logger("VP8Encoder"), 
                   "VP8 编码器初始化成功: %dx%d @ %d kbps, %d 线程",
                   _config.width, _config.height, 
                   _config.targetBitrate / 1000, _config.encodingThreads);
        
        return true;
    }
    
    std::optional<VideoEncoder::EncodedFrame> encode(
        const uint8_t* frameData, 
        int width, 
        int height) {
        
        if (!_isInitialized) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), "编码器未初始化");
            return std::nullopt;
        }
        
        if (width != _config.width || height != _config.height) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), 
                        "帧尺寸不匹配: 期望 %dx%d, 实际 %dx%d",
                        _config.width, _config.height, width, height);
            return std::nullopt;
        }
        
        // 复制 I420 数据到 vpx_image
        size_t ySize = width * height;
        size_t uvSize = ySize / 4;
        
        memcpy(_rawImage.planes[VPX_PLANE_Y], frameData, ySize);
        memcpy(_rawImage.planes[VPX_PLANE_U], frameData + ySize, uvSize);
        memcpy(_rawImage.planes[VPX_PLANE_V], frameData + ySize + uvSize, uvSize);
        
        // 编码标志
        vpx_enc_frame_flags_t flags = 0;
        if (_keyFrameRequested) {
            flags |= VPX_EFLAG_FORCE_KF;
            _keyFrameRequested = false;
        }
        
        // 编码帧
        vpx_codec_err_t res = vpx_codec_encode(
            &_codec,
            &_rawImage,
            _frameCount,
            1,  // duration
            flags,
            VPX_DL_REALTIME  // 实时模式
        );
        
        if (res != VPX_CODEC_OK) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), 
                        "编码失败: %s", vpx_codec_err_to_string(res));
            return std::nullopt;
        }
        
        // 获取编码数据
        EncodedFrame encoded;
        encoded.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        
        vpx_codec_iter_t iter = nullptr;
        const vpx_codec_cx_pkt_t* pkt;
        
        while ((pkt = vpx_codec_get_cx_data(&_codec, &iter)) != nullptr) {
            if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
                // 复制编码数据
                const uint8_t* data = static_cast<const uint8_t*>(pkt->data.frame.buf);
                size_t size = pkt->data.frame.sz;
                encoded.data.assign(data, data + size);
                
                // 检查是否为关键帧
                encoded.isKeyFrame = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
                
                RCLCPP_DEBUG(rclcpp::get_logger("VP8Encoder"), 
                            "编码帧 %d: %zu bytes, 关键帧: %s",
                            _frameCount, size,
                            encoded.isKeyFrame ? "是" : "否");
                
                _frameCount++;
                return encoded;
            }
        }
        
        RCLCPP_WARN(rclcpp::get_logger("VP8Encoder"), "未获取到编码数据");
        return std::nullopt;
    }
    
    void requestKeyFrame() {
        _keyFrameRequested = true;
        RCLCPP_INFO(rclcpp::get_logger("VP8Encoder"), "请求关键帧");
    }
    
private:
    EncoderConfig _config;
    bool _isInitialized{false};
    bool _keyFrameRequested{false};
    int _frameCount{0};
    
    vpx_codec_ctx_t _codec;
    vpx_codec_enc_cfg_t _cfg;
    vpx_image_t _rawImage;
};

#else  // 没有 libvpx，使用框架实现

class VP8Encoder::Impl {
public:
    Impl(const EncoderConfig& config) : _config(config) {}
    
    bool initialize() {
        RCLCPP_WARN(rclcpp::get_logger("VP8Encoder"), 
                   "⚠️  libvpx 未安装，使用框架实现（不产生真实编码数据）");
        RCLCPP_INFO(rclcpp::get_logger("VP8Encoder"), 
                   "VP8 编码器（框架）初始化: %dx%d @ %d kbps",
                   _config.width, _config.height, _config.targetBitrate / 1000);
        _isInitialized = true;
        return true;
    }
    
    std::optional<VideoEncoder::EncodedFrame> encode(
        const uint8_t* frameData, 
        int width, 
        int height) {
        
        if (!_isInitialized) {
            RCLCPP_ERROR(rclcpp::get_logger("VP8Encoder"), "编码器未初始化");
            return std::nullopt;
        }
        
        // 框架实现 - 模拟编码
        EncodedFrame encoded;
        encoded.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();
        
        _frameCount++;
        encoded.isKeyFrame = (_frameCount % 30 == 0) || _keyFrameRequested;
        _keyFrameRequested = false;
        
        // 模拟编码数据
        size_t estimatedSize = width * height / 10;
        encoded.data.resize(estimatedSize);
        
        // 避免未使用参数警告
        (void)frameData;
        
        RCLCPP_DEBUG(rclcpp::get_logger("VP8Encoder"), 
                    "编码帧（框架）: %dx%d, 关键帧: %s, 大小: %zu bytes",
                    width, height, 
                    encoded.isKeyFrame ? "是" : "否",
                    encoded.data.size());
        
        return encoded;
    }
    
    void requestKeyFrame() {
        _keyFrameRequested = true;
        RCLCPP_INFO(rclcpp::get_logger("VP8Encoder"), "请求关键帧");
    }
    
private:
    EncoderConfig _config;
    bool _isInitialized{false};
    bool _keyFrameRequested{false};
    int _frameCount{0};
};

#endif  // USE_LIBVPX

// ============================================================================
// VP8Encoder 公共接口实现
// ============================================================================

VP8Encoder::VP8Encoder(const EncoderConfig& config)
    : VideoEncoder(config), _impl(std::make_unique<Impl>(config)) {
}

VP8Encoder::~VP8Encoder() = default;

bool VP8Encoder::initialize() {
    return _impl->initialize();
}

std::optional<VideoEncoder::EncodedFrame> VP8Encoder::encode(
    const uint8_t* frameData, 
    int width, 
    int height) {
    return _impl->encode(frameData, width, height);
}

void VP8Encoder::requestKeyFrame() {
    _impl->requestKeyFrame();
}

} // namespace u_webrtc
