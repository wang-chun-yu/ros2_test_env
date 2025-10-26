#ifndef U_WEBRTC_WEBRTC_STREAMER_HPP
#define U_WEBRTC_WEBRTC_STREAMER_HPP

#include "image_converter.hpp"
#include "peer_connection_wrapper.hpp"
#include "signaling_client.hpp"
#include "video_encoder.hpp"
#include <memory>
#include <atomic>
#include <string>

namespace u_webrtc {

/**
 * @brief WebRTC 流管理器，核心类
 */
class WebRTCStreamer {
public:
    struct StreamConfig {
        std::string signalingServerUrl;
        std::string stunServer;
        std::string turnServer;
        std::string turnUsername;
        std::string turnPassword;
        int targetBitrate;
        int maxFrameRate;
        std::string codecName;      // "VP8", "VP9", "H264"
        int width;
        int height;
        int encodingThreads;
    };
    
    /**
     * @brief 构造函数
     * @param config 流配置
     */
    explicit WebRTCStreamer(const StreamConfig& config);
    ~WebRTCStreamer();
    
    /**
     * @brief 初始化 WebRTC 连接
     * @return 成功返回 true
     */
    bool initialize();
    
    /**
     * @brief 推送帧数据到 WebRTC
     * @param frame 转换后的帧数据
     * @return 成功返回 true
     */
    bool pushFrame(const ImageConverter::ConvertedFrame& frame);
    
    /**
     * @brief 连接到信令服务器
     * @return 成功返回 true
     */
    bool connectToSignalingServer();
    
    /**
     * @brief 创建并发送 Offer
     */
    void createOffer();
    
    /**
     * @brief 处理 Answer
     * @param sdp Answer SDP 字符串
     */
    void handleAnswer(const std::string& sdp);
    
    /**
     * @brief 添加 ICE 候选
     * @param candidate ICE 候选字符串
     */
    void addIceCandidate(const std::string& candidate);
    
    /**
     * @brief 检查是否已连接
     * @return 已连接返回 true
     */
    bool isConnected() const;
    
private:
    /**
     * @brief ICE 候选回调
     * @param candidate 候选字符串
     */
    void onIceCandidate(const std::string& candidate);
    
    /**
     * @brief 连接状态变化回调
     * @param state 状态字符串
     */
    void onConnectionStateChange(const std::string& state);
    
    /**
     * @brief 处理信令消息
     * @param message JSON 消息
     */
    void handleSignalingMessage(const std::string& message);
    
    std::unique_ptr<PeerConnectionWrapper> _peerConnection;
    std::unique_ptr<SignalingClient> _signalingClient;
    std::unique_ptr<VideoEncoder> _videoEncoder;
    StreamConfig _config;
    std::atomic<bool> _isConnected{false};
    std::atomic<bool> _isInitialized{false};
};

} // namespace u_webrtc

#endif // U_WEBRTC_WEBRTC_STREAMER_HPP



