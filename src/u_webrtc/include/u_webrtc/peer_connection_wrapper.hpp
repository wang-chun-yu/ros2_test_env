#ifndef U_WEBRTC_PEER_CONNECTION_WRAPPER_HPP
#define U_WEBRTC_PEER_CONNECTION_WRAPPER_HPP

#include <string>
#include <functional>
#include <memory>
#include <vector>

namespace u_webrtc {

/**
 * @brief WebRTC PeerConnection 封装类
 */
class PeerConnectionWrapper {
public:
    using IceCandidateCallback = std::function<void(const std::string&)>;
    using ConnectionStateCallback = std::function<void(const std::string&)>;
    
    struct PeerConnectionConfig {
        std::string stunServer;     // "stun:stun.l.google.com:19302"
        std::string turnServer;     // TURN 服务器地址（可选）
        std::string turnUsername;
        std::string turnPassword;
    };
    
    /**
     * @brief 构造函数
     * @param config 连接配置
     */
    explicit PeerConnectionWrapper(const PeerConnectionConfig& config);
    ~PeerConnectionWrapper();
    
    /**
     * @brief 初始化 PeerConnection
     * @return 成功返回 true
     */
    bool initialize();
    
    /**
     * @brief 创建 Offer
     * @return Offer SDP 字符串
     */
    std::string createOffer();
    
    /**
     * @brief 设置远端 SDP（Answer）
     * @param sdp Answer SDP 字符串
     */
    void setRemoteDescription(const std::string& sdp);
    
    /**
     * @brief 添加 ICE 候选
     * @param candidate ICE 候选字符串
     */
    void addIceCandidate(const std::string& candidate);
    
    /**
     * @brief 发送编码后的视频帧
     * @param frameData 帧数据
     * @param isKeyFrame 是否为关键帧
     */
    void sendEncodedFrame(const std::vector<uint8_t>& frameData, bool isKeyFrame);
    
    /**
     * @brief 设置 ICE 候选回调
     * @param callback 回调函数
     */
    void onIceCandidate(IceCandidateCallback callback);
    
    /**
     * @brief 设置连接状态变化回调
     * @param callback 回调函数
     */
    void onConnectionStateChange(ConnectionStateCallback callback);
    
private:
    PeerConnectionConfig _config;
    IceCandidateCallback _iceCandidateCallback;
    ConnectionStateCallback _connectionStateCallback;
    
    // PeerConnection 实现细节（使用 Pimpl 模式）
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace u_webrtc

#endif // U_WEBRTC_PEER_CONNECTION_WRAPPER_HPP



