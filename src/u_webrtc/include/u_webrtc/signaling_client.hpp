#ifndef U_WEBRTC_SIGNALING_CLIENT_HPP
#define U_WEBRTC_SIGNALING_CLIENT_HPP

#include <string>
#include <functional>
#include <memory>
#include <atomic>

namespace u_webrtc {

/**
 * @brief WebSocket 信令客户端，用于 WebRTC 信令交换
 */
class SignalingClient {
public:
    using MessageCallback = std::function<void(const std::string&)>;
    
    /**
     * @brief 构造函数
     * @param serverUrl WebSocket 服务器地址
     */
    explicit SignalingClient(const std::string& serverUrl);
    ~SignalingClient();
    
    /**
     * @brief 连接到信令服务器
     * @return 成功返回 true
     */
    bool connect();
    
    /**
     * @brief 断开连接
     */
    void disconnect();
    
    /**
     * @brief 发送 Offer SDP
     * @param sdp SDP 描述字符串
     */
    void sendOffer(const std::string& sdp);
    
    /**
     * @brief 发送 Answer SDP
     * @param sdp SDP 描述字符串
     */
    void sendAnswer(const std::string& sdp);
    
    /**
     * @brief 发送 ICE 候选
     * @param candidate ICE 候选字符串
     */
    void sendIceCandidate(const std::string& candidate);
    
    /**
     * @brief 设置消息接收回调
     * @param callback 回调函数
     */
    void onMessage(MessageCallback callback);
    
    /**
     * @brief 检查是否已连接
     * @return 已连接返回 true
     */
    bool isConnected() const;
    
private:
    /**
     * @brief 处理 WebSocket 消息
     * @param message 接收到的消息
     */
    void handleWebSocketMessage(const std::string& message);
    
    /**
     * @brief 发送 JSON 消息
     * @param message JSON 字符串
     */
    void sendMessage(const std::string& message);
    
    std::string _serverUrl;
    MessageCallback _messageCallback;
    std::atomic<bool> _isConnected{false};
    
    // WebSocket 连接实现细节（使用 Pimpl 模式）
    class Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace u_webrtc

#endif // U_WEBRTC_SIGNALING_CLIENT_HPP



