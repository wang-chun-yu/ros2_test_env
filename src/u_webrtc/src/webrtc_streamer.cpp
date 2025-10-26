#include "u_webrtc/webrtc_streamer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

namespace u_webrtc {

WebRTCStreamer::WebRTCStreamer(const StreamConfig& config)
    : _config(config) {
}

WebRTCStreamer::~WebRTCStreamer() {
    if (_signalingClient) {
        _signalingClient->disconnect();
    }
}

bool WebRTCStreamer::initialize() {
    RCLCPP_INFO(rclcpp::get_logger("WebRTCStreamer"), "初始化 WebRTC 流管理器");
    
    // 创建 PeerConnection
    PeerConnectionWrapper::PeerConnectionConfig pcConfig;
    pcConfig.stunServer = _config.stunServer;
    pcConfig.turnServer = _config.turnServer;
    pcConfig.turnUsername = _config.turnUsername;
    pcConfig.turnPassword = _config.turnPassword;
    
    _peerConnection = std::make_unique<PeerConnectionWrapper>(pcConfig);
    if (!_peerConnection->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), 
                    "PeerConnection 初始化失败");
        return false;
    }
    
    // 设置 PeerConnection 回调
    _peerConnection->onIceCandidate(
        std::bind(&WebRTCStreamer::onIceCandidate, this, std::placeholders::_1)
    );
    _peerConnection->onConnectionStateChange(
        std::bind(&WebRTCStreamer::onConnectionStateChange, this, std::placeholders::_1)
    );
    
    // 创建信令客户端
    _signalingClient = std::make_unique<SignalingClient>(_config.signalingServerUrl);
    _signalingClient->onMessage(
        std::bind(&WebRTCStreamer::handleSignalingMessage, this, std::placeholders::_1)
    );
    
    // 创建视频编码器
    VideoEncoder::EncoderConfig encoderConfig;
    encoderConfig.codec = _config.codecName;
    encoderConfig.width = _config.width;
    encoderConfig.height = _config.height;
    encoderConfig.targetBitrate = _config.targetBitrate;
    encoderConfig.maxFramerate = _config.maxFrameRate;
    encoderConfig.encodingThreads = _config.encodingThreads;
    
    // 目前仅支持 VP8
    if (_config.codecName == "VP8") {
        _videoEncoder = std::make_unique<VP8Encoder>(encoderConfig);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("WebRTCStreamer"), 
                   "不支持的编码器: %s，使用 VP8", _config.codecName.c_str());
        _videoEncoder = std::make_unique<VP8Encoder>(encoderConfig);
    }
    
    if (!_videoEncoder->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), 
                    "视频编码器初始化失败");
        return false;
    }
    
    _isInitialized = true;
    RCLCPP_INFO(rclcpp::get_logger("WebRTCStreamer"), "WebRTC 流管理器初始化成功");
    return true;
}

bool WebRTCStreamer::pushFrame(const ImageConverter::ConvertedFrame& frame) {
    if (!_isInitialized) {
        RCLCPP_WARN(rclcpp::get_logger("WebRTCStreamer"), "流管理器未初始化");
        return false;
    }
    
    if (!_isConnected) {
        // 还未建立连接，跳过此帧
        return false;
    }
    
    // 编码帧
    auto encodedFrame = _videoEncoder->encode(
        frame.data.data(), 
        frame.width, 
        frame.height
    );
    
    if (!encodedFrame) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), "帧编码失败");
        return false;
    }
    
    // 发送编码后的帧
    _peerConnection->sendEncodedFrame(encodedFrame->data, encodedFrame->isKeyFrame);
    
    return true;
}

bool WebRTCStreamer::connectToSignalingServer() {
    if (!_signalingClient) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), "信令客户端未创建");
        return false;
    }
    
    return _signalingClient->connect();
}

void WebRTCStreamer::createOffer() {
    if (!_peerConnection) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), "PeerConnection 未创建");
        return;
    }
    
    std::string offer = _peerConnection->createOffer();
    
    if (_signalingClient && _signalingClient->isConnected()) {
        _signalingClient->sendOffer(offer);
        RCLCPP_INFO(rclcpp::get_logger("WebRTCStreamer"), "已发送 Offer");
    }
}

void WebRTCStreamer::handleAnswer(const std::string& sdp) {
    if (_peerConnection) {
        _peerConnection->setRemoteDescription(sdp);
        RCLCPP_INFO(rclcpp::get_logger("WebRTCStreamer"), "已处理 Answer");
    }
}

void WebRTCStreamer::addIceCandidate(const std::string& candidate) {
    if (_peerConnection) {
        _peerConnection->addIceCandidate(candidate);
    }
}

bool WebRTCStreamer::isConnected() const {
    return _isConnected;
}

void WebRTCStreamer::onIceCandidate(const std::string& candidate) {
    if (_signalingClient && _signalingClient->isConnected()) {
        _signalingClient->sendIceCandidate(candidate);
        RCLCPP_DEBUG(rclcpp::get_logger("WebRTCStreamer"), "已发送 ICE 候选");
    }
}

void WebRTCStreamer::onConnectionStateChange(const std::string& state) {
    RCLCPP_INFO(rclcpp::get_logger("WebRTCStreamer"), 
               "连接状态变化: %s", state.c_str());
    
    if (state == "connected") {
        _isConnected = true;
    } else if (state == "disconnected" || state == "failed" || state == "closed") {
        _isConnected = false;
    }
}

void WebRTCStreamer::handleSignalingMessage(const std::string& message) {
    try {
        auto json = nlohmann::json::parse(message);
        
        std::string type = json["type"];
        
        if (type == "answer") {
            std::string sdp = json["sdp"];
            handleAnswer(sdp);
        } else if (type == "candidate") {
            std::string candidate = json["candidate"];
            addIceCandidate(candidate);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("WebRTCStreamer"), 
                       "未知的信令消息类型: %s", type.c_str());
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("WebRTCStreamer"), 
                    "解析信令消息失败: %s", e.what());
    }
}

} // namespace u_webrtc



