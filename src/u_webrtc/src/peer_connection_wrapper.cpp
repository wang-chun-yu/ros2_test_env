#include "u_webrtc/peer_connection_wrapper.hpp"
#include "u_webrtc/rtp_packetizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rtc/rtc.hpp>
#include <algorithm>
#include <cstddef>
#include <thread>
// #include <nlohmann/json.hpp>


namespace u_webrtc {

class PeerConnectionWrapper::Impl {
public:
    Impl(const PeerConnectionConfig& config) : _config(config) {
        // 配置 ICE 服务器
        rtc::Configuration rtcConfig;
        
        // 添加 STUN 服务器
        rtcConfig.iceServers.emplace_back(_config.stunServer);
        
        // 添加 TURN 服务器（如果配置了）
        if (!_config.turnServer.empty()) {
            // libdatachannel 的 IceServer 构造函数：
            // IceServer(string url) 或
            // IceServer(hostname, port, username, password, relayType)
            // 这里简单地将整个 TURN URL 作为一个字符串
            // 格式: turn:hostname:port?transport=udp
            rtcConfig.iceServers.emplace_back(_config.turnServer);
        }
        
        // 创建 PeerConnection
        _pc = std::make_shared<rtc::PeerConnection>(rtcConfig);
    }
    
    bool initialize() {
        // 设置回调
        _pc->onLocalDescription([this](rtc::Description desc) {
            _localSdp = std::string(desc);
            RCLCPP_INFO(rclcpp::get_logger("PeerConnection"), 
                       "Local SDP created");
        });
        
        _pc->onLocalCandidate([this](rtc::Candidate cand) {
            if (_iceCandidateCallback) {
                _iceCandidateCallback(std::string(cand));
            }
        });
        
        _pc->onStateChange([this](rtc::PeerConnection::State state) {
            std::string stateStr = stateToString(state);
            if (_connectionStateCallback) {
                _connectionStateCallback(stateStr);
            }
        });
        
        // 添加视频轨道
        auto video = rtc::Description::Video("video", 
            rtc::Description::Direction::SendOnly);
        video.addVP8Codec(96);  // 使用 VP8 编码（与 VP8Encoder 匹配）
        
        _videoTrack = _pc->addTrack(video);
        
        _videoTrack->onOpen([this]() {
            _isTrackOpen = true;
            RCLCPP_INFO(rclcpp::get_logger("PeerConnection"), 
                       "Video track opened");
        });
        
        // 初始化 RTP 分包器
        RTPPacketizer::Config rtpConfig;
        rtpConfig.payloadType = 96;  // H.264/VP8 动态负载类型
        rtpConfig.clockRate = 90000;  // 90kHz 视频时钟
        rtpConfig.maxPayloadSize = 1200;  // 避免 MTU 问题
        
        _rtpPacketizer = std::make_unique<RTPPacketizer>(rtpConfig);
        
        RCLCPP_INFO(rclcpp::get_logger("PeerConnection"), 
                   "PeerConnection 和 RTP 分包器初始化完成");
        
        return true;
    }
    
    std::string createOffer() {
        // 主动创建 offer
        // libdatachannel 需要设置本地描述来生成 SDP
        _pc->setLocalDescription();
        
        // 等待 onLocalDescription 回调被触发
        // 回调会设置 _localSdp
        int maxWait = 50; // 最多等待 500ms
        for (int i = 0; i < maxWait && _localSdp.empty(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (_localSdp.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("PeerConnection"), 
                        "创建 Offer 失败：超时");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("PeerConnection"), 
                       "Offer 创建成功，SDP 长度: %zu", _localSdp.length());
        }
        
        return _localSdp;
    }
    
    void setRemoteDescription(const std::string& sdp) {
        rtc::Description desc(sdp, "answer");
        _pc->setRemoteDescription(desc);
    }
    
    void addIceCandidate(const std::string& candidate) {
        _pc->addRemoteCandidate(rtc::Candidate(candidate));
    }
    
    void sendEncodedFrame(const std::vector<uint8_t>& frameData, 
                         bool isKeyFrame) {
        if (!_isTrackOpen || !_videoTrack || !_rtpPacketizer) {
            return;
        }
        
        // 关键修复：确保每个 VP8 帧作为单个 RTP 包发送
        // 避免分片导致的 VP8 descriptor 问题
        
        // 获取时间戳（90kHz 时钟，不是毫秒！）
        static uint32_t rtpTimestamp = 0;
        rtpTimestamp += 3000;  // 约 30fps (90000/30=3000)
        
        // 手动构造单个 RTP 包
        std::vector<uint8_t> rtpPacket;
        rtpPacket.reserve(12 + 1 + frameData.size());  // RTP header + VP8 descriptor + data
        
        // 1. RTP Header (12 bytes)
        rtpPacket.push_back(0x80);  // V=2, P=0, X=0, CC=0
        rtpPacket.push_back(0x60 | 96);  // M=0, PT=96 (VP8)
        
        // 序列号（大端序）
        static uint16_t seqNum = 0;
        seqNum++;
        rtpPacket.push_back((seqNum >> 8) & 0xFF);
        rtpPacket.push_back(seqNum & 0xFF);
        
        // 时间戳（大端序）
        rtpPacket.push_back((rtpTimestamp >> 24) & 0xFF);
        rtpPacket.push_back((rtpTimestamp >> 16) & 0xFF);
        rtpPacket.push_back((rtpTimestamp >> 8) & 0xFF);
        rtpPacket.push_back(rtpTimestamp & 0xFF);
        
        // SSRC（固定值）
        uint32_t ssrc = _rtpPacketizer->getConfig().ssrc;
        rtpPacket.push_back((ssrc >> 24) & 0xFF);
        rtpPacket.push_back((ssrc >> 16) & 0xFF);
        rtpPacket.push_back((ssrc >> 8) & 0xFF);
        rtpPacket.push_back(ssrc & 0xFF);
        
        // 2. VP8 Payload Descriptor (1 byte)
        uint8_t descriptor = 0x10;  // S=1 (start of partition), PID=0
        rtpPacket.push_back(descriptor);
        
        // 3. VP8 Encoded Data
        rtpPacket.insert(rtpPacket.end(), frameData.begin(), frameData.end());
        
        // 转换并发送
        std::vector<std::byte> byteData(rtpPacket.size());
        std::transform(rtpPacket.begin(), rtpPacket.end(), byteData.begin(),
                      [](uint8_t b) { return std::byte(b); });
        
        try {
            _videoTrack->send(byteData);
            
            _frameSent++;
            if (_frameSent % 100 == 0) {
                RCLCPP_INFO(rclcpp::get_logger("PeerConnection"),
                           "视频统计: 已发送 %lu 帧, 关键帧: %s",
                           _frameSent, isKeyFrame ? "是" : "否");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("PeerConnection"),
                        "发送帧失败: %s", e.what());
        }
    }
    
    void setIceCandidateCallback(PeerConnectionWrapper::IceCandidateCallback callback) {
        _iceCandidateCallback = std::move(callback);
    }
    
    void setConnectionStateCallback(PeerConnectionWrapper::ConnectionStateCallback callback) {
        _connectionStateCallback = std::move(callback);
    }
    
private:
    std::string stateToString(rtc::PeerConnection::State state) {
        switch (state) {
            case rtc::PeerConnection::State::New: return "new";
            case rtc::PeerConnection::State::Connecting: return "connecting";
            case rtc::PeerConnection::State::Connected: return "connected";
            case rtc::PeerConnection::State::Disconnected: return "disconnected";
            case rtc::PeerConnection::State::Failed: return "failed";
            case rtc::PeerConnection::State::Closed: return "closed";
            default: return "unknown";
        }
    }

    PeerConnectionConfig _config;
    std::shared_ptr<rtc::PeerConnection> _pc;
    std::shared_ptr<rtc::Track> _videoTrack;
    std::string _localSdp;
    bool _isTrackOpen{false};
    
    // RTP 分包器
    std::unique_ptr<RTPPacketizer> _rtpPacketizer;
    uint64_t _frameSent{0};
    uint64_t _bytesSent{0};

    PeerConnectionWrapper::IceCandidateCallback _iceCandidateCallback;
    PeerConnectionWrapper::ConnectionStateCallback _connectionStateCallback;
};

PeerConnectionWrapper::PeerConnectionWrapper(const PeerConnectionConfig& config)
    : _config(config), _impl(std::make_unique<Impl>(config)) {
}

PeerConnectionWrapper::~PeerConnectionWrapper() = default;

bool PeerConnectionWrapper::initialize() {
    return _impl->initialize();
}

std::string PeerConnectionWrapper::createOffer() {
    return _impl->createOffer();
}

void PeerConnectionWrapper::setRemoteDescription(const std::string& sdp) {
    _impl->setRemoteDescription(sdp);
}

void PeerConnectionWrapper::addIceCandidate(const std::string& candidate) {
    _impl->addIceCandidate(candidate);
}

void PeerConnectionWrapper::sendEncodedFrame(
    const std::vector<uint8_t>& frameData, 
    bool isKeyFrame) {
    _impl->sendEncodedFrame(frameData, isKeyFrame);
}

void PeerConnectionWrapper::onIceCandidate(IceCandidateCallback callback) {
    _iceCandidateCallback = std::move(callback);
    _impl->setIceCandidateCallback(_iceCandidateCallback);
}

void PeerConnectionWrapper::onConnectionStateChange(ConnectionStateCallback callback) {
    _connectionStateCallback = std::move(callback);
    _impl->setConnectionStateCallback(_connectionStateCallback);
}

} // namespace u_webrtc



