#include "u_webrtc/signaling_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#ifdef USE_WEBSOCKETPP
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <thread>
#include <chrono>

// 支持两种客户端类型
typedef websocketpp::client<websocketpp::config::asio_client> ws_client;
typedef websocketpp::client<websocketpp::config::asio_tls_client> wss_client;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;
#endif

namespace u_webrtc {

#ifdef USE_WEBSOCKETPP

// ============================================================================
// 真实的 WebSocket 实现（使用 websocketpp）
// ============================================================================

class SignalingClient::Impl {
public:
    Impl(const std::string& serverUrl) : _serverUrl(serverUrl) {
        try {
            // 检测是否使用 TLS
            _useTls = (_serverUrl.find("wss://") == 0);
            
            if (_useTls) {
                // 使用 WSS 客户端
                _wssClient = std::make_unique<wss_client>();
                
                // 设置日志级别
                _wssClient->clear_access_channels(websocketpp::log::alevel::all);
                _wssClient->clear_error_channels(websocketpp::log::elevel::all);
                
                // 初始化 ASIO
                _wssClient->init_asio();
                
                // 设置 TLS 初始化处理器
                _wssClient->set_tls_init_handler([this](websocketpp::connection_hdl) {
                    return onTlsInit();
                });
                
                // 设置消息处理器
                _wssClient->set_message_handler([this](websocketpp::connection_hdl hdl, 
                                                       wss_client::message_ptr msg) {
                    onMessage(hdl, msg);
                });
                
                // 设置打开处理器
                _wssClient->set_open_handler([this](websocketpp::connection_hdl hdl) {
                    onOpen(hdl);
                });
                
                // 设置关闭处理器
                _wssClient->set_close_handler([this](websocketpp::connection_hdl hdl) {
                    onClose(hdl);
                });
                
                // 设置失败处理器
                _wssClient->set_fail_handler([this](websocketpp::connection_hdl hdl) {
                    onFail(hdl);
                });
                
                RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                           "WebSocket 客户端初始化完成 (WSS 模式)");
            } else {
                // 使用 WS 客户端
                _wsClient = std::make_unique<ws_client>();
                
                // 设置日志级别
                _wsClient->clear_access_channels(websocketpp::log::alevel::all);
                _wsClient->clear_error_channels(websocketpp::log::elevel::all);
                
                // 初始化 ASIO
                _wsClient->init_asio();
                
                // 设置消息处理器
                _wsClient->set_message_handler([this](websocketpp::connection_hdl hdl, 
                                                      ws_client::message_ptr msg) {
                    onMessage(hdl, msg);
                });
                
                // 设置打开处理器
                _wsClient->set_open_handler([this](websocketpp::connection_hdl hdl) {
                    onOpen(hdl);
                });
                
                // 设置关闭处理器
                _wsClient->set_close_handler([this](websocketpp::connection_hdl hdl) {
                    onClose(hdl);
                });
                
                // 设置失败处理器
                _wsClient->set_fail_handler([this](websocketpp::connection_hdl hdl) {
                    onFail(hdl);
                });
                
                RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                           "WebSocket 客户端初始化完成 (WS 模式)");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                        "初始化失败: %s", e.what());
        }
    }
    
    ~Impl() {
        disconnect();
        if (_ioThread.joinable()) {
            _ioThread.join();
        }
    }
    
    bool connect() {
        try {
            websocketpp::lib::error_code ec;
            
            if (_useTls) {
                // WSS 连接
                wss_client::connection_ptr con = _wssClient->get_connection(_serverUrl, ec);
                
                if (ec) {
                    RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                                "WSS 连接创建失败: %s", ec.message().c_str());
                    return false;
                }
                
                // 保存连接句柄
                _hdl = con->get_handle();
                
                // 连接
                _wssClient->connect(con);
                
                // 在单独的线程中运行 io_service
                _ioThread = std::thread([this]() {
                    try {
                        _wssClient->run();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                                    "WSS IO 线程异常: %s", e.what());
                    }
                });
            } else {
                // WS 连接
                ws_client::connection_ptr con = _wsClient->get_connection(_serverUrl, ec);
                
                if (ec) {
                    RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                                "WS 连接创建失败: %s", ec.message().c_str());
                    return false;
                }
                
                // 保存连接句柄
                _hdl = con->get_handle();
                
                // 连接
                _wsClient->connect(con);
                
                // 在单独的线程中运行 io_service
                _ioThread = std::thread([this]() {
                    try {
                        _wsClient->run();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                                    "WS IO 线程异常: %s", e.what());
                    }
                });
            }
            
            // 等待连接建立（最多 5 秒）
            for (int i = 0; i < 50; ++i) {
                if (_isConnected) {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            RCLCPP_WARN(rclcpp::get_logger("SignalingClient"), 
                       "连接超时，但仍在尝试...");
            return _isConnected;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                        "连接异常: %s", e.what());
            return false;
        }
    }
    
    void disconnect() {
        if (_isConnected) {
            try {
                websocketpp::lib::error_code ec;
                
                if (_useTls) {
                    _wssClient->close(_hdl, websocketpp::close::status::going_away, 
                                     "Client closing", ec);
                } else {
                    _wsClient->close(_hdl, websocketpp::close::status::going_away, 
                                    "Client closing", ec);
                }
                
                if (ec) {
                    RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                                "关闭连接失败: %s", ec.message().c_str());
                }
                
                _isConnected = false;
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                            "断开连接异常: %s", e.what());
            }
        }
        
        // 停止 io_service
        if (_useTls && _wssClient) {
            _wssClient->stop();
        } else if (_wsClient) {
            _wsClient->stop();
        }
    }
    
    void sendMessage(const std::string& message) {
        if (!_isConnected) {
            RCLCPP_WARN(rclcpp::get_logger("SignalingClient"), 
                       "未连接，无法发送消息");
            return;
        }
        
        try {
            websocketpp::lib::error_code ec;
            
            if (_useTls) {
                _wssClient->send(_hdl, message, websocketpp::frame::opcode::text, ec);
            } else {
                _wsClient->send(_hdl, message, websocketpp::frame::opcode::text, ec);
            }
            
            if (ec) {
                RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                            "发送消息失败: %s", ec.message().c_str());
            } else {
                RCLCPP_DEBUG(rclcpp::get_logger("SignalingClient"), 
                            "发送消息: %s", message.c_str());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                        "发送消息异常: %s", e.what());
        }
    }
    
    bool isConnected() const {
        return _isConnected;
    }
    
    void setMessageCallback(SignalingClient::MessageCallback callback) {
        _messageCallback = std::move(callback);
    }
    
private:
    context_ptr onTlsInit() {
        context_ptr ctx = std::make_shared<boost::asio::ssl::context>(
            boost::asio::ssl::context::sslv23
        );
        
        try {
            ctx->set_options(boost::asio::ssl::context::default_workarounds |
                           boost::asio::ssl::context::no_sslv2 |
                           boost::asio::ssl::context::no_sslv3 |
                           boost::asio::ssl::context::single_dh_use);
        } catch (std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                        "TLS 初始化失败: %s", e.what());
        }
        
        return ctx;
    }
    
    template<typename MessagePtr>
    void onMessage(websocketpp::connection_hdl, MessagePtr msg) {
        const std::string& payload = msg->get_payload();
        
        RCLCPP_DEBUG(rclcpp::get_logger("SignalingClient"), 
                    "收到消息: %s", payload.c_str());
        
        if (_messageCallback) {
            _messageCallback(payload);
        }
    }
    
    void onOpen(websocketpp::connection_hdl) {
        _isConnected = true;
        RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                   "✅ WebSocket 连接已建立: %s", _serverUrl.c_str());
    }
    
    void onClose(websocketpp::connection_hdl hdl) {
        _isConnected = false;
        
        std::string closeReason;
        std::string errorMsg;
        
        try {
            if (_useTls) {
                wss_client::connection_ptr con = _wssClient->get_con_from_hdl(hdl);
                closeReason = con->get_remote_close_reason();
                errorMsg = con->get_ec().message();
            } else {
                ws_client::connection_ptr con = _wsClient->get_con_from_hdl(hdl);
                closeReason = con->get_remote_close_reason();
                errorMsg = con->get_ec().message();
            }
        } catch (...) {
            closeReason = "Unknown";
            errorMsg = "Unknown";
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                   "WebSocket 连接已关闭: %s (%s)", 
                   closeReason.c_str(), errorMsg.c_str());
    }
    
    void onFail(websocketpp::connection_hdl hdl) {
        _isConnected = false;
        
        std::string errorMsg;
        
        try {
            if (_useTls) {
                wss_client::connection_ptr con = _wssClient->get_con_from_hdl(hdl);
                errorMsg = con->get_ec().message();
            } else {
                ws_client::connection_ptr con = _wsClient->get_con_from_hdl(hdl);
                errorMsg = con->get_ec().message();
            }
        } catch (...) {
            errorMsg = "Unknown error";
        }
        
        RCLCPP_ERROR(rclcpp::get_logger("SignalingClient"), 
                    "WebSocket 连接失败: %s", errorMsg.c_str());
    }
    
    std::string _serverUrl;
    bool _useTls{false};
    std::unique_ptr<ws_client> _wsClient;
    std::unique_ptr<wss_client> _wssClient;
    websocketpp::connection_hdl _hdl;
    std::atomic<bool> _isConnected{false};
    std::thread _ioThread;
    SignalingClient::MessageCallback _messageCallback;
};

#else  // 没有 websocketpp，使用框架实现

// ============================================================================
// 框架实现（模拟）
// ============================================================================

class SignalingClient::Impl {
public:
    Impl(const std::string& serverUrl) : _serverUrl(serverUrl) {}
    
    bool connect() {
        RCLCPP_WARN(rclcpp::get_logger("SignalingClient"), 
                   "⚠️  websocketpp 未安装，使用框架实现");
        RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                   "模拟连接到: %s", _serverUrl.c_str());
        _isConnected = true;
        return true;
    }
    
    void disconnect() {
        if (_isConnected) {
            _isConnected = false;
            RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), 
                       "模拟断开连接");
        }
    }
    
    void sendMessage(const std::string& message) {
        if (!_isConnected) {
            return;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("SignalingClient"), 
                    "模拟发送: %s", message.c_str());
    }
    
    bool isConnected() const {
        return _isConnected;
    }
    
    void setMessageCallback(SignalingClient::MessageCallback callback) {
        _messageCallback = std::move(callback);
    }
    
private:
    std::string _serverUrl;
    std::atomic<bool> _isConnected{false};
    SignalingClient::MessageCallback _messageCallback;
};

#endif  // USE_WEBSOCKETPP

// ============================================================================
// SignalingClient 公共接口实现
// ============================================================================

SignalingClient::SignalingClient(const std::string& serverUrl)
    : _serverUrl(serverUrl), _impl(std::make_unique<Impl>(serverUrl)) {
}

SignalingClient::~SignalingClient() {
    disconnect();
}

bool SignalingClient::connect() {
    bool success = _impl->connect();
    _isConnected = success;
    return success;
}

void SignalingClient::disconnect() {
    _impl->disconnect();
    _isConnected = false;
}

void SignalingClient::sendOffer(const std::string& sdp) {
    nlohmann::json message;
    message["type"] = "offer";
    message["sdp"] = sdp;
    
    sendMessage(message.dump());
    RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), "已发送 Offer");
}

void SignalingClient::sendAnswer(const std::string& sdp) {
    nlohmann::json message;
    message["type"] = "answer";
    message["sdp"] = sdp;
    
    sendMessage(message.dump());
    RCLCPP_INFO(rclcpp::get_logger("SignalingClient"), "已发送 Answer");
}

void SignalingClient::sendIceCandidate(const std::string& candidate) {
    nlohmann::json message;
    message["type"] = "candidate";
    message["candidate"] = candidate;
    
    sendMessage(message.dump());
    RCLCPP_DEBUG(rclcpp::get_logger("SignalingClient"), "已发送 ICE 候选");
}

void SignalingClient::onMessage(MessageCallback callback) {
    _messageCallback = std::move(callback);
    _impl->setMessageCallback(_messageCallback);
}

bool SignalingClient::isConnected() const {
    return _isConnected;
}

void SignalingClient::sendMessage(const std::string& message) {
    _impl->sendMessage(message);
}

void SignalingClient::handleWebSocketMessage(const std::string& message) {
    if (_messageCallback) {
        _messageCallback(message);
    }
}

} // namespace u_webrtc
