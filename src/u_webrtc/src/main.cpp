#include "u_webrtc/image_subscriber.hpp"
#include "u_webrtc/image_converter.hpp"
#include "u_webrtc/webrtc_streamer.hpp"
#include "u_webrtc/config_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

/**
 * @brief WebRTC 视频流节点主类
 */
class WebRTCStreamNode : public rclcpp::Node {
public:
    WebRTCStreamNode() : Node("webrtc_streamer") {
        // 声明参数
        u_webrtc::ConfigManager::declareParameters(this);
        
        // 加载配置
        auto config = u_webrtc::ConfigManager::loadFromNode(this);
        
        RCLCPP_INFO(this->get_logger(), "=== WebRTC 视频流节点配置 ===");
        RCLCPP_INFO(this->get_logger(), "图像话题: %s", config.imageTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "信令服务器: %s", config.signalingServerUrl.c_str());
        RCLCPP_INFO(this->get_logger(), "编码格式: %s", config.codec.c_str());
        RCLCPP_INFO(this->get_logger(), "目标比特率: %d kbps", config.targetBitrate / 1000);
        RCLCPP_INFO(this->get_logger(), "分辨率: %dx%d @ %d fps", 
                   config.width, config.height, config.maxFramerate);
        
        // 创建图像转换器
        _imageConverter = std::make_unique<u_webrtc::ImageConverter>();
        
        // 创建 WebRTC 流管理器
        u_webrtc::WebRTCStreamer::StreamConfig streamConfig;
        streamConfig.signalingServerUrl = config.signalingServerUrl;
        streamConfig.stunServer = config.stunServer;
        streamConfig.turnServer = config.turnServer;
        streamConfig.turnUsername = config.turnUsername;
        streamConfig.turnPassword = config.turnPassword;
        streamConfig.targetBitrate = config.targetBitrate;
        streamConfig.maxFrameRate = config.maxFramerate;
        streamConfig.codecName = config.codec;
        streamConfig.width = config.width;
        streamConfig.height = config.height;
        streamConfig.encodingThreads = config.encodingThreads;
        
        _webrtcStreamer = std::make_unique<u_webrtc::WebRTCStreamer>(streamConfig);
        
        // 初始化 WebRTC 流管理器
        if (!_webrtcStreamer->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "WebRTC 流管理器初始化失败");
            throw std::runtime_error("WebRTC initialization failed");
        }
        
        // 连接到信令服务器
        if (_webrtcStreamer->connectToSignalingServer()) {
            RCLCPP_INFO(this->get_logger(), "已连接到信令服务器");
            
            // 创建 Offer 并发送
            _webrtcStreamer->createOffer();
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "无法连接到信令服务器，将在接收到图像数据后重试");
        }
        
        // 创建图像订阅器（使用 std::shared_ptr 来共享 this 指针）
        _imageSubscriber = std::make_shared<u_webrtc::ImageSubscriber>(
            "image_sub", 
            config.imageTopic
        );
        
        // 设置图像回调
        _imageSubscriber->setImageCallback(
            std::bind(&WebRTCStreamNode::onImageReceived, 
                     this, 
                     std::placeholders::_1)
        );
        
        // 创建定时器，定期检查连接状态
        _statusTimer = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&WebRTCStreamNode::onStatusTimer, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "WebRTC 视频流节点启动成功");
    }
    
private:
    /**
     * @brief 图像接收回调
     */
    void onImageReceived(const sensor_msgs::msg::Image::SharedPtr msg) {
        _frameCount++;
        
        // 转换图像格式
        auto frame = _imageConverter->convertToWebRTCFormat(msg);
        
        if (!frame) {
            RCLCPP_ERROR(this->get_logger(), "图像格式转换失败");
            return;
        }
        
        // 推送到 WebRTC 流
        if (_webrtcStreamer->pushFrame(*frame)) {
            _successCount++;
        }
        
        // 每 100 帧输出一次统计信息
        if (_frameCount % 100 == 0) {
            double successRate = static_cast<double>(_successCount) / _frameCount * 100.0;
            RCLCPP_INFO(this->get_logger(), 
                       "已处理 %ld 帧，成功率: %.2f%%", 
                       _frameCount, successRate);
        }
    }
    
    /**
     * @brief 状态检查定时器回调
     */
    void onStatusTimer() {
        if (!_webrtcStreamer->isConnected()) {
            RCLCPP_WARN(this->get_logger(), "WebRTC 连接未建立");
            
            // 尝试重新连接
            if (_webrtcStreamer->connectToSignalingServer()) {
                _webrtcStreamer->createOffer();
            }
        } else {
            RCLCPP_INFO(this->get_logger(), 
                       "WebRTC 连接正常，已处理 %ld 帧", _frameCount);
        }
    }
    
    std::shared_ptr<u_webrtc::ImageSubscriber> _imageSubscriber;
    std::unique_ptr<u_webrtc::ImageConverter> _imageConverter;
    std::unique_ptr<u_webrtc::WebRTCStreamer> _webrtcStreamer;
    rclcpp::TimerBase::SharedPtr _statusTimer;
    
    int64_t _frameCount{0};
    int64_t _successCount{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<WebRTCStreamNode>();
        
        // 使用 MultiThreadedExecutor 提高性能
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "节点异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}



