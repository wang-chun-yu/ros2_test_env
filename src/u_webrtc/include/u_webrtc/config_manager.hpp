#ifndef U_WEBRTC_CONFIG_MANAGER_HPP
#define U_WEBRTC_CONFIG_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace u_webrtc {

/**
 * @brief 配置管理器，从 ROS2 参数中读取配置
 */
class ConfigManager {
public:
    struct Config {
        // ROS2 配置
        std::string imageTopic;
        int qosDepth;
        
        // WebRTC 配置
        std::string signalingServerUrl;
        std::string stunServer;
        std::string turnServer;
        std::string turnUsername;
        std::string turnPassword;
        
        // 视频编码配置
        std::string codec;
        int targetBitrate;
        int maxFramerate;
        int width;
        int height;
        
        // 性能配置
        int encodingThreads;
        bool useHardwareEncoding;
    };
    
    /**
     * @brief 从 ROS2 节点参数加载配置
     * @param node ROS2 节点指针
     * @return 配置对象
     */
    static Config loadFromNode(rclcpp::Node* node);
    
    /**
     * @brief 声明所有 ROS2 参数（带默认值）
     * @param node ROS2 节点指针
     */
    static void declareParameters(rclcpp::Node* node);
};

} // namespace u_webrtc

#endif // U_WEBRTC_CONFIG_MANAGER_HPP



