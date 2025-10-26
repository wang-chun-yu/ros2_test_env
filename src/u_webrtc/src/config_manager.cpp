#include "u_webrtc/config_manager.hpp"

namespace u_webrtc {

void ConfigManager::declareParameters(rclcpp::Node* node) {
    // ROS2 配置
    node->declare_parameter("image_topic", "/camera/image_raw");
    node->declare_parameter("qos_depth", 10);
    
    // WebRTC 配置
    node->declare_parameter("signaling_server_url", "ws://localhost:8080");
    node->declare_parameter("stun_server", "stun:stun.l.google.com:19302");
    node->declare_parameter("turn_server", "");
    node->declare_parameter("turn_username", "");
    node->declare_parameter("turn_password", "");
    
    // 视频编码配置
    node->declare_parameter("codec", "VP8");
    node->declare_parameter("target_bitrate", 2000000);  // 2 Mbps
    node->declare_parameter("max_framerate", 30);
    node->declare_parameter("width", 1280);
    node->declare_parameter("height", 720);
    
    // 性能配置
    node->declare_parameter("encoding_threads", 4);
    node->declare_parameter("use_hardware_encoding", true);
}

ConfigManager::Config ConfigManager::loadFromNode(rclcpp::Node* node) {
    Config config;
    
    // ROS2 配置
    config.imageTopic = node->get_parameter("image_topic").as_string();
    config.qosDepth = node->get_parameter("qos_depth").as_int();
    
    // WebRTC 配置
    config.signalingServerUrl = node->get_parameter("signaling_server_url").as_string();
    config.stunServer = node->get_parameter("stun_server").as_string();
    config.turnServer = node->get_parameter("turn_server").as_string();
    config.turnUsername = node->get_parameter("turn_username").as_string();
    config.turnPassword = node->get_parameter("turn_password").as_string();
    
    // 视频编码配置
    config.codec = node->get_parameter("codec").as_string();
    config.targetBitrate = node->get_parameter("target_bitrate").as_int();
    config.maxFramerate = node->get_parameter("max_framerate").as_int();
    config.width = node->get_parameter("width").as_int();
    config.height = node->get_parameter("height").as_int();
    
    // 性能配置
    config.encodingThreads = node->get_parameter("encoding_threads").as_int();
    config.useHardwareEncoding = node->get_parameter("use_hardware_encoding").as_bool();
    
    return config;
}

} // namespace u_webrtc



