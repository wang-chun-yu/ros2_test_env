#include "u_webrtc/image_subscriber.hpp"

namespace u_webrtc {

ImageSubscriber::ImageSubscriber(const std::string& nodeName, 
                                const std::string& topicName)
    : rclcpp::Node(nodeName), _topicName(topicName) {
    
    // 创建订阅器，使用 RELIABLE QoS 以兼容相机话题
    // 注意：大多数相机节点使用 RELIABLE QoS，所以我们也使用 RELIABLE 以确保兼容
    auto qos = rclcpp::QoS(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        _topicName,
        qos,
        std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), 
                "ImageSubscriber 已创建，订阅话题: %s (RELIABLE QoS)", _topicName.c_str());
}

void ImageSubscriber::setImageCallback(ImageCallback callback) {
    _userCallback = std::move(callback);
}

void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "接收到空图像消息");
        return;
    }
    
    // 使用 DEBUG 级别日志，避免每帧都输出
    RCLCPP_DEBUG(this->get_logger(), 
                "接收到图像: %dx%d, 编码: %s", 
                msg->width, msg->height, msg->encoding.c_str());
    
    // 调用用户定义的回调函数
    if (_userCallback) {
        _userCallback(msg);
    }
}

} // namespace u_webrtc



