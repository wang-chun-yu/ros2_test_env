#include "u_webrtc/image_subscriber.hpp"

namespace u_webrtc {

ImageSubscriber::ImageSubscriber(const std::string& nodeName, 
                                const std::string& topicName)
    : rclcpp::Node(nodeName), _topicName(topicName) {
    
    // 创建订阅器，使用传感器数据 QoS 配置
    _subscription = this->create_subscription<sensor_msgs::msg::Image>(
        _topicName,
        rclcpp::SensorDataQoS(),
        std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), 
                "ImageSubscriber 已创建，订阅话题: %s", _topicName.c_str());
}

void ImageSubscriber::setImageCallback(ImageCallback callback) {
    _userCallback = std::move(callback);
}

void ImageSubscriber::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "接收到空图像消息");
        return;
    }
    
    // 调用用户定义的回调函数
    if (_userCallback) {
        _userCallback(msg);
    } else {
        RCLCPP_DEBUG(this->get_logger(), 
                    "接收到图像: %dx%d, 编码: %s", 
                    msg->width, msg->height, msg->encoding.c_str());
    }
}

} // namespace u_webrtc



