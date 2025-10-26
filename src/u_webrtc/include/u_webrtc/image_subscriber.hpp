#ifndef U_WEBRTC_IMAGE_SUBSCRIBER_HPP
#define U_WEBRTC_IMAGE_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <functional>
#include <string>

namespace u_webrtc {

/**
 * @brief ROS2 图像订阅器，用于接收 sensor_msgs/msg/Image 消息
 */
class ImageSubscriber : public rclcpp::Node {
public:
    using ImageCallback = std::function<void(const sensor_msgs::msg::Image::SharedPtr)>;
    
    /**
     * @brief 构造函数
     * @param nodeName 节点名称
     * @param topicName 订阅的话题名称
     */
    explicit ImageSubscriber(const std::string& nodeName, 
                            const std::string& topicName);
    
    /**
     * @brief 设置图像回调函数
     * @param callback 用户定义的回调函数
     */
    void setImageCallback(ImageCallback callback);
    
private:
    /**
     * @brief 内部图像回调函数
     * @param msg 接收到的图像消息
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _subscription;
    ImageCallback _userCallback;
    std::string _topicName;
};

} // namespace u_webrtc

#endif // U_WEBRTC_IMAGE_SUBSCRIBER_HPP



