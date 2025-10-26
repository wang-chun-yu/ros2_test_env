#ifndef U_WEBRTC_IMAGE_CONVERTER_HPP
#define U_WEBRTC_IMAGE_CONVERTER_HPP

#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <optional>
#include <string>

namespace u_webrtc {

/**
 * @brief 图像格式转换器，将 ROS Image 转换为 WebRTC 可用格式
 */
class ImageConverter {
public:
    struct ConvertedFrame {
        std::vector<uint8_t> data;
        int width;
        int height;
        std::string format; // "I420", "RGB", etc.
    };
    
    ImageConverter() = default;
    ~ImageConverter() = default;
    
    /**
     * @brief 将 ROS Image 转换为 WebRTC 可用格式
     * @param rosImage ROS 图像消息
     * @return 转换后的帧数据
     */
    std::optional<ConvertedFrame> convertToWebRTCFormat(
        const sensor_msgs::msg::Image::SharedPtr& rosImage);
    
    /**
     * @brief 转换为 I420 格式（WebRTC 常用格式）
     * @param image OpenCV Mat 格式的图像
     * @return 转换后的 I420 格式帧数据
     */
    std::optional<ConvertedFrame> convertToI420(const cv::Mat& image);
    
private:
    /**
     * @brief 将 ROS Image 转换为 OpenCV Mat
     * @param rosImage ROS 图像消息
     * @return OpenCV Mat 格式的图像
     */
    cv::Mat rosImageToMat(const sensor_msgs::msg::Image::SharedPtr& rosImage);
};

} // namespace u_webrtc

#endif // U_WEBRTC_IMAGE_CONVERTER_HPP



