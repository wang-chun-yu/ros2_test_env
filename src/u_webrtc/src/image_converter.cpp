#include "u_webrtc/image_converter.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

namespace u_webrtc {

std::optional<ImageConverter::ConvertedFrame> 
ImageConverter::convertToWebRTCFormat(const sensor_msgs::msg::Image::SharedPtr& rosImage) {
    try {
        // 将 ROS Image 转换为 OpenCV Mat
        cv::Mat image = rosImageToMat(rosImage);
        
        if (image.empty()) {
            return std::nullopt;
        }
        
        // 转换为 I420 格式（WebRTC 标准格式）
        return convertToI420(image);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ImageConverter"), 
                    "图像转换失败: %s", e.what());
        return std::nullopt;
    }
}

std::optional<ImageConverter::ConvertedFrame> 
ImageConverter::convertToI420(const cv::Mat& image) {
    if (image.empty()) {
        return std::nullopt;
    }
    
    ConvertedFrame frame;
    frame.width = image.cols;
    frame.height = image.rows;
    frame.format = "I420";
    
    cv::Mat yuv;
    
    // 根据输入格式进行转换
    if (image.channels() == 3) {
        // BGR 转 YUV I420
        cv::cvtColor(image, yuv, cv::COLOR_BGR2YUV_I420);
    } else if (image.channels() == 4) {
        // BGRA 转 BGR 再转 YUV
        cv::Mat bgr;
        cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
        cv::cvtColor(bgr, yuv, cv::COLOR_BGR2YUV_I420);
    } else if (image.channels() == 1) {
        // 灰度图，创建 Y 平面，U 和 V 平面填充 128
        yuv = cv::Mat(image.rows * 3 / 2, image.cols, CV_8UC1);
        
        // 复制 Y 平面
        image.copyTo(yuv(cv::Rect(0, 0, image.cols, image.rows)));
        
        // 填充 U 和 V 平面
        cv::Mat uv = yuv(cv::Rect(0, image.rows, image.cols, image.rows / 2));
        uv.setTo(cv::Scalar(128));
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ImageConverter"), 
                    "不支持的图像通道数: %d", image.channels());
        return std::nullopt;
    }
    
    // 将数据复制到 vector
    frame.data.assign(yuv.data, yuv.data + yuv.total() * yuv.elemSize());
    
    return frame;
}

cv::Mat ImageConverter::rosImageToMat(const sensor_msgs::msg::Image::SharedPtr& rosImage) {
    try {
        // 使用 cv_bridge 转换
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(rosImage, rosImage->encoding);
        return cvPtr->image;
        
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ImageConverter"), 
                    "cv_bridge 转换失败: %s", e.what());
        return cv::Mat();
    }
}

} // namespace u_webrtc



