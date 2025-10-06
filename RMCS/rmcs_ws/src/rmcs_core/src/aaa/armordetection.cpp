#include "identify.hpp"
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::detect{
class ArmorDetection
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmorDetection()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        camera_profile_.invert_image  = get_parameter("invert_image").as_bool();
        camera_profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        camera_profile_.gain          = static_cast<float>(get_parameter("gain").as_double());
        image_capture_                = std::make_unique<hikcamera::ImageCapturer>(camera_profile_);
    
        // 读取HSV颜色阈值参数并创建颜色范围
        std::vector<ColorRange> color_ranges;
        
        // 红色范围1
        ColorRange red1;
        red1.lower = cv::Scalar(get_parameter("red1_L_H").as_double(), get_parameter("red1_L_S").as_double(), get_parameter("red1_L_V").as_double());
        red1.upper = cv::Scalar(get_parameter("red1_U_H").as_double(), get_parameter("red1_U_S").as_double(), get_parameter("red1_U_V").as_double());
        red1.colorName = "Red";
        color_ranges.push_back(red1);

        // 红色范围2
        ColorRange red2;
        red2.lower = cv::Scalar(get_parameter("red2_L_H").as_double(), get_parameter("red2_L_S").as_double(), get_parameter("red2_L_V").as_double());
        red2.upper = cv::Scalar(get_parameter("red2_U_H").as_double(), get_parameter("red2_U_S").as_double(), get_parameter("red2_U_V").as_double());
        red2.colorName = "Red";
        color_ranges.push_back(red2);

        // 蓝色范围
        ColorRange blue;
        blue.lower = cv::Scalar(get_parameter("blue_L_H").as_double(), get_parameter("blue_L_S").as_double(), get_parameter("blue_L_V").as_double());
        blue.upper = cv::Scalar(get_parameter("blue_U_H").as_double(), get_parameter("blue_U_S").as_double(), get_parameter("blue_U_V").as_double());
        blue.colorName = "Blue";
        color_ranges.push_back(blue);

        // 白色范围（可选）
        ColorRange white;
        white.lower = cv::Scalar(get_parameter("white_L_H").as_double(), get_parameter("white_L_S").as_double(), get_parameter("white_L_V").as_double());
        white.upper = cv::Scalar(get_parameter("white_U_H").as_double(), get_parameter("white_U_S").as_double(), get_parameter("white_U_V").as_double());
        white.colorName = "White";
        color_ranges.push_back(white);

        // 设置识别器的颜色范围
        identifier_.setColorRanges(color_ranges);
        
        register_output("/armor_detect/camera/camera_image", camera_image_);
        register_output("/armor_detect/camera/display_image", display_image_);

        camera_thread_ = std::thread(&ArmorDetection::camera_update, this);
    }

    ~ArmorDetection() {
        // 确保线程安全退出
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }
    }

    void update() override {
        // 主更新函数，如果需要可以在这里添加逻辑
    }

private:
    void camera_update() {
        while (rclcpp::ok()) {
            cv::Mat read = image_capture_->read();
            if (read.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            // 输出原始相机图像
            *camera_image_ = read;
            
            // 创建显示图像副本
            cv::Mat display = read.clone();
            
            // 进行装甲板识别
            identifier_.update(display);
            
            // 输出处理后的显示图像
            *display_image_ = display;
            
            // 添加适当的延迟，避免过度占用CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    rclcpp::Logger logger_;

    std::thread camera_thread_;
    hikcamera::ImageCapturer::CameraProfile camera_profile_;
    std::unique_ptr<hikcamera::ImageCapturer> image_capture_;

    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> display_image_;

    Armor_Identifier identifier_;
};
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::detect::ArmorDetection, rmcs_executor::Component)