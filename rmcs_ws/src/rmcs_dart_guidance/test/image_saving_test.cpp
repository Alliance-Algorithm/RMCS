#include <chrono>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance {

class TestImageSaver : public rmcs_executor::Component, public rclcpp::Node {
public:
    TestImageSaver() 
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        
        // 读取测试参数
        std::string save_dir = "./test_images";
        if (has_parameter("save_directory")) {
            save_dir = get_parameter("save_directory").as_string();
        }
        
        int test_count = 5;
        if (has_parameter("test_count")) {
            test_count = static_cast<int>(get_parameter("test_count").as_int());
        }
        
        int interval_ms = 1000;
        if (has_parameter("interval_ms")) {
            interval_ms = static_cast<int>(get_parameter("interval_ms").as_int());
        }
        
        RCLCPP_INFO(logger_, "Starting image saver test:");
        RCLCPP_INFO(logger_, "  Directory: %s", save_dir.c_str());
        RCLCPP_INFO(logger_, "  Count: %d", test_count);
        RCLCPP_INFO(logger_, "  Interval: %d ms", interval_ms);
        
        // 创建目录
        try {
            std::filesystem::create_directories(save_dir);
            RCLCPP_INFO(logger_, "Created directory: %s", save_dir.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to create directory: %s", e.what());
            return;
        }
        
        // 生成测试图像并保存
        for (int i = 0; i < test_count; ++i) {
            test_save_image(save_dir, i);
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
        
        RCLCPP_INFO(logger_, "Test completed. Check directory: %s", save_dir.c_str());
    }
    
    void update() override {
        // 测试完成后停止更新
    }
    
private:
    void test_save_image(const std::string& directory, int index) {
        // 创建测试图像
        cv::Mat test_image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // 添加一些内容
        cv::rectangle(test_image, cv::Point(100, 100), cv::Point(540, 380), cv::Scalar(0, 255, 0), 2);
        cv::circle(test_image, cv::Point(320, 240), 50, cv::Scalar(255, 0, 0), -1);
        cv::putText(test_image, "Test " + std::to_string(index), cv::Point(200, 400),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        
        // 生成文件名
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&time_t, &tm_buf);
        
        std::ostringstream oss;
        oss << directory << "/test_" 
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") 
            << "_" << std::setw(3) << std::setfill('0') << index
            << ".png";
        
        std::string filename = oss.str();
        
        // 保存图像
        try {
            bool success = cv::imwrite(filename, test_image);
            if (success) {
                RCLCPP_INFO(logger_, "Saved test image: %s", filename.c_str());
                
                // 验证文件存在
                if (std::filesystem::exists(filename)) {
                    auto file_size = std::filesystem::file_size(filename);
                    RCLCPP_INFO(logger_, "  File size: %ld bytes", file_size);
                } else {
                    RCLCPP_ERROR(logger_, "  File not found after saving!");
                }
            } else {
                RCLCPP_ERROR(logger_, "Failed to save image: %s", filename.c_str());
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(logger_, "OpenCV exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Exception: %s", e.what());
        }
    }
    
    rclcpp::Logger logger_;
};

} // namespace rmcs_dart_guidance

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::TestImageSaver, rmcs_executor::Component)