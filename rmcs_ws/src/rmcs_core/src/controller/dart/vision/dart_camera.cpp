
#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>

namespace rmcs_core::controller::dart {

class DartCamera
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartCamera()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_output("/dart/vision/camera_frame", dart_camera_frame_);

        profile_.invert_image  = get_parameter("invert_image").as_bool();
        profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        profile_.gain          = 16.9807;
        capture_               = std::make_unique<hikcamera::ImageCapturer>(profile_);

        frame_buffer_[0]    = cv::Mat(720, 1440, CV_8UC3, cv::Scalar(0, 0, 0));
        camera_read_thread_ = std::thread(&DartCamera::camera_read, this);
    }

    void update() override {
        if (!frame_buffer_[0].empty()) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            *dart_camera_frame_ = frame_buffer_[0];
        }
    }

private:
    void camera_read() {
        while (camera_enable_) {
            frame_buffer_[1] = capture_->read();

            std::lock_guard<std::mutex> lock(buffer_mutex_);
            frame_buffer_[0] = frame_buffer_[1];
        }
    }

    std::thread camera_read_thread_;
    bool camera_enable_ = true;
    cv::Mat frame_buffer_[2];
    std::mutex buffer_mutex_;

    rclcpp::Logger logger_;

    hikcamera::ImageCapturer::CameraProfile profile_;
    std::unique_ptr<hikcamera::ImageCapturer> capture_;

    OutputInterface<cv::Mat> dart_camera_frame_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartCamera, rmcs_executor::Component)