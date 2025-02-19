
#include "controller/dart/dart_resource.hpp"
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
        profile_.gain          = 0;
        capture_               = std::make_unique<hikcamera::ImageCapturer>(profile_);

        latest_frame_.init();
        camera_read_thread_ = std::thread(&DartCamera::camera_read, this);
    }

    void update() override {
        if (!latest_frame_.image.empty()) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            *dart_camera_frame_ = latest_frame_;
        }
    }

private:
    void camera_read() {
        while (camera_control_mode_ == ControllerState::Enable) {

            cv::Mat image_buffer = capture_->read();

            std::lock_guard<std::mutex> lock(buffer_mutex_);
            latest_frame_.image = image_buffer;
            latest_frame_.id    = (latest_frame_.id + 1) % 0xFF;
        }
    }

    std::thread camera_read_thread_;
    ControllerState camera_control_mode_ = ControllerState::Enable;
    CameraFrame latest_frame_;
    std::mutex buffer_mutex_;

    rclcpp::Logger logger_;

    hikcamera::ImageCapturer::CameraProfile profile_;
    std::unique_ptr<hikcamera::ImageCapturer> capture_;

    OutputInterface<CameraFrame> dart_camera_frame_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartCamera, rmcs_executor::Component)