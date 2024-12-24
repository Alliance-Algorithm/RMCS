/*
    镖架视觉
    施工中
*/
#include "hikcamera/image_capturer.hpp"
#include "image_processing.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class VisualGuidance
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VisualGuidance()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        debug_mode_ = get_parameter("debug_mode").as_bool();

        register_output("/dart/firction/working_velocity", friction_working_velocity_, nan);
        register_output("/dart/yaw/control_angle_error", yaw_error_, nan);
        register_output("/dart/camera/frame", camera_image_);
        register_output("/dart/camera/processed_image", processed_image_);

        // profile.invert_image  = get_parameter("invert_image").as_bool();
        // profile.exposure_time =
        // std::chrono::milliseconds(get_parameter("exposure_time").as_int()); profile.gain =
        // static_cast<float>(get_parameter("gain").as_double());
        capturer = std::make_unique<hikcamera::ImageCapturer>(profile);
    }

    void update() override {
        frame_         = capturer->read();
        *camera_image_ = frame_;
        update_target_position(frame_);
        update_yaw_control_errors();

        if (angle_ready_ == true) {
            calculate_relative_distance();
            calculate_launch_velocity();
        }
    }

private:
    void update_target_position(cv::Mat& image) {
        if (image.empty()) {
            RCLCPP_WARN(logger_, "VisualGuidance::update_target_position : image not load");
            return;
        }

        cv::Mat processed_image;
        ImageProcess::hybrid_image_processing(frame_, processed_image);
        // ImageProcess::image_to_brightMask(frame_, processed_image);
        std::vector<cv::Point> target = ImageProcess::target_find(processed_image);
        *processed_image_             = processed_image;

        if (target.size() == 0) {
            // RCLCPP_WARN(logger_, "VisualGuidance::update_target_position : target not found");
            return;
        }
        if (target.size() > 1) {
            RCLCPP_WARN(logger_, "VisualGuidance::update_target_position : found multiple targets");
            return;
        }

        target_position_ = target.front();
        RCLCPP_INFO(logger_, "position : %d,%d", target_position_.x, target_position_.y);
    }

    void update_yaw_control_errors() {
        double mid_col = frame_.cols / 2.0;
        *yaw_error_    = target_position_.x - mid_col;
    }

    void calculate_relative_distance() {}
    void calculate_launch_velocity() {}

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    cv::Mat frame_;
    bool debug_mode_;
    bool angle_ready_ = false;

    cv::Point target_position_;

    OutputInterface<double> yaw_error_;
    OutputInterface<double> friction_working_velocity_;
    OutputInterface<cv::Mat> camera_image_;
    OutputInterface<cv::Mat> processed_image_;

    hikcamera::ImageCapturer::CameraProfile profile;
    std::unique_ptr<hikcamera::ImageCapturer> capturer;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::VisualGuidance, rmcs_executor::Component)