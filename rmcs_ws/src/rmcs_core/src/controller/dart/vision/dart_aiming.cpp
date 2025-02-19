
#include "controller/dart/dart_resource.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
#include <vector>

namespace rmcs_core::controller::dart {

class AimController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AimController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , yaw_aim_position_(get_parameter("yaw_aim_position").as_double())
        , pitch_aim_position_(get_parameter("pitch_aim_position").as_double()) {

        register_input("/dart/vision/camera_frame", dart_camera_frame_);
        register_output("/dart/vision/display_image", display_image_);
        register_output("/dart/vision/error_vector", error_vector_, Eigen::Vector2d::Zero());

        lowerlimit = cv::Scalar(
            get_parameter("lowerlimit_H").as_double(), get_parameter("lowerlimit_L").as_double(),
            get_parameter("lowerlimit_S").as_double());
        upperlimit = cv::Scalar(
            get_parameter("upperlimit_H").as_double(), get_parameter("upperlimit_L").as_double(),
            get_parameter("upperlimit_S").as_double());

        camera_frame_.init();
        identify_thread_ = std::thread(&AimController::dart_guidance, this);
    }

    void update() override {
        camera_frame_ = *dart_camera_frame_;

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        error_vector_->x() = latest_target_position_.x - yaw_aim_position_;
        error_vector_->y() = latest_target_position_.y - pitch_aim_position_;
    }

private:
    void dart_guidance() {
        while (guidance_enable_) {
            if (camera_frame_.id == latest_frame_id_) {
                continue;
            }

            identifier(camera_frame_.image);
            latest_frame_id_ = dart_camera_frame_->id;
        }
    }

    void identifier(const cv::Mat& camera_image_) {
        processed_image_ = image_processing(camera_image_);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(processed_image_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat display   = camera_image_.clone();
        int target_number = 0;
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);

            if (area < 16)
                continue;
            else
                target_number++;

            cv::RotatedRect minRect = cv::minAreaRect(contour);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            for (int i = 0; i < 4; i++) {
                cv::line(display, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
            }
            if (target_number == 1) {
                std::lock_guard<std::mutex> lock(buffer_mutex_);
                latest_target_position_ = minRect.center;
                cv::circle(display, latest_target_position_, 2, cv::Scalar(255, 0, 255), -1);
            }
        }

        // TODO: 当检测到多个目标的处理代码

        // RCLCPP_INFO(
        //     logger_, "contours:%zu,target:%d,position:(%d,%d),error:(%lf,%lf)", contours.size(), target_number,
        //     target_position_.x, target_position_.y, error_vector_->x(), error_vector_->y());

        display_image_->image = display;
        display_image_->id    = latest_frame_id_;
    }

    cv::Mat image_processing(const cv::Mat& input) {
        cv::Mat HLS;
        cv::cvtColor(input, HLS, cv::COLOR_BGR2HLS);
        cv::Mat process;
        cv::inRange(HLS, lowerlimit, upperlimit, process);

        static cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::morphologyEx(process, process, cv::MORPH_OPEN, open_kernel);

        int kernel_size              = 8;
        static cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(process, process, dilate_kernel);

        return process;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;
    const double yaw_aim_position_;
    const double pitch_aim_position_;
    cv::Scalar lowerlimit;
    cv::Scalar upperlimit;

    CameraFrame camera_frame_;
    long latest_frame_id_;
    cv::Mat processed_image_;
    cv::Point latest_target_position_;

    InputInterface<CameraFrame> dart_camera_frame_;

    OutputInterface<CameraFrame> display_image_;
    OutputInterface<Eigen::Vector2d> error_vector_;

    bool guidance_enable_ = true;
    std::thread identify_thread_;
    std::mutex buffer_mutex_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AimController, rmcs_executor::Component)