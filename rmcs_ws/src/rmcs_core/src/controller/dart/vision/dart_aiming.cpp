
#include "controller/dart/dart_resource.hpp"
#include <cmath>
#include <cstdlib>
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

        latest_target_position_ = cv::Point2d(yaw_aim_position_, pitch_aim_position_);

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
        identify_thread_ = std::thread(&AimController::camera_frame_process, this);
    }

    void update() override {
        camera_frame_ = *dart_camera_frame_;

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        error_vector_->x() = latest_target_position_.x - yaw_aim_position_;
        error_vector_->y() = latest_target_position_.y - pitch_aim_position_;
    }

private:
    void camera_frame_process() {
        while (guidance_enable_) {
            if (camera_frame_.id == latest_frame_id_) {
                continue;
            }

            identifier(camera_frame_.image);
            latest_frame_id_ = dart_camera_frame_->id;
        }
    }

    cv::Mat image_process(const cv::Mat& input) {
        cv::Mat HLS;
        cv::cvtColor(input, HLS, cv::COLOR_BGR2HLS);
        cv::Mat process;
        cv::inRange(HLS, lowerlimit, upperlimit, process);

        static cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(process, process, cv::MORPH_OPEN, open_kernel);

        int kernel_size              = 7;
        static cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(process, process, dilate_kernel);
        //
        return process;
    }

    void identifier(const cv::Mat& camera_image_) {
        processed_image_ = image_process(camera_image_);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(processed_image_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat display = camera_image_;
        std::vector<cv::Point> possible_targets;

        // 对每一个识别出的轮廓进行筛选
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);

            // 面积筛选
            if (area <= 64)
                continue;

            double perimeter        = cv::arcLength(contour, true);
            cv::RotatedRect minRect = cv::minAreaRect(contour);
            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            double a = sqrt(pow(rectPoints[1].x - rectPoints[0].x, 2) + pow(rectPoints[1].y - rectPoints[0].y, 2));
            double b = sqrt(pow(rectPoints[1].x - rectPoints[2].x, 2) + pow(rectPoints[1].y - rectPoints[2].y, 2));

            // 形状和长宽筛选
            if (perimeter > 2 * (a + b))
                continue;

            if (a / b > 1.5 || b / a > 1.5)
                continue;

            // 用最小矩形框出轮廓
            for (int i = 0; i < 4; i++) {
                cv::line(display, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
            }
            possible_targets.emplace_back(minRect.center);
        }

        if (possible_targets.empty()) {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            latest_target_position_ = cv::Point2d(yaw_aim_position_, pitch_aim_position_);
            last_possible_targets_.clear();
        }

        if (possible_targets.size() == 1) {
            if (last_possible_targets_.size() == 0) {
                last_possible_targets_ = possible_targets;
                return;
            }

            std::lock_guard<std::mutex> lock(buffer_mutex_);
            latest_target_position_ = possible_targets.front();
            cv::circle(display, latest_target_position_, 2, cv::Scalar(255, 0, 255), -1);
        }

        if (possible_targets.size() >= 2) {
            // TODO: 当检测到多个目标的处理代码
        }

        // RCLCPP_INFO(
        //     logger_, "num:%zu,error:(%lf,%lf)", possible_targets.size(), error_vector_->x(), error_vector_->y());

        display_image_->image = display;
        display_image_->id    = latest_frame_id_;
    }
    std::vector<cv::Point> last_possible_targets_;

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