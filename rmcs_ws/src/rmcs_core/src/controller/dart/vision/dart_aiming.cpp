
#include <eigen3/Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

namespace rmcs_core::controller::dart {

class DartAiming
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAiming()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/vision/camera_frame", dart_camera_frame_);
        register_output("/dart/vision/display_image", display_image_);

        image_show_enable_ = get_parameter("image_show_enable").as_bool();

        lowerlimit = cv::Scalar(
            get_parameter("lowerlimit_H").as_double(), get_parameter("lowerlimit_L").as_double(),
            get_parameter("lowerlimit_S").as_double());
        upperlimit = cv::Scalar(
            get_parameter("upperlimit_H").as_double(), get_parameter("upperlimit_L").as_double(),
            get_parameter("upperlimit_S").as_double());
    }

    void update() override {
        camera_image_ = *dart_camera_frame_;
        identifier(camera_image_);
    }

private:
    void update_errors() {}

    void identifier(const cv::Mat& camera_image_) {
        processed_image_ = image_processing(camera_image_);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(processed_image_, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat display = camera_image_.clone();
        for (const auto& contour : contours) {
            cv::RotatedRect minRect = cv::minAreaRect(contour);

            cv::Point2f rectPoints[4];
            minRect.points(rectPoints);

            for (int i = 0; i < 4; i++) {
                cv::line(display, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(255, 0, 255), 1);
            }

            target_position_ = minRect.center;
            cv::circle(display, target_position_, 2, cv::Scalar(0, 0, 255), -1);
            RCLCPP_INFO(logger_, "(%lf,%lf)", target_position_.x, target_position_.y);
        }

        *display_image_ = display;
    }

    cv::Mat image_processing(const cv::Mat& input) {
        cv::Mat HSV_image;
        cv::cvtColor(input, HSV_image, cv::COLOR_BGR2HLS);
        cv::Mat processing;
        cv::inRange(HSV_image, lowerlimit, upperlimit, processing);

        static cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::morphologyEx(processing, processing, cv::MORPH_OPEN, open_kernel);

        int kernel_size              = 8;
        static cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::dilate(processing, processing, dilate_kernel);

        return processing;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    bool image_show_enable_ = false;
    cv::Scalar lowerlimit;
    cv::Scalar upperlimit;

    cv::Mat camera_image_;
    cv::Mat processed_image_;
    cv::Point2d target_position_;

    InputInterface<cv::Mat> dart_camera_frame_;

    OutputInterface<cv::Mat> display_image_;
    OutputInterface<Eigen::Vector3d> error_vector_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartAiming, rmcs_executor::Component)