
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::dart {

class DartAiming
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAiming()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/camera/frame", dart_camera_frame_);

        register_output("/dart/yaw/control_angle_error", yaw_error_, nan);
        register_output("/dart/pitch_left/control_angle_error", pitch_left_error_, nan);
        register_output("/dart/pitch_right/control_angle_error", pitch_right_error_, nan);

        register_output("/dart/vision/processed_image", display_image_);

        image_show_enable_ = get_parameter("image_show_enable").as_bool();
        lowerlimit[0]      = get_parameter("lowerlimit_H").as_double();
        lowerlimit[1]      = get_parameter("lowerlimit_L").as_double();
        lowerlimit[2]      = get_parameter("lowerlimit_S").as_double();
        upperlimit[0]      = get_parameter("upperlimit_H").as_double();
        upperlimit[1]      = get_parameter("upperlimit_L").as_double();
        upperlimit[2]      = get_parameter("upperlimit_S").as_double();
    }

    void update() override {
        camera_image_    = *dart_camera_frame_;
        processed_image_ = image_processing(camera_image_);
        *display_image_  = processed_image_;
    }

private:
    void update_errors() {}
    void guide_light_identifier() {}

    cv::Mat image_processing(const cv::Mat& input) {
        cv::Mat HSV_image;
        cv::cvtColor(input, HSV_image, cv::COLOR_BGR2HLS);
        cv::Mat processing;
        cv::inRange(HSV_image, lowerlimit, upperlimit, processing);
        static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(processing, processing, cv::MORPH_OPEN, kernel);

        return processing;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Logger logger_;

    bool image_show_enable_ = false;
    cv::Scalar lowerlimit;
    cv::Scalar upperlimit;
    cv::Mat camera_image_;
    cv::Mat processed_image_;

    InputInterface<cv::Mat> dart_camera_frame_;

    OutputInterface<cv::Mat> display_image_;
    OutputInterface<double> yaw_error_;
    OutputInterface<double> pitch_left_error_;
    OutputInterface<double> pitch_right_error_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartAiming, rmcs_executor::Component)