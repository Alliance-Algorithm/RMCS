
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::controller::dart {

class DartCamera
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartCamera()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_output("/dart/camera/frame", dart_camera_frame_);

        image_show_enable_     = get_parameter("image_show_enable").as_bool();
        profile_.invert_image  = get_parameter("invert_image").as_bool();
        profile_.exposure_time = std::chrono::microseconds(get_parameter("exposure_time").as_int());
        profile_.gain          = 16.9807;
        capture_               = std::make_unique<hikcamera::ImageCapturer>(profile_);
    }

    void update() override {
        *dart_camera_frame_ = capture_->read();

        if (image_show_enable_) {
            if (dart_camera_frame_->empty()) {
                RCLCPP_WARN(logger_, "DartCamera: can't read image");
                return;
            }
            cv::imshow("camera image", *dart_camera_frame_);
            cv::waitKey(1);
        }
    };

private:
    rclcpp::Logger logger_;
    hikcamera::ImageCapturer::CameraProfile profile_;
    std::unique_ptr<hikcamera::ImageCapturer> capture_;
    bool image_show_enable_ = false;

    OutputInterface<cv::Mat> dart_camera_frame_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartCamera, rmcs_executor::Component)