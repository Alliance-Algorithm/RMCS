
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rmcs_core::controller::dart {

class ImageSubscriber
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ImageSubscriber()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        register_input("/dart/camera/frame", frame_);
        register_input("/dart/camera/processed_image", processed_image_);
    }
    void update() override {
        if (frame_->empty()) {
            RCLCPP_WARN(logger_, "dart::ImageSubscriber : No image");
            return;
        }
        cv::imshow("camera", *frame_);
        cv::imshow("processed", *processed_image_);
        cv::waitKey(1);
    };

private:
    rclcpp::Logger logger_;
    InputInterface<cv::Mat> frame_;
    InputInterface<cv::Mat> processed_image_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::ImageSubscriber, rmcs_executor::Component)
