#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core/cvdef.h>
#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>

namespace rmcs_dart_guidance {

class ImagePublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ImagePublisher()
        : Node(get_component_name(), rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input(get_parameter("Interface_name").as_string(), input_image_);

        image_publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>(get_parameter("topic_name").as_string(), 1000);

        publish_freq_ = get_parameter("publish_freq").as_double();
        publish_freq_ = MAX(0, MIN(publish_freq_, 1000));
        image_type_   = get_parameter("image_type").as_string();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / publish_freq_)), [this]() {
            sensor_msgs::msg::Image publish_image_msg;
            cv_bridge::CvImage cv_image(std_msgs::msg::Header(), image_type_, *input_image_);
            publish_image_msg = *cv_image.toImageMsg();
            image_publisher_->publish(publish_image_msg);
        });
    }

    void update() override {}

private:
    rclcpp::Logger logger_;

    double publish_freq_ = 60.0; // Hz
    std::string image_type_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    InputInterface<cv::Mat> input_image_;
};
} // namespace rmcs_dart_guide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::ImagePublisher, rmcs_executor::Component)
