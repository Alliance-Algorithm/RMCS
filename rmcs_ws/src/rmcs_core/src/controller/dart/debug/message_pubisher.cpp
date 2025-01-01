
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>

namespace rmcs_core::controller::dart {

class MessagePublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MessagePublisher()
        : Node(get_component_name()) {

        debug_mode_ = get_parameter("debug_enable").as_bool();

        register_input("friction_lf_current_velocity_", friction_lf_current_velocity_);
        register_input("friction_lb_current_velocity_", friction_lb_current_velocity_);
        register_input("friction_rb_current_velocity_", friction_rb_current_velocity_);
        register_input("friction_rf_current_velocity_", friction_rf_current_velocity_);

        publisher_1_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lf_current_velocity_", 10);
        publisher_2_ = this->create_publisher<std_msgs::msg::String>("msg_friction_lb_current_velocity_", 10);
        publisher_3_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rb_current_velocity_", 10);
        publisher_4_ = this->create_publisher<std_msgs::msg::String>("msg_friction_rf_current_velocity_", 10);
        timer_       = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->publish_message(); });
    }

    void update() override {
        msg_friction_lf_current_velocity_.data = std::to_string(*friction_lf_current_velocity_);
        msg_friction_lb_current_velocity_.data = std::to_string(*friction_lb_current_velocity_);
        msg_friction_rb_current_velocity_.data = std::to_string(*friction_rb_current_velocity_);
        msg_friction_rf_current_velocity_.data = std::to_string(*friction_rf_current_velocity_);
    }

private:
    void publish_message() {
        publisher_1_->publish(msg_friction_lf_current_velocity_);
        publisher_2_->publish(msg_friction_lb_current_velocity_);
        publisher_3_->publish(msg_friction_rb_current_velocity_);
        publisher_4_->publish(msg_friction_rf_current_velocity_);
    }

    bool debug_mode_ = false;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_3_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_4_;

    std_msgs::msg::String msg_friction_lf_current_velocity_;
    std_msgs::msg::String msg_friction_lb_current_velocity_;
    std_msgs::msg::String msg_friction_rb_current_velocity_;
    std_msgs::msg::String msg_friction_rf_current_velocity_;

    InputInterface<double> friction_lf_current_velocity_;
    InputInterface<double> friction_lb_current_velocity_;
    InputInterface<double> friction_rb_current_velocity_;
    InputInterface<double> friction_rf_current_velocity_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::MessagePublisher, rmcs_executor::Component)
