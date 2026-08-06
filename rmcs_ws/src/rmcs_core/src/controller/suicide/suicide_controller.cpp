#include <exception>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace rmcs_core::controller::suicide {

class SuicideController : public rmcs_executor::Component {
public:
    SuicideController() { register_input("/remote/keyboard", keyboard_); }

    void update() override {
        if (keyboard_->ctrl && keyboard_->shift && keyboard_->r) {
            RCLCPP_WARN(rclcpp::get_logger("suicide"), "TERMINATE TRIGGERED");
            std::terminate();
        }
    }

private:
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
};

} // namespace rmcs_core::controller::suicide

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::suicide::SuicideController, rmcs_executor::Component)