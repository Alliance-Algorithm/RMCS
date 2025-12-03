#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
namespace rmcs_core::hardware {
class ServoTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ServoTest()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        trigger_lock_angle_ = get_parameter("trigger_lock_angle").as_int();
        trigger_free_angle_ = get_parameter("trigger_free_angle").as_int();
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        register_output("/dart/trigger_servo/control_angle", trigger_control_angle);
    }

    void update() override {
        using namespace rmcs_msgs;
        if (*switch_left_ == Switch::UP && *switch_right_ == Switch::DOWN) {
            *trigger_control_angle = trigger_lock_angle_;
        } else if (*switch_left_ == Switch::UP && *switch_right_ == Switch::UP) {
            *trigger_control_angle = trigger_free_angle_;
        }
    }

private:
    rclcpp::Logger logger_;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    uint16_t trigger_lock_angle_;
    uint16_t trigger_free_angle_;
    OutputInterface<uint16_t> trigger_control_angle;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ServoTest, rmcs_executor::Component)

// FF FF 01 03 03 16 E2
// FF FF 01 04 02 16 02 E0

// FF FF 01 03 03 18 E0
// FF FF 01 04 02 18 02 DE

// FF FF 01 05 03 1C 00 00 DB
// FF FF 01 05 03 1C 00 01 DA

// FF FF 01 07 03 41 00 14 00 00 9F
// FF FF 01 07 03 41 00 32 00 00 81

// FF FF 01 05 03 41 00 14 A1
// FF FF 01 04 02 1D 01 DA