#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

// DartLaunchSettingV2 仅负责扳机锁定状态到 PWM 值的映射。
class DartLaunchSettingV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartLaunchSettingV2()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , trigger_free_value_(get_parameter("trigger_free_value").as_double())
        , trigger_lock_value_(get_parameter("trigger_lock_value").as_double()) {
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);
        register_output("/dart/trigger_servo/value", trigger_value_);
    }

    void update() override {
        *trigger_value_ = *trigger_lock_enable_ ? trigger_lock_value_ : trigger_free_value_;
    }

private:
    double trigger_free_value_;
    double trigger_lock_value_;

    InputInterface<bool> trigger_lock_enable_;
    OutputInterface<double> trigger_value_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::DartLaunchSettingV2, rmcs_executor::Component)
