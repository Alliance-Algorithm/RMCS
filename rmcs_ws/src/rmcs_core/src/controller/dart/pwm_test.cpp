
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::test {
class PWMTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PWMTest()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        set_value_ = get_parameter("set_value").as_double();
        register_output("/dart/trigger_servo/value", value_);
    }

    void update() override { *value_ = set_value_; }

private:
    double set_value_ = 0.0;
    OutputInterface<double> value_;
};
} // namespace rmcs_core::controller::test

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::test::PWMTest, rmcs_executor::Component)