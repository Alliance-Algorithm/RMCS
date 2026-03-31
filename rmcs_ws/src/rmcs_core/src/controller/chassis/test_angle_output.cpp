#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::test::output {

class AngleOutput
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleOutput()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        {
        register_input("/chassis/left_front_joint/angle", lf_input_angle_);
        register_input("/chassis/left_back_joint/angle", lb_input_angle_);
        register_input("/chassis/right_back_joint/angle", rb_input_angle_);
        register_input("/chassis/right_front_joint/angle", rf_input_angle_);
        
    }

    void update() override {
        RCLCPP_INFO(
            get_logger(),
            "Angles (deg): LF=%.2f, LB=%.2f, RB=%.2f, RF=%.2f",
            *lf_input_angle_ * rad2deg, *lb_input_angle_ * rad2deg, *rb_input_angle_ * rad2deg, *rf_input_angle_ * rad2deg);
    }

private:
    InputInterface<double> lf_input_angle_;
    InputInterface<double> lb_input_angle_;
    InputInterface<double> rb_input_angle_;
    InputInterface<double> rf_input_angle_;

    double rad2deg = 180.0 / std::numbers::pi;

};

} // namespace rmcs_core::test::output

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::test::output::AngleOutput, rmcs_executor::Component)