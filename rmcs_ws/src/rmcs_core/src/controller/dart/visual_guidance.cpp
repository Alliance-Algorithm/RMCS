
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class VisualGuidance
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VisualGuidance()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        debug_mode = get_parameter("debug_mode").as_bool();
        register_output(
            "/dart/guidance/control_direction", control_dirction_, Eigen::Vector3d::Zero());
        register_output("/dart/firction/working_velocity", friction_working_velocity_, nan);
    }

    void update() override {
        identifier();
        update_control_direction();
    }

private:
    void identifier() {}
    void update_control_direction() {}
    void calculate_relative_distance() {}
    void claculate_launch_velocity() {}

    rclcpp::Logger logger_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    bool debug_mode;

    OutputInterface<double> friction_working_velocity_;
    OutputInterface<Eigen::Vector3d> control_dirction_;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::VisualGuidance, rmcs_executor::Component)