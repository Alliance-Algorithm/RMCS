
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::dart {

using namespace rmcs_description;

class AngleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AngleController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        yaw_velocity_limit_     = get_parameter("yaw_velocity_limit").as_double();
        yaw_left_angle_limit_   = get_parameter("yaw_left_angle_limit").as_double();
        yaw_right_angle_limit_  = get_parameter("yaw_right_angle_limit").as_double();
        pitch_velocity_limit_   = get_parameter("pitch_velocity_limit").as_double();
        pitch_up_angle_limit_   = get_parameter("pitch_up_angle_limit").as_double();
        pitch_down_angle_limit_ = get_parameter("pitch_down_angle_limit").as_double();

        register_input("/remote/switch/right", switch_right_input_);
        register_input("/remote/switch/left", switch_left_input_);
    }

    void update() override {
        using namespace rmcs_msgs;

        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;

        if ((switch_left_ == Switch::UNKNOWN || switch_right_ == Switch::UNKNOWN)
            || (switch_left_ == Switch::DOWN || switch_right_ == Switch::DOWN)) {
            reset_all_controls();
        } else {
            control_enabled_ = true;
            update_control_errors();
        }
    }

private:
    void reset_all_controls() { control_enabled_ = false; }

    void update_control_errors() {}

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;
    bool control_enabled_ = false;

    double yaw_velocity_limit_, pitch_up_angle_limit_, pitch_down_angle_limit_;
    double pitch_velocity_limit_, yaw_left_angle_limit_, yaw_right_angle_limit_;

    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::AngleController, rmcs_executor::Component)