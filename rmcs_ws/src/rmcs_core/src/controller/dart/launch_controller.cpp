
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>
namespace rmcs_core::controller::dart {

class LauncherController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    LauncherController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        debug_enable_              = get_parameter("debug_enable").as_bool();
        conveyor_working_velocity_ = get_parameter("conveyor_working_velocity").as_double();

        if (debug_enable_) {
            friction_velocity_debug_ = get_parameter("friction_working_velocity").as_double();
        } else {
            register_input("/dart/firction/working_velocity", friction_velocity_input_);
        }

        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);

        register_output("/dart/friction_lf/control_velocity", friction_lf_control_velocity_, nan);
        register_output("/dart/friction_lb/control_velocity", friction_lb_control_velocity_, nan);
        register_output("/dart/friction_rb/control_velocity", friction_rb_control_velocity_, nan);
        register_output("/dart/friction_rf/control_velocity", friction_rf_control_velocity_, nan);
        register_output("/dart/conveyor/control_velocity", conveyor_control_velocity_, nan);
    }

    void update() override {
        switch_left_  = *switch_left_input_;
        switch_right_ = *switch_right_input_;
        using namespace rmcs_msgs;

        if ((switch_left_ == Switch::UNKNOWN || switch_right_ == Switch::UNKNOWN)
            || (switch_left_ == Switch::DOWN && switch_right_ == Switch::DOWN)) {
            reset_all_controls();
        } else if (switch_right_ == Switch::MIDDLE) {
            friction_enable_ = (switch_right_ == Switch::DOWN) ? false : true;
            if (switch_left_ == Switch::MIDDLE)
                conveyor_enable_ = -1;
            if (switch_left_ == Switch::UP)
                conveyor_enable_ = 1;
            update_motor_velocities();
        } else {
            friction_enable_ = (switch_right_ == Switch::DOWN) ? false : true;
            update_motor_velocities();
        }
    }

private:
    void reset_all_controls() {
        friction_enable_               = false;
        conveyor_enable_               = 0;
        *conveyor_control_velocity_    = nan;
        *friction_lf_control_velocity_ = nan;
        *friction_lb_control_velocity_ = nan;
        *friction_rb_control_velocity_ = nan;
        *friction_rf_control_velocity_ = nan;
    }

    void update_motor_velocities() {
        friction_working_velocity_ = debug_enable_ ? friction_velocity_debug_ : *friction_velocity_input_;
        double friction_velocity   = friction_enable_ ? friction_working_velocity_ : 0.0;
        double conveyor_velocity   = conveyor_enable_ * conveyor_working_velocity_;

        *conveyor_control_velocity_    = conveyor_velocity;
        *friction_lf_control_velocity_ = friction_velocity;
        *friction_lb_control_velocity_ = friction_velocity;
        *friction_rb_control_velocity_ = friction_velocity;
        *friction_rf_control_velocity_ = friction_velocity;
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    double friction_velocity_debug_;
    InputInterface<double> friction_velocity_input_;
    double conveyor_working_velocity_;
    double friction_working_velocity_;

    bool debug_enable_    = true;
    bool friction_enable_ = false;
    int conveyor_enable_  = 0;

    OutputInterface<double> friction_lf_control_velocity_;
    OutputInterface<double> friction_lb_control_velocity_;
    OutputInterface<double> friction_rb_control_velocity_;
    OutputInterface<double> friction_rf_control_velocity_;
    OutputInterface<double> conveyor_control_velocity_;

    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::LauncherController, rmcs_executor::Component)