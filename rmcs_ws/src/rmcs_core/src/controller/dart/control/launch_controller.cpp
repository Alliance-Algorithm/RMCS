
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
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
        conveyor_working_velocity_ = get_parameter("conveyor_working_velocity").as_double();
        error_velocity_            = get_parameter("error_velocity").as_double();

        auto parameter = get_parameter("friction_working_velocity");
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            parameter_immediate_value_ = parameter.as_double();
            friction_working_velocity_.bind_directly(parameter_immediate_value_);
        } else {
            register_input(parameter.as_string(), friction_working_velocity_);
        }

        register_input("/dart/conveyor/velocity", conveyor_velocity_);
        register_input("/remote/switch/right", switch_right_input_, false);
        register_input("/remote/switch/left", switch_left_input_, false);

        register_output("/dart/conveyor/control_velocity", conveyor_control_velocity_, nan);
        register_output("/dart/friction_front/control_velocity", friction_front_control_velocity_, nan);
        register_output("/dart/friction_back/control_velocity", friction_back_control_velocity_, nan);
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
            if (switch_left_ == Switch::MIDDLE) {
                auto_filling();
            } else if (switch_left_ == Switch::UP) {
                conveyor_enable_ = 1;
            } else {
                conveyor_enable_ = 0;
            }
            update_motor_velocities();
        } else {
            friction_enable_ = (switch_right_ == Switch::DOWN) ? false : true;
            conveyor_enable_ = 0;
            update_motor_velocities();
        }
    }

private:
    void reset_all_controls() {
        friction_enable_                  = false;
        conveyor_enable_                  = 0;
        *conveyor_control_velocity_       = nan;
        *friction_front_control_velocity_ = nan;
        *friction_back_control_velocity_  = nan;
    }

    void update_motor_velocities() {
        double friction_velocity = friction_enable_ ? *friction_working_velocity_ : 0.0;
        double conveyor_velocity = conveyor_enable_ * conveyor_working_velocity_;

        *conveyor_control_velocity_       = conveyor_velocity;
        *friction_front_control_velocity_ = friction_velocity;
        *friction_back_control_velocity_  = friction_velocity + error_velocity_;
    }

    void auto_filling() {
        double velocity = *conveyor_velocity_;

        if (velocity == 0 && !reverse_ready_) {
            reverse_ready_   = true;
            conveyor_enable_ = -1 * conveyor_enable_ - 0.5;
        } else if (abs(velocity) >= 10.0) {
            reverse_ready_ = false;
        }

        RCLCPP_INFO(logger_, "conveyor:%5.1lf,bool:%5d", conveyor_enable_, reverse_ready_);
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Logger logger_;

    InputInterface<double> friction_working_velocity_;
    InputInterface<double> conveyor_velocity_;
    double parameter_immediate_value_;
    double conveyor_working_velocity_;
    double error_velocity_;

    bool friction_enable_   = false;
    double conveyor_enable_ = 0;
    bool reverse_ready_     = false;

    OutputInterface<double> conveyor_control_velocity_;
    OutputInterface<double> friction_front_control_velocity_;
    OutputInterface<double> friction_back_control_velocity_;

    InputInterface<rmcs_msgs::Switch> switch_left_input_;
    InputInterface<rmcs_msgs::Switch> switch_right_input_;

    rmcs_msgs::Switch switch_left_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch switch_right_ = rmcs_msgs::Switch::UNKNOWN;
};
} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::LauncherController, rmcs_executor::Component)