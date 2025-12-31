#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::smc {

class SlidingModeController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SlidingModeController()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);

        c_       = get_parameter("c").as_double();
        epsilon_ = get_parameter("epsilon").as_double();
        k_       = get_parameter("k").as_double();
        phi_     = get_parameter("phi").as_double();

        moment_of_inertia_    = get_parameter("J").as_double();
        damping_coefficient_ = get_parameter("B").as_double();
        gear_ratio            = get_parameter("gear_ratio").as_double();

        register_input(get_parameter("angle_error").as_string(), angle_error_);
        register_input(get_parameter("current_velocity").as_string(), current_velocity_);
        register_input(get_parameter("target_velocity").as_string(), target_velocity_);
        register_input(get_parameter("target_acceleration").as_string(), target_acceleration_);

        register_output(get_parameter("control_torque").as_string(), control_torque_);

        // debug
        // register_output("/smc/s_value", sliding_surface_value_);
    }

    void update() override {

        if ((*switch_left_ == rmcs_msgs::Switch::DOWN && *switch_right_ == rmcs_msgs::Switch::DOWN)
            || (*switch_left_ == rmcs_msgs::Switch::UNKNOWN && *switch_right_ == rmcs_msgs::Switch::UNKNOWN)) {
            reset();
            return;
        }
            // if(*angle_error_ < 0.8 && *angle_error_ > -0.8){             // dead zone
            //     *control_torque_ =0.0;
            // } else {
                *control_torque_ = std::clamp(calc_control_value(), -2.5, 2.5);
            // }
        // RCLCPP_INFO(logger_, "control:%8lf", *control_torque_);    //debug
    }

private:
    void reset() {
        *control_torque_                = nan;
        sliding_surface_value_integral_ = 0;
    }

    double calc_control_value() {
        double velocity_error = *current_velocity_ - *target_velocity_;
        double s              = c_ * (-*angle_error_) + velocity_error;

        // debug code begin
        *sliding_surface_value_ = s;

        // damping term
        double damping_term = damping_coefficient_ * (*current_velocity_);

        // acceleration feed forward term
        double feedforward_term = moment_of_inertia_ * (*target_acceleration_);

        // integral term
        sliding_surface_value_integral_ = sliding_surface_value_integral_ + s * 0.001;

        // control law
        double control_value = damping_term + feedforward_term
                             - moment_of_inertia_ * (c_ * velocity_error + epsilon_ * tanh(s / phi_) + k_ * (s));

        return control_value;
    }

    double sat(double s) const {
        if (phi_ == 0) {
            RCLCPP_ERROR(logger_, "SlidingModeController: sat(s): phi_ == 0");
        }
        auto sp = s / phi_;
        if (sp > 1) {
            return 1;
        } else if (sp < -1) {
            return -1;
        } else {
            return sp;
        }
    }

    rclcpp::Logger logger_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    // controller parameters
    double c_;       // sliding_surface_parameter_
    double epsilon_; // switch_gain_
    double k_;       // reaching_law_gain_
    double phi_;     // boundary_layer_thickness_

    // mechanical parameters
    double moment_of_inertia_;
    double damping_coefficient_;
    double sliding_surface_value_integral_ = 0;
    double gear_ratio; // 传动增益，传动比*效率的倒数

    // observed variables
    InputInterface<double> angle_error_;
    InputInterface<double> current_velocity_;
    InputInterface<double> target_velocity_;
    InputInterface<double> target_acceleration_;

    // output
    OutputInterface<double> control_torque_;

    // debug
    OutputInterface<double> sliding_surface_value_;
};
} // namespace rmcs_core::controller::smc

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::smc::SlidingModeController, rmcs_executor::Component)