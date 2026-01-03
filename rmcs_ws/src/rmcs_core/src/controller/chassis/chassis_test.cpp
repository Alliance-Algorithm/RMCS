#include "controller/chassis/qcp_solver.hpp"
#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/mouse.hpp>

namespace rmcs_core::controller::chassis {

class ChassisTestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisTestController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))

        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
        , left_front_joint_offset_(get_parameter_or("left_front_joint_offset", 0.0))
        , left_back_joint_offset_(get_parameter_or("left_back_joint_offset", 0.0))
        , right_front_joint_offset_(get_parameter_or("right_front_joint_offset", 0.0))
        , right_back_joint_offset_(get_parameter_or("right_back_joint_offset", 0.0))
        , Bx_(get_parameter_or("Rod_relative_horizontal_coordinate", 0.0))
        , By_(get_parameter_or("Rod_relative_longitudinal_coordinate", 0.0))
        , L_(get_parameter_or("Rod_length", 0.0)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", right_switch_);
        register_input("/remote/switch/left", left_switch_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/chassis/left_front_joint/encoder_angle", left_front_joint_angle_);
        register_input("/chassis/left_back_joint/encoder_angle", left_back_joint_angle_);
        register_input("/chassis/right_front_joint/encoder_angle", right_front_joint_angle_);
        register_input("/chassis/right_back_joint/encoder_angle", right_back_joint_angle_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/control_angle_error", lf_angle_error_, nan_);
        register_output("/chassis/left_back_joint/control_angle_error", lb_angle_error_, nan_);
        register_output("/chassis/right_front_joint/control_angle_error", rf_angle_error_, nan_);
        register_output("/chassis/right_back_joint/control_angle_error", rb_angle_error_, nan_);

        register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, nan_);*processed_encoder_angle_ = nan_;


        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = max_angle_;
    }

    void update() override {

        using rmcs_msgs::Switch;

        const auto right_switch = *right_switch_;
        const auto left_switch  = *left_switch_;
        const auto keyboard     = *keyboard_;
        do{
        if ((left_switch == Switch::UNKNOWN || right_switch == Switch::UNKNOWN)
            || (left_switch == Switch::DOWN && right_switch == Switch::DOWN)) {
            reset_all_controls();
            break;
            }

        update_chassis_mode(right_switch, left_switch, keyboard);
        update_velocity_control();

        update_lift_target_toggle(left_switch, right_switch);
        update_lift_angle_error();
        }while(false);
        last_right_switch_ = right_switch;
        last_left_switch_  = left_switch;
        last_keyboard_     = keyboard;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double translational_velocity_max_ = 5.0;
    static constexpr double angular_velocity_max_       = 10.0;
    static constexpr double spin_ratio_default_         = 0.6;

    static double wrap_deg(double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg >= 180.0)
            deg -= 360.0;
        if (deg < -180.0)
            deg += 360.0;
        return deg;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = min_angle_;
        test_init_ = false;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
        *rb_angle_error_ = nan_;
    }

    void update_chassis_mode(
        rmcs_msgs::Switch right_switch,
        rmcs_msgs::Switch left_switch,
        const rmcs_msgs::Keyboard& keyboard) {

        auto mode = *mode_;
        if (left_switch != rmcs_msgs::Switch::DOWN) {
            if (last_right_switch_ == rmcs_msgs::Switch::MIDDLE && right_switch == rmcs_msgs::Switch::DOWN) {
                mode = (mode == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::SPIN;
                if (mode == rmcs_msgs::ChassisMode::SPIN)
                    spinning_forward_ = !spinning_forward_;
            } else if (!last_keyboard_.c && keyboard.c) {
                mode = (mode == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::AUTO
                                                             : rmcs_msgs::ChassisMode::SPIN;
                if (mode == rmcs_msgs::ChassisMode::SPIN)
                    spinning_forward_ = !spinning_forward_;
            }
        }
        *mode_ = mode;
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity                = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity = (*joystick_right_) + keyboard_move;

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max_;
        return translational_velocity;
    }

    double update_angular_velocity_control() {
        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO:
            return angular_velocity_max_ * joystick_left_->y();

        // case rmcs_msgs::ChassisMode::SPIN: {
        //     double ratio = spin_ratio_default_;

        //     const double rotary_knob = *rotary_knob_;
        //     if (std::isfinite(rotary_knob)) {
        //         if (rotary_knob >= -1.0 && rotary_knob <= 1.0)
        //             ratio = std::abs(rotary_knob);
        //         else
        //             ratio = std::clamp(std::abs(rotary_knob), 0.0, 1.0);
        //     }

        //     const double angular_velocity = ratio * angular_velocity_max_;
        //     return spinning_forward_ ? angular_velocity : -angular_velocity;
        // }

        default:
            return 0.0;
        }
    }

    void update_lift_target_toggle(rmcs_msgs::Switch left_switch, rmcs_msgs::Switch right_switch) {
        const bool toggle_condition =
            (left_switch == rmcs_msgs::Switch::MIDDLE) && (right_switch == rmcs_msgs::Switch::UP);

        const bool last_toggle_condition =
            (last_left_switch_ == rmcs_msgs::Switch::MIDDLE)
            && (last_right_switch_ == rmcs_msgs::Switch::UP);

        if (toggle_condition && !last_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
        }
    }

    void update_lift_angle_error() {
        s_target_ = trapezoidal_calculator(current_target_angle_);

        const double alpha_lf =
            wrap_deg(left_front_joint_offset_) - wrap_deg(*left_front_joint_angle_) + 61.0;
        const double alpha_lb = 
            wrap_deg(left_back_joint_offset_) - wrap_deg(*left_back_joint_angle_) + 61.0;
        const double alpha_rf =
            wrap_deg(right_front_joint_offset_) - wrap_deg(*right_front_joint_angle_) + 61.0;
        const double alpha_rb =
            wrap_deg(right_back_joint_offset_) - wrap_deg(*right_back_joint_angle_) + 61.0;

        *processed_encoder_angle_ = (alpha_lb + alpha_lf + alpha_rf + alpha_rb) / 4.0;

        s_lf_ = trapezoidal_calculator(alpha_lf);
        s_lb_ = trapezoidal_calculator(alpha_lb);
        s_rf_ = trapezoidal_calculator(alpha_rf);
        s_rb_ = trapezoidal_calculator(alpha_rb);

        const double lf_err = s_lf_ - s_target_;
        const double lb_err = s_lb_ - s_target_;
        const double rf_err = s_rf_ - s_target_;
        const double rb_err = s_rb_ - s_target_;

        *lf_angle_error_ = lf_err;
        *lb_angle_error_ = lb_err;
        *rf_angle_error_ = rf_err;
        *rb_angle_error_ = rb_err;
    }

    double trapezoidal_calculator(double alpha_deg) const {
        const double rad = alpha_deg * pi_ / 180.0;

        const double term = Bx_ * std::cos(rad) + By_ * std::sin(rad);
        const double t = (Bx_ * std::sin(rad) - By_ * std::cos(rad) + 15.0);

        const double inside = L_ * L_ - t * t;
        return term + std::sqrt(std::max(0.0, inside));
    }

    double angle_calculator(double s) const {
        const double A = 2.0 * s * Bx_ + 30.0 * By_;
        const double B = 2.0 * s * By_ - 30.0 * Bx_;
        const double C = s * s + Bx_ * Bx_ + By_ * By_ - 9775.0;

        double cos_val = C / std::sqrt(A * A + B * B);
        cos_val = std::max(-1.0, std::min(1.0, cos_val));

        return std::atan2(B, A) - std::acos(cos_val);
    }

    void init_calculator() {
        const double alpha_lf =
            wrap_deg(left_front_joint_offset_ - *left_front_joint_angle_);
        const double alpha_lb =
            wrap_deg(left_back_joint_offset_ - *left_back_joint_angle_);
        const double alpha_rf =
            wrap_deg(right_front_joint_offset_ - *right_front_joint_angle_);
        const double alpha_rb =
            wrap_deg(right_back_joint_offset_ - *right_back_joint_angle_);

        lf_ = trapezoidal_calculator(alpha_lf);
        lb_ = trapezoidal_calculator(alpha_lb);
        rf_ = trapezoidal_calculator(alpha_rf);
        rb_ = trapezoidal_calculator(alpha_rb);

        s_lf_ = lf_;
        s_lb_ = lb_;
        s_rf_ = rf_;
        s_rb_ = rb_;
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> right_switch_;
    InputInterface<rmcs_msgs::Switch> left_switch_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    rmcs_msgs::Switch last_right_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_left_switch_  = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_   = rmcs_msgs::Keyboard::zero();

    bool spinning_forward_ = true;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> right_back_joint_angle_;

    OutputInterface<double> lf_angle_error_;
    OutputInterface<double> lb_angle_error_;
    OutputInterface<double> rf_angle_error_;
    OutputInterface<double> rb_angle_error_;

    OutputInterface<double> processed_encoder_angle_;

    double min_angle_;
    double max_angle_;
    double left_front_joint_offset_;
    double left_back_joint_offset_;
    double right_front_joint_offset_;
    double right_back_joint_offset_;

    double current_target_angle_ = 55.0;

    double lf_ = 0.0, lb_ = 0.0, rf_ = 0.0, rb_ = 0.0;
    double s_lf_ = 0.0, s_lb_ = 0.0, s_rf_ = 0.0, s_rb_ = 0.0;
    double s_target_ = 0.0;

    double Bx_ = 0.0;
    double By_ = 0.0;
    double L_  = 0.0;

    bool test_init_ = false;

    static constexpr double pi_ = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisTestController, rmcs_executor::Component)
