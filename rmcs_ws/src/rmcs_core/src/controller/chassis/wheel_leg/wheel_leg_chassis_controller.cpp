#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/wheel_leg_mode.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(7.0, 0.0, 0.0) {
        following_velocity_controller_.output_max = angular_velocity_max;
        following_velocity_controller_.output_min = -angular_velocity_max;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/yaw/angle", chassis_yaw_angle_imu_);

        register_output("/chassis/angle", chassis_angle_, nan);
        register_output("/chassis/control_angle", chassis_control_angle_, nan);

        register_output("/chassis/control_mode", mode_);

        register_output("/chassis/status/balanceless", is_balanceless_, false);
        register_output("/chassis/status/rescue_tip_over", is_rescue_tip_over_, false);

        register_output("/chassis/status/leg_extended", is_leg_extended_, false);
        register_output("/chassis/status/jump", is_jump_active_, false);
        register_output("/chassis/status/climb", is_climb_active_, false);

        register_output("/chassis/control_velocity", chassis_control_velocity_);
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {
                if (last_switch_right_ == Switch::DOWN && switch_right == Switch::MIDDLE) {
                    if (mode == rmcs_msgs::WheelLegMode::BALANCELESS) {
                        mode = rmcs_msgs::WheelLegMode::FOLLOW;
                    }
                }

                if ((last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN)
                    || (!last_keyboard_.c && keyboard.c)) {
                    if (mode == rmcs_msgs::WheelLegMode::SPIN) {
                        mode = rmcs_msgs::WheelLegMode::FOLLOW;
                    } else {
                        mode = rmcs_msgs::WheelLegMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode == rmcs_msgs::WheelLegMode::LAUNCH_RAMP
                             ? rmcs_msgs::WheelLegMode::FOLLOW
                             : rmcs_msgs::WheelLegMode::LAUNCH_RAMP;
                } else if (!last_keyboard_.b && keyboard.b) {
                    mode = mode == rmcs_msgs::WheelLegMode::BALANCELESS
                             ? rmcs_msgs::WheelLegMode::FOLLOW
                             : rmcs_msgs::WheelLegMode::BALANCELESS;
                }

                // Change leg length
                if (!last_keyboard_.z && keyboard.z) {
                    leg_extended_ = !leg_extended_;
                }

                // Jump
                jump_active_ = !last_keyboard_.e && keyboard.e;

                // Climb
                climb_active_ = !last_keyboard_.q && keyboard.q;

                // Rescue tip-over
                rescue_tip_over_ = !last_keyboard_.r && keyboard.r;

                *is_balanceless_ = mode == rmcs_msgs::WheelLegMode::BALANCELESS;
                *is_leg_extended_ = leg_extended_;
                *is_jump_active_ = jump_active_;
                *is_climb_active_ = climb_active_;
                *is_rescue_tip_over_ = rescue_tip_over_;

                *mode_ = mode;
            }

            update_velocity_control();
            update_chassis_angle_control();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::WheelLegMode::BALANCELESS;

        *chassis_control_velocity_ = {nan, nan, nan};
    }

    void update_velocity_control() {
        auto translational_velocity = update_translational_velocity_control();
        auto angular_velocity = update_angular_velocity_control();

        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max;

        return translational_velocity;
    }

    void update_chassis_angle_control() {
        double chassis_control_angle = nan;

        *chassis_control_angle_ = chassis_control_angle;
    }

    double update_angular_velocity_control() {
        double angular_velocity = 0.0;
        double chassis_control_angle = nan;

        switch (*mode_) {
        case rmcs_msgs::WheelLegMode::BALANCELESS:
        case rmcs_msgs::WheelLegMode::RESCUE_TIP_OVER: break;
        case rmcs_msgs::WheelLegMode::SPIN: {
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
            chassis_control_angle =
                *chassis_yaw_angle_imu_; // When spinning, yaw angle's output is set to zero.
        } break;
        case rmcs_msgs::WheelLegMode::FOLLOW:
        case rmcs_msgs::WheelLegMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);
            // err: [0, 2pi) -> signed
            // Only one direction can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);

            chassis_control_angle = 0.0;
        } break;
        }
        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;
        // chassis_control_angle: [0, 2pi).

        // err = setpoint         -       measurement
        //          ^                          ^
        //          |gimbal_yaw_angle_error    |chassis_angle
        //                                            ^
        //                                            |(2pi - gimbal_yaw_angle)
        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;
        // err: [0, 2pi).

        return err;
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // Maximum control velocities
    static constexpr double translational_velocity_max = 2.0;
    static constexpr double angular_velocity_max = 16.0;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    InputInterface<double> chassis_yaw_angle_imu_;

    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::WheelLegMode> mode_;

    OutputInterface<bool> is_balanceless_;
    OutputInterface<bool> is_rescue_tip_over_;

    OutputInterface<bool> is_leg_extended_;
    OutputInterface<bool> is_jump_active_;
    OutputInterface<bool> is_climb_active_;

    bool spinning_forward_ = true;
    bool leg_extended_ = false, jump_active_ = false, climb_active_ = false,
         rescue_tip_over_ = false, launch_ramp_active_ = false;

    pid::PidCalculator following_velocity_controller_;

    OutputInterface<double> chassis_yaw_control_angle_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegChassisController, rmcs_executor::Component)