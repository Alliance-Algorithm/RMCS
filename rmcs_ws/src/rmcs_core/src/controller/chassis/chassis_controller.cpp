#include "controller/pid/pid_calculator.hpp"

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        following_velocity_controller_.output_max = +kAngularVelocityMax;
        following_velocity_controller_.output_min = -kAngularVelocityMax;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/rmcs_navigation/enable_control", navigation_enable_control_, false);
        register_input("/rmcs_navigation/chassis_velocity", navigation_command_velocity_, false);
        register_input("/rmcs_navigation/chassis_behavior", navigation_chassis_behavior_, false);

        register_output("/chassis/angle", chassis_angle_, kNaN);
        register_output("/chassis/control_angle", chassis_control_angle_, kNaN);
        register_output("/chassis/control_mode", mode_, ChassisMode::ALIGNMENT);
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

        // Navigation Control Unavailable, Make It Manual Mode
        if (!navigation_enable_control_.ready()) {
            navigation_enable_control_.make_and_bind_directly(false);
            navigation_command_velocity_.make_and_bind_directly(Eigen::Vector2d::Zero());
            navigation_chassis_behavior_.make_and_bind_directly(ChassisMode::NONE);

            RCLCPP_INFO(get_logger(), "Manual mode without navigation");
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

                // Navigation Control Here
                const auto behavior = *navigation_chassis_behavior_;
                const auto control = *navigation_enable_control_;
                if (control && behavior != ChassisMode::NONE)
                    mode = behavior;

                // Right Switch: MIDDLE -> DOWN
                // Switch ChassisMode Between SPIN And STEP_DOWN
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == ChassisMode::SPIN_FAST || mode == ChassisMode::SPIN_SLOW) {
                        mode = ChassisMode::STEP_DOWN;
                    } else {
                        mode = spinning_forward_ ? ChassisMode::SPIN_FAST : ChassisMode::SPIN_SLOW;
                        spinning_forward_ = !spinning_forward_;
                    }
                }
                // Press Key: Z
                // Switch ChassisMode To STEP_DOWN
                else if (!last_keyboard_.z && keyboard.z) {
                    mode = (mode == ChassisMode::STEP_DOWN) ? ChassisMode::AUTO
                                                            : ChassisMode::STEP_DOWN;
                }
                // Press Key: C
                // Switch ChassisMode To SPIN
                else if (!last_keyboard_.c && keyboard.c) {
                    if (mode == ChassisMode::SPIN_FAST) {
                        mode = ChassisMode::AUTO;
                    } else {
                        mode = ChassisMode::SPIN_FAST;
                        spinning_forward_ = !spinning_forward_;
                    }
                }
                // Press Key: X
                // Switch ChassisMode To LAUNCH_RAMP
                else if (!last_keyboard_.x && keyboard.x) {
                    mode = (mode == ChassisMode::LAUNCH_RAMP) ? ChassisMode::AUTO
                                                              : ChassisMode::LAUNCH_RAMP;
                }

                *mode_ = mode;
            }

            update_velocity_control();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

    void reset_all_controls() {
        *mode_ = ChassisMode::ALIGNMENT;
        *chassis_control_velocity_ = {kNaN, kNaN, kNaN};
    }

    void update_velocity_control() {
        auto translational_velocity = update_translational_velocity_control();
        auto angular_velocity = update_angular_velocity_control();

        Eigen::Vector3d control_velocity;
        control_velocity << translational_velocity, angular_velocity;

        chassis_control_velocity_->vector = control_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        // Handle Navigation Control
        //
        // YawLink(LidarLink) -> ChassisLink
        //
        auto nav_speed = Eigen::Vector2d{Eigen::Vector2d::Zero()};
        if (*navigation_enable_control_) {
            auto raw_command = *navigation_command_velocity_;
            if (std::isfinite(raw_command.x()) && std::isfinite(raw_command.y())) {
                auto yaw_rotation = Eigen::Rotation2Dd{*gimbal_yaw_angle_};
                nav_speed = yaw_rotation * raw_command;
            }
        }

        const auto keyboard = *keyboard_;
        const auto keyboard_move =
            Eigen::Vector2d{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        auto manual_speed = Eigen::Rotation2Dd{*gimbal_yaw_angle_}
                          * Eigen::Vector2d{*joystick_right_ + keyboard_move};

        if (manual_speed.norm() > 1.0)
            manual_speed.normalize();

        manual_speed *= kTranslationalVelocityMax;

        return manual_speed + nav_speed;
    }

    double update_angular_velocity_control() {
        auto angular_velocity = 0.0;
        auto chassis_control_angle = kNaN;

        switch (*mode_) {
        case ChassisMode::NONE:
        case ChassisMode::AUTO: break;

        case ChassisMode::SPIN_FAST: {
            angular_velocity =
                1.0 * (spinning_forward_ ? kAngularVelocityMax : -kAngularVelocityMax);
        } break;
        case ChassisMode::SPIN_SLOW: {
            angular_velocity =
                0.5 * (spinning_forward_ ? kAngularVelocityMax : -kAngularVelocityMax);
        } break;

        case ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> [0, alignment) -> signed.
            // In step-down mode, two sides of the chassis can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;
        case ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            // err: [0, 2pi) -> signed
            // In launch ramp mode, only one direction can be used for alignment.
            // TODO: Dynamically determine the split angle based on chassis velocity.
            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        case ChassisMode::ALIGNMENT_POWERED:
        case ChassisMode::ALIGNMENT: {
            const auto speed = Eigen::Vector2d{chassis_control_velocity_->vector.head<2>()};
            const auto line1 = Eigen::Vector2d{speed.x(), 0};
            const auto line2 = Eigen::Vector2d{0, speed.y()};

            const auto signed_angle = [](const Eigen::Vector2d& from, const Eigen::Vector2d& to) {
                return std::atan2(from.x() * to.y() - from.y() * to.x(), from.dot(to));
            };
            const double angle1 = signed_angle(speed, line1);
            const double angle2 = signed_angle(speed, line2);
            const double min = (std::abs(angle1) < std::abs(angle2)) ? angle1 : angle2;

            angular_velocity = following_velocity_controller_.update(-min);
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
    using ChassisMode = rmcs_msgs::ChassisMode;

    // Maximum control velocities
    static constexpr double kTranslationalVelocityMax = 20.0;
    static constexpr double kAngularVelocityMax = 1.2 * std::numbers::pi * 2.0;

    static constexpr double kInf = std::numeric_limits<double>::infinity();
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    bool spinning_forward_ = true;

    pid::PidCalculator following_velocity_controller_{
        get_parameter_or<double>("following_velocity_kp", 7.0),
        get_parameter_or<double>("following_velocity_ki", 0.0),
        get_parameter_or<double>("following_velocity_kd", 0.0),
    };
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    // For Navigation
    InputInterface<bool> navigation_enable_control_;
    InputInterface<Eigen::Vector2d> navigation_command_velocity_;
    InputInterface<ChassisMode> navigation_chassis_behavior_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)
