#include "controller/pid/pid_calculator.hpp"

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    ChassisController()
        : Node{get_component_name(), node::options()} {

        following_velocity_controller_.output_max = angular_velocity_max;
        following_velocity_controller_.output_min = -angular_velocity_max;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, false);
        register_input("/chassis/climbing_forward_velocity", climbing_forward_velocity_, false);

        register_input("/rmcs_navigation/enable_control", navigation_enable_control_, false);
        register_input("/rmcs_navigation/chassis_velocity", navigation_command_velocity_, false);
        register_input("/rmcs_navigation/chassis_behavior", navigation_chassis_behavior_, false);

        register_output("/chassis/angle", chassis_angle_, kNaN);
        register_output("/chassis/control_angle", chassis_control_angle_, kNaN);
        register_output("/chassis/control_mode", mode_, rmcs_msgs::ChassisMode::ALIGNMENT);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            node::warn("Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            node::warn("Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }

        if (!climbing_forward_velocity_.ready()) {
            climbing_forward_velocity_.make_and_bind_directly(kNaN);
        }

        if (!navigation_enable_control_.ready()) {
            navigation_enable_control_.make_and_bind_directly(false);
        }
        if (!navigation_command_velocity_.ready()) {
            navigation_command_velocity_.make_and_bind_directly(Eigen::Vector2d::Zero());
        }
        if (!navigation_chassis_behavior_.ready()) {
            navigation_chassis_behavior_.make_and_bind_directly(rmcs_msgs::ChassisMode::AUTO);
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
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode != rmcs_msgs::ChassisMode::SPIN_FAST) {
                        mode = rmcs_msgs::ChassisMode::SPIN_FAST;
                        spinning_forward_ = !spinning_forward_;
                    } else {
                        mode = rmcs_msgs::ChassisMode::STEP_DOWN;
                    }
                } else if (!last_keyboard_.c && keyboard.c) {
                    if (mode != rmcs_msgs::ChassisMode::SPIN_FAST) {
                        mode = rmcs_msgs::ChassisMode::SPIN_FAST;
                        spinning_forward_ = !spinning_forward_;
                    } else {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode != rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             ? rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             : rmcs_msgs::ChassisMode::AUTO;
                } else if (!last_keyboard_.z && keyboard.z) {
                    mode = mode != rmcs_msgs::ChassisMode::STEP_DOWN
                             ? rmcs_msgs::ChassisMode::STEP_DOWN
                             : rmcs_msgs::ChassisMode::AUTO;
                }

                if (*navigation_enable_control_
                    && navigation_command_velocity_->array().isFinite().all()) {
                    mode = *navigation_chassis_behavior_;
                }

                update_spin_stuck_watchdog(mode);
                *mode_ = mode;
            }

            update_velocity_control();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::ALIGNMENT;
        *chassis_control_velocity_ = {kNaN, kNaN, kNaN};

        spin_stuck_count_ = 0;
        spin_recovery_count_ = 0;
    }

    auto update_spin_stuck_watchdog(rmcs_msgs::ChassisMode& mode) -> void {
        constexpr auto kSpinStuckConfirmTicks = std::size_t{300};
        constexpr auto kSpinRecoveryTicks = std::size_t{1000};
        constexpr auto kSpinStuckAngularVelocityRatio = double{0.2};

        using rmcs_msgs::ChassisMode;

        if (spin_recovery_count_ > 0) {
            mode = ChassisMode::ALIGNMENT_POWERED;

            if (--spin_recovery_count_ == 0)
                mode = mode_before_watchdog_;

            spin_stuck_count_ = 0;
            return;
        }

        if (!rmcs_msgs::is_spining(mode) || !chassis_yaw_velocity_imu_.ready()) {
            spin_stuck_count_ = 0;
            return;
        }

        const auto expected = (mode == ChassisMode::SPIN_FAST ? 0.6 : 0.3) * angular_velocity_max;
        if (std::abs(*chassis_yaw_velocity_imu_) >= kSpinStuckAngularVelocityRatio * expected) {
            spin_stuck_count_ = 0;
            return;
        }

        if (++spin_stuck_count_ < kSpinStuckConfirmTicks)
            return;

        mode_before_watchdog_ = mode;
        mode = ChassisMode::ALIGNMENT_POWERED;
        spin_recovery_count_ = kSpinRecoveryTicks;
        spin_stuck_count_ = 0;

        node::warn("Spin stuck detected, disable spinning for 1s.");
    }

    void update_velocity_control() {
        auto translational_velocity = update_translational_velocity_control();
        auto angular_velocity = update_angular_velocity_control();

        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        if (!std::isnan(*climbing_forward_velocity_))
            return {*climbing_forward_velocity_, 0.0};

        if (*navigation_enable_control_) {
            const auto command = *navigation_command_velocity_;
            if (command.array().isFinite().all()) {
                Eigen::Vector2d superimposed = command + *joystick_right_ * translational_velocity_max;
                if (superimposed.norm() > translational_velocity_max)
                    superimposed *= translational_velocity_max / superimposed.norm();

                return Eigen::Rotation2Dd{*gimbal_yaw_angle_} * superimposed;
            }
        }

        auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max;

        return translational_velocity;
    }

    double update_angular_velocity_control() {
        double angular_velocity = 0.0;
        double chassis_control_angle = kNaN;

        if (!std::isnan(*climbing_forward_velocity_)) {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);
            if (err > std::numbers::pi)
                err -= 2 * std::numbers::pi;
            angular_velocity = following_velocity_controller_.update(err);

            *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
            *chassis_control_angle_ = chassis_control_angle;
            return angular_velocity;
        }

        using namespace rmcs_msgs;
        switch (*mode_) {
        case ChassisMode::AUTO: break;

        case ChassisMode::SPIN_FAST:
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
            break;
        case ChassisMode::SPIN_SLOW:
            angular_velocity =
                0.3 * (spinning_forward_ ? angular_velocity_max : -angular_velocity_max);
            break;

            // @NOTE: Align With 4 Sides
        case ChassisMode::ALIGNMENT_POWERED: [[fallthrough]];
        case ChassisMode::ALIGNMENT: {
            const auto speed = chassis_control_velocity_->vector.head<2>();
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

            // @NOTE: Align With 2 Sides
        case ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;

            // @NOTE: Align With 1 Sides
        case ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
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

        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;

        return err;
    }

private:
    static constexpr double kInf = std::numeric_limits<double>::infinity();
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    const double translational_velocity_max{node::param_or("translational_velocity_max", 10.0)};
    const double angular_velocity_max{node::param_or("angular_velocity_max", 16.0)};

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<double> climbing_forward_velocity_;

    InputInterface<bool> navigation_enable_control_;
    InputInterface<Eigen::Vector2d> navigation_command_velocity_;
    InputInterface<rmcs_msgs::ChassisMode> navigation_chassis_behavior_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    bool spinning_forward_ = true;

    std::size_t spin_stuck_count_ = 0;
    std::size_t spin_recovery_count_ = 0;
    rmcs_msgs::ChassisMode mode_before_watchdog_ = rmcs_msgs::ChassisMode::AUTO;

    pid::PidCalculator following_velocity_controller_{
        node::param_or("following_velocity_kp", 8.0),
        node::param_or("following_velocity_ki", 0.0),
        node::param_or("following_velocity_kd", 0.0),
    };

    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)
