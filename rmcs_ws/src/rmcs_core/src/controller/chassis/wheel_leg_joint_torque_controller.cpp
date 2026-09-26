#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "wheel_leg_joint_pair_geometry.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegJointTorqueController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Clock = std::chrono::steady_clock;

public:
    WheelLegJointTorqueController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/wheel_leg/joint_enable", joint_enable_, false);
        register_input("/wheel_leg/joint_mit_active", joint_mit_active_, false);
        register_input("/chassis/reset_count", reset_count_);
        register_output("/wheel_leg/joint_controller/healthy", healthy_, false);

        constexpr std::array<const char*, 4> kJoints{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < kJoints.size(); ++i) {
            const std::string base = std::string{"/wheel_leg/"} + kJoints[i];
            register_input(base + "/angle", position_[i], false);
            register_input(base + "/velocity", velocity_[i], false);
            register_input(base + "/feedback_valid", feedback_valid_[i], false);
            register_input(base + "/control_angle", target_[i], false);
            register_output(base + "/control_torque", torque_[i], 0.0);
        }

        kp_ = get_parameter_or<double>("joint_kp", 30.0);
        kd_ = get_parameter_or<double>("joint_kd", 1.0);
        max_torque_ = get_parameter_or<double>("max_torque", 40.0);
        max_reference_speed_ = get_parameter_or<double>("max_reference_speed", 6.0);
        max_following_error_ = get_parameter_or<double>("max_following_error", 0.75);
        max_feedback_speed_ = get_parameter_or<double>("max_feedback_speed", 45.0);
        max_feedback_jump_ = get_parameter_or<double>("max_feedback_jump", 3.0);
        min_motor_difference_ = get_parameter_or<double>(
            "min_motor_difference", WheelLegJointPairGeometry::kDefaultMinDifference);
        max_motor_difference_ = get_parameter_or<double>(
            "max_motor_difference", WheelLegJointPairGeometry::kDefaultMaxDifference);
        motor_difference_margin_ = get_parameter_or<double>("motor_difference_margin", 0.04);
        motor_difference_feedback_tolerance_ =
            get_parameter_or<double>("motor_difference_feedback_tolerance", 0.04);
        if (!std::isfinite(kp_) || kp_ < 0.0 || !std::isfinite(kd_) || kd_ < 0.0
            || !std::isfinite(max_torque_) || max_torque_ <= 0.0 || max_torque_ > 40.0
            || !std::isfinite(max_reference_speed_) || max_reference_speed_ <= 0.0
            || !std::isfinite(max_following_error_) || max_following_error_ <= 0.0
            || !std::isfinite(max_feedback_speed_) || max_feedback_speed_ <= 0.0
            || !std::isfinite(max_feedback_jump_) || max_feedback_jump_ <= 0.0
            || max_feedback_jump_ >= std::numbers::pi || !std::isfinite(min_motor_difference_)
            || !std::isfinite(max_motor_difference_)
            || min_motor_difference_ >= max_motor_difference_
            || !std::isfinite(motor_difference_margin_) || motor_difference_margin_ < 0.0
            || 2.0 * motor_difference_margin_ >= max_motor_difference_ - min_motor_difference_
            || !std::isfinite(motor_difference_feedback_tolerance_)
            || motor_difference_feedback_tolerance_ < 0.0
            || max_motor_difference_ - min_motor_difference_
                       + 2.0 * motor_difference_feedback_tolerance_
                   >= WheelLegJointPairGeometry::kPeriod)
            throw std::invalid_argument("WheelLegJointTorqueController: invalid control parameter");
    }

    void update() override {
        if (reset_count_.ready() && *reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            reset_();
            zero_outputs_();
            return;
        }
        if (!joint_enable_.ready() || !*joint_enable_) {
            reset_();
            zero_outputs_();
            return;
        }
        if (fault_latched_) {
            zero_outputs_();
            return;
        }

        for (std::size_t i = 0; i < target_.size(); ++i) {
            if (!position_[i].ready() || !velocity_[i].ready() || !feedback_valid_[i].ready()
                || !*feedback_valid_[i] || !target_[i].ready() || !std::isfinite(*position_[i])
                || !std::isfinite(*velocity_[i]) || !std::isfinite(*target_[i])) {
                if (trajectory_initialized_)
                    latch_fault_("joint feedback or target became invalid");
                zero_outputs_();
                return;
            }
        }

        const auto now = Clock::now();
        if (!update_feedback_history_(now)) {
            zero_outputs_();
            return;
        }
        if (!update_pair_feedback_()) {
            zero_outputs_();
            return;
        }
        if (!joint_mit_active_.ready() || !*joint_mit_active_) {
            for (std::size_t i = 0; i < reference_.size(); ++i)
                reference_[i] = measured_[i];
            last_update_ = now;
            trajectory_initialized_ = true;
            for (auto& output : torque_)
                *output = 0.0;
            *healthy_ = true;
            return;
        }
        if (!trajectory_initialized_) {
            for (std::size_t i = 0; i < reference_.size(); ++i)
                reference_[i] = measured_[i];
            last_update_ = now;
            trajectory_initialized_ = true;
        } else {
            const double dt = std::chrono::duration<double>{now - last_update_}.count();
            if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.05) {
                latch_fault_("controller update interval is invalid");
                zero_outputs_();
                return;
            }
            last_update_ = now;

            for (std::size_t i = 0; i < reference_.size(); ++i) {
                const double position = measured_[i];
                if (std::abs(reference_[i] - position) > max_following_error_) {
                    latch_fault_("joint exceeded the trajectory following limit");
                    zero_outputs_();
                    return;
                }
            }

            // Lift the two targets together to one feasible multi-turn branch.
            // Linear interpolation then keeps the motor difference within its limits.
            for (std::size_t first : {std::size_t{0}, std::size_t{2}}) {
                const auto side = first == 0 ? WheelLegJointPairGeometry::Side::kLeft
                                             : WheelLegJointPairGeometry::Side::kRight;
                const auto target = WheelLegJointPairGeometry::nearest_feasible_target(
                    side, reference_[first], reference_[first + 1], *target_[first],
                    *target_[first + 1], min_motor_difference_, max_motor_difference_,
                    motor_difference_margin_);
                if (!target) {
                    latch_fault_("joint pair target is invalid");
                    zero_outputs_();
                    return;
                }
                if (target->difference_clamped)
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 1000,
                        "WheelLegJointTorqueController: projected an unreachable motor pair "
                        "target");
                const double hip_delta = target->hip - reference_[first];
                const double knee_delta = target->knee - reference_[first + 1];
                const double remaining = std::max(std::abs(hip_delta), std::abs(knee_delta));
                if (remaining == 0.0)
                    continue;
                const double progress = std::min(1.0, max_reference_speed_ * dt / remaining);
                reference_[first] += progress * hip_delta;
                reference_[first + 1] += progress * knee_delta;
            }
        }

        for (std::size_t i = 0; i < torque_.size(); ++i) {
            const double error = reference_[i] - measured_[i];
            if (std::abs(error) > max_following_error_) {
                latch_fault_("joint exceeded the trajectory following limit");
                zero_outputs_();
                return;
            }
            *torque_[i] = std::clamp(kp_ * error - kd_ * *velocity_[i], -max_torque_, max_torque_);
        }
        *healthy_ = true;
    }

private:
    bool update_pair_feedback_() {
        if (!pair_feedback_initialized_) {
            for (std::size_t pair = 0; pair < knee_offset_.size(); ++pair) {
                const std::size_t first = 2 * pair;
                const auto side = pair == 0 ? WheelLegJointPairGeometry::Side::kLeft
                                            : WheelLegJointPairGeometry::Side::kRight;
                const auto offset = WheelLegJointPairGeometry::feedback_knee_offset(
                    side, *position_[first], *position_[first + 1], min_motor_difference_,
                    max_motor_difference_, motor_difference_feedback_tolerance_);
                if (!offset) {
                    latch_fault_("joint pair feedback is outside the mechanical branch");
                    return false;
                }
                knee_offset_[pair] = *offset;
            }
            pair_feedback_initialized_ = true;
        }
        for (std::size_t pair = 0; pair < knee_offset_.size(); ++pair) {
            const std::size_t first = 2 * pair;
            const auto side = pair == 0 ? WheelLegJointPairGeometry::Side::kLeft
                                        : WheelLegJointPairGeometry::Side::kRight;
            measured_[first] = *position_[first];
            measured_[first + 1] = *position_[first + 1] + knee_offset_[pair];
            const double difference =
                WheelLegJointPairGeometry::difference(side, measured_[first], measured_[first + 1]);
            if (difference < min_motor_difference_ - motor_difference_feedback_tolerance_
                || difference > max_motor_difference_ + motor_difference_feedback_tolerance_) {
                latch_fault_("joint pair exceeded the mechanical difference limit");
                return false;
            }
        }
        return true;
    }

    bool update_feedback_history_(Clock::time_point now) {
        for (std::size_t i = 0; i < last_position_.size(); ++i) {
            const double position = *position_[i];
            const double delta = std::abs(position - last_position_[i]);
            if (trajectory_initialized_ && delta > 0.0) {
                const double elapsed =
                    std::chrono::duration<double>{now - last_position_change_time_[i]}.count();
                if (!std::isfinite(elapsed) || elapsed < 0.0 || delta > max_feedback_jump_
                    || delta > max_feedback_speed_ * elapsed + 0.1) {
                    latch_fault_("joint angle feedback is discontinuous");
                    return false;
                }
            }
            if (!trajectory_initialized_ || delta > 0.0)
                last_position_change_time_[i] = now;
            last_position_[i] = position;
        }
        return true;
    }

    void reset_() {
        trajectory_initialized_ = false;
        fault_latched_ = false;
        pair_feedback_initialized_ = false;
        reference_.fill(0.0);
        measured_.fill(0.0);
        knee_offset_.fill(0.0);
        last_position_.fill(0.0);
        last_position_change_time_.fill(Clock::time_point{});
        last_update_ = Clock::time_point{};
    }

    void latch_fault_(const char* reason) {
        if (!fault_latched_)
            RCLCPP_ERROR(get_logger(), "WheelLegJointTorqueController: %s", reason);
        fault_latched_ = true;
    }

    void zero_outputs_() {
        for (auto& output : torque_)
            *output = 0.0;
        *healthy_ = false;
    }

    InputInterface<bool> joint_enable_;
    InputInterface<bool> joint_mit_active_;
    InputInterface<std::size_t> reset_count_;
    std::array<InputInterface<double>, 4> position_;
    std::array<InputInterface<double>, 4> velocity_;
    std::array<InputInterface<bool>, 4> feedback_valid_;
    std::array<InputInterface<double>, 4> target_;
    std::array<OutputInterface<double>, 4> torque_;
    OutputInterface<bool> healthy_;

    std::array<double, 4> reference_{};
    std::array<double, 4> measured_{};
    std::array<double, 2> knee_offset_{};
    std::array<double, 4> last_position_{};
    std::array<Clock::time_point, 4> last_position_change_time_{};
    Clock::time_point last_update_{};
    std::size_t last_reset_count_ = 0;
    bool trajectory_initialized_ = false;
    bool pair_feedback_initialized_ = false;
    bool fault_latched_ = false;

    double kp_ = 30.0;
    double kd_ = 1.0;
    double max_torque_ = 40.0;
    double max_reference_speed_ = 6.0;
    double max_following_error_ = 0.75;
    double max_feedback_speed_ = 45.0;
    double max_feedback_jump_ = 3.0;
    double min_motor_difference_ = WheelLegJointPairGeometry::kDefaultMinDifference;
    double max_motor_difference_ = WheelLegJointPairGeometry::kDefaultMaxDifference;
    double motor_difference_margin_ = 0.04;
    double motor_difference_feedback_tolerance_ = 0.04;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegJointTorqueController, rmcs_executor::Component)
