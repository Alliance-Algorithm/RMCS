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

class WheelLegJointVelocityController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
    using Geometry = WheelLegJointPairGeometry;

public:
    WheelLegJointVelocityController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/wheel_leg/joint_enable", joint_enable_, false);
        register_input("/wheel_leg/joint_control_active", joint_control_active_, false);
        register_input("/chassis/reset_count", reset_count_);
        register_output("/wheel_leg/joint_controller/healthy", healthy_, false);
        register_output("/wheel_leg/joint_controller/fault_reason", fault_reason_, std::string{});
        constexpr std::array<const char*, 4> kJoints{
            "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
        for (std::size_t i = 0; i < kJoints.size(); ++i) {
            const std::string base = std::string{"/wheel_leg/"} + kJoints[i];
            register_input(base + "/angle", position_[i], false);
            register_input(base + "/velocity", velocity_[i], false);
            register_input(base + "/feedback_valid", feedback_valid_[i], false);
            register_input(base + "/control_angle", target_[i], false);
            register_output(base + "/control_velocity", command_[i], 0.0);
        }
        config_.angle_kp = get_parameter_or<double>("angle_kp", 6.0);
        config_.max_velocity = get_parameter_or<double>("max_joint_velocity", 2.0);
        config_.max_acceleration = get_parameter_or<double>("max_joint_acceleration", 4.0);
        config_.min_difference =
            get_parameter_or<double>("min_motor_difference", Geometry::kDefaultMinDifference);
        config_.max_difference =
            get_parameter_or<double>("max_motor_difference", Geometry::kDefaultMaxDifference);
        target_margin_ = get_parameter_or<double>("motor_difference_margin", 0.04);
        feedback_tolerance_ = get_parameter_or<double>("motor_difference_tolerance", 0.03);
        max_feedback_speed_ = get_parameter_or<double>("max_feedback_speed", 45.0);
        max_feedback_jump_ = get_parameter_or<double>("max_feedback_jump", 0.5);
        if (!std::isfinite(config_.angle_kp) || config_.angle_kp <= 0.0
            || !std::isfinite(config_.max_velocity) || config_.max_velocity <= 0.0
            || !std::isfinite(config_.max_acceleration) || config_.max_acceleration <= 0.0
            || !std::isfinite(config_.min_difference) || !std::isfinite(config_.max_difference)
            || !std::isfinite(target_margin_) || target_margin_ < 0.0
            || config_.min_difference + 2.0 * target_margin_ >= config_.max_difference
            || !std::isfinite(feedback_tolerance_) || feedback_tolerance_ < 0.0
            || config_.min_difference - feedback_tolerance_ <= -std::numbers::pi
            || config_.max_difference + feedback_tolerance_ >= std::numbers::pi
            || !std::isfinite(max_feedback_speed_) || max_feedback_speed_ <= 0.0
            || !std::isfinite(max_feedback_jump_) || max_feedback_jump_ <= 0.0
            || max_feedback_jump_ >= std::numbers::pi)
            throw std::invalid_argument("WheelLegJointVelocityController: invalid parameter");
    }

    void update() override { update_at(Clock::now()); }

    // Use the same component in an offline, deterministic physics simulation.
    // The runtime entry above always supplies the real steady clock.
    void update_at(Clock::time_point now) {
        const bool reset = reset_count_.ready() && *reset_count_ != last_reset_count_;
        if (reset)
            last_reset_count_ = *reset_count_;
        if (reset || !joint_enable_.ready() || !*joint_enable_) {
            reset_();
            return;
        }
        if (fault_latched_)
            return;
        for (std::size_t i = 0; i < position_.size(); ++i) {
            if (!position_[i].ready() || !velocity_[i].ready() || !feedback_valid_[i].ready()
                || !*feedback_valid_[i] || !target_[i].ready() || !std::isfinite(*position_[i])
                || !std::isfinite(*velocity_[i]) || !std::isfinite(*target_[i])) {
                unavailable_("joint " + std::to_string(i) + " feedback or target invalid");
                return;
            }
            const double delta = std::abs(Geometry::wrap(*position_[i] - last_position_[i]));
            const double elapsed = std::chrono::duration<double>{now - last_change_[i]}.count();
            if (std::abs(*velocity_[i]) > max_feedback_speed_
                || (feedback_initialized_
                    && (delta > max_feedback_jump_
                        || delta > max_feedback_speed_ * elapsed + 0.02))) {
                unavailable_("joint " + std::to_string(i) + " feedback discontinuity");
                return;
            }
            if (!feedback_initialized_ || delta > 0.0)
                last_change_[i] = now;
            last_position_[i] = *position_[i];
        }
        std::array<WheelLegPairPose, 2> measured;
        std::array<WheelLegPairPose, 2> desired;
        for (std::size_t pair = 0; pair < 2; ++pair) {
            const auto side = pair == 0 ? Geometry::Side::kLeft : Geometry::Side::kRight;
            const auto first = 2 * pair;
            const auto pose = Geometry::feedback(
                side, *position_[first], *position_[first + 1], config_.min_difference,
                config_.max_difference, feedback_tolerance_);
            if (!pose) {
                unavailable_(
                    std::string{pair == 0 ? "left" : "right"}
                    + " pair outside V5 assembly branch, d="
                    + std::to_string(
                        Geometry::decode(side, *position_[first], *position_[first + 1])
                            .difference));
                return;
            }
            measured[pair] = *pose;
            desired[pair] = Geometry::target(
                side, *target_[first], *target_[first + 1], config_.min_difference,
                config_.max_difference, target_margin_);
        }
        if (!feedback_initialized_)
            RCLCPP_INFO(
                get_logger(),
                "[joint pair] initial inner angle: left=%.2f right=%.2f deg; "
                "instantaneous phase feedback, no turn accumulation",
                Geometry::inner_angle_degrees(measured[0].difference),
                Geometry::inner_angle_degrees(measured[1].difference));
        feedback_initialized_ = true;
        *fault_reason_ = "";
        *healthy_ = true;
        if (!joint_control_active_.ready() || !*joint_control_active_ || !was_active_) {
            zero_commands_();
            last_update_ = now;
            was_active_ = joint_control_active_.ready() && *joint_control_active_;
            return;
        }
        const double dt = std::chrono::duration<double>{now - last_update_}.count();
        last_update_ = now;
        if (dt <= 0.0 || dt > 0.05) {
            unavailable_("controller update interval invalid");
            return;
        }
        control_started_ = true;
        for (std::size_t pair = 0; pair < 2; ++pair) {
            const auto side = pair == 0 ? Geometry::Side::kLeft : Geometry::Side::kRight;
            previous_command_[pair] = wheel_leg_pair_velocity(
                side, measured[pair], desired[pair], previous_command_[pair], config_, dt);
            *command_[2 * pair] = previous_command_[pair].hip;
            *command_[2 * pair + 1] = previous_command_[pair].knee;
        }
    }

private:
    void zero_commands_() {
        previous_command_ = {};
        for (auto& command : command_)
            *command = 0.0;
    }

    void unavailable_(const std::string& reason) {
        zero_commands_();
        *healthy_ = false;
        *fault_reason_ = reason;
        was_active_ = false;
        // Startup feedback can recover; a running fault needs explicit reset.
        if (control_started_) {
            fault_latched_ = true;
            RCLCPP_ERROR(get_logger(), "[joint pair] stopped: %s", reason.c_str());
        } else
            feedback_initialized_ = false;
    }

    void reset_() {
        zero_commands_();
        *healthy_ = false;
        *fault_reason_ = "";
        feedback_initialized_ = false;
        control_started_ = false;
        was_active_ = false;
        fault_latched_ = false;
        last_position_ = {};
        last_change_ = {};
        last_update_ = {};
    }

    InputInterface<bool> joint_enable_;
    InputInterface<bool> joint_control_active_;
    InputInterface<std::size_t> reset_count_;
    std::array<InputInterface<double>, 4> position_;
    std::array<InputInterface<double>, 4> velocity_;
    std::array<InputInterface<bool>, 4> feedback_valid_;
    std::array<InputInterface<double>, 4> target_;
    std::array<OutputInterface<double>, 4> command_;
    OutputInterface<bool> healthy_;
    OutputInterface<std::string> fault_reason_;

    WheelLegPairVelocityConfig config_;
    double target_margin_ = 0.04;
    double feedback_tolerance_ = 0.03;
    double max_feedback_speed_ = 45.0;
    double max_feedback_jump_ = 0.5;
    std::array<WheelLegJointPair, 2> previous_command_{};
    std::array<double, 4> last_position_{};
    std::array<Clock::time_point, 4> last_change_{};
    Clock::time_point last_update_{};
    std::size_t last_reset_count_ = 0;
    bool feedback_initialized_ = false;
    bool control_started_ = false;
    bool was_active_ = false;
    bool fault_latched_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegJointVelocityController, rmcs_executor::Component)
