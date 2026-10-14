#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_arm_dwell.hpp"

namespace rmcs_core::controller::wheelleg {

using ArmDwell = rmcs_core::controller::identification::WheelLegArmDwell;

// STAND state machine (top layer, no self-righting).
//
// Operator contract (matches the rest of the real-robot graph):
//   * both DOWN            -> fully disabled (disarmed; drive bridge zeroes torque)
//   * both MIDDLE (dwelled)-> armed
//
// On arm it captures the measured near-standing pose and drives the leg toward
// the nominal stance (the trajectory controller ramps the joint references from
// the measured state; the planner holds the nominal leg length and zero
// velocity; the ADRC balance controller holds pitch). It reports
//
//   /wheelleg/stand/armed   bool
//   /wheelleg/stand/phase   0 disabled | 1 converging | 2 stable
//   /wheelleg/stand/ready   bool   (stable for `stable_seconds`)
//
// `ready` is the hand-off condition for the MOVE state machine (layered: STAND
// is the prerequisite of MOVE), so a positive velocity command is only accepted
// once the robot is standing stably.
//
// This layer owns no motor command itself; it only gates/observes. The pose is
// produced by the shared command -> trajectory -> balance chain.
class WheelLegStandStateMachine
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegStandStateMachine()
        : Node{get_component_name(), node::options()} {
        load_config_();

        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_, false);

        register_input("/wheelleg/state/leg_length", state_leg_length_, false);
        register_input("/wheelleg/state/knee_angle", state_knee_angle_, false);
        register_input("/wheelleg/state/shank_angle", state_shank_, false);
        register_input("/wheelleg/state/pitch", state_pitch_, false);
        register_input("/wheelleg/state/pitch_velocity", state_pitch_velocity_, false);
        register_input("/wheelleg/state/wheel_velocity", state_wheel_velocity_, false);

        register_output("/wheelleg/stand/armed", armed_out_, false);
        register_output("/wheelleg/stand/phase", phase_out_, std::uint8_t{0});
        register_output("/wheelleg/stand/ready", ready_out_, false);
    }

    void before_updating() override {
        arm_ = {};
        reset_();
    }

    void update() override {
        using Switch = rmcs_msgs::Switch;
        const bool fresh = dr16_fresh_.ready() && *dr16_fresh_;
        const auto left = fresh ? *switch_left_ : Switch::UNKNOWN;
        const auto right = fresh ? *switch_right_ : Switch::UNKNOWN;

        if (arm_.update(Clock::now(), fresh, left, right, arm_dwell_s_))
            armed_ = true;
        if (left == Switch::DOWN && right == Switch::DOWN)
            reset_();

        if (!armed_) {
            *phase_out_ = kDisabled;
            *ready_out_ = false;
            *armed_out_ = false;
            return;
        }
        *armed_out_ = true;

        if (!converged_()) {
            *phase_out_ = kConverging;
            *ready_out_ = false;
            stable_elapsed_ = 0.0;
            return;
        }

        stable_elapsed_ += dt_;
        if (stable_elapsed_ >= stable_seconds_) {
            *phase_out_ = kStable;
            *ready_out_ = true;
        } else {
            *phase_out_ = kConverging;
            *ready_out_ = false;
        }

        if (dbg_count_++ % 500 == 0)
            info(
                "stand armed phase={} L0={:.3f}/{:.3f} pitch={:.4f} ready={}",
                static_cast<int>(*phase_out_),
                state_leg_length_.ready() ? *state_leg_length_ : kNaN, nominal_leg_length_,
                state_pitch_.ready() ? *state_pitch_ : kNaN, *ready_out_);
    }

private:
    using Clock = ArmDwell::Clock;

    enum Phase : std::uint8_t { kDisabled = 0, kConverging = 1, kStable = 2 };

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    void reset_() {
        armed_ = false;
        stable_elapsed_ = 0.0;
    }

    [[nodiscard]] bool converged_() const {
        if (!state_leg_length_.ready() || !state_pitch_.ready())
            return false;
        if (std::abs(*state_leg_length_ - nominal_leg_length_) > leg_tolerance_)
            return false;
        if (std::abs(*state_pitch_) > pitch_tolerance_)
            return false;
        if (state_wheel_velocity_.ready() && std::abs(*state_wheel_velocity_) > wheel_velocity_tolerance_)
            return false;
        return true;
    }

    void load_config_() {
        dt_ = node::param_or("dt", 0.001);
        nominal_leg_length_ = node::param_or("nominal_leg_length", 0.334);
        leg_tolerance_ = node::param_or("leg_tolerance", 0.02);
        pitch_tolerance_ = node::param_or("pitch_tolerance_deg", 6.0) * std::numbers::pi / 180.0;
        wheel_velocity_tolerance_ = node::param_or("wheel_velocity_tolerance", 0.5);
        stable_seconds_ = node::param_or("stable_seconds", 0.5);
        arm_dwell_s_ = node::param_or("arm_dwell_s", 0.5);
    }

    double dt_ = 0.001;
    double nominal_leg_length_ = 0.334;
    double leg_tolerance_ = 0.02;
    double pitch_tolerance_ = 6.0 * std::numbers::pi / 180.0;
    double wheel_velocity_tolerance_ = 0.5;
    double stable_seconds_ = 0.5;
    double arm_dwell_s_ = 0.5;

    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_;
    InputInterface<double> state_leg_length_, state_knee_angle_, state_shank_;
    InputInterface<double> state_pitch_, state_pitch_velocity_, state_wheel_velocity_;

    OutputInterface<bool> armed_out_, ready_out_;
    OutputInterface<std::uint8_t> phase_out_;

    ArmDwell arm_;
    bool armed_ = false;
    double stable_elapsed_ = 0.0;
    std::uint64_t dbg_count_ = 0;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegStandStateMachine, rmcs_executor::Component)
