#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Self-righting (get-up) trajectory source for the wheel-leg.
//
// It produces the recovery pose on the task interface consumed by
// WheelLegTrajectoryController (which blends it with the balance trajectory by
// the alpha_r weight):
//   /wheelleg/task/leg_length  recovery leg length L0
//   /wheelleg/task/shank       recovery leg orientation (shank angle phi)
//   /wheelleg/task/recovery    alpha_r (0 = recovery, 1 = balance)
//
// Trajectory: two TIME-parameterised segments in the (L0, phi) task space
// (执行路线1 §4: L0 = leg length, phi = equivalent shank angle). Both DOF move
// together along each segment, so nothing snaps:
//   T1 PLANT : from the measured pose to a "wheels planted" pose over
//              plant_seconds; switches to T2 as soon as the wheel carries load.
//   T2 STAND : from the planted pose to the standing pose over stand_seconds.
// alpha_r ramps 1 -> 0 on activation and 0 -> 1 after the stance is reached, so
// the swap in the trajectory controller is continuous.
//
// NOTE: the two DOF are NOT independent in time: the segment duration is fixed
// and both L0 and phi interpolate across it, so the leg never changes length at
// a different rate than it changes orientation (a shared rate would shorten the
// leg ~70x faster than the shank sweep, which flung the robot).
class WheelLegRecoveryController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegRecoveryController()
        : Node{get_component_name(), node::options()} {
        load_config_();

        register_input("/wheelleg/state/leg_length", state_leg_length_);
        register_input("/wheelleg/state/shank_angle", state_shank_);
        register_input("/wheelleg/state/pitch", state_pitch_);
        register_input("/wheelleg/state/contact_normal_l", contact_l_, false);
        register_input("/wheelleg/state/contact_normal_r", contact_r_, false);

        register_output("/wheelleg/task/leg_length", out_leg_length_, kNaN);
        register_output("/wheelleg/task/shank", out_shank_, kNaN);
        register_output("/wheelleg/task/recovery", out_recovery_, 1.0);

        register_input(trigger_topic_, trigger_, false);
    }

    void update() override {
        const bool triggered = start_on_boot_ || (trigger_.ready() && *trigger_);
        if (!triggered) {
            phase_ = Phase::kIdle;
            *out_leg_length_ = kNaN;
            *out_shank_ = kNaN;
            *out_recovery_ = 1.0;
            return;
        }
        if (!state_leg_length_.ready() || !state_shank_.ready() || !state_pitch_.ready()) {
            *out_leg_length_ = kNaN;
            *out_shank_ = kNaN;
            *out_recovery_ = 1.0;
            return;
        }

        const double l0 = *state_leg_length_;
        const double shank = *state_shank_;
        const double pitch = *state_pitch_;

        if (phase_ == Phase::kIdle) {
            // Start from the measured pose (continuous on activation) and hold it
            // for hold_seconds so the bridge/plant has settled before moving.
            ref_l0_ = l0;
            ref_shank_ = shank;
            alpha_r_ = 0.0;
            elapsed_ = 0.0;
            if (start_phase_ == "done") {
                // Skip the get-up and hand straight to the stand chain
                // (planner + trajectory balance). Used to test the hand-off.
                phase_ = Phase::kDone;
            } else {
                start_segment_(plant_l0_, plant_shank_);
                phase_ = Phase::kPlant;
            }
        }
        elapsed_ += dt_;

        const bool loaded = (contact_l_.ready() ? *contact_l_ : 0.0) > contact_eps_
                         || (contact_r_.ready() ? *contact_r_ : 0.0) > contact_eps_;

        // The switch ALWAYS runs so alpha_r is set every tick (alpha_r = 0 means
        // "recovery owns the leg"); only the actual step is held back until the
        // hold time elapses. (Gating the whole switch left alpha_r stuck at its
        // initial 1.0 during the hold -> the TC ran balance and flung the robot.)
        switch (phase_) {
        case Phase::kPlant:
            if (elapsed_ >= hold_seconds_)
                step_segment_();
            if (loaded) {
                start_segment_(stand_l0_, stand_shank_);
                phase_ = Phase::kStand;
            }
            alpha_r_ = 0.0;
            break;
        case Phase::kStand:
            if (elapsed_ >= hold_seconds_)
                step_segment_();
            // Only hand off once the pose has actually REACHED the stance (no
            // snap): the leg is never yanked to the target.
            if (segment_reached_() && loaded && std::abs(pitch) < handover_tilt_)
                phase_ = Phase::kDone;
            alpha_r_ = 0.0;
            break;
        case Phase::kDone:
        default:
            // Hold the reached pose; ramp alpha_r so the balance takes over.
            // Also honour the start hold, so the viewer can catch the hand-off.
            if (elapsed_ >= hold_seconds_)
                alpha_r_ = std::min(1.0, alpha_r_ + dt_ / std::max(handoff_seconds_, kEpsilon));
            break;
        }

        // Body-pitch feedback on the leg orientation: while the wheel is planted
        // the leg angle rotates the body, so add k * theta to drive theta -> 0.
        const double shank_cmd = ref_shank_ + theta_kp_ * pitch;

        // Debug: the first few ticks (initial transient), then every 500 ticks.
        if (dbg_count_ < 10 || dbg_count_ % 500 == 0)
            info(
                "recovery {} t={:.3f} L0={:.4f}/{:.4f} shank={:.4f}/{:.4f} "
                "pitch={:.4f} loaded={} alpha_r={:.3f}",
                std::string_view{phase_name_()}, elapsed_, ref_l0_, l0,
                shank_cmd, shank, pitch, loaded, alpha_r_);
        ++dbg_count_;

        *out_leg_length_ = ref_l0_;
        *out_shank_ = shank_cmd;
        *out_recovery_ = alpha_r_;
    }

private:
    enum class Phase { kIdle, kPlant, kStand, kDone };

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kEpsilon = 1e-9;

    // Velocity-limited segment: the reference approaches the goal at bounded
    // rates (rad/s for the shank, m/s for the length), taking the SHORT way
    // around for the ground-referenced shank angle. A fixed-duration segment
    // forces a high velocity, and when the leg has been carried off by gravity
    // the PD needs a huge torque to keep up -- which flung the robot. Bounding
    // the rate lets the leg follow the plan instead.
    void start_segment_(double goal_l0, double goal_shank) {
        seg_goal_l0_ = goal_l0;
        seg_goal_shank_ = ref_shank_ + wrap_to_pi_(goal_shank - ref_shank_);
        seg_t_ = 0.0;
    }

    void step_segment_() {
        seg_t_ += dt_;
        ref_l0_ += std::clamp(seg_goal_l0_ - ref_l0_, -l0_rate_ * dt_, l0_rate_ * dt_);
        ref_shank_ +=
            std::clamp(seg_goal_shank_ - ref_shank_, -shank_rate_ * dt_, shank_rate_ * dt_);
    }

    [[nodiscard]] bool segment_reached_() const {
        return std::abs(seg_goal_l0_ - ref_l0_) < 0.002
            && std::abs(seg_goal_shank_ - ref_shank_) < 0.01;
    }

    [[nodiscard]] static double wrap_to_pi_(double a) {
        return std::remainder(a, 2.0 * std::numbers::pi);
    }

    [[nodiscard]] const char* phase_name_() const {
        switch (phase_) {
        case Phase::kPlant: return "plant";
        case Phase::kStand: return "stand";
        case Phase::kDone: return "done";
        default: return "idle";
        }
    }

    void load_config_() {
        dt_ = node::param_or("dt", 0.001);
        trigger_topic_ =
            node::param_or<std::string>("trigger_topic", std::string{"/wheelleg/recovery/trigger"});
        start_on_boot_ = node::param_or("start_on_boot", false);
        // "auto" = run the get-up; "done" = skip it and hand straight to the
        // stand chain (for testing the alpha_r hand-off from a near-stance start).
        start_phase_ = node::param_or<std::string>("start_phase", std::string{"auto"});

        // T1 PLANT waypoint: legs down, wheels reaching the ground.
        plant_l0_ = node::param_or("plant_leg_length", 0.22);
        plant_shank_ = node::param_or("plant_shank", 0.2);

        // T2 STAND waypoint: the standing pose.
        stand_l0_ = node::param_or("stand_leg_length", 0.334);
        stand_shank_ = node::param_or("stand_shank", 2.26);

        // Bounded reference rates (velocity limit of the get-up plan).
        shank_rate_ = node::param_or("shank_rate", 0.5);  // rad/s
        l0_rate_ = node::param_or("l0_rate", 0.03);       // m/s

        theta_kp_ = node::param_or("theta_kp", 0.0);

        contact_eps_ = node::param_or("contact_force_eps", 1.0);
        handover_tilt_ = node::param_or("handover_tilt_deg", 8.0) * std::numbers::pi / 180.0;
        handoff_seconds_ = node::param_or("handoff_seconds", 0.5);
        hold_seconds_ = node::param_or("start_hold_seconds", 0.1);
    }

    double dt_;
    std::string trigger_topic_;
    bool start_on_boot_ = false;
    std::string start_phase_ = "auto";

    double plant_l0_, plant_shank_;
    double stand_l0_, stand_shank_;
    double shank_rate_, l0_rate_;
    double theta_kp_;
    double contact_eps_;
    double handover_tilt_;
    double handoff_seconds_;
    double hold_seconds_ = 0.1;

    Phase phase_ = Phase::kIdle;
    double ref_l0_ = 0.0;
    double ref_shank_ = 0.0;
    double alpha_r_ = 1.0;
    double elapsed_ = 0.0;

    // Current trajectory segment.
    double seg_t_ = 0.0;
    double seg_goal_l0_ = 0.0;
    double seg_goal_shank_ = 0.0;

    InputInterface<double> state_leg_length_;
    InputInterface<double> state_shank_;
    InputInterface<double> state_pitch_;
    InputInterface<double> contact_l_;
    InputInterface<double> contact_r_;
    InputInterface<bool> trigger_;

    OutputInterface<double> out_leg_length_;
    OutputInterface<double> out_shank_;
    OutputInterface<double> out_recovery_;

    std::uint64_t dbg_count_ = 0;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegRecoveryController, rmcs_executor::Component)
