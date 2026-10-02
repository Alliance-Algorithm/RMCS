#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Motion command planner for the wheel-leg chassis. It is the single producer
// of the /wheelleg/command/* interface consumed by the trajectory / balance /
// yaw / roll controllers, so the command layer is shared by simulation and
// hardware (the MuJoCo bridge is a pure plant + state source).
//
// Interface: a body-frame twist goal (v, omega):
//   /wheelleg/move/velocity  (m/s,   body forward)
//   /wheelleg/move/yaw_rate  (rad/s)
// A coordinated jerk/acceleration-limited profile smooths both channels, so any
// goal change (step, mode switch, simultaneous translation+rotation) becomes a
// continuous reference instead of a command jump. The yaw angle command is the
// integral of the planned yaw rate.
//
// Test-only behaviours (params) generate the twist goal internally:
//   behavior = "twist"  : follow /wheelleg/move/*  (default)
//   behavior = "square" : legacy square-wave drive/turn test
//   behavior = "patrol" : straight -> pivot 180deg -> straight, back and forth
class WheelLegMotionPlanner
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegMotionPlanner()
        : Node{get_component_name(), node::options()} {
        load_config_();

        register_input("/wheelleg/move/velocity", move_velocity_, false);
        register_input("/wheelleg/move/yaw_rate", move_yaw_rate_, false);
        register_input("/wheelleg/state/yaw", state_yaw_, false);
        register_input("/wheelleg/state/roll", state_roll_, false);

        register_output("/wheelleg/command/velocity", command_velocity_output_, 0.0);
        register_output("/wheelleg/command/yaw_rate", command_yaw_rate_output_, 0.0);
        register_output("/wheelleg/command/yaw", command_yaw_output_, 0.0);
        register_output("/wheelleg/command/acceleration", command_acceleration_output_, 0.0);
        register_output("/wheelleg/command/leg_length", command_leg_length_output_, kNaN);
        // Lateral plan (combined translation + turning): the bank angle the body
        // must hold for the commanded centripetal acceleration, and the matching
        // differential-leg-length feedforward (roll bias).
        register_output("/wheelleg/command/bank", bank_output_, 0.0);
        register_output("/wheelleg/command/roll_bias_ff", roll_bias_ff_output_, 0.0);
        // Roll tracking error (measured roll - bank): the roll PD acts on this so
        // its steady-state output is zero and it does not oppose the bank.
        register_output("/wheelleg/command/roll_error", roll_error_output_, 0.0);
    }

    void update() override {
        time_ += dt_;

        double v_goal = 0.0;
        double w_goal = 0.0;
        goal_(v_goal, w_goal);

        // Coordinated profile: bounded acceleration and jerk on both channels,
        // so translation and rotation can ramp together and switch smoothly.
        step_channel_(velocity_, accel_, v_goal, velocity_accel_, velocity_jerk_);
        step_channel_(yaw_rate_, yaw_accel_, w_goal, yaw_accel_limit_, yaw_jerk_);

        yaw_command_ += yaw_rate_ * dt_;
        if (std::isfinite(yaw_setpoint_cap_)) {
            yaw_command_ = yaw_cap_dir_ >= 0.0
                             ? std::min(yaw_command_, yaw_setpoint_cap_)
                             : std::max(yaw_command_, yaw_setpoint_cap_);
        }

        // Lateral acceleration plan, computed in real time from the commanded
        // twist: a steady (v, omega) turn needs the centripetal acceleration
        // a_lat = v*omega. The body must bank inward by phi = atan(a_lat/g) so
        // gravity + inertia stay along the body up-axis; the lateral CoM shift
        // is produced by a differential leg length (mirror of the fore-aft
        // accel -> leg-angle chain), fed forward as a differential knee bias.
        // a_lat is limited by the physical feasibility (friction + tip-over),
        // see update_inputs: lat_accel_limit_.
        const double a_lat = std::clamp(velocity_ * yaw_rate_, -lat_accel_limit_, lat_accel_limit_);
        const double bank = std::atan(a_lat / gravity_);
        *bank_output_ = bank;
        *roll_bias_ff_output_ = bank_to_roll_bias_ * bank;
        const double roll_meas =
            (state_roll_.ready() && std::isfinite(*state_roll_)) ? *state_roll_ : 0.0;
        *roll_error_output_ = roll_meas - bank;

        *command_velocity_output_ = velocity_;
        *command_yaw_rate_output_ = yaw_rate_;
        *command_yaw_output_ = yaw_command_;
        // The trajectory controller runs its own velocity S-curve and
        // outer-loop accel; keep the explicit-accel channel unused here.
        *command_acceleration_output_ = 0.0;
        *command_leg_length_output_ = leg_length_;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    // One jerk/acceleration-limited tracking channel: value tracks goal with
    // bounded rate (accel_limit) and rate-of-rate (jerk_limit).
    void step_channel_(
        double& value, double& rate, double goal, double accel_limit, double jerk_limit) const {
        const double rate_desired = std::clamp(
            channel_kp_ * (goal - value), -accel_limit, accel_limit);
        rate += std::clamp(rate_desired - rate, -jerk_limit * dt_, jerk_limit * dt_);
        value += rate * dt_;
    }

    void goal_(double& v_goal, double& w_goal) {
        if (behavior_ == "square")
            goal_square_(v_goal, w_goal);
        else if (behavior_ == "patrol")
            goal_patrol_(v_goal, w_goal);
        else if (behavior_ == "circle")
            goal_circle_(v_goal, w_goal);
        else if (behavior_ == "stadium")
            goal_stadium_(v_goal, w_goal);
        else // "twist"
            goal_twist_(v_goal, w_goal);

        v_goal = std::clamp(v_goal, -max_velocity_, max_velocity_);
        w_goal = std::clamp(w_goal, -max_yaw_rate_, max_yaw_rate_);
    }

    // Stadium (oval) track: straight at a high speed, then a 180deg arc driven at
    // a lower (non-zero) centreline speed -> a moving turn, not a pivot. Repeat,
    // always turning the same way, tracing two straights and two semicircular
    // ends like a running track.
    void goal_stadium_(double& v_goal, double& w_goal) {
        v_goal = 0.0;
        w_goal = 0.0;
        if (time_ < stadium_start_)
            return;
        if (!stadium_started_) {
            stadium_started_ = true;
            stadium_phase_ = kStadiumStraight;
            stadium_phase_start_ = time_;
        }
        if (stadium_phase_ == kStadiumStraight) {
            v_goal = stadium_straight_velocity_;
            if (time_ - stadium_phase_start_ >= stadium_straight_time_) {
                stadium_phase_ = kStadiumArc;
                stadium_phase_start_ = time_;
                stadium_turn_yaw_start_ = measured_yaw_();
                // Heading setpoint the arc ramps to; clamped so the ramp cannot
                // overshoot while the yaw loop lags (which would keep turning
                // through the following straight).
                yaw_setpoint_cap_ = yaw_command_ + stadium_yaw_sign_ * std::numbers::pi;
                yaw_cap_dir_ = stadium_yaw_sign_ >= 0.0 ? 1.0 : -1.0;
            }
        } else {
            v_goal = stadium_arc_velocity_;
            w_goal = stadium_yaw_sign_ * stadium_arc_yaw_rate_;
            const double turned = std::abs(measured_yaw_() - stadium_turn_yaw_start_);
            const double timeout =
                std::max(2.0, 2.0 * std::numbers::pi / std::max(stadium_arc_yaw_rate_, 1e-6));
            if (turned >= std::numbers::pi || time_ - stadium_phase_start_ >= timeout) {
                stadium_phase_ = kStadiumStraight;
                stadium_phase_start_ = time_;
                yaw_setpoint_cap_ = kNaN;
            }
        }
    }

    // Constant body twist (v, omega) after a start time: a steady circle of
    // radius v/omega. Used to commission combined translation + turning.
    void goal_circle_(double& v_goal, double& w_goal) const {
        const bool active = time_ >= circle_start_;
        v_goal = active ? circle_velocity_ : 0.0;
        w_goal = active ? circle_yaw_rate_ : 0.0;
    }

    void goal_twist_(double& v_goal, double& w_goal) const {
        v_goal = (move_velocity_.ready() && std::isfinite(*move_velocity_))
                   ? *move_velocity_
                   : 0.0;
        w_goal = (move_yaw_rate_.ready() && std::isfinite(*move_yaw_rate_))
                   ? *move_yaw_rate_
                   : 0.0;
    }

    // Legacy commissioning test: velocity square wave (or two-target switch)
    // plus a yaw-rate square wave, both started after their start time.
    void goal_square_(double& v_goal, double& w_goal) const {
        double v = command_velocity_;
        if (std::isfinite(command_velocity_period_) && command_velocity_period_ > 0.0
            && std::isfinite(command_velocity_2_)) {
            const auto half = static_cast<long long>(time_ / command_velocity_period_);
            v = (half % 2 == 0) ? command_velocity_ : command_velocity_2_;
        } else if (
            std::isfinite(command_velocity_switch_) && time_ >= command_velocity_switch_
            && std::isfinite(command_velocity_2_)) {
            v = command_velocity_2_;
        }
        v_goal = (time_ >= command_velocity_start_) ? v : 0.0;

        double w = command_yaw_rate_;
        if (std::isfinite(command_yaw_rate_period_) && command_yaw_rate_period_ > 0.0) {
            const auto half = static_cast<long long>(time_ / command_yaw_rate_period_);
            w = (half % 2 == 0) ? command_yaw_rate_ : -command_yaw_rate_;
        }
        w_goal = (time_ >= command_yaw_rate_start_) ? w : 0.0;
    }

    // Back-and-forth patrol (test): straight -> stop -> pivot -> straight ...
    // The profile layer smooths the goals; the pivot is a trapezoid that stops
    // at the measured heading change.
    void goal_patrol_(double& v_goal, double& w_goal) {
        v_goal = 0.0;
        w_goal = 0.0;
        if (time_ < patrol_start_)
            return;

        if (!patrol_started_) {
            patrol_started_ = true;
            patrol_phase_ = kPatrolStraight;
            patrol_phase_start_ = time_;
        }

        switch (patrol_phase_) {
        case kPatrolStraight:
            v_goal = patrol_velocity_;
            if (time_ - patrol_phase_start_ >= patrol_straight_time_) {
                patrol_phase_ = kPatrolStop;
                patrol_phase_start_ = time_;
            }
            break;
        case kPatrolStop:
            if (std::abs(velocity_) <= 0.05) {
                patrol_phase_ = kPatrolTurn;
                patrol_phase_start_ = time_;
                patrol_turn_yaw_start_ = measured_yaw_();
                patrol_turn_rate_ = 0.0;
            }
            break;
        case kPatrolTurn: {
            const double turned =
                std::abs(measured_yaw_() - patrol_turn_yaw_start_);
            const double remaining = std::max(0.0, patrol_turn_angle_ - turned);
            const double stop_rate =
                std::sqrt(std::max(0.0, 2.0 * patrol_yaw_accel_ * remaining));
            const double rate_target = std::min(std::abs(patrol_yaw_rate_), stop_rate);
            patrol_turn_rate_ += std::clamp(
                rate_target - patrol_turn_rate_, -patrol_yaw_accel_ * dt_,
                patrol_yaw_accel_ * dt_);
            patrol_turn_rate_ = std::max(patrol_turn_rate_, 0.0);
            w_goal = patrol_yaw_sign_ * patrol_turn_rate_;
            const double timeout = std::max(
                1.0, 3.0 * patrol_turn_angle_ / std::max(std::abs(patrol_yaw_rate_), 1e-6));
            if (remaining <= 1e-3 || time_ - patrol_phase_start_ >= timeout) {
                patrol_phase_ = kPatrolStraight;
                patrol_phase_start_ = time_;
            }
            break;
        }
        default: break;
        }
    }

    [[nodiscard]] double measured_yaw_() const {
        return (state_yaw_.ready() && std::isfinite(*state_yaw_)) ? *state_yaw_
                                                                  : yaw_command_;
    }

    void load_config_() {
        dt_ = node::param_or("dt", 0.001);
        behavior_ = node::param_or<std::string>("behavior", std::string{"twist"});

        max_velocity_ = node::param_or("max_velocity", 5.0);
        max_yaw_rate_ = node::param_or("max_yaw_rate", 12.0);

        channel_kp_ = node::param_or("channel_kp", 10.0);
        velocity_accel_ = node::param_or("velocity_accel", 1.5);
        velocity_jerk_ = node::param_or("velocity_jerk", 4.0);
        yaw_accel_limit_ = node::param_or("yaw_accel", 6.28);
        yaw_jerk_ = node::param_or("yaw_jerk", 40.0);

        leg_length_ = node::param_or("leg_length", kNaN);

        // "square" behaviour.
        command_velocity_ = node::param_or("command_velocity", 0.0);
        command_velocity_2_ = node::param_or("command_velocity_2", kNaN);
        command_velocity_switch_ = node::param_or("command_velocity_switch", kNaN);
        command_velocity_period_ = node::param_or("command_velocity_period", kNaN);
        command_velocity_start_ = node::param_or("command_velocity_start", 0.0);
        command_yaw_rate_ = node::param_or("command_yaw_rate", 0.0);
        command_yaw_rate_start_ = node::param_or("command_yaw_rate_start", 0.0);
        command_yaw_rate_period_ = node::param_or("command_yaw_rate_period", kNaN);

        // "patrol" behaviour.
        patrol_velocity_ = node::param_or("patrol_velocity", 3.0);
        patrol_straight_time_ = node::param_or("patrol_straight_time", 5.0);
        patrol_yaw_rate_ = node::param_or("patrol_yaw_rate", 3.14);
        patrol_yaw_accel_ = node::param_or("patrol_yaw_accel", 6.28);
        patrol_turn_degrees_ = node::param_or("patrol_turn_degrees", 180.0);
        patrol_turn_angle_ = patrol_turn_degrees_ * std::numbers::pi / 180.0;
        patrol_start_ = node::param_or("patrol_start", 3.0);
        patrol_yaw_sign_ = node::param_or("patrol_yaw_sign", 1.0);

        // "circle" behaviour.
        circle_velocity_ = node::param_or("circle_velocity", 1.0);
        circle_yaw_rate_ = node::param_or("circle_yaw_rate", 0.5);
        circle_start_ = node::param_or("circle_start", 3.0);

        // "stadium" behaviour.
        stadium_straight_velocity_ = node::param_or("stadium_straight_velocity", 3.0);
        stadium_straight_time_ = node::param_or("stadium_straight_time", 4.0);
        stadium_arc_velocity_ = node::param_or("stadium_arc_velocity", 2.0);
        stadium_arc_yaw_rate_ = node::param_or("stadium_arc_yaw_rate", 1.0);
        stadium_start_ = node::param_or("stadium_start", 3.0);
        stadium_yaw_sign_ = node::param_or("stadium_yaw_sign", 1.0);

        // Lateral plan: a_lat = v*omega -> bank = atan(a_lat/g) -> differential
        // leg length via roll_bias_ff = bank_to_roll_bias * bank (calibrated
        // static gain roll/roll_bias).
        gravity_ = node::param_or("gravity", 9.81);
        bank_to_roll_bias_ = node::param_or("bank_to_roll_bias", 2.11);
        // Feasible lateral acceleration (m/s^2), computed from the vehicle:
        //   friction  : a_lat <= mu * g
        //   tip-over  : the per-wheel normal force N = m g / 2 -/+ m a_lat h / w
        //               must stay positive, i.e. a_lat <= g w / (2 h)
        // A safety factor trims the limit. lat_accel_max > 0 is an optional hard
        // cap on top of the physical limit.
        friction_mu_ = node::param_or("friction_mu", 2.0);
        track_width_ = node::param_or("track_width", 0.396);
        com_height_ = node::param_or("com_height", 0.26);
        lat_accel_safety_ = node::param_or("lat_accel_safety", 0.7);
        lat_accel_max_ = node::param_or("lat_accel_max", 0.0);
        {
            const double tip_over = gravity_ * track_width_ / (2.0 * com_height_);
            const double friction = friction_mu_ * gravity_;
            const double physical = lat_accel_safety_ * std::min(tip_over, friction);
            lat_accel_limit_ = lat_accel_max_ > 0.0
                                 ? std::min(lat_accel_max_, physical)
                                 : physical;
        }
    }

    double dt_ = 0.001;
    std::string behavior_ = "twist";

    double max_velocity_ = 5.0;
    double max_yaw_rate_ = 12.0;
    double channel_kp_ = 10.0;
    double velocity_accel_ = 1.5;
    double velocity_jerk_ = 4.0;
    double yaw_accel_limit_ = 6.28;
    double yaw_jerk_ = 40.0;
    double leg_length_ = kNaN;

    double command_velocity_ = 0.0;
    double command_velocity_2_ = kNaN;
    double command_velocity_switch_ = kNaN;
    double command_velocity_period_ = kNaN;
    double command_velocity_start_ = 0.0;
    double command_yaw_rate_ = 0.0;
    double command_yaw_rate_start_ = 0.0;
    double command_yaw_rate_period_ = kNaN;

    double patrol_velocity_ = 3.0;
    double patrol_straight_time_ = 5.0;
    double patrol_yaw_rate_ = 3.14;
    double patrol_yaw_accel_ = 6.28;
    double patrol_turn_degrees_ = 180.0;
    double patrol_turn_angle_ = std::numbers::pi;
    double patrol_start_ = 3.0;
    double patrol_yaw_sign_ = 1.0;

    double circle_velocity_ = 1.0;
    double circle_yaw_rate_ = 0.5;
    double circle_start_ = 3.0;

    double stadium_straight_velocity_ = 3.0;
    double stadium_straight_time_ = 4.0;
    double stadium_arc_velocity_ = 2.0;
    double stadium_arc_yaw_rate_ = 1.0;
    double stadium_start_ = 3.0;
    double stadium_yaw_sign_ = 1.0;
    static constexpr int kStadiumStraight = 0;
    static constexpr int kStadiumArc = 1;
    bool stadium_started_ = false;
    int stadium_phase_ = kStadiumStraight;
    double stadium_phase_start_ = 0.0;
    double stadium_turn_yaw_start_ = 0.0;
    double yaw_setpoint_cap_ = kNaN;
    double yaw_cap_dir_ = 1.0;

    double gravity_ = 9.81;
    double bank_to_roll_bias_ = 2.11;
    double friction_mu_ = 2.0;
    double track_width_ = 0.396;
    double com_height_ = 0.26;
    double lat_accel_safety_ = 0.7;
    double lat_accel_max_ = 0.0;
    double lat_accel_limit_ = 3.0;

    static constexpr int kPatrolStraight = 0;
    static constexpr int kPatrolStop = 1;
    static constexpr int kPatrolTurn = 2;
    bool patrol_started_ = false;
    int patrol_phase_ = kPatrolStraight;
    double patrol_phase_start_ = 0.0;
    double patrol_turn_yaw_start_ = 0.0;
    double patrol_turn_rate_ = 0.0;

    double time_ = 0.0;
    double velocity_ = 0.0;
    double accel_ = 0.0;
    double yaw_rate_ = 0.0;
    double yaw_accel_ = 0.0;
    double yaw_command_ = 0.0;

    InputInterface<double> move_velocity_;
    InputInterface<double> move_yaw_rate_;
    InputInterface<double> state_yaw_;
    InputInterface<double> state_roll_;

    OutputInterface<double> command_velocity_output_;
    OutputInterface<double> command_yaw_rate_output_;
    OutputInterface<double> command_yaw_output_;
    OutputInterface<double> command_acceleration_output_;
    OutputInterface<double> command_leg_length_output_;
    OutputInterface<double> bank_output_;
    OutputInterface<double> roll_bias_ff_output_;
    OutputInterface<double> roll_error_output_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegMotionPlanner, rmcs_executor::Component)
