#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Core>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class AdjustableChassisController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Switch = rmcs_msgs::Switch;
    using Clock = std::chrono::steady_clock;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double joystick_deadzone = 0.05;
    static constexpr double angular_velocity_limit = 10.0;
    // A finite, nonbinding budget keeps the existing steering solver's interface usable.
    static constexpr double unrestricted_power = std::numeric_limits<float>::max();
    // Angle offset used when seeking a lift limit.
    static constexpr double lift_homing_lead = 0.75;
    static constexpr std::size_t master_lift_count = 4;
    static constexpr std::array lift_prefixes{
        "/chassis/lift/master_left_front", "/chassis/lift/master_left_back",
        "/chassis/lift/master_right_back", "/chassis/lift/master_right_front",
        "/chassis/lift/slave_left", "/chassis/lift/slave_right"};

    enum class LiftState { idle, homing, calibrated, fault };
    enum class LiftMode { inactive, extend, retract_front, retract_front_and_slave, retract_all };

    struct LiftJoint {
        InputInterface<double> angle, velocity, torque;
        InputInterface<bool> feedback_valid;
        OutputInterface<double> target;
        double direction = 0.0;
        double zero = nan;
        std::optional<Clock::time_point> blocked_since;
        std::optional<double> stalled_target;
    };

public:
    AdjustableChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);

        register_output("/chassis/control_velocity", control_velocity_, nan, nan, nan);
        register_output("/chassis/control_power_limit", control_power_limit_, 0.0);
        register_output("/chassis/lift/master_retracting", master_lift_retracting_);
        configure_lifts();
    }

    void update() override {
        master_lift_retracting_->fill(false);
        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const auto& stick = *joystick_left_;
        const double magnitude = std::hypot(stick.x(), stick.y());
        const double yaw_input = joystick_right_->y();

        const bool joystick_valid = std::isfinite(magnitude) && std::isfinite(yaw_input);
        const bool translating = magnitude > joystick_deadzone;
        const double angular_velocity = std::abs(yaw_input) > joystick_deadzone
            ? angular_velocity_limit * 0.3 * yaw_input
            : 0.0;
        const bool rotating = angular_velocity != 0.0;

        const bool can_arm = left == Switch::DOWN && right == Switch::DOWN && joystick_valid
            && !translating && !rotating;
        if (!armed_) {
            reset_motor();
            armed_ = can_arm;
            return;
        }

        if ((left == Switch::UNKNOWN || right == Switch::UNKNOWN)
            || (left == Switch::DOWN && right == Switch::DOWN) || !joystick_valid) {
            reset_motor();
            armed_ = can_arm;
            return;
        }

        const auto lift_mode = mode_selection(left, right);
        const bool lifts_allow_drive = update_lifts(lift_mode);
        if ((left != Switch::MIDDLE && (!lift_enabled_ || lift_mode == LiftMode::inactive))
            || !lifts_allow_drive || (!translating && !rotating)) {
            stop_chassis();
            return;
        }

        const Eigen::Vector2d velocity = stick * (translating ? 3.0 : 0.0);
        control_velocity_->vector << velocity.x(), velocity.y(), angular_velocity;
        *control_power_limit_ = unrestricted_power;
    }

private:
    static LiftMode mode_selection(Switch left, Switch right) {
        if (left == Switch::MIDDLE && right == Switch::MIDDLE)
            return LiftMode::extend;
        if (left == Switch::MIDDLE && right == Switch::UP)
            return LiftMode::retract_front;
        if (left == Switch::UP && right == Switch::UP)
            return LiftMode::retract_front_and_slave;
        if (left == Switch::UP && right == Switch::MIDDLE)
            return LiftMode::retract_all;
        return LiftMode::inactive;
    }

    void configure_lifts() {
        lift_enabled_ = get_parameter_or("lift.enabled", false);
        if (!lift_enabled_)
            return;

        const auto directions = get_parameter("lift.down_directions").as_double_array();
        lift_stall_torque_ = get_parameter("lift.stall_torque").as_double();
        lift_stall_speed_ = get_parameter("lift.stall_speed").as_double();
        lift_stall_time_ = get_parameter("lift.stall_time").as_double();
        lift_position_tolerance_ = get_parameter("lift.position_tolerance").as_double();
        front_retract_travel_ = get_parameter_or("lift.front_retract_travel", 0.0);
        slave_retract_travel_ = get_parameter_or("lift.slave_retract_travel", 0.0);

        const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
        const auto valid_direction = [](double v) {
            return v == 1.0 || v == -1.0 || v == 0.0;
        };
        if ((directions.size() != master_lift_count && directions.size() != lifts_.size())
            || !std::all_of(directions.begin(), directions.end(), valid_direction)
            || std::all_of(directions.begin(), directions.end(), [](double v) { return v == 0.0; })
            || !positive(lift_position_tolerance_) || !positive(lift_stall_torque_)
            || !positive(lift_stall_speed_) || !positive(lift_stall_time_)
            || !std::isfinite(front_retract_travel_) || front_retract_travel_ < 0.0
            || !std::isfinite(slave_retract_travel_) || slave_retract_travel_ < 0.0) {
            throw std::invalid_argument("Invalid lift limits or directions");
        }

        for (std::size_t i = 0; i < directions.size(); ++i)
            lifts_[i].direction = directions[i];

        for (std::size_t i = 0; i < lifts_.size(); ++i) {
            const std::string prefix = lift_prefixes[i];
            auto& lift = lifts_[i];
            register_output(prefix + "/target_angle", lift.target, nan);
            if (lift.direction == 0.0)
                continue;
            register_input(prefix + "/angle", lift.angle);
            register_input(prefix + "/velocity", lift.velocity);
            register_input(prefix + "/torque", lift.torque);
            register_input(prefix + "/feedback_valid", lift.feedback_valid);
        }
    }

    bool update_lifts(LiftMode mode) {
        if (!lift_enabled_)
            return true;
        if (lift_state_ == LiftState::fault)
            return false;
        const bool down = mode == LiftMode::extend;
        if (lift_state_ == LiftState::idle && !down)
            return true;

        for (std::size_t i = 0; i < lifts_.size(); ++i) {
            const auto& lift = lifts_[i];
            if (lift.direction == 0.0)
                continue;
            if (!*lift.feedback_valid || !std::isfinite(*lift.angle)
                || !std::isfinite(*lift.velocity) || !std::isfinite(*lift.torque)) {
                return lift_fault("Missing or invalid feedback", lift_prefixes[i]);
            }
        }

        if (lift_state_ == LiftState::homing && !down) {
            release_lifts();
            return mode != LiftMode::inactive;
        }
        if (mode == LiftMode::inactive) {
            cancel_lift_retraction();
            return false;
        }
        if (lift_state_ == LiftState::calibrated)
            update_calibrated_lift(mode, Clock::now());
        else
            update_lift_homing(Clock::now());

        return true;
    }

    void update_lift_homing(Clock::time_point now) {
        lift_state_ = LiftState::homing;
        for (auto& lift : lifts_) {
            if (lift.direction == 0.0)
                continue;
            if (!std::isfinite(lift.zero) && blocked_long_enough(lift, now))
                lift.zero = *lift.angle;
            *lift.target = std::isfinite(lift.zero) ? lift.zero
                                                  : *lift.angle + lift.direction * lift_homing_lead;
        }

        if (std::all_of(lifts_.begin(), lifts_.end(), [](const LiftJoint& lift) {
                return lift.direction == 0.0 || std::isfinite(lift.zero);
            })) {
            lift_state_ = LiftState::calibrated;
            RCLCPP_INFO(
                get_logger(), "Lift zeros (master LF, LB, RB, RF, slave L, R; nan means disabled): "
                              "%.4f, %.4f, %.4f, %.4f, %.4f, %.4f rad",
                lifts_[0].zero, lifts_[1].zero, lifts_[2].zero, lifts_[3].zero, lifts_[4].zero,
                lifts_[5].zero);
        }
    }

    void update_calibrated_lift(LiftMode mode, Clock::time_point now) {
        const double front_travel = mode == LiftMode::extend ? 0.0 : front_retract_travel_;
        const double back_travel = mode == LiftMode::retract_all ? front_retract_travel_ : 0.0;
        const double slave_travel =
            (mode == LiftMode::retract_front_and_slave || mode == LiftMode::retract_all)
                ? slave_retract_travel_
                : 0.0;
        const std::array travel{
            front_travel, back_travel, back_travel, front_travel, slave_travel, slave_travel};

        std::array<double, lift_prefixes.size()> targets{};
        bool hold_lifts = false;
        for (std::size_t i = 0; i < lifts_.size(); ++i) {
            auto& lift = lifts_[i];
            if (lift.direction == 0.0)
                continue;

            targets[i] = lift.zero - lift.direction * travel[i];
            if (lift.stalled_target == targets[i]) {
                hold_lifts = true;
            } else if (lift.stalled_target) {
                lift.stalled_target.reset();
                *lift.target = targets[i];
                lift.blocked_since.reset();
            }
        }

        bool stalled = false;
        for (std::size_t i = 0; i < lifts_.size(); ++i) {
            auto& lift = lifts_[i];
            if (lift.direction == 0.0 || lift.stalled_target)
                continue;

            if (!hold_lifts && *lift.target != targets[i]) {
                *lift.target = targets[i];
                lift.blocked_since.reset();
            } else if (!std::isfinite(*lift.target)) {
                *lift.target = *lift.angle;
            }

            if (std::abs(*lift.target - *lift.angle) <= lift_position_tolerance_) {
                lift.blocked_since.reset();
            } else if (blocked_long_enough(lift, now)) {
                lift.stalled_target = targets[i];
                *lift.target = nan;
                lift.blocked_since.reset();
                stalled = true;
                RCLCPP_WARN(get_logger(), "Lift stalled during travel: %s", lift_prefixes[i]);
            }
        }

        if (stalled) {
            for (auto& lift : lifts_) {
                if (lift.direction != 0.0 && !lift.stalled_target
                    && (std::abs(*lift.target - *lift.angle) > lift_position_tolerance_
                        || std::abs(*lift.velocity) >= lift_stall_speed_)) {
                    *lift.target = *lift.angle;
                    lift.blocked_since.reset();
                }
            }
        }

        for (std::size_t i = 0; i < master_lift_count; ++i) {
            const auto& lift = lifts_[i];
            (*master_lift_retracting_)[i] = lift.direction != 0.0 && travel[i] > 0.0
                && !lift.stalled_target && *lift.target == targets[i];
        }
    }

    bool blocked_long_enough(LiftJoint& lift, Clock::time_point now) const {
        if (std::abs(*lift.velocity) >= lift_stall_speed_
            || std::abs(*lift.torque) < lift_stall_torque_) {
            lift.blocked_since.reset();
            return false;
        }
        if (!lift.blocked_since)
            lift.blocked_since = now;
        return std::chrono::duration<double>(now - *lift.blocked_since).count() >= lift_stall_time_;
    }

    void cancel_lift_retraction() {
        for (auto& lift : lifts_) {
            lift.blocked_since.reset();
            if (*lift.target != lift.zero)
                *lift.target = nan;
        }
    }

    void release_lifts() {
        for (auto& lift : lifts_) {
            *lift.target = nan;
            lift.blocked_since.reset();
        }
    }

    bool lift_fault(const char* reason, const char* joint) {
        lift_state_ = LiftState::fault;
        release_lifts();
        RCLCPP_ERROR(get_logger(), "Lift fault: %s %s", reason, joint);
        return false;
    }

    void reset_motor() {
        if (lift_enabled_)
            release_lifts();
        stop_chassis();
    }

    void stop_chassis() {
        control_velocity_->vector.setConstant(nan);
        *control_power_limit_ = 0.0;
    }

    bool armed_ = false;

    InputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    InputInterface<Switch> switch_left_, switch_right_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    OutputInterface<double> control_power_limit_;
    OutputInterface<std::array<bool, master_lift_count>> master_lift_retracting_;

    std::array<LiftJoint, lift_prefixes.size()> lifts_;
    double front_retract_travel_ = 0.0, slave_retract_travel_ = 0.0;
    double lift_position_tolerance_ = 0.0;
    double lift_stall_torque_ = 0.0, lift_stall_speed_ = 0.0, lift_stall_time_ = 0.0;
    LiftState lift_state_ = LiftState::idle;
    bool lift_enabled_ = false;
};

} // namespace rmcs_core::controller::chassis

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::AdjustableChassisController, rmcs_executor::Component)
