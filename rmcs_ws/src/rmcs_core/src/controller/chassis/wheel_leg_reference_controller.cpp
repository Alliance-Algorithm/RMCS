#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "wheel_leg_arm_sequence.hpp"

namespace rmcs_core::controller::chassis {

// Independent commissioning graph: motor API zero, with the same host PD as V6.
class WheelLegReferenceController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegReferenceController()
        : Node{get_component_name(), node::options()} {
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            const std::string prefix = std::string{"/wheel_leg/"} + kJointNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/max_torque", max_torque_[i]);
            register_input(prefix + "/status_code", status_[i]);
            register_input(prefix + "/fault_code", fault_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.0);
            register_output(prefix + "/control_angle", angle_target_[i], kNaN);
            register_output(prefix + "/reference/error", error_[i], kNaN);
        }
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
        register_input("/wheel_leg/dm_control_ready", drives_ready_);
        register_input("/predefined/update_rate", update_rate_);
        register_output("/wheel_leg/left_wheel/control_torque", wheel_torque_[0], 0.0);
        register_output("/wheel_leg/right_wheel/control_torque", wheel_torque_[1], 0.0);
        register_output("/wheel_leg/enable_request", enable_request_, false);
        register_output("/wheel_leg/reference/state", state_, 0);
        register_output("/wheel_leg/reference/failure", failure_, 0);
        register_output("/wheel_leg/reference/ready", ready_, false);

        const auto values = get_parameter("reference_motor_angles").as_double_array();
        if (values.size() != kJointNames.size())
            throw std::invalid_argument("reference_motor_angles requires four motor API angles");
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (!std::isfinite(values[i]) || std::abs(values[i]) > kPositionMax)
                throw std::invalid_argument("Reference target exceeds DM motor API range");
            reference_[i] = values[i];
        }
        kp_ = get_parameter_or("reference_kp", 160.0);
        kd_ = get_parameter_or("reference_kd", 2.5);
        ramp_velocity_ = get_parameter_or("reference_ramp_velocity", 0.25);
        torque_limit_ = get_parameter_or("reference_torque_limit", 40.0);
        frequency_ = get_parameter_or("reference_pd_frequency", 200.0);
        position_tolerance_ = get_parameter_or("reference_position_tolerance", 0.03);
        velocity_tolerance_ = get_parameter_or("reference_velocity_tolerance", 0.1);
        stable_seconds_ = get_parameter_or("reference_stable_seconds", 0.2);
        following_error_max_ = get_parameter_or("reference_following_error_max", 0.2);
        for (const double value :
             {kp_, ramp_velocity_, torque_limit_, frequency_, position_tolerance_,
              velocity_tolerance_, stable_seconds_, following_error_max_}) {
            if (!std::isfinite(value) || value <= 0.0)
                throw std::invalid_argument("Reference control parameters must be positive");
        }
        if (!std::isfinite(kd_) || kd_ < 0.0 || following_error_max_ <= position_tolerance_)
            throw std::invalid_argument("Invalid reference damping or following-error bound");
    }

    void update() override {
        // Executor publishes the update rate after before_updating().
        if (divisor_ == 0) {
            const double ratio = *update_rate_ / frequency_;
            if (!std::isfinite(ratio) || ratio < 1.0 || std::abs(ratio - std::round(ratio)) > 1e-6)
                throw std::invalid_argument("Reference PD frequency must divide executor rate");
            divisor_ = static_cast<std::size_t>(std::llround(ratio));
            period_ = 1.0 / frequency_;
        }
        ++tick_;
        stop_outputs_();
        using rmcs_msgs::Switch;
        const auto left = *dr16_fresh_ ? *switch_left_ : Switch::UNKNOWN;
        const auto right = *dr16_fresh_ ? *switch_right_ : Switch::UNKNOWN;
        if (left == Switch::DOWN && right == Switch::DOWN) {
            reset_session_();
            *failure_ = 0;
            arm_.update(left, right);
            return;
        }
        if (!arm_.update(left, right)) {
            reset_session_();
            if (*failure_ != 0)
                *state_ = kFault;
            return;
        }
        if (*failure_ != 0) {
            arm_ = {};
            *state_ = kFault;
            return;
        }

        const auto now = Clock::now();
        if (const auto failure = read_feedback_(); failure != kNone) {
            fail_(failure);
            return;
        }
        if (!active_) {
            active_ = true;
            enable_started_ = now;
        }
        if (pd_started_ && now - last_pd_time_ > std::chrono::milliseconds{20}) {
            fail_(kControlInterval);
            return;
        }
        *enable_request_ = true;
        *state_ = kArming;
        if (!*drives_ready_) {
            if (pd_started_ || now - enable_started_ > std::chrono::seconds{1})
                fail_(kDriveNotReady);
            else {
                target_ = q_;
                publish_target_();
            }
            return;
        }
        for (const auto& status : status_) {
            if (*status != 1) {
                fail_(kDriveNotReady);
                return;
            }
        }
        if (!pd_started_ || tick_ - last_pd_tick_ >= divisor_) {
            if (!pd_started_) {
                // The mechanism can move while FC/neutral MIT handshaking is in progress.
                captured_ = q_;
                ramp_duration_ = (reference_ - captured_).cwiseAbs().maxCoeff() / ramp_velocity_;
            }
            ramp_elapsed_ += period_;
            const double fraction =
                ramp_duration_ > 0.0 ? std::min(ramp_elapsed_ / ramp_duration_, 1.0) : 1.0;
            // One fraction for the coupled shafts preserves the relative-angle path.
            target_ = captured_ + fraction * (reference_ - captured_);
            const Eigen::Vector4d following_error = target_ - q_;
            if (following_error.cwiseAbs().maxCoeff() > following_error_max_) {
                fail_(kFollowingError);
                return;
            }
            const Eigen::Vector4d requested = kp_ * following_error - kd_ * dq_;
            for (std::size_t i = 0; i < kJointNames.size(); ++i) {
                const double bound = std::min(torque_limit_, *max_torque_[i]);
                held_torque_[i] = std::clamp(requested[i], -bound, bound);
            }
            const bool settled = fraction == 1.0
                              && (reference_ - q_).cwiseAbs().maxCoeff() < position_tolerance_
                              && dq_.cwiseAbs().maxCoeff() < velocity_tolerance_;
            if (settled) {
                if (!settling_)
                    stable_started_ = now;
                stable_elapsed_ = std::chrono::duration<double>(now - stable_started_).count();
            } else {
                stable_elapsed_ = 0.0;
            }
            settling_ = settled;
            pd_started_ = true;
            last_pd_tick_ = tick_;
            last_pd_time_ = now;
        }
        *ready_ = stable_elapsed_ + 1e-12 >= stable_seconds_;
        *state_ = *ready_ ? kHolding : kMoving;
        publish_target_();
        for (std::size_t i = 0; i < kJointNames.size(); ++i)
            *torque_[i] = held_torque_[i];
    }

private:
    using Clock = std::chrono::steady_clock;
    enum State { kIdle, kArming, kMoving, kHolding, kFault };
    enum Failure {
        kNone,
        kInvalidFeedback,
        kMotorFault,
        kControlInterval,
        kFollowingError,
        kDriveNotReady
    };
    static constexpr std::array kJointNames{
        "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kPositionMax = 12.4;

    void stop_outputs_() {
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            *torque_[i] = 0.0;
            *angle_target_[i] = kNaN;
            *error_[i] = kNaN;
        }
        for (auto& wheel : wheel_torque_)
            *wheel = 0.0;
        *enable_request_ = false;
        *ready_ = false;
        *state_ = kIdle;
    }

    void reset_session_() {
        active_ = pd_started_ = settling_ = false;
        ramp_elapsed_ = stable_elapsed_ = 0.0;
        held_torque_.setZero();
    }

    void fail_(Failure failure) {
        *failure_ = failure;
        arm_ = {};
        reset_session_();
        stop_outputs_();
        *state_ = kFault;
        node::error("Reference hold disabled, failure {}", static_cast<int>(failure));
    }

    Failure read_feedback_() {
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            q_[i] = *angle_[i];
            dq_[i] = *velocity_[i];
            if (!std::isfinite(q_[i]) || !std::isfinite(dq_[i]) || !std::isfinite(*max_torque_[i])
                || *max_torque_[i] <= 0.0 || std::abs(q_[i]) > kPositionMax
                || std::abs(dq_[i]) > 45.0)
                return kInvalidFeedback;
            if (*fault_[i] != 0 || (*status_[i] != 0 && *status_[i] != 1))
                return kMotorFault;
        }
        return kNone;
    }

    void publish_target_() {
        for (std::size_t i = 0; i < kJointNames.size(); ++i) {
            *angle_target_[i] = target_[i];
            *error_[i] = reference_[i] - q_[i];
        }
    }

    std::array<InputInterface<double>, 4> angle_, velocity_, max_torque_;
    std::array<InputInterface<int>, 4> status_, fault_;
    std::array<OutputInterface<double>, 4> torque_, angle_target_, error_;
    std::array<OutputInterface<double>, 2> wheel_torque_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_, drives_ready_;
    InputInterface<double> update_rate_;
    OutputInterface<bool> enable_request_, ready_;
    OutputInterface<int> state_, failure_;
    WheelLegArmSequence arm_;
    Eigen::Vector4d reference_, captured_, target_, q_, dq_, held_torque_;
    Clock::time_point enable_started_{}, last_pd_time_{}, stable_started_{};
    std::size_t tick_ = 0, last_pd_tick_ = 0, divisor_ = 0;
    double kp_, kd_, ramp_velocity_, torque_limit_, frequency_, position_tolerance_;
    double velocity_tolerance_, stable_seconds_, following_error_max_;
    double period_ = 0.005, ramp_duration_ = 0.0, ramp_elapsed_ = 0.0, stable_elapsed_ = 0.0;
    bool active_ = false, pd_started_ = false, settling_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegReferenceController, rmcs_executor::Component)
