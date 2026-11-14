#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

#include "wheel_leg/joint_solver.hpp"
#include "wheel_leg/jump.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegRlController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegRlController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , jump_(get_parameter("jump_apex").as_double()) {
        register_input("/wheel_leg/state/position", position_);
        register_input("/wheel_leg/state/velocity", velocity_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/dm_control_ready", dm_control_ready_);
        register_input("/chassis/control_enable", control_enable_);
        register_input("/chassis/control_velocity", control_velocity_);
        register_input("/chassis/control_height", control_height_);
        register_input("/chassis/control_mode", control_mode_);
        register_input("/chassis/reset_count", reset_count_);
        register_input("/chassis/jump/request_count", jump_request_count_);
        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/wheel_leg/rl/valid", policy_valid_);
        register_input("/wheel_leg/rl/healthy", healthy_);
        register_input("/wheel_leg/rl/action_seq", action_sequence_);
        register_input("/wheel_leg/rl/policy_rate", policy_rate_);
        for (std::size_t i = 0; i < wheel_leg::kJointNames.size(); ++i) {
            const auto name = std::string{wheel_leg::kJointNames[i]};
            register_input("/wheel_leg/rl/raw/" + name, action_[i]);
            register_input("/wheel_leg/rl/raw/" + name + "/raw", raw_action_[i]);
            register_output(
                "/wheel_leg/" + name + "/control_torque", torque_[i],
                std::numeric_limits<double>::quiet_NaN());
        }

        register_output("/wheel_leg/rl/input/enable", policy_enable_, false);
        register_output("/wheel_leg/rl/input/command", policy_command_, Eigen::Vector3d::Zero());
        register_output(
            "/wheel_leg/rl/input/height", policy_height_, wheel_leg::jump_v4::kStandingHeight);
        register_output(
            "/wheel_leg/rl/input/context", policy_context_,
            std::vector<double>{1., 0., 0., 0., 0., 0., 0.});
        register_output("/wheel_leg/rl/applied_action", applied_action_, wheel_leg::JointVector{});
        register_output("/wheel_leg/rl/action_applied", action_applied_, false);
        register_output("/wheel_leg/rl/state", state_, static_cast<int>(State::kIdle));
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            reset();
            fault_ = false;
        }
        const bool jump_requested = *jump_request_count_ != last_jump_request_count_;
        last_jump_request_count_ = *jump_request_count_;
        if (!*control_enable_ || fault_) {
            if (*state_ != static_cast<int>(State::kIdle))
                reset();
            return;
        }
        if (!update_state())
            return;
        update_jump(jump_requested);
        update_command();
        if (!update_action())
            return;
        if (have_target_ && (!last_pd_tick_ || *update_count_ - *last_pd_tick_ >= kPdDivisor))
            update_torque();
    }

private:
    enum class State : int { kIdle = 1, kWaiting = 2, kRunning = 3 };
    static constexpr std::size_t kPdDivisor = 5;
    static constexpr std::size_t kActionTimeoutTicks = 60;
    static constexpr std::size_t kDriveLossTicks = 100;

    void reset() {
        jump_.reset();
        jump_velocity_ = 0.;
        for (auto& output : torque_)
            *output = std::numeric_limits<double>::quiet_NaN();
        *policy_enable_ = false;
        *applied_action_ = wheel_leg::JointVector{};
        *action_applied_ = false;
        policy_command_->setZero();
        *policy_height_ = wheel_leg::jump_v4::kStandingHeight;
        *policy_context_ = {1., 0., 0., 0., 0., 0., 0.};
        *state_ = static_cast<int>(State::kIdle);
        have_target_ = false;
        target_ = decoded_ = pending_history_ = wheel_leg::JointVector{};
        accepted_sequence_ = *action_sequence_;
        last_jump_request_count_ = *jump_request_count_;
        ticks_since_action_ = drive_loss_ticks_ = 0;
        last_pd_tick_.reset();
        last_command_tick_.reset();
    }

    void fail(const char* reason) {
        RCLCPP_ERROR(get_logger(), "Wheel-leg RL stopped: %s", reason);
        reset();
        fault_ = true;
    }

    bool update_state() {
        if (*state_ == static_cast<int>(State::kIdle)) {
            *state_ = static_cast<int>(State::kWaiting);
            write_torque(wheel_leg::JointVector{});
        }
        if (*state_ == static_cast<int>(State::kWaiting)) {
            if (!*dm_control_ready_)
                return false;
            *policy_enable_ = true;
            ticks_since_action_ = 0;
            accepted_sequence_ = *action_sequence_;
            *state_ = static_cast<int>(State::kRunning);
            last_pd_tick_.reset();
            last_command_tick_.reset();
        } else if (!*dm_control_ready_) {
            jump_.reset();
            jump_velocity_ = 0.;
            *policy_context_ = {1., 0., 0., 0., 0., 0., 0.};
            write_torque(wheel_leg::JointVector{});
            *action_applied_ = false;
            if (++drive_loss_ticks_ >= kDriveLossTicks)
                fail("DM drive readiness lost");
            return false;
        }
        drive_loss_ticks_ = 0;
        return true;
    }

    double now() const {
        return std::chrono::duration<double>(timestamp_->time_since_epoch()).count();
    }

    void update_jump(bool requested) {
        const double time = now();
        const bool busy = jump_.active();
        jump_.update(time);
        if (requested && !busy && have_target_ && *policy_valid_ > 0. && *healthy_ > 0.
            && std::abs(control_velocity_->vector.x()) <= wheel_leg::jump_v4::kMovingVxMax
            && std::abs(control_velocity_->vector.y()) < 1e-6
            && std::abs(control_velocity_->vector.z()) < 1e-6
            && !rmcs_msgs::is_spining(*control_mode_)
            && gravity_->z() < -std::cos(std::numbers::pi / 15.) && gyro_->norm() < .8
            && jump_.request(time)) {
            jump_velocity_ = control_velocity_->vector.x();
            last_command_tick_.reset();
        }
        if (busy && !jump_.active())
            last_command_tick_.reset();
        const auto context = jump_.context(time);
        policy_context_->assign(context.begin(), context.end());
    }

    void update_command() {
        const auto divisor = static_cast<std::size_t>(1000. / *policy_rate_);
        if (last_command_tick_ && *update_count_ - *last_command_tick_ < divisor)
            return;
        last_command_tick_ = *update_count_;
        if (jump_.active()) {
            *policy_command_ = Eigen::Vector3d{jump_velocity_, 0., 0.};
            *policy_height_ = jump_.height(now());
            return;
        }
        const double vx = rmcs_msgs::is_spining(*control_mode_)
                            ? 0.
                            : std::clamp(
                                  control_velocity_->vector.x(), -wheel_leg::jump_v4::kForwardLimit,
                                  wheel_leg::jump_v4::kForwardLimit);
        *policy_command_ = Eigen::Vector3d{
            vx, 0.,
            std::clamp(
                control_velocity_->vector.z(), -wheel_leg::jump_v4::kYawLimit,
                wheel_leg::jump_v4::kYawLimit)};
        const double step = wheel_leg::jump_v4::kHeightRate / *policy_rate_;
        *policy_height_ += std::clamp(
            std::clamp(
                *control_height_, wheel_leg::jump_v4::kHeightMin, wheel_leg::jump_v4::kHeightMax)
                - *policy_height_,
            -step, step);
    }

    bool update_action() {
        if (*healthy_ <= 0.) {
            fail("inference worker failed");
            return false;
        }
        if (*policy_valid_ > 0. && *action_sequence_ != accepted_sequence_) {
            for (std::size_t i = 0; i < decoded_.size(); ++i) {
                decoded_[i] = *action_[i];
                pending_history_[i] = *raw_action_[i];
            }
            target_ = wheel_leg::decoded_action_target(
                decoded_, *position_, wheel_leg::jump_target_bounds(jump_.active()));
            accepted_sequence_ = *action_sequence_;
            have_target_ = true;
            ticks_since_action_ = 0;
        } else if (++ticks_since_action_ > kActionTimeoutTicks) {
            fail("policy result timeout");
            return false;
        }
        return true;
    }

    void update_torque() {
        target_ = wheel_leg::decoded_action_target(
            decoded_, *position_, wheel_leg::jump_target_bounds(jump_.active()));
        auto torque = wheel_leg::pd_torque(target_, *position_, *velocity_);
        const auto& q = *position_;
        const auto extra = wheel_leg::tuck_torque(
            {q[0], q[1], q[2], q[3]}, {target_[0], target_[1], target_[2], target_[3]},
            {torque[0], torque[1], torque[2], torque[3]},
            jump_.phase(now()) == wheel_leg::JumpCommand::Phase::kFlight);
        for (std::size_t i = 0; i < extra.size(); ++i)
            torque[i] += extra[i];
        write_torque(wheel_leg::limit_torque(torque, *position_));
        *applied_action_ = pending_history_;
        *action_applied_ = true;
        last_pd_tick_ = *update_count_;
    }

    void write_torque(const wheel_leg::JointVector& torque) {
        for (std::size_t i = 0; i < torque.size(); ++i)
            *torque_[i] = torque[i];
    }

    InputInterface<wheel_leg::JointVector> position_, velocity_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_;
    InputInterface<bool> dm_control_ready_;
    InputInterface<bool> control_enable_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<double> control_height_;
    InputInterface<rmcs_msgs::ChassisMode> control_mode_;
    InputInterface<std::size_t> reset_count_, jump_request_count_, update_count_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<double> policy_valid_, healthy_, policy_rate_;
    InputInterface<std::size_t> action_sequence_;
    std::array<InputInterface<double>, 6> action_, raw_action_;

    std::array<OutputInterface<double>, 6> torque_;
    OutputInterface<bool> policy_enable_, action_applied_;
    OutputInterface<Eigen::Vector3d> policy_command_;
    OutputInterface<double> policy_height_;
    OutputInterface<std::vector<double>> policy_context_;
    OutputInterface<wheel_leg::JointVector> applied_action_;
    OutputInterface<int> state_;

    bool fault_ = false, have_target_ = false;
    wheel_leg::JointVector target_{}, decoded_{}, pending_history_{};
    std::size_t last_reset_count_ = 0, last_jump_request_count_ = 0, accepted_sequence_ = 0;
    std::size_t ticks_since_action_ = 0, drive_loss_ticks_ = 0;
    std::optional<std::size_t> last_pd_tick_, last_command_tick_;
    wheel_leg::JumpCommand jump_;
    double jump_velocity_ = 0.;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlController, rmcs_executor::Component)
