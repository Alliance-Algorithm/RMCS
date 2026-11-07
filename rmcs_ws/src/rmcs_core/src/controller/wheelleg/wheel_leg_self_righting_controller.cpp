#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "controller/chassis/wheel_leg/capture.hpp"
#include "controller/chassis/wheel_leg/model/recovery_profile.hpp"
#include "identification/wheel_leg_arm_dwell.hpp"

namespace rmcs_core::controller::wheelleg {

namespace native = rmcs_core::controller::chassis::wheel_leg;

class WheelLegSelfRightingController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegSelfRightingController()
        : Node{get_component_name(), node::options()}
        , config_(native::make_recovery_config())
        , recovery_(config_)
        , observer_(native::recovery_leg_geometry()) {
        enabled_ = get_parameter_or("enabled", false);
        admitted_ = !get_parameter_or("simulation_only", true)
                 && (get_parameter_or("hardware_mapping_verified", false)
                     || (get_parameter_or("hardware_commissioning_enabled", false)
                         && !get_parameter_or("hardware_calibration_id", std::string{}).empty()));
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/dm_control_ready", drives_ready_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/wheel_leg/imu/quaternion", quaternion_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/imu/acceleration", acceleration_);
        for (std::size_t i = 0; i < axes_.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + axes_[i];
            register_input(prefix + "/angle", position_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_output(
                std::string{"/wheelleg/recovery/"} + axes_[i] + "/torque", torque_[i], 0.);
        }
        register_output("/wheelleg/recovery/active", active_, false);
        register_output("/wheelleg/recovery/ready", ready_, false);
        register_output("/wheelleg/recovery/fault", fault_, std::uint8_t{0});
        register_output("/wheelleg/recovery/phase", phase_, std::uint8_t{0});
        register_output("/chassis/reset_count", reset_count_, std::size_t{0});
    }

    void before_updating() override { reset(); }

    void update() override {
        using Switch = rmcs_msgs::Switch;
        const bool down = *left_ == Switch::DOWN && *right_ == Switch::DOWN;
        const auto now = Arm::Clock::now();
        const bool arm_event = arm_.update(now, *remote_fresh_, *left_, *right_, .5);
        if (down && !previous_down_)
            ++*reset_count_;
        previous_down_ = down;
        if (!enabled_ || !admitted_ || !*remote_fresh_ || down || *left_ == Switch::UNKNOWN
            || *right_ == Switch::UNKNOWN) {
            reset();
            return;
        }
        if (!(*left_ == Switch::MIDDLE && (*right_ == Switch::MIDDLE || *right_ == Switch::UP))) {
            reset();
            return;
        }
        if (arm_event)
            armed_ = true;
        if (!armed_)
            return;
        if (released_) {
            *active_ = false;
            *ready_ = true;
            return;
        }
        *active_ = true;
        if (*fault_ != 0 || !*drives_ready_) {
            clear_torques();
            return;
        }
        if (!*feedback_fresh_ || !quaternion_->coeffs().allFinite() || quaternion_->norm() < .9
            || quaternion_->norm() > 1.1 || !gyro_->allFinite() || !acceleration_->allFinite()) {
            *fault_ = 9;
            clear_torques();
            return;
        }
        if (++ticks_ % 5 != 0)
            return;
        clear_torques();
        native::JointReferenceRecoveryVector6 api, velocity;
        for (std::size_t i = 0; i < axes_.size(); ++i) {
            api[i] = *position_[i];
            velocity[i] = *velocity_[i];
        }
        auto q = api;
        for (int i = 0; i < 4; ++i) {
            q[i] = -api[i] + rmcs_core::controller::chassis::kMotorOffsets[i];
            velocity[i] = -velocity[i];
        }
        const Eigen::Vector3d gravity =
            quaternion_->normalized().conjugate() * -Eigen::Vector3d::UnitZ();
        auto feedback = observer_.observe(
            q, velocity, gravity, *gyro_, *acceleration_,
            started_ ? recovery_.command().phase : native::JointReferenceRecoveryPhase::kSelect);
        if (!feedback.q.allFinite()) {
            *fault_ = 9;
            return;
        }
        if (!started_) {
            recovery_.reset(feedback.q);
            started_ = true;
        }
        feedback.rl_capture_ready =
            native::recovery_capture_blockers(feedback, *acceleration_, config_.nominal) == 0;
        const auto& command = recovery_.update(feedback);
        *phase_ = static_cast<std::uint8_t>(command.phase);
        *fault_ = static_cast<std::uint8_t>(command.failure_code);
        if (command.failed)
            return;
        if (command.pure_rl) {
            released_ = true;
            *active_ = false;
            *ready_ = true;
            return;
        }
        feedback.wheel_probe_torque =
            observer_.probe_command(command.scripted && command.release_finished);
        const auto effort =
            recovery_.torques(feedback, native::JointReferenceRecoveryVector6::Zero());
        for (std::size_t i = 0; i < axes_.size(); ++i)
            *torque_[i] = i < 4 ? -effort[i] : effort[i];
    }

private:
    using Arm = rmcs_core::controller::identification::WheelLegArmDwell;
    void clear_torques() {
        for (auto& output : torque_)
            *output = 0.;
    }
    void reset() {
        clear_torques();
        *active_ = *ready_ = false;
        *fault_ = *phase_ = 0;
        armed_ = released_ = started_ = false;
        ticks_ = 0;
        observer_.reset();
        recovery_.reset(native::JointReferenceRecoveryVector6::Zero(), false);
    }
    static constexpr std::array axes_{"left_hip_joint",   "left_knee_joint", "right_hip_joint",
                                      "right_knee_joint", "left_wheel",      "right_wheel"};
    std::array<InputInterface<double>, 6> position_, velocity_;
    InputInterface<bool> feedback_fresh_, drives_ready_, remote_fresh_;
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<Eigen::Quaterniond> quaternion_;
    InputInterface<Eigen::Vector3d> gyro_, acceleration_;
    std::array<OutputInterface<double>, 6> torque_;
    OutputInterface<bool> active_, ready_;
    OutputInterface<std::uint8_t> fault_, phase_;
    OutputInterface<std::size_t> reset_count_;
    native::JointReferenceRecoveryConfig config_;
    native::JointReferenceRecoveryController recovery_;
    native::ClosedChainSupportObserver observer_;
    rmcs_core::controller::identification::WheelLegArmDwell arm_;
    std::uint64_t ticks_ = 0;
    bool enabled_ = false, admitted_ = false, armed_ = false, released_ = false, started_ = false,
         previous_down_ = false;
};
} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegSelfRightingController, rmcs_executor::Component)
