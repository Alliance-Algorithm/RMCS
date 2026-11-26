#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/wheel_leg_rl_state.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "wheel_leg/joint_coordinates.hpp"

namespace rmcs_core::controller::chassis {
namespace {
constexpr wheel_leg::LegVector kPolicyReference{-.42, .13722330866417465, .42, -.13721605817245947};

class WheelLegActionAdapter : public rmcs_executor::Component {
public:
    WheelLegActionAdapter() {
        for (std::size_t i = 0; i < wheel_leg::kJointNames.size(); ++i)
            register_input(
                std::string{"/wheel_leg/rl/raw/"} + wheel_leg::kJointNames[i], action_[i]);
        register_output("/wheel_leg/rl/action_target", target_, wheel_leg::JointVector{});
    }

    void update() override {
        for (std::size_t i = 0; i < action_.size(); ++i)
            (*target_)[i] =
                i < kPolicyReference.size() ? kPolicyReference[i] + *action_[i] : *action_[i];
    }

private:
    std::array<InputInterface<double>, 6> action_;
    OutputInterface<wheel_leg::JointVector> target_;
};
} // namespace

class WheelLegObservation
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegObservation()
        : Node{get_component_name(), node::options()} {
        register_input("/wheel_leg/state/position", position_);
        register_input("/wheel_leg/state/velocity", velocity_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/rl/input/enable", policy_enable_);
        register_input("/wheel_leg/rl/input/command", policy_command_);
        register_input("/wheel_leg/rl/input/height", policy_height_);
        register_input("/wheel_leg/jump/active", jump_active_);
        register_input("/wheel_leg/jump/apex", jump_apex_);
        register_input("/wheel_leg/jump/elapsed", jump_elapsed_);
        register_input("/wheel_leg/rl/applied_action", applied_action_);
        register_input("/wheel_leg/rl/action_applied", action_applied_);
        register_input("/wheel_leg/rl/state", rl_state_);
        register_input("/wheel_leg/rl/policy_rate", policy_rate_);
        register_input("/chassis/reset_count", reset_count_);
        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/update_rate", update_rate_);

        register_output("/wheel_leg/rl/features/command", feature_command_, std::vector<double>(3));
        register_output("/wheel_leg/rl/features/height", feature_height_, 0.);
        register_output("/wheel_leg/rl/features/gyro", feature_gyro_, std::vector<double>(3));
        register_output("/wheel_leg/rl/features/gravity", feature_gravity_, std::vector<double>(3));
        register_output(
            "/wheel_leg/rl/features/position", feature_position_, std::vector<double>(6));
        register_output(
            "/wheel_leg/rl/features/velocity", feature_velocity_, std::vector<double>(6));
        register_output(
            "/wheel_leg/rl/features/previous_action", feature_action_, std::vector<double>(6));
        register_output(
            "/wheel_leg/rl/features/auxiliary", feature_auxiliary_, std::vector<double>(7));
        register_output("/wheel_leg/rl/features/frame_sequence", frame_sequence_, std::size_t{0});
        register_output("/wheel_leg/rl/features/epoch", epoch_, std::size_t{0});
        register_output(
            "/wheel_leg/rl/input/context", policy_context_,
            std::vector<double>{1., 0., 0., 0., 0., 0., 0.});
    }

    void update() override {
        (*policy_context_)[0] = *jump_active_ ? 0. : 1.;
        (*policy_context_)[4] = *jump_active_ ? 1. : 0.;
        (*policy_context_)[5] = *jump_active_ ? *jump_apex_ : 0.;
        (*policy_context_)[6] = *jump_active_ ? *jump_elapsed_ : 0.;
        constexpr int kRunning = static_cast<int>(rmcs_msgs::WheelLegRlState::kRunning);
        if (*reset_count_ != last_reset_count_ || (!*policy_enable_ && last_policy_enable_))
            reset();
        else if (*rl_state_ == kRunning && last_rl_state_ != kRunning)
            invalidate_frame();
        last_reset_count_ = *reset_count_;
        last_policy_enable_ = *policy_enable_;
        last_rl_state_ = *rl_state_;
        if (!*policy_enable_)
            return;

        const auto divisor = static_cast<std::size_t>(std::lround(*update_rate_ / *policy_rate_));
        if (last_sample_tick_ && *update_count_ - *last_sample_tick_ < divisor)
            return;
        last_sample_tick_ = *update_count_;
        update_features();
        *frame_sequence_ = ++next_frame_sequence_;
    }

private:
    void reset() {
        last_sample_tick_.reset();
        invalidate_frame();
    }

    void invalidate_frame() {
        for (auto* output :
             {&feature_command_, &feature_gyro_, &feature_gravity_, &feature_position_,
              &feature_velocity_, &feature_action_, &feature_auxiliary_})
            std::fill((*output)->begin(), (*output)->end(), 0.);
        *feature_height_ = 0.;
        *frame_sequence_ = 0;
        ++*epoch_;
    }

    void update_features() {
        for (std::size_t i = 0; i < 3; ++i) {
            (*feature_command_)[i] = (*policy_command_)[i];
            (*feature_gyro_)[i] = (*gyro_)[i];
            (*feature_gravity_)[i] = (*gravity_)[i];
        }
        *feature_height_ = *policy_height_;
        const auto& q = *position_;
        const auto nominal = wheel_leg::paired_targets(
            kPolicyReference, {q[0], q[1], q[2], q[3]}, wheel_leg::kWindingBounds);
        for (std::size_t i = 0; i < 6; ++i) {
            (*feature_position_)[i] = i < 4 ? q[i] - nominal[i] : 0.;
            (*feature_velocity_)[i] = (*velocity_)[i];
            (*feature_action_)[i] = *action_applied_ ? (*applied_action_)[i] : 0.;
        }
        std::copy(policy_context_->begin(), policy_context_->end(), feature_auxiliary_->begin());
    }

    InputInterface<wheel_leg::JointVector> position_, velocity_, applied_action_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, policy_command_;
    InputInterface<double> policy_height_, policy_rate_, update_rate_, jump_apex_, jump_elapsed_;
    InputInterface<bool> policy_enable_, action_applied_, jump_active_;
    InputInterface<int> rl_state_;
    InputInterface<std::size_t> reset_count_, update_count_;

    OutputInterface<std::vector<double>> feature_command_, feature_gyro_, feature_gravity_;
    OutputInterface<std::vector<double>> feature_position_, feature_velocity_, feature_action_;
    OutputInterface<std::vector<double>> feature_auxiliary_;
    OutputInterface<double> feature_height_;
    OutputInterface<std::size_t> frame_sequence_, epoch_;
    OutputInterface<std::vector<double>> policy_context_;

    // Actions must reach the controller before the policy samples its applied-action history.
    std::shared_ptr<WheelLegActionAdapter> action_adapter_{
        create_partner_component<WheelLegActionAdapter>(get_component_name() + "_actions")};
    std::optional<std::size_t> last_sample_tick_;
    std::size_t next_frame_sequence_ = 0;
    std::size_t last_reset_count_ = 0;
    int last_rl_state_ = 0;
    bool last_policy_enable_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegObservation, rmcs_executor::Component)
