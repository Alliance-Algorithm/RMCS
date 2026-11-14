#include <algorithm>
#include <optional>
#include <vector>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "wheel_leg/joint_coordinates.hpp"
#include "wheel_leg/jump_v4.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegObservation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegObservation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/wheel_leg/state/position", position_);
        register_input("/wheel_leg/state/velocity", velocity_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/rl/input/enable", policy_enable_);
        register_input("/wheel_leg/rl/input/command", policy_command_);
        register_input("/wheel_leg/rl/input/height", policy_height_);
        register_input("/wheel_leg/rl/input/context", policy_context_);
        register_input("/wheel_leg/rl/applied_action", applied_action_);
        register_input("/wheel_leg/rl/action_applied", action_applied_);
        register_input("/wheel_leg/rl/state", rl_state_);
        register_input("/wheel_leg/rl/policy_rate", policy_rate_);
        register_input("/chassis/reset_count", reset_count_);
        register_input("/predefined/update_count", update_count_);

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
        register_output("/wheel_leg/rl/input/frame_sequence", frame_sequence_, std::size_t{0});
    }

    void update() override {
        constexpr int kRunning = 3;
        if (*reset_count_ != last_reset_count_ || (!*policy_enable_ && last_policy_enable_)
            || (*rl_state_ == kRunning && last_rl_state_ != kRunning))
            reset();
        last_reset_count_ = *reset_count_;
        last_policy_enable_ = *policy_enable_;
        last_rl_state_ = *rl_state_;
        if (!*policy_enable_)
            return;

        const auto divisor = static_cast<std::size_t>(1000. / *policy_rate_);
        if (last_sample_tick_ && *update_count_ - *last_sample_tick_ < divisor)
            return;
        last_sample_tick_ = *update_count_;
        update_features();
        *frame_sequence_ = ++policy_tick_;
    }

private:
    void reset() {
        for (auto* output :
             {&feature_command_, &feature_gyro_, &feature_gravity_, &feature_position_,
              &feature_velocity_, &feature_action_, &feature_auxiliary_})
            std::fill((*output)->begin(), (*output)->end(), 0.);
        *feature_height_ = 0.;
        *frame_sequence_ = policy_tick_ = 0;
        last_sample_tick_.reset();
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
            {wheel_leg::kNominalPosition[0], wheel_leg::kNominalPosition[1],
             wheel_leg::kNominalPosition[2], wheel_leg::kNominalPosition[3]},
            {q[0], q[1], q[2], q[3]}, wheel_leg::jump_v4::kBounds);
        for (std::size_t i = 0; i < 6; ++i) {
            (*feature_position_)[i] = i < 4 ? q[i] - nominal[i] : 0.;
            (*feature_velocity_)[i] = (*velocity_)[i];
            (*feature_action_)[i] = *action_applied_ ? (*applied_action_)[i] : 0.;
        }
        std::copy(policy_context_->begin(), policy_context_->end(), feature_auxiliary_->begin());
    }

    InputInterface<wheel_leg::JointVector> position_, velocity_, applied_action_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, policy_command_;
    InputInterface<double> policy_height_, policy_rate_;
    InputInterface<std::vector<double>> policy_context_;
    InputInterface<bool> policy_enable_, action_applied_;
    InputInterface<int> rl_state_;
    InputInterface<std::size_t> reset_count_, update_count_;

    OutputInterface<std::vector<double>> feature_command_, feature_gyro_, feature_gravity_;
    OutputInterface<std::vector<double>> feature_position_, feature_velocity_, feature_action_;
    OutputInterface<std::vector<double>> feature_auxiliary_;
    OutputInterface<double> feature_height_;
    OutputInterface<std::size_t> frame_sequence_;

    std::optional<std::size_t> last_sample_tick_;
    std::size_t policy_tick_ = 0, last_reset_count_ = 0;
    int last_rl_state_ = 0;
    bool last_policy_enable_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegObservation, rmcs_executor::Component)
