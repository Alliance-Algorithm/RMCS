#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class DeformableRlSuspension
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableRlSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        rl_base_ = get_parameter_or<std::string>("rl_base", "/chassis/rl");
        joint_base_path_ = get_parameter_or<std::string>("joint_base_path", "/chassis");
        joint_suffix_ = get_parameter_or<std::string>("joint_suffix", "_joint");
        angle_suffix_ = get_parameter_or<std::string>("angle_suffix", "/physical_angle");
        output_angle_suffix_ = get_parameter_or<std::string>(
            "output_angle_suffix", "/rl_target_physical_angle");
        output_velocity_suffix_ = get_parameter_or<std::string>(
            "output_velocity_suffix", "/rl_target_physical_velocity");
        output_acceleration_suffix_ = get_parameter_or<std::string>(
            "output_acceleration_suffix", "/rl_target_physical_acceleration");
        output_error_suffix_ =
            get_parameter_or<std::string>("output_error_suffix", "/rl_control_angle_error");

        leg_action_scale_ = get_parameter_or("leg_action_scale", 0.15);
        q_target_min_rad_ = get_parameter_or("q_target_min_rad", 0.0);
        q_target_max_rad_ = get_parameter_or("q_target_max_rad", 1.0563);
        if (q_target_min_rad_ > q_target_max_rad_)
            std::swap(q_target_min_rad_, q_target_max_rad_);

        physical_min_rad_ =
            get_parameter_or("physical_min_rad", -std::numeric_limits<double>::infinity());
        physical_max_rad_ =
            get_parameter_or("physical_max_rad", std::numeric_limits<double>::infinity());
        if (physical_min_rad_ > physical_max_rad_)
            std::swap(physical_min_rad_, physical_max_rad_);

        register_input(rl_base_ + "/valid", valid_, false);
        register_input(rl_base_ + "/healthy", healthy_, false);
        register_input("/chassis/active_suspension/active", active_suspension_);
        register_input("/chassis/deformable/reset_count", reset_count_);
        register_input("/chassis/deformable/rl_q_cmd", q_cmd_);
        register_input("/chassis/rl/calibration/high_physical_angle_rad", high_physical_angle_);
        register_input("/chassis/rl/calibration/low_physical_angle_rad", low_physical_angle_);
        register_input("/chassis/rl/calibration/q_max_rad", q_max_rad_);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            register_input(
                rl_base_ + "/action/joint_leg_" + std::to_string(leg + 1), action_[leg], false);

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            register_input(joint_path_(corner, angle_suffix_), physical_angle_[corner]);
            register_output(
                joint_path_(corner, output_angle_suffix_), target_angle_[corner], nan_);
            register_output(
                joint_path_(corner, output_velocity_suffix_), target_velocity_[corner], nan_);
            register_output(
                joint_path_(corner, output_acceleration_suffix_), target_acceleration_[corner],
                nan_);
            register_output(joint_path_(corner, output_error_suffix_), angle_error_[corner], nan_);
        }
    }

    void before_updating() override {
        if (!valid_.ready())
            valid_.make_and_bind_directly(0.0);
        if (!healthy_.ready())
            healthy_.make_and_bind_directly(0.0);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            if (!action_[leg].ready())
                action_[leg].make_and_bind_directly(nan_);
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!physical_angle_[corner].ready())
                throw std::runtime_error("missing deformable joint physical angle interface");
        }

        last_reset_count_ = *reset_count_;
        publish_nan_targets_();
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            publish_nan_targets_();
            return;
        }

        const bool suspension_active = *active_suspension_;
        if (!suspension_state_initialized_ || suspension_active != last_suspension_active_) {
            suspension_state_initialized_ = true;
            last_suspension_active_ = suspension_active;
            RCLCPP_INFO(
                get_logger(), "RL suspension authority changed: active=%s",
                suspension_active ? "true" : "false");
            if (!suspension_active)
                publish_nan_targets_();
        }

        if (!suspension_active) {
            publish_nan_targets_();
            return;
        }

        const bool authoritative = *valid_ > 0.5 && *healthy_ > 0.5;
        const double q_cmd = *q_cmd_;
        const double high_physical = *high_physical_angle_;
        const double low_physical = *low_physical_angle_;
        const double q_max = *q_max_rad_;
        const double span = high_physical - low_physical;
        const bool calibration_ok = std::isfinite(high_physical) && std::isfinite(low_physical)
                                 && std::isfinite(q_max) && q_max > 0.0 && span > 1e-9;

        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            const auto corner = kCornerForLeg[leg];
            const double physical = *physical_angle_[corner];
            const double raw_action = *action_[leg];

            if (!authoritative || !calibration_ok || !std::isfinite(q_cmd)
                || !std::isfinite(physical) || !std::isfinite(raw_action)) {
                *target_angle_[corner] = nan_;
                *target_velocity_[corner] = nan_;
                *target_acceleration_[corner] = nan_;
                *angle_error_[corner] = nan_;
                continue;
            }

            const double q_target = std::clamp(
                q_cmd + leg_action_scale_ * raw_action, q_target_min_rad_, q_target_max_rad_);
            const double p_target = std::clamp(
                high_physical - q_target * span / q_max, physical_min_rad_, physical_max_rad_);
            *target_angle_[corner] = p_target;
            *target_velocity_[corner] = nan_;
            *target_acceleration_[corner] = nan_;
            *angle_error_[corner] = physical - p_target;
        }
    }

private:
    static constexpr std::size_t kLegCount = 4;
    static constexpr std::size_t kCornerCount = 4;
    static constexpr std::size_t kLeftFront = 0;
    static constexpr std::size_t kLeftBack = 1;
    static constexpr std::size_t kRightBack = 2;
    static constexpr std::size_t kRightFront = 3;

    static constexpr const char* kJointName[kCornerCount] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    static constexpr std::size_t kCornerForLeg[kLegCount] = {
        kRightFront,
        kLeftFront,
        kLeftBack,
        kRightBack,
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    std::string joint_path_(std::size_t corner, const std::string& suffix) const {
        return joint_base_path_ + "/" + kJointName[corner] + joint_suffix_ + suffix;
    }

    void publish_nan_targets_() {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            *target_angle_[corner] = nan_;
            *target_velocity_[corner] = nan_;
            *target_acceleration_[corner] = nan_;
            *angle_error_[corner] = nan_;
        }
    }

    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<bool> active_suspension_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<double> q_cmd_;
    InputInterface<double> high_physical_angle_;
    InputInterface<double> low_physical_angle_;
    InputInterface<double> q_max_rad_;

    std::array<InputInterface<double>, kLegCount> action_;
    std::array<InputInterface<double>, kCornerCount> physical_angle_;
    std::array<OutputInterface<double>, kCornerCount> target_angle_;
    std::array<OutputInterface<double>, kCornerCount> target_velocity_;
    std::array<OutputInterface<double>, kCornerCount> target_acceleration_;
    std::array<OutputInterface<double>, kCornerCount> angle_error_;

    std::string rl_base_;
    std::string joint_base_path_;
    std::string joint_suffix_;
    std::string angle_suffix_;
    std::string output_angle_suffix_;
    std::string output_velocity_suffix_;
    std::string output_acceleration_suffix_;
    std::string output_error_suffix_;

    double leg_action_scale_ = 0.15;
    double q_target_min_rad_ = 0.0;
    double q_target_max_rad_ = 1.0563;
    double physical_min_rad_ = -std::numeric_limits<double>::infinity();
    double physical_max_rad_ = std::numeric_limits<double>::infinity();

    std::size_t last_reset_count_ = 0;
    bool suspension_state_initialized_ = false;
    bool last_suspension_active_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableRlSuspension, rmcs_executor::Component)
