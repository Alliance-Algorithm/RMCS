#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "deformable/rl_contract.hpp"

namespace rmcs_core::controller::chassis {

class DeformableRlSuspension
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableRlSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const auto rl_base = get_parameter_or<std::string>("rl_base", "/chassis/rl");
        const auto joint_base_path = get_parameter_or<std::string>("joint_base_path", "/chassis");
        const auto joint_suffix = get_parameter_or<std::string>("joint_suffix", "_joint");
        const auto angle_suffix = get_parameter_or<std::string>("angle_suffix", "/physical_angle");
        const auto output_angle_suffix =
            get_parameter_or<std::string>("output_angle_suffix", "/rl_target_physical_angle");
        const auto output_velocity_suffix =
            get_parameter_or<std::string>("output_velocity_suffix", "/rl_target_physical_velocity");
        const auto output_acceleration_suffix = get_parameter_or<std::string>(
            "output_acceleration_suffix", "/rl_target_physical_acceleration");
        const auto output_error_suffix =
            get_parameter_or<std::string>("output_error_suffix", "/rl_control_angle_error");

        const auto action_contract = get_parameter_or<std::string>("action_contract_version", "legacy_v1");
        if (action_contract != "legacy_v1" && action_contract != "minangle_physical_v3")
            throw std::invalid_argument("unsupported deformable RL action contract");
        use_v3_ = action_contract == "minangle_physical_v3";
        target_rate_limit_ = get_parameter_or("leg_target_rate_limit", 2.0);
        if (use_v3_ && (!std::isfinite(target_rate_limit_) || target_rate_limit_ <= 0.0))
            throw std::invalid_argument("V3 leg_target_rate_limit must be finite and positive");
        leg_action_scale_ = get_parameter_or("leg_action_scale", 0.15);
        q_target_min_rad_ = get_parameter_or("q_target_min_rad", 0.0);
        q_target_max_rad_ = get_parameter_or("q_target_max_rad", 1.0563);
        if (use_v3_ && (!std::isfinite(q_target_min_rad_) || !std::isfinite(q_target_max_rad_)
            || q_target_min_rad_ >= q_target_max_rad_))
            throw std::invalid_argument("invalid V3 URDF joint limits");
        if (q_target_min_rad_ > q_target_max_rad_)
            std::swap(q_target_min_rad_, q_target_max_rad_);

        physical_min_rad_ =
            get_parameter_or("physical_min_rad", -std::numeric_limits<double>::infinity());
        physical_max_rad_ =
            get_parameter_or("physical_max_rad", std::numeric_limits<double>::infinity());
        if (physical_min_rad_ > physical_max_rad_)
            std::swap(physical_min_rad_, physical_max_rad_);

        const auto joint_path = [&](std::size_t corner, const std::string& suffix) {
            return joint_base_path + "/" + kJointName[corner] + joint_suffix + suffix;
        };

        register_input("/predefined/update_rate", update_rate_, false);
        register_input(rl_base + "/valid", valid_, false);
        register_input(rl_base + "/healthy", healthy_, false);
        register_input("/chassis/active_suspension/active", active_suspension_);
        register_input("/chassis/deformable/reset_count", reset_count_);
        register_input("/chassis/deformable/rl_q_cmd", q_cmd_);
        register_input("/chassis/rl/calibration/high_physical_angle_rad", high_physical_angle_);
        register_input("/chassis/rl/calibration/low_physical_angle_rad", low_physical_angle_);
        register_input("/chassis/rl/calibration/q_max_rad", q_max_rad_);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            register_input(
                rl_base + "/action/joint_leg_" + std::to_string(leg + 1), action_[leg], false);

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            register_input(joint_path(corner, angle_suffix), physical_angle_[corner]);
            register_output(joint_path(corner, output_angle_suffix), target_angle_[corner], kNaN);
            register_output(
                joint_path(corner, output_velocity_suffix), target_velocity_[corner], kNaN);
            register_output(
                joint_path(corner, output_acceleration_suffix), target_acceleration_[corner], kNaN);
            register_output(joint_path(corner, output_error_suffix), angle_error_[corner], kNaN);
        }
    }

    void before_updating() override {
        if (!update_rate_.ready())
            update_rate_.make_and_bind_directly(1000.0);
        if (!valid_.ready())
            valid_.make_and_bind_directly(0.0);
        if (!healthy_.ready())
            healthy_.make_and_bind_directly(0.0);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            if (!action_[leg].ready())
                action_[leg].make_and_bind_directly(kNaN);
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

        if (!*active_suspension_) {
            publish_nan_targets_();
            return;
        }

        const bool authoritative = *valid_ > 0.5 && *healthy_ > 0.5;
        const double q_cmd = *q_cmd_;
        const double high_physical = *high_physical_angle_;
        const double low_physical = *low_physical_angle_;
        const double q_max = *q_max_rad_;
        const double span = high_physical - low_physical;
        const bool calibration_ok = use_v3_
            ? std::isfinite(q_cmd) && q_target_min_rad_ <= q_cmd && q_cmd <= q_target_max_rad_
            : std::isfinite(high_physical) && std::isfinite(low_physical)
                && std::isfinite(q_max) && q_max > 0.0 && span > 1e-9;
        const double dt = std::isfinite(*update_rate_) && *update_rate_ > 0.0
            ? 1.0 / *update_rate_ : kNaN;
        if (!authoritative || !calibration_ok || !std::isfinite(q_cmd) || !std::isfinite(dt)) {
            publish_nan_targets_();
            return;
        }
        // A single invalid corner must relinquish the entire four-leg trajectory.
        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            if (!std::isfinite(*physical_angle_[kCornerForLeg[leg]]) || !std::isfinite(*action_[leg])) {
                publish_nan_targets_();
                return;
            }
        }

        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            const auto corner = kCornerForLeg[leg];
            const double physical = *physical_angle_[corner];
            const double raw_action = *action_[leg];

            const double q_target = use_v3_
                ? deformable_rl::suspension_target(raw_action, q_cmd, q_target_min_rad_, q_target_max_rad_)
                : std::clamp(q_cmd + leg_action_scale_ * raw_action, q_target_min_rad_, q_target_max_rad_);
            double p_target = std::clamp(
                use_v3_ ? deformable_rl::kPhysicalZero - q_target : high_physical - q_target * span / q_max,
                physical_min_rad_, physical_max_rad_);
            if (use_v3_) {
                if (!std::isfinite(previous_target_[corner]))
                    previous_target_[corner] = physical;
                const double max_change = target_rate_limit_ * dt;
                p_target = previous_target_[corner]
                    + std::clamp(p_target - previous_target_[corner], -max_change, max_change);
                previous_target_[corner] = p_target;
            }
            *target_angle_[corner] = p_target;
            *target_velocity_[corner] = kNaN;
            *target_acceleration_[corner] = kNaN;
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

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    void publish_nan_targets_() {
        previous_target_.fill(kNaN);
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            *target_angle_[corner] = kNaN;
            *target_velocity_[corner] = kNaN;
            *target_acceleration_[corner] = kNaN;
            *angle_error_[corner] = kNaN;
        }
    }

    InputInterface<double> update_rate_;
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

    bool use_v3_ = false;
    double target_rate_limit_ = 2.0;
    std::array<double, kCornerCount> previous_target_{};
    double leg_action_scale_ = 0.15;
    double q_target_min_rad_ = 0.0;
    double q_target_max_rad_ = 1.0563;
    double physical_min_rad_ = -std::numeric_limits<double>::infinity();
    double physical_max_rad_ = std::numeric_limits<double>::infinity();

    std::size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableRlSuspension, rmcs_executor::Component)
