#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>

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

        hold_on_invalid_ = get_parameter_or("hold_on_invalid", true);
        posture_fallback_ = get_parameter_or("posture_fallback", true);

        register_input(rl_base_ + "/valid", valid_, false);
        register_input(rl_base_ + "/healthy", healthy_, false);
        register_input("/chassis/active_suspension/active", active_suspension_, false);
        register_input("/chassis/deformable/reset_count", reset_count_, false);
        register_input("/chassis/deformable/rl_q_cmd", q_cmd_, false);
        register_input(
            "/chassis/rl/calibration/high_physical_angle_rad", high_physical_angle_, false);
        register_input(
            "/chassis/rl/calibration/low_physical_angle_rad", low_physical_angle_, false);
        register_input("/chassis/rl/calibration/q_max_rad", q_max_rad_, false);
        register_input("/chassis/deformable/min_angle_deg", min_angle_deg_, false);
        register_input("/chassis/deformable/max_angle_deg", max_angle_deg_, false);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            register_input(
                rl_base_ + "/action/joint_leg_" + std::to_string(leg + 1), action_[leg], false);

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            register_input(joint_path_(corner, angle_suffix_), physical_angle_[corner], false);
            register_input(
                posture_path_(corner), posture_target_angle_[corner], false);
            register_output(
                joint_path_(corner, "/target_physical_angle"), target_angle_[corner], nan_);
        }
    }

    void before_updating() override {
        if (!valid_.ready())
            valid_.make_and_bind_directly(0.0);
        if (!healthy_.ready())
            healthy_.make_and_bind_directly(0.0);
        if (!active_suspension_.ready())
            active_suspension_.make_and_bind_directly(false);
        if (!reset_count_.ready())
            reset_count_.make_and_bind_directly(static_cast<std::size_t>(0));
        if (!q_cmd_.ready())
            q_cmd_.make_and_bind_directly(0.0);
        if (!high_physical_angle_.ready())
            high_physical_angle_.make_and_bind_directly(kDefaultHighPhysicalAngleRad);
        if (!low_physical_angle_.ready())
            low_physical_angle_.make_and_bind_directly(kDefaultLowPhysicalAngleRad);
        if (!q_max_rad_.ready())
            q_max_rad_.make_and_bind_directly(kDefaultQMaxRad);
        if (!min_angle_deg_.ready())
            min_angle_deg_.make_and_bind_directly(kDefaultMinAngleDeg);
        if (!max_angle_deg_.ready())
            max_angle_deg_.make_and_bind_directly(kDefaultMaxAngleDeg);

        for (std::size_t leg = 0; leg < kLegCount; ++leg)
            if (!action_[leg].ready())
                action_[leg].make_and_bind_directly(nan_);
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!physical_angle_[corner].ready())
                physical_angle_[corner].make_and_bind_directly(nan_);
            if (!posture_target_angle_[corner].ready())
                posture_target_angle_[corner].make_and_bind_directly(nan_);
        }

        last_reset_count_ = *reset_count_;
    }

    void update() override {
        // A reset only invalidates RL-side bookkeeping; it must never blank the leg targets.
        // `/chassis/deformable/reset_count` churns every cycle while the remote is UNKNOWN, so
        // returning here would leave /chassis/*/target_physical_angle at NaN forever and the
        // joint controllers would disable their torque output.
        if (*reset_count_ != last_reset_count_)
            last_reset_count_ = *reset_count_;

        const bool suspension_active = *active_suspension_;
        const bool authoritative = suspension_active && *valid_ > 0.5 && *healthy_ > 0.5;

        const double q_cmd = *q_cmd_;
        const double high_physical = *high_physical_angle_;
        const double low_physical = *low_physical_angle_;
        const double q_max = *q_max_rad_;
        const double span = high_physical - low_physical;
        const bool calibration_ok = std::isfinite(high_physical) && std::isfinite(low_physical)
            && std::isfinite(q_max) && q_max > 0.0 && span > 1e-9;

        const double min_angle_deg = *min_angle_deg_;
        const double max_angle_deg = *max_angle_deg_;
        const bool posture_limits_ok = std::isfinite(min_angle_deg) && std::isfinite(max_angle_deg)
            && max_angle_deg > min_angle_deg;
        const double posture_min_rad =
            (min_angle_deg - kMinAngleMarginDeg) * std::numbers::pi / 180.0;
        const double posture_max_rad = max_angle_deg * std::numbers::pi / 180.0;

        for (std::size_t leg = 0; leg < kLegCount; ++leg) {
            const auto corner = kCornerForLeg[leg];
            double& target = *target_angle_[corner];
            const double physical = *physical_angle_[corner];
            const double raw_action = *action_[leg];

            // 未开启主动悬挂：退回老 DeformableSuspension 的非激活行为，跟随底盘姿态目标
            // （默认起立位），RL 不参与。
            if (!suspension_active) {
                const double posture_target = *posture_target_angle_[corner];
                if (posture_fallback_ && posture_limits_ok && std::isfinite(posture_target))
                    target = std::clamp(posture_target, posture_min_rad, posture_max_rad);
                else
                    target = hold_on_invalid_ ? physical : nan_;
                continue;
            }

            if (!authoritative || !calibration_ok || !std::isfinite(q_cmd)
                || !std::isfinite(physical) || !std::isfinite(raw_action)) {
                target = hold_on_invalid_ ? physical : nan_;
                continue;
            }

            const double q_target = std::clamp(
                q_cmd + leg_action_scale_ * raw_action, q_target_min_rad_, q_target_max_rad_);
            const double p_target = std::clamp(
                high_physical - q_target * span / q_max, physical_min_rad_, physical_max_rad_);
            target = p_target;
        }
    }

private:
    static constexpr std::size_t kLegCount = 4;
    static constexpr std::size_t kCornerCount = 4;
    static constexpr std::size_t kLeftFront = 0;
    static constexpr std::size_t kLeftBack = 1;
    static constexpr std::size_t kRightBack = 2;
    static constexpr std::size_t kRightFront = 3;

    // Real corner order used by the hardware / joint controllers.
    static constexpr const char* kJointName[kCornerCount] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    // RL leg order is joint_leg_1..4 = RF, LF, LB, RB.
    static constexpr std::size_t kCornerForLeg[kLegCount] = {
        kRightFront,
        kLeftFront,
        kLeftBack,
        kRightBack,
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kDefaultHighPhysicalAngleRad = 59.0 * std::numbers::pi / 180.0;
    static constexpr double kDefaultLowPhysicalAngleRad = 5.0 * std::numbers::pi / 180.0;
    static constexpr double kDefaultQMaxRad = 1.36;
    static constexpr double kMinAngleMarginDeg = 5.0;
    static constexpr double kDefaultMinAngleDeg = 8.0;
    static constexpr double kDefaultMaxAngleDeg = 59.0;

    std::string joint_path_(std::size_t corner, const std::string& suffix) const {
        return joint_base_path_ + "/" + kJointName[corner] + joint_suffix_ + suffix;
    }

    std::string posture_path_(std::size_t corner) const {
        return std::string{"/chassis/deformable/"} + kJointName[corner]
             + "_joint/posture_target_angle";
    }

    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<bool> active_suspension_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<double> q_cmd_;
    InputInterface<double> high_physical_angle_;
    InputInterface<double> low_physical_angle_;
    InputInterface<double> q_max_rad_;
    InputInterface<double> min_angle_deg_;
    InputInterface<double> max_angle_deg_;

    std::array<InputInterface<double>, kLegCount> action_;
    std::array<InputInterface<double>, kCornerCount> physical_angle_;
    std::array<InputInterface<double>, kCornerCount> posture_target_angle_;
    std::array<OutputInterface<double>, kCornerCount> target_angle_;

    std::string rl_base_;
    std::string joint_base_path_;
    std::string joint_suffix_;
    std::string angle_suffix_;

    double leg_action_scale_ = 0.15;
    double q_target_min_rad_ = 0.0;
    double q_target_max_rad_ = 1.0563;
    double physical_min_rad_ = -std::numeric_limits<double>::infinity();
    double physical_max_rad_ = std::numeric_limits<double>::infinity();
    bool hold_on_invalid_ = true;
    bool posture_fallback_ = true;

    std::size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableRlSuspension, rmcs_executor::Component)
