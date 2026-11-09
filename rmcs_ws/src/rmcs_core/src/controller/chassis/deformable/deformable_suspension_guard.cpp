#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/deformable/suspension_guard.hpp"

namespace rmcs_core::controller::chassis {

class DeformableSuspensionGuard
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableSuspensionGuard()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , guard_({get_parameter_or("flat_enter_angle_deg", 3.0),
              get_parameter_or("flat_exit_angle_deg", 3.5),
              get_parameter_or("support_plane_tolerance_m", 0.006),
              get_parameter_or("flat_confirmation_s", 0.1),
              get_parameter_or("attitude_time_constant_s", 0.25),
              get_parameter_or("acceleration_time_constant_s", 0.05),
              get_parameter_or("acceleration_fusion_scale", 0.5)}) {
        max_imu_age_s_ = get_parameter_or("max_imu_age_s", 0.02);
        if (!std::isfinite(max_imu_age_s_) || max_imu_age_s_ <= 0.0)
            throw std::invalid_argument("max_imu_age_s must be finite and positive");
        register_input("/predefined/update_rate", update_rate_);
        register_input("/chassis/active_suspension/active", active_);
        register_input("/chassis/deformable/reset_count", reset_count_);
        register_input("/chassis/imu/quaternion", attitude_);
        register_input("/chassis/rl/gyro_b", gyro_);
        register_input("/chassis/imu/specific_force_b", specific_force_);
        register_input("/chassis/imu/sample_age", imu_age_);
        register_input("/chassis/rl/encoder_twist_b", encoder_twist_);
        constexpr const char* names[4] = {"right_front", "left_front", "left_back", "right_back"};
        for (std::size_t i = 0; i < 4; ++i)
            register_input(std::string("/chassis/") + names[i] + "_joint/urdf_angle", joint_q_[i]);
        register_output("/chassis/rl/flat_hold", flat_hold_, false);
        register_output("/chassis/rl/compensated_gravity_b", gravity_, Eigen::Vector3d::Constant(kNaN));
        register_output("/chassis/rl/linear_acceleration_b", acceleration_, Eigen::Vector3d::Constant(kNaN));
        register_output("/chassis/rl/estimated_support_grade_deg", grade_, kNaN);
        register_output("/chassis/rl/support_plane_error_m", plane_error_, kNaN);
    }

    void before_updating() override {
        last_reset_count_ = *reset_count_;
        last_active_ = *active_;
        clear_();
    }

    void update() override {
        if (*active_ != last_active_) {
            last_active_ = *active_;
            guard_.release_flat_hold();
        }
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            clear_();
            return;
        }
        if (!std::isfinite(*imu_age_) || *imu_age_ < 0.0 || *imu_age_ > max_imu_age_s_) {
            clear_();
            return;
        }
        Eigen::Vector4d q;
        for (std::size_t i = 0; i < 4; ++i)
            q[i] = *joint_q_[i];
        const auto result = guard_.update(*attitude_, *gyro_, *specific_force_, *encoder_twist_, q,
            1.0 / *update_rate_);
        *flat_hold_ = *active_ && result.flat_hold;
        *gravity_ = result.gravity_b;
        *acceleration_ = result.linear_acceleration_b;
        *grade_ = result.grade_deg;
        *plane_error_ = result.plane_error_m;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    void clear_() {
        guard_.reset();
        *flat_hold_ = false;
        *gravity_ = Eigen::Vector3d::Constant(kNaN);
        *acceleration_ = Eigen::Vector3d::Constant(kNaN);
        *grade_ = *plane_error_ = kNaN;
    }
    InputInterface<double> update_rate_, imu_age_;
    InputInterface<bool> active_;
    InputInterface<std::size_t> reset_count_;
    InputInterface<Eigen::Quaterniond> attitude_;
    InputInterface<Eigen::Vector3d> gyro_, specific_force_, encoder_twist_;
    std::array<InputInterface<double>, 4> joint_q_;
    OutputInterface<bool> flat_hold_;
    OutputInterface<Eigen::Vector3d> gravity_, acceleration_;
    OutputInterface<double> grade_, plane_error_;
    deformable_rl::SuspensionGuard guard_;
    double max_imu_age_s_ = 0.02;
    std::size_t last_reset_count_ = 0;
    bool last_active_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableSuspensionGuard, rmcs_executor::Component)
