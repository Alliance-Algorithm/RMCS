#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::chassis {

/**
 * 变形舵轮底盘控制器。
 *
 * 结构参照 HeroSteeringWheelController：
 *   1. 规划器：期望速度状态 chassis_velocity_expected_ + 加速度指令 + 限幅 + 低通 + 积分；
 *   2. 转向解析前馈（由期望速度/期望加速度直接给出转向角速度）；
 *   3. 轮速前馈（由期望加速度分配轮力矩）+ 轮速反馈。
 *
 * 在此基础上增加“变轮距”项：
 *   - 每个模块半径 r_i = chassis_radius + rod_length * cos(alpha_i)；
 *   - 接触点速度含径向速度 r_dot_i * e_r_i；
 *   - 接触点加速度含径向加速度 r_ddot_i * e_r_i 与耦合项 omega * r_dot_i * e_r_i；
 *   - 转向前馈、轮速参考、力矩分配、功率约束中的半径均按模块各自半径计算。
 *
 * 坐标系：BaseLink（x 前，y 左）。模块位置在四个对角线上（±45°），
 * 舵向角约定为“轮子朝向在 BaseLink 下的角度”，可用 steering_angle_offset 统一平移零点。
 */
class DeformableSteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double, double, double, double>;

public:
    DeformableSteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mass_(get_parameter("mass").as_double())
        , moment_of_inertia_(get_parameter("moment_of_inertia").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double())
        , friction_coefficient_(get_parameter_or("friction_coefficient", 0.6))
        , k1_(get_parameter("k1").as_double())
        , k2_(get_parameter("k2").as_double())
        , no_load_power_(get_parameter("no_load_power").as_double())
        , chassis_radius_base_(get_parameter_or("chassis_radius", 0.2341741))
        , rod_length_(get_parameter_or("rod_length", 0.140))
        , steering_angle_offset_(get_parameter_or("steering_angle_offset", 0.0))
        , control_acceleration_filter_hz_(get_parameter_or("control_acceleration_filter_hz", 5.0))
        , energy_clamp_enabled_(get_parameter_or("chassis_energy_clamp", true))

        , deform_speed_limit_(get_parameter_or("deform_speed_limit", 0.35))
        , deform_speed_threshold_(get_parameter_or("deform_speed_threshold", 0.004))
        , deform_rate_filter_cutoff_hz_(get_parameter_or("deform_rate_filter_cutoff_hz", 20.0))
        , steering_angle_velocity_limit_(get_parameter_or("steering_angle_velocity_limit", 10.0))
        , steering_aligned_tolerance_(get_parameter_or("steering_aligned_tolerance", 0.06))
        , steering_angle_kp_(get_parameter_or("steering_angle_kp", 12.0))
        , steering_angle_ki_(get_parameter_or("steering_angle_ki", 0.0))
        , steering_angle_kd_(get_parameter_or("steering_angle_kd", 0.0))
        , steering_velocity_kp_(get_parameter_or("steering_velocity_kp", 0.5))
        , steering_velocity_ki_(get_parameter_or("steering_velocity_ki", 0.0))
        , steering_velocity_kd_(get_parameter_or("steering_velocity_kd", 0.0))
        , steering_torque_limit_(get_parameter_or("steering_torque_limit", 4.0))
        , chassis_translational_velocity_pid_(5.0, 0.0, 1.0)
        , chassis_angular_velocity_pid_(5.0, 0.0, 1.0)
        , wheel_velocity_pid_(0.6, 0.0, 0.0)
        , steering_angle_pid_(steering_angle_kp_, steering_angle_ki_, steering_angle_kd_)
        , steering_velocity_pid_(
              steering_velocity_kp_, steering_velocity_ki_, steering_velocity_kd_)
        , control_acceleration_filter_(1.0)
        , module_radial_rate_filter_(1.0)
        , chassis_velocity_expected_(Eigen::Vector3d::Zero())
        , cos_varphi_(-s2, -s2, +s2, +s2)
        , sin_varphi_(+s2, -s2, -s2, +s2) {

        register_input("/predefined/update_rate", update_rate_, false);

        register_input("/chassis/left_front_wheel/velocity", wheel_velocity_[0]);
        register_input("/chassis/left_back_wheel/velocity", wheel_velocity_[1]);
        register_input("/chassis/right_back_wheel/velocity", wheel_velocity_[2]);
        register_input("/chassis/right_front_wheel/velocity", wheel_velocity_[3]);

        register_input("/chassis/left_front_steering/angle", steering_angle_[0]);
        register_input("/chassis/left_back_steering/angle", steering_angle_[1]);
        register_input("/chassis/right_back_steering/angle", steering_angle_[2]);
        register_input("/chassis/right_front_steering/angle", steering_angle_[3]);

        register_input("/chassis/left_front_steering/velocity", steering_velocity_[0]);
        register_input("/chassis/left_back_steering/velocity", steering_velocity_[1]);
        register_input("/chassis/right_back_steering/velocity", steering_velocity_[2]);
        register_input("/chassis/right_front_steering/velocity", steering_velocity_[3]);

        register_input("/chassis/left_front_joint/physical_angle", joint_angle_[0]);
        register_input("/chassis/left_back_joint/physical_angle", joint_angle_[1]);
        register_input("/chassis/right_back_joint/physical_angle", joint_angle_[2]);
        register_input("/chassis/right_front_joint/physical_angle", joint_angle_[3]);

        register_input("/chassis/left_front_joint/physical_velocity", joint_velocity_[0]);
        register_input("/chassis/left_back_joint/physical_velocity", joint_velocity_[1]);
        register_input("/chassis/right_back_joint/physical_velocity", joint_velocity_[2]);
        register_input("/chassis/right_front_joint/physical_velocity", joint_velocity_[3]);

        // 变形规划器给出的关节目标角加速度（DeformableSuspension 发布），用于径向加速度前馈。
        // 可选：未就绪或空闲时为 NaN，前馈按 0 处理。
        register_input(
            "/chassis/left_front_joint/target_physical_acceleration", joint_acceleration_[0],
            false);
        register_input(
            "/chassis/left_back_joint/target_physical_acceleration", joint_acceleration_[1], false);
        register_input(
            "/chassis/right_back_joint/target_physical_acceleration", joint_acceleration_[2],
            false);
        register_input(
            "/chassis/right_front_joint/target_physical_acceleration", joint_acceleration_[3],
            false);

        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, false);
        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_power_limit", power_limit_);

        register_output(
            "/chassis/left_front_steering/control_torque", steering_control_torque_[0], 0.0);
        register_output(
            "/chassis/left_back_steering/control_torque", steering_control_torque_[1], 0.0);
        register_output(
            "/chassis/right_back_steering/control_torque", steering_control_torque_[2], 0.0);
        register_output(
            "/chassis/right_front_steering/control_torque", steering_control_torque_[3], 0.0);

        register_output("/chassis/left_front_wheel/control_torque", wheel_control_torque_[0], 0.0);
        register_output("/chassis/left_back_wheel/control_torque", wheel_control_torque_[1], 0.0);
        register_output("/chassis/right_back_wheel/control_torque", wheel_control_torque_[2], 0.0);
        register_output("/chassis/right_front_wheel/control_torque", wheel_control_torque_[3], 0.0);

        register_output("/chassis/steering_aligned", steering_aligned_, false);
        register_output("/chassis/deform/active", deform_active_, false);

        steering_angle_pid_.output_min.setConstant(-steering_angle_velocity_limit_);
        steering_angle_pid_.output_max.setConstant(+steering_angle_velocity_limit_);
        steering_velocity_pid_.output_min.setConstant(-steering_torque_limit_);
        steering_velocity_pid_.output_max.setConstant(+steering_torque_limit_);

        control_acceleration_filter_.set_cutoff(control_acceleration_filter_hz_, 1000.0);
    }

    void before_updating() override {
        if (!chassis_yaw_velocity_imu_.ready()) {
            chassis_yaw_velocity_imu_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/chassis/yaw/velocity_imu\". Set to 0.0.");
        }
        for (auto& input : joint_acceleration_)
            if (!input.ready())
                input.make_and_bind_directly(nan_);

        if (!update_rate_.ready() || !std::isfinite(*update_rate_) || *update_rate_ <= 1e-6)
            update_rate_hz_ = 1000.0;
        else
            update_rate_hz_ = *update_rate_;

        control_acceleration_filter_.set_cutoff(control_acceleration_filter_hz_, update_rate_hz_);
        module_radial_rate_filter_.set_cutoff(deform_rate_filter_cutoff_hz_, update_rate_hz_);

        RCLCPP_INFO(
            get_logger(), "Deformable steering chassis: mass %.2f kg, wheel radius %.3f m", mass_,
            wheel_radius_);
    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        const double dt = 1.0 / update_rate_hz_;

        update_deform_state();

        const auto steering_status = calculate_steering_status();
        if (!steering_status.valid) {
            reset_all_controls();
            return;
        }

        const auto wheel_velocities = calculate_wheel_velocities();
        const auto measured_velocity =
            calculate_chassis_velocity(steering_status, wheel_velocities);

        // ---- 规划器：期望速度 → 加速度指令 → 限幅 → 低通 → 积分 ----
        integral_yaw_angle_imu(dt);
        const auto chassis_status_expected = calculate_chassis_status_expected(measured_velocity);

        auto chassis_control_velocity = chassis_control_velocity_->vector;
        if (deform_active_state_)
            chassis_control_velocity.head<2>() =
                clamp_norm(chassis_control_velocity.head<2>(), deform_speed_limit_);

        auto chassis_acceleration = calculate_chassis_control_acceleration(
            chassis_status_expected.velocity, chassis_control_velocity);

        double available_power =
            *power_limit_ - no_load_power_ - k2_ * wheel_velocities.array().pow(2).sum();

        const auto wheel_pid_torques =
            calculate_wheel_pid_torques(steering_status, wheel_velocities, chassis_status_expected);

        const auto constrained_acceleration = constrain_chassis_control_acceleration(
            steering_status, wheel_velocities, chassis_acceleration, wheel_pid_torques,
            available_power);

        const auto filtered_acceleration =
            odom_to_base_link_vector(control_acceleration_filter_.update(
                base_link_to_odom_vector(constrained_acceleration)));

        const auto steering_torques = calculate_steering_control_torques(
            steering_status, chassis_status_expected, filtered_acceleration);
        const auto wheel_torques = calculate_wheel_control_torques(
            steering_status, filtered_acceleration, wheel_pid_torques);

        update_control_torques(steering_torques, wheel_torques);
        update_chassis_velocity_expected(filtered_acceleration, dt);
    }

private:
    static constexpr size_t kWheelCount = 4;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double g_ = 9.81;
    static constexpr double s2 = std::numbers::sqrt2 / 2;

    struct SteeringStatus {
        bool valid = false;
        Eigen::Vector4d angle, cos_angle, sin_angle;
        Eigen::Vector4d velocity;
    };

    struct ChassisStatus {
        Eigen::Vector3d velocity;
        Eigen::Vector4d wheel_velocity_x, wheel_velocity_y;
    };

    // ---------------------------------------------------------------- 变形状态

    /// 模块半径：r_i = chassis_radius + rod_length * cos(alpha_i)。
    double module_radius(size_t i) const {
        const double alpha = *joint_angle_[i];
        if (!std::isfinite(alpha))
            return chassis_radius_base_ + rod_length_; // 与硬件 kDefaultRadius 一致
        return std::clamp(chassis_radius_base_ + rod_length_ * std::cos(alpha), 1e-3, 1.0);
    }

    double mean_module_radius() const {
        double sum = 0.0;
        for (size_t i = 0; i < kWheelCount; ++i)
            sum += module_radius(i);
        return sum / static_cast<double>(kWheelCount);
    }

    /// 模块径向方向（BaseLink，单位向量，四个对角线）。
    Eigen::Vector2d module_radial_direction(size_t i) const {
        return {cos_varphi_[i], sin_varphi_[i]};
    }

    /// 模块位置（BaseLink）。
    Eigen::Vector2d module_position(size_t i) const {
        return module_radius(i) * module_radial_direction(i);
    }

    /// 径向速度：r_dot = -rod_length * sin(alpha) * alpha_dot，逐轮低通。
    /// 径向加速度：r_ddot = -rod_length * (cos(alpha) * alpha_dot^2 + sin(alpha) * alpha_ddot)，
    /// alpha_ddot 取自变形规划器的目标角加速度（拿不到时按 0 处理）。
    void update_deform_state() {
        Eigen::Vector4d raw_rate;
        for (size_t i = 0; i < kWheelCount; ++i) {
            const double alpha = *joint_angle_[i];
            const double alpha_dot = *joint_velocity_[i];
            raw_rate[i] = (std::isfinite(alpha) && std::isfinite(alpha_dot))
                            ? -rod_length_ * std::sin(alpha) * alpha_dot
                            : 0.0;

            const double alpha_ddot =
                std::isfinite(*joint_acceleration_[i]) ? *joint_acceleration_[i] : 0.0;
            module_radial_acceleration_[i] =
                (std::isfinite(alpha) && std::isfinite(alpha_dot))
                    ? -rod_length_
                          * (std::cos(alpha) * alpha_dot * alpha_dot + std::sin(alpha) * alpha_ddot)
                    : 0.0;
        }

        module_radial_rate_ = module_radial_rate_filter_.update(raw_rate);

        deform_active_state_ = std::abs(module_radial_rate_.mean()) > deform_speed_threshold_;
        *deform_active_ = deform_active_state_;
    }

    // ---------------------------------------------------------------- 状态读取

    SteeringStatus calculate_steering_status() const {
        SteeringStatus status;
        status.valid = true;

        for (size_t i = 0; i < kWheelCount; ++i) {
            if (!std::isfinite(*steering_angle_[i]) || !std::isfinite(*steering_velocity_[i])) {
                status.valid = false;
                return status;
            }
            status.angle[i] = *steering_angle_[i] - steering_angle_offset_;
            status.velocity[i] = *steering_velocity_[i];
        }
        status.cos_angle = status.angle.array().cos();
        status.sin_angle = status.angle.array().sin();
        return status;
    }

    Eigen::Vector4d calculate_wheel_velocities() const {
        return {*wheel_velocity_[0], *wheel_velocity_[1], *wheel_velocity_[2], *wheel_velocity_[3]};
    }

    /// 测速正解（英雄舵轮同款“平均分配”闭式解，另加径向补偿）：
    ///
    ///   b_i   = w_i * wheel_radius - r_dot_i * (d_i · e_i)          [m/s]
    ///   v     = 1/4       * sum( b_i * d_i )
    ///   omega = 1/(4 * R) * sum( b_i * sin(theta_i - phi_i) )
    ///
    /// 权重固定（不随舵角变化），分母永远不为零，因此不会出现“四轮平行时奇异”，
    /// 代价是过渡位姿下有界近似。目前只用于能量钳位。
    Eigen::Vector3d calculate_chassis_velocity(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities) const {

        const double mean_radius = std::max(mean_module_radius(), 1e-3);

        double vx = 0.0, vy = 0.0, omega = 0.0;
        for (size_t i = 0; i < kWheelCount; ++i) {
            const Eigen::Vector2d radial = module_radial_direction(i);
            const double cos_angle = steering_status.cos_angle[i];
            const double sin_angle = steering_status.sin_angle[i];

            const double compensated =
                wheel_velocities[i] * wheel_radius_
                - module_radial_rate_[i] * (cos_angle * radial.x() + sin_angle * radial.y());

            vx += compensated * cos_angle;
            vy += compensated * sin_angle;
            omega += (cos_varphi_[i] * sin_angle - sin_varphi_[i] * cos_angle) * compensated;
        }

        Eigen::Vector3d velocity;
        velocity << vx / 4.0, vy / 4.0, omega / (4.0 * mean_radius);
        return velocity;
    }

    /// 期望状态：把计划速度从 odom 转回 base，并给出每轮的期望接触点速度分量。
    ChassisStatus calculate_chassis_status_expected(const Eigen::Vector3d& measured_velocity) {
        // 能量钳位（与英雄车一致）：计划动能明显超过实测动能时，按比例把计划速度压回来。
        // 作用：车顶住墙/被卡住时，防止计划速度一路积分、松开的瞬间窜出去。
        if (energy_clamp_enabled_) {
            const auto energy = [this](const Eigen::Vector3d& velocity) {
                return mass_ * velocity.head<2>().squaredNorm()
                     + moment_of_inertia_ * velocity.z() * velocity.z();
            };
            const double measured_energy = energy(measured_velocity);
            const double planned_energy = energy(chassis_velocity_expected_);
            if (planned_energy > measured_energy && planned_energy > 1e-9) {
                const double scale = std::sqrt(std::max(measured_energy, 0.0) / planned_energy);
                chassis_velocity_expected_ *= scale;
            }
        }

        ChassisStatus status;
        status.velocity = odom_to_base_link_vector(chassis_velocity_expected_);

        const auto& [vx, vy, vz] = status.velocity;
        for (size_t i = 0; i < kWheelCount; ++i) {
            const Eigen::Vector2d position = module_position(i);
            const Eigen::Vector2d radial = module_radial_direction(i);
            status.wheel_velocity_x[i] =
                vx - vz * position.y() + module_radial_rate_[i] * radial.x();
            status.wheel_velocity_y[i] =
                vy + vz * position.x() + module_radial_rate_[i] * radial.y();
        }
        return status;
    }

    // ---------------------------------------------------------------- 规划器

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_velocity_expected,
        const Eigen::Vector3d& chassis_control_velocity) {

        const Eigen::Vector2d translational_acceleration =
            chassis_translational_velocity_pid_.update(
                chassis_control_velocity.head<2>() - chassis_velocity_expected.head<2>());
        const double angular_acceleration = chassis_angular_velocity_pid_.update(
            chassis_control_velocity[2] - chassis_velocity_expected[2]);

        Eigen::Vector3d chassis_acceleration;
        chassis_acceleration << translational_acceleration, angular_acceleration;

        if (chassis_acceleration.lpNorm<1>() < 1e-1)
            chassis_acceleration.setZero();
        return chassis_acceleration;
    }

    Eigen::Vector4d calculate_wheel_pid_torques(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const ChassisStatus& chassis_status_expected) {

        const Eigen::Vector4d wheel_control_velocity =
            (chassis_status_expected.wheel_velocity_x.array() * steering_status.cos_angle.array()
             + chassis_status_expected.wheel_velocity_y.array() * steering_status.sin_angle.array())
            / wheel_radius_;

        return wheel_velocity_pid_.update(wheel_control_velocity - wheel_velocities);
    }

    /// 把加速度指令限制在摩擦/功率允许范围内（与英雄车相同的椭圆约束）。
    Eigen::Vector3d constrain_chassis_control_acceleration(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const Eigen::Vector3d& chassis_acceleration, const Eigen::Vector4d& wheel_pid_torques,
        double available_power) const {

        Eigen::Vector2d translational_acceleration_direction = chassis_acceleration.head<2>();
        const double translational_acceleration_max = translational_acceleration_direction.norm();
        if (translational_acceleration_max > 0.0)
            translational_acceleration_direction /= translational_acceleration_max;

        double angular_acceleration_max = chassis_acceleration.z();
        const double angular_acceleration_direction = angular_acceleration_max > 0 ? 1.0 : -1.0;
        angular_acceleration_max *= angular_acceleration_direction;

        const double mean_radius = std::max(mean_module_radius(), 1e-3);
        const double rhombus_right = friction_coefficient_ * g_;
        const double rhombus_top =
            rhombus_right * mass_ * mean_radius / std::max(moment_of_inertia_, 1e-6);

        auto [a, b, c, d, e, f] = calculate_ellipse_parameters(
            steering_status, wheel_velocities, translational_acceleration_direction,
            angular_acceleration_direction, wheel_pid_torques);

        const Eigen::Vector2d best_point = qcp_solver_.solve(
            {1.0, 0.2},
            {std::max(translational_acceleration_max, 0.0),
             std::max(angular_acceleration_max, 0.0)},
            {rhombus_right, rhombus_top}, {a, b, c, d, e, f - available_power});

        if (!best_point.array().isFinite().all())
            return Eigen::Vector3d::Zero();

        Eigen::Vector3d best_acceleration;
        best_acceleration << best_point.x() * translational_acceleration_direction,
            best_point.y() * angular_acceleration_direction;
        return best_acceleration;
    }

    /// 功率约束的椭圆系数；半径按每个模块的实际半径 r_i 代入（英雄车为常数 vehicle_radius）。
    Formula calculate_ellipse_parameters(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const Eigen::Vector2d& translational_acceleration_direction,
        double angular_acceleration_direction, const Eigen::Vector4d& wheel_torque_base) const {

        Eigen::Vector4d cos_alpha_minus_gamma =
            steering_status.cos_angle.array() * translational_acceleration_direction.x()
            + steering_status.sin_angle.array() * translational_acceleration_direction.y();
        Eigen::Vector4d sin_alpha_minus_varphi =
            cos_varphi_.array() * steering_status.sin_angle.array()
            - sin_varphi_.array() * steering_status.cos_angle.array();
        Eigen::Vector4d double_k1_torque_base_plus_wheel_velocities =
            2 * k1_ * wheel_torque_base.array() + wheel_velocities.array();

        Eigen::Vector4d inverse_radius;
        Eigen::Vector4d inverse_radius_squared;
        for (size_t i = 0; i < kWheelCount; ++i) {
            const double radius = std::max(module_radius(i), 1e-3);
            inverse_radius[i] = 1.0 / radius;
            inverse_radius_squared[i] = inverse_radius[i] * inverse_radius[i];
        }

        const double r2 = wheel_radius_ * wheel_radius_;

        Formula formula;
        auto& [a, b, c, d, e, f] = formula;

        a = (k1_ * mass_ * mass_ * r2 / 16.0) * cos_alpha_minus_gamma.array().pow(2).sum();
        b = (k1_ * mass_ * moment_of_inertia_ * r2 / 8.0) * angular_acceleration_direction
          * (cos_alpha_minus_gamma.array() * sin_alpha_minus_varphi.array()
             * inverse_radius.array())
                .sum();
        c = (k1_ * moment_of_inertia_ * moment_of_inertia_ * r2 / 16.0)
          * (sin_alpha_minus_varphi.array().pow(2) * inverse_radius_squared.array()).sum();
        d = (mass_ * wheel_radius_ / 4.0)
          * (double_k1_torque_base_plus_wheel_velocities.array() * cos_alpha_minus_gamma.array())
                .sum();
        e = (moment_of_inertia_ * wheel_radius_ / 4.0) * angular_acceleration_direction
          * (double_k1_torque_base_plus_wheel_velocities.array() * sin_alpha_minus_varphi.array()
             * inverse_radius.array())
                .sum();
        f = (wheel_torque_base.array()
             * (k1_ * wheel_torque_base.array() + wheel_velocities.array()))
                .sum();

        return formula;
    }

    // ---------------------------------------------------------------- 转向

    /// 转向前馈：接触点速度方向的解析导数。
    ///
    /// V_i  = v + omega x p_i + r_dot_i e_i
    /// Vdot = a + alpha_dot x p_i + omega x (r_dot_i e_i) + r_ddot_i e_i
    /// theta_dot = (V x Vdot)_z / |V|^2
    Eigen::Vector4d calculate_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected,
        const Eigen::Vector3d& chassis_acceleration) {

        const Eigen::Vector3d& velocity = chassis_status_expected.velocity;
        const auto& [vx, vy, vz] = velocity;
        const auto& [ax, ay, az] = chassis_acceleration;

        Eigen::Vector4d feedforward = Eigen::Vector4d::Zero();
        Eigen::Vector4d targets = Eigen::Vector4d::Constant(nan_);
        Eigen::Vector4d errors = Eigen::Vector4d::Zero();
        bool aligned = true;

        for (size_t i = 0; i < kWheelCount; ++i) {
            const Eigen::Vector2d position = module_position(i);
            const Eigen::Vector2d radial = module_radial_direction(i);
            const double radial_rate = module_radial_rate_[i];
            const double radial_acceleration = module_radial_acceleration_[i];

            const Eigen::Vector2d contact_velocity(
                vx - vz * position.y() + radial_rate * radial.x(),
                vy + vz * position.x() + radial_rate * radial.y());
            const Eigen::Vector2d contact_acceleration(
                ax - az * position.y() - vz * radial_rate * radial.y()
                    + radial_acceleration * radial.x(),
                ay + az * position.x() + vz * radial_rate * radial.x()
                    + radial_acceleration * radial.y());

            const double speed_squared = contact_velocity.squaredNorm();

            if (speed_squared > 1e-4) {
                feedforward[i] = (contact_velocity.x() * contact_acceleration.y()
                                  - contact_velocity.y() * contact_acceleration.x())
                               / speed_squared;
                targets[i] = std::atan2(contact_velocity.y(), contact_velocity.x());
            } else if (contact_acceleration.squaredNorm() > 1e-6) {
                feedforward[i] = 0.0;
                targets[i] = std::atan2(contact_acceleration.y(), contact_acceleration.x());
            } else {
                feedforward[i] = 0.0;
                targets[i] = nan_;
            }

            if (std::isfinite(targets[i])) {
                errors[i] = wrap_to_half_pi(targets[i] - steering_status.angle[i]);
                if (std::abs(errors[i]) > steering_aligned_tolerance_)
                    aligned = false;
            } else {
                errors[i] = 0.0;
                aligned = false;
            }
        }

        Eigen::Vector4d torques = steering_velocity_pid_.update(
            feedforward + steering_angle_pid_.update(errors) - steering_status.velocity);

        for (size_t i = 0; i < kWheelCount; ++i) {
            if (!std::isfinite(targets[i]) || !std::isfinite(torques[i]))
                torques[i] = 0.0;
        }

        steering_errors_ = errors;
        for (size_t i = 0; i < kWheelCount; ++i)
            if (!std::isfinite(targets[i]))
                steering_errors_[i] = nan_;

        *steering_aligned_ = aligned;
        return torques;
    }

    // ---------------------------------------------------------------- 轮力矩

    /// 轮力矩前馈：由期望加速度分配到每个轮；未对齐时按 cos(误差) 缩放，
    /// 避免轮子还没转到位就满力推着走（会横向蹭地、也可能触发过流）。
    Eigen::Vector4d calculate_wheel_control_torques(
        const SteeringStatus& steering_status, const Eigen::Vector3d& chassis_acceleration,
        const Eigen::Vector4d& wheel_pid_torques) const {

        const auto& [ax, ay, az] = chassis_acceleration;

        Eigen::Vector4d wheel_torques;
        for (size_t i = 0; i < kWheelCount; ++i) {
            const double radius = std::max(module_radius(i), 1e-3);
            const double sin_alpha_minus_varphi = cos_varphi_[i] * steering_status.sin_angle[i]
                                                - sin_varphi_[i] * steering_status.cos_angle[i];

            const double feedforward =
                wheel_radius_ / 4.0
                * (ax * mass_ * steering_status.cos_angle[i]
                   + ay * mass_ * steering_status.sin_angle[i]
                   + az * moment_of_inertia_ * sin_alpha_minus_varphi / radius);

            const double error = steering_errors_[i];
            const double scale = std::isfinite(error) ? std::clamp(std::cos(error), 0.0, 1.0) : 0.0;

            wheel_torques[i] = (feedforward + wheel_pid_torques[i]) * scale;
            if (!std::isfinite(wheel_torques[i]))
                wheel_torques[i] = 0.0;
        }
        return wheel_torques;
    }

    void update_chassis_velocity_expected(const Eigen::Vector3d& chassis_acceleration, double dt) {
        chassis_velocity_expected_ += dt * base_link_to_odom_vector(chassis_acceleration);
    }

    void update_control_torques(
        const Eigen::Vector4d& steering_torques, const Eigen::Vector4d& wheel_torques) {
        for (size_t i = 0; i < kWheelCount; ++i) {
            *steering_control_torque_[i] = steering_torques[i];
            *wheel_control_torque_[i] = wheel_torques[i];
        }
    }

    // ---------------------------------------------------------------- 工具

    void reset_all_controls() {
        for (size_t i = 0; i < kWheelCount; ++i) {
            *steering_control_torque_[i] = 0.0;
            *wheel_control_torque_[i] = 0.0;
        }
        chassis_translational_velocity_pid_.reset();
        chassis_angular_velocity_pid_.reset();
        wheel_velocity_pid_.reset();
        steering_angle_pid_.reset();
        steering_velocity_pid_.reset();
        control_acceleration_filter_.reset();
        module_radial_rate_filter_.reset();

        chassis_velocity_expected_.setZero();
        module_radial_rate_.setZero();
        module_radial_acceleration_.setZero();
        steering_errors_.setConstant(nan_);
        chassis_yaw_angle_imu_ = 0.0;

        *steering_aligned_ = false;
        deform_active_state_ = false;
        *deform_active_ = false;
    }

    void integral_yaw_angle_imu(double dt) {
        if (std::isfinite(*chassis_yaw_velocity_imu_))
            chassis_yaw_angle_imu_ += *chassis_yaw_velocity_imu_ * dt;
        chassis_yaw_angle_imu_ = std::remainder(chassis_yaw_angle_imu_, 2 * std::numbers::pi);
    }

    Eigen::Vector3d base_link_to_odom_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    Eigen::Vector3d odom_to_base_link_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(-chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    static Eigen::Vector2d clamp_norm(const Eigen::Vector2d& value, double limit) {
        const double norm = value.norm();
        if (norm > limit && norm > 1e-9)
            return value * (limit / norm);
        return value;
    }

    /// 把角度差折到 (-pi/2, pi/2]：舵轮允许“翻转 180° 反向转”，
    /// 取更近的那一侧，轮速参考与力矩会沿实测舵向自动反向（与英雄车一致）。
    static double wrap_to_half_pi(double angle) {
        angle = std::remainder(angle, std::numbers::pi);
        return angle;
    }

    // ---------------------------------------------------------------- 参数

    const double mass_;
    const double moment_of_inertia_;
    const double wheel_radius_;
    const double friction_coefficient_;
    const double k1_, k2_, no_load_power_;
    const double chassis_radius_base_;
    const double rod_length_;
    const double steering_angle_offset_;
    const double control_acceleration_filter_hz_;
    const bool energy_clamp_enabled_;
    const double deform_speed_limit_;
    const double deform_speed_threshold_;
    const double deform_rate_filter_cutoff_hz_;
    const double steering_angle_velocity_limit_;
    const double steering_aligned_tolerance_;
    const double steering_angle_kp_, steering_angle_ki_, steering_angle_kd_;
    const double steering_velocity_kp_, steering_velocity_ki_, steering_velocity_kd_;
    const double steering_torque_limit_;

    // ---------------------------------------------------------------- 接口

    InputInterface<double> update_rate_;
    InputInterface<double> chassis_yaw_velocity_imu_;

    std::array<InputInterface<double>, kWheelCount> wheel_velocity_;
    std::array<InputInterface<double>, kWheelCount> steering_angle_;
    std::array<InputInterface<double>, kWheelCount> steering_velocity_;
    std::array<InputInterface<double>, kWheelCount> joint_angle_;
    std::array<InputInterface<double>, kWheelCount> joint_velocity_;
    std::array<InputInterface<double>, kWheelCount> joint_acceleration_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    std::array<OutputInterface<double>, kWheelCount> steering_control_torque_;
    std::array<OutputInterface<double>, kWheelCount> wheel_control_torque_;
    OutputInterface<bool> steering_aligned_;
    OutputInterface<bool> deform_active_;

    // ---------------------------------------------------------------- 状态

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;
    pid::MatrixPidCalculator<4> wheel_velocity_pid_;
    pid::MatrixPidCalculator<4> steering_angle_pid_;
    pid::MatrixPidCalculator<4> steering_velocity_pid_;

    filter::LowPassFilter<3> control_acceleration_filter_;
    filter::LowPassFilter<4> module_radial_rate_filter_;

    QcpSolver qcp_solver_;

    Eigen::Vector3d chassis_velocity_expected_;
    Eigen::Vector4d module_radial_rate_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d module_radial_acceleration_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d steering_errors_ = Eigen::Vector4d::Constant(nan_);

    double chassis_yaw_angle_imu_ = 0.0;
    double update_rate_hz_ = 1000.0;
    bool deform_active_state_ = false;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableSteeringWheelController, rmcs_executor::Component)
