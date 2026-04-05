#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/chassis/qcp_solver.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::chassis {

class WheelDemoController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit WheelDemoController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mess_(get_parameter("mess").as_double())
        , moment_of_inertia_(get_parameter("moment_of_inertia").as_double())
        , chassis_radius_(get_parameter("chassis_radius").as_double())
        , rod_length_(get_parameter("rod_length").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double())
        , friction_coefficient_(get_parameter("friction_coefficient").as_double())
        , k1_(get_parameter("k1").as_double())
        , k2_(get_parameter("k2").as_double())
        , no_load_power_(get_parameter("no_load_power").as_double())
        , vehicle_radius_(Eigen::Vector4d::Constant(chassis_radius_ + rod_length_))
        , control_acceleration_filter_(5.0, 1000.0)
        , chassis_velocity_expected_(Eigen::Vector3d::Zero())
        , chassis_translational_velocity_pid_(5.0, 0.0, 1.0)
        , chassis_angular_velocity_pid_(5.0, 0.0, 1.0)
        , steering_velocity_pid_(0.15, 0.0, 0.0)
        , steering_angle_pid_(30.0, 0.0, 0.0)
        , wheel_velocity_pid_(0.6, 0.0, 0.0) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);

        register_input("/chassis/left_front_steering/angle", left_front_steering_angle_);
        register_input("/chassis/left_back_steering/angle", left_back_steering_angle_);
        register_input("/chassis/right_back_steering/angle", right_back_steering_angle_);
        register_input("/chassis/right_front_steering/angle", right_front_steering_angle_);

        register_input("/chassis/left_front_steering/velocity", left_front_steering_velocity_);
        register_input("/chassis/left_back_steering/velocity", left_back_steering_velocity_);
        register_input("/chassis/right_back_steering/velocity", right_back_steering_velocity_);
        register_input("/chassis/right_front_steering/velocity", right_front_steering_velocity_);

        register_input("/chassis/left_front_wheel/velocity", left_front_wheel_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_wheel_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_wheel_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_wheel_velocity_);

        register_input("/chassis/left_front_joint/physical_angle", left_front_joint_angle_);
        register_input("/chassis/left_back_joint/physical_angle", left_back_joint_angle_);
        register_input("/chassis/right_back_joint/physical_angle", right_back_joint_angle_);
        register_input("/chassis/right_front_joint/physical_angle", right_front_joint_angle_);

        register_input("/chassis/left_front_joint/physical_velocity", left_front_joint_velocity_);
        register_input("/chassis/left_back_joint/physical_velocity", left_back_joint_velocity_);
        register_input("/chassis/right_back_joint/physical_velocity", right_back_joint_velocity_);
        register_input("/chassis/right_front_joint/physical_velocity", right_front_joint_velocity_);

        register_input("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_);
        register_input("/chassis/control_velocity", chassis_control_velocity_);
        register_input("/chassis/control_power_limit", power_limit_);

        register_output(
            "/chassis/left_front_steering/control_torque", left_front_steering_control_torque_);
        register_output(
            "/chassis/left_back_steering/control_torque", left_back_steering_control_torque_);
        register_output(
            "/chassis/right_back_steering/control_torque", right_back_steering_control_torque_);
        register_output(
            "/chassis/right_front_steering/control_torque", right_front_steering_control_torque_);

        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_wheel_control_torque_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_wheel_control_torque_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_wheel_control_torque_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_wheel_control_torque_);

        register_output("/chassis/encoder/alpha", encoder_alpha_);
        register_output("/chassis/encoder/alpha_dot", encoder_alpha_dot_);
        register_output("/chassis/radius", radius_);
    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        JointStates joint = update_joint_states_();
        if (joint.valid) {
            vehicle_radius_ = joint.radius;
            *radius_ = vehicle_radius_.mean();
            *encoder_alpha_ = joint.alpha_rad.mean();
            *encoder_alpha_dot_ = joint.alpha_dot_rad.mean();
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "physical joint angle[deg] lf=%.2f lb=%.2f rb=%.2f rf=%.2f, radius[m] lf=%.3f "
                "lb=%.3f rb=%.3f rf=%.3f",
                joint.alpha_rad[0] * 180.0 / std::numbers::pi,
                joint.alpha_rad[1] * 180.0 / std::numbers::pi,
                joint.alpha_rad[2] * 180.0 / std::numbers::pi,
                joint.alpha_rad[3] * 180.0 / std::numbers::pi, vehicle_radius_[0],
                vehicle_radius_[1], vehicle_radius_[2], vehicle_radius_[3]);
        } else {
            *radius_ = nan_;
            *encoder_alpha_ = nan_;
            *encoder_alpha_dot_ = nan_;
        }

        integral_yaw_angle_imu();

        const auto steering_status = calculate_steering_status();
        const auto wheel_velocities = calculate_wheel_velocities();
        const auto chassis_velocity =
            calculate_chassis_velocity(steering_status, wheel_velocities, joint);
        auto chassis_status_expected = calculate_chassis_status_expected(chassis_velocity, joint);
        const auto chassis_control_velocity = calculate_chassis_control_velocity();

        const auto chassis_acceleration = calculate_chassis_control_acceleration(
            chassis_status_expected.velocity, chassis_control_velocity);

        const double power_limit =
            *power_limit_ - no_load_power_ - k2_ * wheel_velocities.array().pow(2).sum();

        const auto wheel_pid_torques =
            calculate_wheel_pid_torques(steering_status, wheel_velocities, chassis_status_expected);

        const auto constrained_chassis_acceleration = constrain_chassis_control_acceleration(
            steering_status, wheel_velocities, chassis_acceleration, wheel_pid_torques,
            power_limit);

        const auto filtered_chassis_acceleration =
            odom_to_base_link_vector(control_acceleration_filter_.update(
                base_link_to_odom_vector(constrained_chassis_acceleration)));

        const auto steering_torques = calculate_steering_control_torques(
            steering_status, chassis_status_expected, joint, filtered_chassis_acceleration);
        const auto wheel_torques = calculate_wheel_control_torques(
            steering_status, joint, filtered_chassis_acceleration, wheel_pid_torques);

        update_control_torques(steering_torques, wheel_torques);
        update_chassis_velocity_expected(filtered_chassis_acceleration);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle = Eigen::Vector4d::Zero();
        Eigen::Vector4d cos_angle = Eigen::Vector4d::Zero();
        Eigen::Vector4d sin_angle = Eigen::Vector4d::Zero();
        Eigen::Vector4d velocity = Eigen::Vector4d::Zero();
    };

    struct ChassisStatus {
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector4d wheel_velocity_x = Eigen::Vector4d::Zero();
        Eigen::Vector4d wheel_velocity_y = Eigen::Vector4d::Zero();
    };

    struct JointStates {
        Eigen::Vector4d alpha_rad = Eigen::Vector4d::Zero();
        Eigen::Vector4d alpha_dot_rad = Eigen::Vector4d::Zero();
        Eigen::Vector4d alpha_ddot_rad = Eigen::Vector4d::Zero();
        Eigen::Vector4d radius = Eigen::Vector4d::Zero();
        Eigen::Vector4d radius_dot = Eigen::Vector4d::Zero();
        Eigen::Vector4d radius_ddot = Eigen::Vector4d::Zero();
        bool valid = false;
    };

    JointStates update_joint_states_() {
        JointStates joint;
        joint.alpha_rad = {
            *left_front_joint_angle_, *left_back_joint_angle_, *right_back_joint_angle_,
            *right_front_joint_angle_};
        joint.alpha_dot_rad = {
            *left_front_joint_velocity_, *left_back_joint_velocity_, *right_back_joint_velocity_,
            *right_front_joint_velocity_};

        if (!joint.alpha_rad.array().isFinite().all()
            || !joint.alpha_dot_rad.array().isFinite().all())
            return joint;

        if (last_joint_velocity_valid_) {
            joint.alpha_ddot_rad = (joint.alpha_dot_rad - last_joint_velocity_) / dt_;
        }

        last_joint_velocity_ = joint.alpha_dot_rad;
        last_joint_velocity_valid_ = true;

        joint.radius = chassis_radius_ + rod_length_ * joint.alpha_rad.array().cos();
        joint.radius_dot =
            -rod_length_ * joint.alpha_rad.array().sin() * joint.alpha_dot_rad.array();
        joint.radius_ddot =
            -rod_length_ * joint.alpha_rad.array().cos() * joint.alpha_dot_rad.array().square()
            - rod_length_ * joint.alpha_rad.array().sin() * joint.alpha_ddot_rad.array();

        joint.valid = joint.radius.array().isFinite().all()
                   && joint.radius_dot.array().isFinite().all()
                   && joint.radius_ddot.array().isFinite().all();
        return joint;
    }

    void reset_all_controls() {
        control_acceleration_filter_.reset();

        chassis_yaw_angle_imu_ = 0.0;
        chassis_velocity_expected_ = Eigen::Vector3d::Zero();
        vehicle_radius_ = Eigen::Vector4d::Constant(chassis_radius_ + rod_length_);
        last_joint_velocity_ = Eigen::Vector4d::Zero();
        last_joint_velocity_valid_ = false;

        *encoder_alpha_ = nan_;
        *encoder_alpha_dot_ = nan_;
        *radius_ = nan_;

        *left_front_steering_control_torque_ = 0.0;
        *left_back_steering_control_torque_ = 0.0;
        *right_back_steering_control_torque_ = 0.0;
        *right_front_steering_control_torque_ = 0.0;

        *left_front_wheel_control_torque_ = 0.0;
        *left_back_wheel_control_torque_ = 0.0;
        *right_back_wheel_control_torque_ = 0.0;
        *right_front_wheel_control_torque_ = 0.0;
    }

    void integral_yaw_angle_imu() {
        chassis_yaw_angle_imu_ += *chassis_yaw_velocity_imu_ * dt_;
        chassis_yaw_angle_imu_ = std::fmod(chassis_yaw_angle_imu_, 2 * std::numbers::pi);
    }

    SteeringStatus calculate_steering_status() const {
        SteeringStatus steering_status;
        steering_status.angle = {
            *left_front_steering_angle_, *left_back_steering_angle_, *right_back_steering_angle_,
            *right_front_steering_angle_};
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();
        steering_status.velocity = {
            *left_front_steering_velocity_, *left_back_steering_velocity_,
            *right_back_steering_velocity_, *right_front_steering_velocity_};
        return steering_status;
    }

    Eigen::Vector4d calculate_wheel_velocities() const {
        return {
            *left_front_wheel_velocity_, *left_back_wheel_velocity_, *right_back_wheel_velocity_,
            *right_front_wheel_velocity_};
    }

    Eigen::Vector3d calculate_chassis_velocity(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const JointStates& joint) const {
        if (!joint.valid)
            return Eigen::Vector3d::Zero();

        Eigen::Matrix<double, 4, 3> a;
        Eigen::Vector4d b;
        for (int i = 0; i < 4; ++i) {
            a(i, 0) = steering_status.cos_angle[i];
            a(i, 1) = steering_status.sin_angle[i];
            a(i, 2) = joint.radius[i] * std::sin(steering_status.angle[i] - phi_[i]);
            b[i] = wheel_radius_ * wheel_velocities[i]
                 - joint.radius_dot[i] * std::cos(steering_status.angle[i] - phi_[i]);
        }
        return a.colPivHouseholderQr().solve(b);
    }

    ChassisStatus calculate_chassis_status_expected(
        const Eigen::Vector3d& chassis_velocity, const JointStates& joint) {
        auto calculate_energy = [this](const Eigen::Vector3d& velocity) {
            return mess_ * velocity.head<2>().squaredNorm()
                 + moment_of_inertia_ * velocity.z() * velocity.z();
        };

        const double chassis_energy = calculate_energy(chassis_velocity);
        const double chassis_energy_expected = calculate_energy(chassis_velocity_expected_);

        if (std::isfinite(chassis_energy) && std::isfinite(chassis_energy_expected)
            && chassis_energy_expected > chassis_energy && chassis_energy_expected > 1e-12) {
            const double k = std::sqrt(chassis_energy / chassis_energy_expected);
            if (std::isfinite(k) && k >= 0.0)
                chassis_velocity_expected_ *= k;
        }

        ChassisStatus chassis_status_expected;
        chassis_status_expected.velocity = odom_to_base_link_vector(chassis_velocity_expected_);

        const double vx = chassis_status_expected.velocity.x();
        const double vy = chassis_status_expected.velocity.y();
        const double vz = chassis_status_expected.velocity.z();
        for (int i = 0; i < 4; ++i) {
            const Eigen::Vector2d wheel_velocity = Eigen::Vector2d(vx, vy)
                                                 + vz * joint.radius[i] * tangential_unit_(phi_[i])
                                                 + joint.radius_dot[i] * radial_unit_(phi_[i]);
            chassis_status_expected.wheel_velocity_x[i] = wheel_velocity.x();
            chassis_status_expected.wheel_velocity_y[i] = wheel_velocity.y();
        }

        return chassis_status_expected;
    }

    Eigen::Vector3d calculate_chassis_control_velocity() const {
        return chassis_control_velocity_->vector;
    }

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_velocity_expected,
        const Eigen::Vector3d& chassis_control_velocity) {
        Eigen::Vector2d translational_control_acceleration =
            chassis_translational_velocity_pid_.update(
                chassis_control_velocity.head<2>() - chassis_velocity_expected.head<2>());

        const double angular_control_acceleration = chassis_angular_velocity_pid_.update(
            chassis_control_velocity[2] - chassis_velocity_expected[2]);

        Eigen::Vector3d chassis_control_acceleration;
        chassis_control_acceleration << translational_control_acceleration,
            angular_control_acceleration;
        if (chassis_control_acceleration.lpNorm<1>() < 1e-1)
            chassis_control_acceleration.setZero();
        return chassis_control_acceleration;
    }

    Eigen::Vector4d calculate_wheel_pid_torques(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const ChassisStatus& chassis_status_expected) {
        const Eigen::Vector4d wheel_control_velocity =
            chassis_status_expected.wheel_velocity_x.array() * steering_status.cos_angle.array()
            + chassis_status_expected.wheel_velocity_y.array() * steering_status.sin_angle.array();
        return wheel_velocity_pid_.update(
            wheel_control_velocity / wheel_radius_ - wheel_velocities);
    }

    Eigen::Vector3d constrain_chassis_control_acceleration(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const Eigen::Vector3d& chassis_acceleration, const Eigen::Vector4d& wheel_pid_torques,
        const double& power_limit) {
        Eigen::Vector2d translational_acceleration_direction = chassis_acceleration.head<2>();
        double translational_acceleration_max = translational_acceleration_direction.norm();
        if (translational_acceleration_max > 0.0)
            translational_acceleration_direction /= translational_acceleration_max;

        double angular_acceleration_max = chassis_acceleration.z();
        double angular_acceleration_direction = angular_acceleration_max > 0 ? 1.0 : -1.0;
        angular_acceleration_max *= angular_acceleration_direction;

        const double rhombus_right = friction_coefficient_ * g_;
        const double rhombus_top =
            rhombus_right * mess_ * vehicle_radius_.mean() / moment_of_inertia_;

        const auto formula = calculate_ellipse_parameters(
            steering_status, wheel_velocities, translational_acceleration_direction,
            angular_acceleration_direction, wheel_pid_torques);
        const double a = formula[0];
        const double b = formula[1];
        const double c = formula[2];
        const double d = formula[3];
        const double e = formula[4];
        const double f = formula[5];

        const QcpSolver::QuadraticConstraint quadratic_constraint{a, b, c, d, e, f - power_limit};

        const Eigen::Vector2d best_point = qcp_solver_.solve(
            {1.0, 0.2}, {translational_acceleration_max, angular_acceleration_max},
            {rhombus_right, rhombus_top}, quadratic_constraint);

        Eigen::Vector3d best_acceleration;
        best_acceleration << best_point.x() * translational_acceleration_direction,
            best_point.y() * angular_acceleration_direction;
        return best_acceleration;
    }

    Eigen::Vector<double, 6> calculate_ellipse_parameters(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const Eigen::Vector2d& translational_acceleration_direction,
        const double& angular_acceleration_direction,
        const Eigen::Vector4d& wheel_torque_base) const {
        Eigen::Vector<double, 6> formula;
        formula.setZero();

        for (int i = 0; i < 4; ++i) {
            const double cos_alpha_minus_gamma =
                steering_status.cos_angle[i] * translational_acceleration_direction.x()
                + steering_status.sin_angle[i] * translational_acceleration_direction.y();
            const double sin_alpha_minus_varphi = std::sin(steering_status.angle[i] - phi_[i]);
            const double double_k1_torque_base_plus_wheel_velocity =
                2 * k1_ * wheel_torque_base[i] + wheel_velocities[i];

            formula[0] += (k1_ * mess_ * mess_ * wheel_radius_ * wheel_radius_ / 16.0)
                        * cos_alpha_minus_gamma * cos_alpha_minus_gamma;
            formula[1] += (k1_ * mess_ * moment_of_inertia_ * wheel_radius_ * wheel_radius_ / 8.0)
                        * angular_acceleration_direction * cos_alpha_minus_gamma
                        * sin_alpha_minus_varphi / vehicle_radius_[i];
            formula[2] += (k1_ * moment_of_inertia_ * moment_of_inertia_ * wheel_radius_
                           * wheel_radius_ / 16.0)
                        * sin_alpha_minus_varphi * sin_alpha_minus_varphi
                        / (vehicle_radius_[i] * vehicle_radius_[i]);
            formula[3] += (mess_ * wheel_radius_ / 4.0) * double_k1_torque_base_plus_wheel_velocity
                        * cos_alpha_minus_gamma;
            formula[4] += (moment_of_inertia_ * wheel_radius_ / 4.0)
                        * angular_acceleration_direction * double_k1_torque_base_plus_wheel_velocity
                        * sin_alpha_minus_varphi / vehicle_radius_[i];
            formula[5] += wheel_torque_base[i] * (k1_ * wheel_torque_base[i] + wheel_velocities[i]);
        }

        return formula;
    }

    Eigen::Vector4d calculate_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected,
        const JointStates& joint, const Eigen::Vector3d& chassis_acceleration) {
        const double vz = chassis_status_expected.velocity.z();
        const double ax = chassis_acceleration.x();
        const double ay = chassis_acceleration.y();
        const double az = chassis_acceleration.z();

        Eigen::Vector4d steering_control_velocity;
        Eigen::Vector4d steering_control_angle;

        for (int i = 0; i < 4; ++i) {
            const double ux = chassis_status_expected.wheel_velocity_x[i];
            const double uy = chassis_status_expected.wheel_velocity_y[i];
            const double dot_r_squared = ux * ux + uy * uy;

            const Eigen::Vector2d dot_u = Eigen::Vector2d(ax, ay)
                                        + az * joint.radius[i] * tangential_unit_(phi_[i])
                                        + vz * joint.radius_dot[i] * tangential_unit_(phi_[i])
                                        + joint.radius_ddot[i] * radial_unit_(phi_[i]);

            if (dot_r_squared > 1e-2) {
                steering_control_velocity[i] = (ux * dot_u.y() - uy * dot_u.x()) / dot_r_squared;
                steering_control_angle[i] = std::atan2(uy, ux);
            } else if (dot_u.squaredNorm() > 1e-6) {
                steering_control_velocity[i] = 0.0;
                steering_control_angle[i] = std::atan2(dot_u.y(), dot_u.x());
            } else {
                steering_control_velocity[i] = nan_;
                steering_control_angle[i] = nan_;
            }
        }

        Eigen::Vector4d steering_torque = steering_velocity_pid_.update(
            steering_control_velocity
            + steering_angle_pid_.update(
                (steering_control_angle - steering_status.angle).unaryExpr([](double diff) {
                    diff = std::fmod(diff, std::numbers::pi);
                    if (diff < -std::numbers::pi / 2)
                        diff += std::numbers::pi;
                    else if (diff > std::numbers::pi / 2)
                        diff -= std::numbers::pi;
                    return diff;
                }))
            - steering_status.velocity);

        return steering_torque.unaryExpr([](double v) { return std::isnan(v) ? 0.0 : v; });
    }

    Eigen::Vector4d calculate_wheel_control_torques(
        const SteeringStatus& steering_status, const JointStates& joint,
        const Eigen::Vector3d& chassis_acceleration,
        const Eigen::Vector4d& wheel_pid_torques) const {
        Eigen::Matrix<double, 3, 4> map;
        for (int i = 0; i < 4; ++i) {
            map(0, i) = steering_status.cos_angle[i];
            map(1, i) = steering_status.sin_angle[i];
            map(2, i) = joint.radius[i] * std::sin(steering_status.angle[i] - phi_[i]);
        }

        const Eigen::Vector3d desired_wrench{
            mess_ * chassis_acceleration.x(),
            mess_ * chassis_acceleration.y(),
            moment_of_inertia_ * chassis_acceleration.z(),
        };

        Eigen::Vector4d wheel_torque = Eigen::Vector4d::Zero();
        const Eigen::Matrix3d gram = map * map.transpose();
        if (std::abs(gram.determinant()) > 1e-8) {
            wheel_torque = wheel_radius_ * (map.transpose() * gram.inverse() * desired_wrench);
        }
        wheel_torque += wheel_pid_torques;
        return wheel_torque;
    }

    void update_control_torques(
        const Eigen::Vector4d& steering_torque, const Eigen::Vector4d& wheel_torque) {
        *left_front_steering_control_torque_ = steering_torque[0];
        *left_back_steering_control_torque_ = steering_torque[1];
        *right_back_steering_control_torque_ = steering_torque[2];
        *right_front_steering_control_torque_ = steering_torque[3];

        *left_front_wheel_control_torque_ = wheel_torque[0];
        *left_back_wheel_control_torque_ = wheel_torque[1];
        *right_back_wheel_control_torque_ = wheel_torque[2];
        *right_front_wheel_control_torque_ = wheel_torque[3];
    }

    void update_chassis_velocity_expected(const Eigen::Vector3d& chassis_acceleration) {
        chassis_velocity_expected_ += dt_ * base_link_to_odom_vector(chassis_acceleration);
    }

    Eigen::Vector3d base_link_to_odom_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    Eigen::Vector3d odom_to_base_link_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(-chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    static Eigen::Vector2d radial_unit_(double phi) { return {std::cos(phi), std::sin(phi)}; }

    static Eigen::Vector2d tangential_unit_(double phi) { return {-std::sin(phi), std::cos(phi)}; }

    static double wrap_to_half_pi_(double diff) {
        diff = std::fmod(diff, std::numbers::pi);
        if (diff < -std::numbers::pi / 2)
            diff += std::numbers::pi;
        else if (diff > std::numbers::pi / 2)
            diff -= std::numbers::pi;
        return diff;
    }

    static constexpr std::array<double, 4> phi_ = {
        std::numbers::pi / 4,
        std::numbers::pi * 3 / 4,
        -std::numbers::pi * 3 / 4,
        -std::numbers::pi / 4,
    };

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.81;

    const double mess_;
    const double moment_of_inertia_;
    const double chassis_radius_;
    const double rod_length_;
    const double wheel_radius_;
    const double friction_coefficient_;
    const double k1_;
    const double k2_;
    const double no_load_power_;

    Eigen::Vector4d vehicle_radius_;
    Eigen::Vector4d last_joint_velocity_ = Eigen::Vector4d::Zero();
    bool last_joint_velocity_valid_ = false;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;

    InputInterface<double> left_front_steering_angle_;
    InputInterface<double> left_back_steering_angle_;
    InputInterface<double> right_back_steering_angle_;
    InputInterface<double> right_front_steering_angle_;

    InputInterface<double> left_front_steering_velocity_;
    InputInterface<double> left_back_steering_velocity_;
    InputInterface<double> right_back_steering_velocity_;
    InputInterface<double> right_front_steering_velocity_;

    InputInterface<double> left_front_wheel_velocity_;
    InputInterface<double> left_back_wheel_velocity_;
    InputInterface<double> right_back_wheel_velocity_;
    InputInterface<double> right_front_wheel_velocity_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;

    InputInterface<double> left_front_joint_velocity_;
    InputInterface<double> left_back_joint_velocity_;
    InputInterface<double> right_back_joint_velocity_;
    InputInterface<double> right_front_joint_velocity_;

    InputInterface<double> chassis_yaw_velocity_imu_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    OutputInterface<double> encoder_alpha_;
    OutputInterface<double> encoder_alpha_dot_;
    OutputInterface<double> radius_;

    QcpSolver qcp_solver_;
    filter::LowPassFilter<3> control_acceleration_filter_;

    double chassis_yaw_angle_imu_ = 0.0;
    Eigen::Vector3d chassis_velocity_expected_ = Eigen::Vector3d::Zero();

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;
    pid::MatrixPidCalculator<4> steering_velocity_pid_;
    pid::MatrixPidCalculator<4> steering_angle_pid_;
    pid::MatrixPidCalculator<4> wheel_velocity_pid_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelDemoController, rmcs_executor::Component)
