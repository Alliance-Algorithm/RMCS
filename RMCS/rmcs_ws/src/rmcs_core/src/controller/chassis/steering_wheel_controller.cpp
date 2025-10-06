#include <cmath>

#include <eigen3/Eigen/Dense>
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

class SteeringWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit SteeringWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mess_(get_parameter("mess").as_double())
        , moment_of_inertia_(get_parameter("moment_of_inertia").as_double())
        , vehicle_radius_(get_parameter("vehicle_radius").as_double())
        , wheel_radius_(get_parameter("wheel_radius").as_double())
        , friction_coefficient_(get_parameter("friction_coefficient").as_double())
        , k1_(get_parameter("k1").as_double())
        , k2_(get_parameter("k2").as_double())
        , no_load_power_(get_parameter("no_load_power").as_double())
        , control_acceleration_filter_(5.0, 1000.0)
        , chassis_velocity_expected_(Eigen::Vector3d::Zero())
        , chassis_translational_velocity_pid_(5.0, 0.0, 1.0)
        , chassis_angular_velocity_pid_(5.0, 0.0, 1.0)
        , cos_varphi_(1, 0, -1, 0) // 0, pi/2, pi, 3pi/2
        , sin_varphi_(0, 1, 0, -1)
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
    }

    void update() override {
        if (std::isnan(chassis_control_velocity_->vector[0])) {
            reset_all_controls();
            return;
        }

        integral_yaw_angle_imu();

        auto steering_status = calculate_steering_status();
        auto wheel_velocities = calculate_wheel_velocities();
        auto chassis_velocity = calculate_chassis_velocity(steering_status, wheel_velocities);

        auto chassis_status_expected = calculate_chassis_status_expected(chassis_velocity);
        auto chassis_control_velocity = calculate_chassis_control_velocity();

        auto chassis_acceleration = calculate_chassis_control_acceleration(
            chassis_status_expected.velocity, chassis_control_velocity);

        double power_limit =
            *power_limit_ - no_load_power_ - k2_ * wheel_velocities.array().pow(2).sum();

        auto wheel_pid_torques =
            calculate_wheel_pid_torques(steering_status, wheel_velocities, chassis_status_expected);

        auto constrained_chassis_acceleration = constrain_chassis_control_acceleration(
            steering_status, wheel_velocities, chassis_acceleration, wheel_pid_torques,
            power_limit);
        auto filtered_chassis_acceleration =
            odom_to_base_link_vector(control_acceleration_filter_.update(
                base_link_to_odom_vector(constrained_chassis_acceleration)));

        auto steering_torques = calculate_steering_control_torques(
            steering_status, chassis_status_expected, filtered_chassis_acceleration);
        auto wheel_torques = calculate_wheel_control_torques(
            steering_status, filtered_chassis_acceleration, wheel_pid_torques);

        update_control_torques(steering_torques, wheel_torques);
        update_chassis_velocity_expected(filtered_chassis_acceleration);
    }

private:
    struct SteeringStatus {
        Eigen::Vector4d angle, cos_angle, sin_angle;
        Eigen::Vector4d velocity;
    };

    struct ChassisStatus {
        Eigen::Vector3d velocity;
        Eigen::Vector4d wheel_velocity_x, wheel_velocity_y;
    };

    void reset_all_controls() {
        control_acceleration_filter_.reset();

        chassis_yaw_angle_imu_ = 0.0;
        chassis_velocity_expected_ = Eigen::Vector3d::Zero();

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

    SteeringStatus calculate_steering_status() {
        SteeringStatus steering_status;

        steering_status.angle = {
            *left_front_steering_angle_,    //
            *left_back_steering_angle_,     //
            *right_back_steering_angle_,    //
            *right_front_steering_angle_    //
        };
        steering_status.angle.array() -= std::numbers::pi / 4;
        steering_status.cos_angle = steering_status.angle.array().cos();
        steering_status.sin_angle = steering_status.angle.array().sin();

        steering_status.velocity = {
            *left_front_steering_velocity_, //
            *left_back_steering_velocity_,  //
            *right_back_steering_velocity_, //
            *right_front_steering_velocity_ //
        };

        return steering_status;
    }

    Eigen::Vector4d calculate_wheel_velocities() {
        return {
            *left_front_wheel_velocity_,    //
            *left_back_wheel_velocity_,     //
            *right_back_wheel_velocity_,    //
            *right_front_wheel_velocity_    //
        };
    }

    Eigen::Vector3d calculate_chassis_velocity(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities) const {
        Eigen::Vector3d velocity;
        double one_quarter_r = wheel_radius_ / 4.0;
        velocity.x() = one_quarter_r * wheel_velocities.dot(steering_status.cos_angle);
        velocity.y() = one_quarter_r * wheel_velocities.dot(steering_status.sin_angle);
        velocity.z() = -one_quarter_r / vehicle_radius_
                     * (-wheel_velocities[0] * steering_status.sin_angle[0]
                        + wheel_velocities[1] * steering_status.cos_angle[1]
                        + wheel_velocities[2] * steering_status.sin_angle[2]
                        - wheel_velocities[3] * steering_status.cos_angle[3]);
        return velocity;
    }

    ChassisStatus calculate_chassis_status_expected(const Eigen::Vector3d& chassis_velocity) {
        auto calculate_energy = [this](const Eigen::Vector3d& velocity) {
            return mess_ * velocity.head<2>().squaredNorm()
                 + moment_of_inertia_ * velocity.z() * velocity.z();
        };
        auto chassis_energy = calculate_energy(chassis_velocity);
        auto chassis_energy_expected = calculate_energy(chassis_velocity_expected_);
        if (chassis_energy_expected > chassis_energy) {
            double k = std::sqrt(chassis_energy / chassis_energy_expected);
            chassis_velocity_expected_ *= k;
        }

        ChassisStatus chassis_status_expected;
        chassis_status_expected.velocity = odom_to_base_link_vector(chassis_velocity_expected_);

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;
        chassis_status_expected.wheel_velocity_x = vx - vehicle_radius_ * vz * sin_varphi_.array();
        chassis_status_expected.wheel_velocity_y = vy + vehicle_radius_ * vz * cos_varphi_.array();

        return chassis_status_expected;
    }

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity = chassis_control_velocity_->vector;
        chassis_control_velocity.head<2>() =
            Eigen::Rotation2Dd(-std::numbers::pi / 4) * chassis_control_velocity.head<2>();

        return chassis_control_velocity;
    }

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_velocity_expected,
        const Eigen::Vector3d& chassis_control_velocity) {

        Eigen::Vector2d translational_control_velocity = chassis_control_velocity.head<2>();
        Eigen::Vector2d translational_velocity = chassis_velocity_expected.head<2>();
        Eigen::Vector2d translational_control_acceleration =
            chassis_translational_velocity_pid_.update(
                translational_control_velocity - translational_velocity);

        const double& angular_control_velocity = chassis_control_velocity[2];
        const double& angular_velocity = chassis_velocity_expected[2];
        double angular_control_acceleration =
            chassis_angular_velocity_pid_.update(angular_control_velocity - angular_velocity);

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
        Eigen::Vector4d wheel_control_velocity =
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
        const double rhombus_top = rhombus_right * mess_ * vehicle_radius_ / moment_of_inertia_;

        auto [a, b, c, d, e, f] = calculate_ellipse_parameters(
            steering_status, wheel_velocities, translational_acceleration_direction,
            angular_acceleration_direction, wheel_pid_torques);

        Eigen::Vector2d best_point = qcp_solver_.solve(
            {1.0, 0.2}, {translational_acceleration_max, angular_acceleration_max},
            {rhombus_right, rhombus_top}, {a, b, c, d, e, f - power_limit});

        Eigen::Vector3d best_acceleration;
        best_acceleration << best_point.x() * translational_acceleration_direction,
            best_point.y() * angular_acceleration_direction;
        return best_acceleration;
    }

    Eigen::Vector<double, 6> calculate_ellipse_parameters(
        const SteeringStatus& steering_status, const Eigen::Vector4d& wheel_velocities,
        const Eigen::Vector2d& translational_acceleration_direction,
        const double& angular_acceleration_direction, const Eigen::Vector4d& wheel_torque_base) {
        Eigen::Vector4d cos_alpha_minus_gamma =
            steering_status.cos_angle.array() * translational_acceleration_direction.x()
            + steering_status.sin_angle.array() * translational_acceleration_direction.y();
        Eigen::Vector4d sin_alpha_minus_varphi =
            cos_varphi_.array() * steering_status.sin_angle.array()
            - sin_varphi_.array() * steering_status.cos_angle.array();
        Eigen::Vector4d double_k1_torque_base_plus_wheel_velocities =
            2 * k1_ * wheel_torque_base.array() + wheel_velocities.array();

        Eigen::Vector<double, 6> formula;
        auto& [a, b, c, d, e, f] = formula;

        a = (k1_ * mess_ * mess_ * wheel_radius_ * wheel_radius_ / 16.0)
          * cos_alpha_minus_gamma.array().pow(2).sum();
        b = ((k1_ * mess_ * moment_of_inertia_ * wheel_radius_ * wheel_radius_)
             / (8.0 * vehicle_radius_))
          * angular_acceleration_direction
          * (cos_alpha_minus_gamma.array() * sin_alpha_minus_varphi.array()).sum();
        c = ((k1_ * moment_of_inertia_ * moment_of_inertia_ * wheel_radius_ * wheel_radius_)
             / (16.0 * vehicle_radius_ * vehicle_radius_))
          * sin_alpha_minus_varphi.array().pow(2).sum();
        d = (mess_ * wheel_radius_ / 4.0)
          * (double_k1_torque_base_plus_wheel_velocities.array() * cos_alpha_minus_gamma.array())
                .sum();
        e = ((moment_of_inertia_ * wheel_radius_) / (4.0 * vehicle_radius_))
          * angular_acceleration_direction
          * (double_k1_torque_base_plus_wheel_velocities.array() * sin_alpha_minus_varphi.array())
                .sum();
        f = (wheel_torque_base.array()
             * (k1_ * wheel_torque_base.array() + wheel_velocities.array()))
                .sum();

        return formula;
    }

    Eigen::Vector4d calculate_steering_control_torques(
        const SteeringStatus& steering_status, const ChassisStatus& chassis_status_expected,
        const Eigen::Vector3d& chassis_acceleration) {

        const auto& [vx, vy, vz] = chassis_status_expected.velocity;
        const auto& [ax, ay, az] = chassis_acceleration;

        Eigen::Vector4d dot_r_squared = chassis_status_expected.wheel_velocity_x.array().square()
                                      + chassis_status_expected.wheel_velocity_y.array().square();

        Eigen::Vector4d steering_control_velocities =
            vx * ay - vy * ax - vz * (vx * vx + vy * vy)
            + vehicle_radius_ * (az * vx - vz * (ax + vz * vy)) * cos_varphi_.array()
            + vehicle_radius_ * (az * vy - vz * (ay - vz * vx)) * sin_varphi_.array();
        Eigen::Vector4d steering_control_angles;

        for (int i = 0; i < steering_control_velocities.size(); ++i) {
            if (dot_r_squared[i] > 1e-2) {
                steering_control_velocities[i] /= dot_r_squared[i];
                steering_control_angles[i] = std::atan2(
                    chassis_status_expected.wheel_velocity_y[i],
                    chassis_status_expected.wheel_velocity_x[i]);
            } else {
                auto x = ax - vehicle_radius_ * (az * sin_varphi_[i] + 0 * cos_varphi_[i]);
                auto y = ay + vehicle_radius_ * (az * cos_varphi_[i] - 0 * sin_varphi_[i]);
                if (x * x + y * y > 1e-6) {
                    steering_control_velocities[i] = 0.0;
                    steering_control_angles[i] = std::atan2(y, x);
                } else {
                    steering_control_velocities[i] = nan_;
                    steering_control_angles[i] = nan_;
                }
            }
        }

        Eigen::Vector4d steering_torques = steering_velocity_pid_.update(
            steering_control_velocities
            + steering_angle_pid_.update(
                (steering_control_angles - steering_status.angle).unaryExpr([](double diff) {
                    diff = std::fmod(diff, std::numbers::pi);
                    if (diff < -std::numbers::pi / 2) {
                        diff += std::numbers::pi;
                    } else if (diff > std::numbers::pi / 2) {
                        diff -= std::numbers::pi;
                    }
                    return diff;
                }))
            - steering_status.velocity);
        return steering_torques.unaryExpr([](double v) { return std::isnan(v) ? 0.0 : v; });
    }

    Eigen::Vector4d calculate_wheel_control_torques(
        const SteeringStatus& steering_status, const Eigen::Vector3d& chassis_acceleration,
        const Eigen::Vector4d& wheel_pid_torques) {

        const auto& [ax, ay, az] = chassis_acceleration;
        Eigen::Vector4d wheel_torques =
            wheel_radius_
            * (ax * mess_ * steering_status.cos_angle.array()
               + ay * mess_ * steering_status.sin_angle.array()
               + az * moment_of_inertia_
                     * (cos_varphi_.array() * steering_status.sin_angle.array()
                        - sin_varphi_.array() * steering_status.cos_angle.array())
                     / vehicle_radius_)
            / 4.0;

        wheel_torques += wheel_pid_torques;

        return wheel_torques;
    }

    void update_control_torques(
        const Eigen::Vector4d& steering_torques, const Eigen::Vector4d& wheel_torques) {
        *left_front_steering_control_torque_ = steering_torques[0];
        *left_back_steering_control_torque_ = steering_torques[1];
        *right_back_steering_control_torque_ = steering_torques[2];
        *right_front_steering_control_torque_ = steering_torques[3];

        *left_front_wheel_control_torque_ = wheel_torques[0];
        *left_back_wheel_control_torque_ = wheel_torques[1];
        *right_back_wheel_control_torque_ = wheel_torques[2];
        *right_front_wheel_control_torque_ = wheel_torques[3];
    }

    void update_chassis_velocity_expected(const Eigen::Vector3d& chassis_acceleration) {
        auto acceleration_odom = base_link_to_odom_vector(chassis_acceleration);
        chassis_velocity_expected_ += dt_ * acceleration_odom;
    }

    Eigen::Vector3d base_link_to_odom_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    Eigen::Vector3d odom_to_base_link_vector(Eigen::Vector3d vector) const {
        vector.head<2>() = Eigen::Rotation2Dd(-chassis_yaw_angle_imu_) * vector.head<2>();
        return vector;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double dt_ = 1e-3;
    static constexpr double g_ = 9.81;

    const double mess_;
    const double moment_of_inertia_;
    const double vehicle_radius_;
    const double wheel_radius_;
    const double friction_coefficient_;

    const double k1_, k2_, no_load_power_;

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

    QcpSolver qcp_solver_;
    filter::LowPassFilter<3> control_acceleration_filter_;

    double chassis_yaw_angle_imu_ = 0.0;
    Eigen::Vector3d chassis_velocity_expected_ = Eigen::Vector3d::Zero();

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;

    pid::MatrixPidCalculator<4> steering_velocity_pid_, steering_angle_pid_, wheel_velocity_pid_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelController, rmcs_executor::Component)