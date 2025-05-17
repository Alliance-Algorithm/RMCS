#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include <controller/pid/matrix_pid_calculator.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "rmcs_utility/eigen_structured_bindings.hpp"

namespace rmcs_core::controller::chassis {

class SteeringWheelControllerWithPlanner
    : public rmcs_executor::Component
    , public rclcpp::Node {

    using Formula = std::tuple<double, double, double>;

public:
    explicit SteeringWheelControllerWithPlanner()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , chassis_velocity_expected_(Eigen::Vector3d::Zero())
        , chassis_translational_velocity_pid_(5.0, 0.0, 1.0)
        , chassis_angular_velocity_pid_(5.0, 0.0, 1.0)
        , cos_varphi_(1, 0, -1, 0) // 0, pi/2, pi, 3pi/2
        , sin_varphi_(0, 1, 0, -1)
        , steering_velocity_pid_(0.15, 0.0, 0.0)
        , steering_angle_pid_(80.0, 0.0, 0.0)
        , wheel_velocity_pid_(0.3, 0.000, 0.0) {

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

        register_input("/tf", tf_);
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

        auto chassis_control_velocity = calculate_chassis_control_velocity();
        auto chassis_status_expected  = calculate_chassis_status_expected();

        auto wheel_velocities = calculate_wheel_velocities();
        auto steering_status  = calculate_steering_status();

        Eigen::Vector3d chassis_acceleration = calculate_chassis_control_acceleration(
            chassis_status_expected.velocity, chassis_control_velocity);

        update_steering_control_torques(
            steering_status, chassis_status_expected, chassis_acceleration);
        update_wheel_torques(
            wheel_velocities, steering_status, chassis_status_expected, chassis_acceleration);

        update_chassis_velocity_expected(chassis_acceleration);
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
        chassis_velocity_expected_.vector = Eigen::Vector3d::Zero();

        *left_front_steering_control_torque_  = 0.0;
        *left_back_steering_control_torque_   = 0.0;
        *right_back_steering_control_torque_  = 0.0;
        *right_front_steering_control_torque_ = 0.0;

        *left_front_wheel_control_torque_  = 0.0;
        *left_back_wheel_control_torque_   = 0.0;
        *right_back_wheel_control_torque_  = 0.0;
        *right_front_wheel_control_torque_ = 0.0;
    }

    Eigen::Vector4d calculate_wheel_velocities() {
        return {
            *left_front_wheel_velocity_,    //
            *left_back_wheel_velocity_,     //
            *right_back_wheel_velocity_,    //
            *right_front_wheel_velocity_    //
        };
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

    Eigen::Vector3d calculate_chassis_control_velocity() {
        Eigen::Vector3d chassis_control_velocity =
            fast_tf::cast<rmcs_description::BaseLink>(*chassis_control_velocity_, *tf_).vector;
        chassis_control_velocity.head<2>() =
            Eigen::Rotation2Dd(-std::numbers::pi / 4) * chassis_control_velocity.head<2>();
        Eigen::Vector2d err = chassis_control_velocity.head<2>() - translational_control_velocity;
        translational_control_velocity = translational_control_velocity
                                       + std::clamp(err.norm(), -0.002, 0.002) * err.normalized();
        chassis_control_velocity.head<2>() = translational_control_velocity;
        return chassis_control_velocity;
    }

    ChassisStatus calculate_chassis_status_expected() {
        ChassisStatus chassis_status_expected;

        chassis_status_expected.velocity =
            fast_tf::cast<rmcs_description::BaseLink>(chassis_velocity_expected_, *tf_).vector;
        chassis_status_expected.velocity.head<2>() =
            Eigen::Rotation2Dd(-std::numbers::pi / 4) * chassis_status_expected.velocity.head<2>();

        const double vz = chassis_status_expected.velocity(2);

        const auto& [vx, vy, _vz] = calculate_chassis_control_velocity();
        // std::cerr << "vx" << vx << ",vy" << vy << std::endl;
        chassis_status_expected.wheel_velocity_x = vx - vehicle_radius_ * vz * sin_varphi_.array();
        chassis_status_expected.wheel_velocity_y = vy + vehicle_radius_ * vz * cos_varphi_.array();

        return chassis_status_expected;
    }

    Eigen::Vector3d calculate_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_velocity_expected,
        const Eigen::Vector3d& chassis_control_velocity) {

        Eigen::Vector2d translational_control_velocity = chassis_control_velocity.head<2>();
        Eigen::Vector2d translational_velocity         = chassis_velocity_expected.head<2>();
        Eigen::Vector2d translational_control_acceleration =
            chassis_translational_velocity_pid_.update(
                translational_control_velocity - translational_velocity);

        const double& angular_control_velocity = chassis_control_velocity[2];
        const double& angular_velocity         = chassis_velocity_expected[2];
        double angular_control_acceleration =
            chassis_angular_velocity_pid_.update(angular_control_velocity - angular_velocity);

        Eigen::Vector3d chassis_control_acceleration;
        chassis_control_acceleration << translational_control_acceleration,
            angular_control_acceleration;

        if (chassis_control_acceleration.lpNorm<1>() < 1e-1)
            chassis_control_acceleration.setZero();
        else
            chassis_control_acceleration = constrain_chassis_control_acceleration(
                chassis_velocity_expected, chassis_control_acceleration);

        return chassis_control_acceleration;
    }

    Eigen::Vector3d constrain_chassis_control_acceleration(
        const Eigen::Vector3d& chassis_velocity_expected,
        const Eigen::Vector3d& chassis_acceleration) {

        Eigen::Vector2d translational_acceleration_direction = chassis_acceleration.head<2>();
        double translational_acceleration_max = translational_acceleration_direction.norm();
        translational_acceleration_direction.normalize();

        double angular_acceleration_max    = chassis_acceleration.z();
        bool angular_acceleration_positive = angular_acceleration_max > 0;
        if (!angular_acceleration_positive)
            angular_acceleration_max = -angular_acceleration_max;
        double angular_acceleration_sign = angular_acceleration_positive ? 1.0 : -1.0;

        double rhombus_right = friction_coefficient_ * g_;
        double rhombus_top   = rhombus_right * mess_ * vehicle_radius_ / moment_of_inertia_;
        auto constrain       = calculate_polygon_constraints(
            rhombus_right, rhombus_top, translational_acceleration_max, -inf_,
            angular_acceleration_max);

        Eigen::Vector2d power_constrain = {
            mess_ * chassis_velocity_expected.head<2>().dot(translational_acceleration_direction),
            angular_acceleration_sign * moment_of_inertia_ * chassis_velocity_expected.z()};
        double power_limit = *power_limit_;
        sutherland_hodgman(
            constrain,
            [&power_constrain, &power_limit](const Eigen::Vector2d& point) {
                return point.dot(power_constrain) - power_limit;
            },
            [&power_constrain, &power_limit](
                const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) -> Eigen::Vector2d {
                Eigen::Vector3d line =
                    Eigen::Vector3d(p1.x(), p1.y(), 1).cross(Eigen::Vector3d(p2.x(), p2.y(), 1));

                Eigen::Matrix2d matrix;
                matrix << power_constrain.x(), power_constrain.y(), line.x(), line.y();

                return matrix.colPivHouseholderQr().solve(Eigen::Vector2d(power_limit, -line.z()));
            });

        double best_value          = -inf_;
        Eigen::Vector2d best_point = Eigen::Vector2d::Zero();
        for (auto& point : constrain) {
            double value = point.dot(Eigen::Vector2d(1.0, 0.1));
            // double value = point.dot(Eigen::Vector2d(0.3, 0.7));
            if (value > best_value)
                best_value = value, best_point = point;
        }

        Eigen::Vector3d best_acceleration;
        best_acceleration << best_point.x() * translational_acceleration_direction,
            angular_acceleration_sign * best_point.y();

        return best_acceleration;
    }

    /**
     * Calculates the feasible region of a linear program as a polygon. Returns a non-repeating list
     * of vertices in counter-clockwise order representing the polygon. The function assumes that
     * `rhombus_right` and `rhombus_top` are always positive values.
     */
    static std::vector<Eigen::Vector2d> calculate_polygon_constraints(
        double rhombus_right, double rhombus_top, double x_max, double y_min, double y_max) {
        std::vector<Eigen::Vector2d> polygon;

        do {
            if (y_min > y_max) [[unlikely]]
                break;

            auto calculate_intersecting_x = [&rhombus_right, &rhombus_top](double y) {
                if (y == 0.0)
                    return rhombus_right;
                else if (y > 0.0)
                    return (rhombus_right / rhombus_top) * (rhombus_top - y);
                else
                    return -(rhombus_right / rhombus_top) * (-rhombus_top - y);
            };

            if (y_min == y_max) [[unlikely]] {
                auto x = calculate_intersecting_x(y_max);
                if (x < 0.0) [[unlikely]]
                    break;
                if (x != 0.0) [[likely]]
                    polygon.emplace_back(x, y_max);
                polygon.emplace_back(0.0, y_max);
                break;
            }

            if (y_min < 0 && y_max > 0) {
                polygon.emplace_back(rhombus_right, 0.0);
            }
            if (y_max < rhombus_top) {
                polygon.emplace_back(calculate_intersecting_x(y_max), y_max);
                polygon.emplace_back(0.0, y_max);
            } else {
                polygon.emplace_back(0.0, rhombus_top);
            }
            if (y_min > -rhombus_top) {
                polygon.emplace_back(0.0, y_min);
                polygon.emplace_back(calculate_intersecting_x(y_min), y_min);
            } else {
                polygon.emplace_back(0.0, -rhombus_top);
            }
        } while (false);

        sutherland_hodgman(
            polygon, [&x_max](const Eigen::Vector2d& point) { return point.x() - x_max; },
            [&rhombus_right, &rhombus_top, &x_max](
                const Eigen::Vector2d& current_point,
                const Eigen::Vector2d& prev_point) -> Eigen::Vector2d {
                if (current_point.y() == prev_point.y()) {
                    return {x_max, current_point.y()};
                } else {
                    if (current_point.y() > 0.0)
                        return {x_max, -(rhombus_top / rhombus_right) * x_max + rhombus_top};
                    else
                        return {x_max, (rhombus_top / rhombus_right) * x_max - rhombus_top};
                }
            });

        return polygon;
    }

    /**
     * Sutherland-Hodgman polygon clipping algorithm. The function modifies the input polygon by
     * removing vertices that are outside the clipping region defined by the edge_compare and
     * calculate_intersecting_point functions.
     */
    static void sutherland_hodgman(
        std::vector<Eigen::Vector2d>& polygon, const auto& edge_compare,
        const auto& calculate_intersecting_point) {

        std::vector<Eigen::Vector2d> new_polygon;
        for (size_t i = polygon.size() - 1, j = 0; j < polygon.size(); i = j++) {
            auto& current_point = polygon[j];
            auto& prev_point    = polygon[i];

            if (edge_compare(current_point) == 0) {
                new_polygon.emplace_back(current_point);
            } else if (edge_compare(current_point) < 0) {
                if (edge_compare(prev_point) > 0)
                    new_polygon.emplace_back(
                        calculate_intersecting_point(current_point, prev_point));
                new_polygon.emplace_back(current_point);
            } else if (edge_compare(prev_point) < 0) {
                if (polygon.size() != 2) [[likely]] // Prevent point duplicates when 2 vertices
                    new_polygon.emplace_back(
                        calculate_intersecting_point(current_point, prev_point));
            }
        }
        polygon.swap(new_polygon);
    }

    void update_steering_control_torques(
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
                    steering_control_angles[i]     = std::atan2(y, x);
                } else {
                    steering_control_velocities[i] = nan_;
                    steering_control_angles[i]     = nan_;
                }
            }
        }

        Eigen::Vector4d control_torques = steering_velocity_pid_.update(
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

        *left_front_steering_control_torque_  = control_torques[0];
        *left_back_steering_control_torque_   = control_torques[1];
        *right_back_steering_control_torque_  = control_torques[2];
        *right_front_steering_control_torque_ = control_torques[3];
    }

    void update_wheel_torques(
        const Eigen::Vector4d& wheel_velocities, const SteeringStatus& steering_status,
        const ChassisStatus& chassis_status_expected, const Eigen::Vector3d&) {

        // const auto& [ax, ay, az]      = chassis_acceleration;
        Eigen::Vector4d wheel_torques = {0, 0, 0, 0};

        Eigen::Vector4d wheel_control_velocity =
            chassis_status_expected.wheel_velocity_x.array() * steering_status.cos_angle.array()
            + chassis_status_expected.wheel_velocity_y.array() * steering_status.sin_angle.array();
        Eigen::Vector4d vel_err = wheel_control_velocity / wheel_radius_ - wheel_velocities;
        // for (int i = 0; i < 4; i++) {
        //     vel_err(i) = std::clamp(vel_err(i), -3.0, 3.0);
        // }
        wheel_torques += wheel_velocity_pid_.update(vel_err);
        // std::cerr << "control:" << wheel_control_velocity / wheel_radius_ << std::endl;
        // std::cerr << "current" << wheel_velocities << std::endl;

        *left_front_wheel_control_torque_  = wheel_torques[0];
        *left_back_wheel_control_torque_   = wheel_torques[1];
        *right_back_wheel_control_torque_  = wheel_torques[2];
        *right_front_wheel_control_torque_ = wheel_torques[3];
    }

    void update_chassis_velocity_expected(Eigen::Vector3d chassis_acceleration) {
        chassis_acceleration.head<2>() =
            Eigen::Rotation2Dd(std::numbers::pi / 4) * chassis_acceleration.head<2>();
        auto acceleration_base_link = fast_tf::cast<rmcs_description::YawLink>(
            rmcs_description::BaseLink::DirectionVector{chassis_acceleration}, *tf_);

        constexpr double dt = 1e-3;
        chassis_velocity_expected_.vector += dt * acceleration_base_link.vector;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double g_ = 9.81;

    // static constexpr double mess_                 = 22.0;
    // static constexpr double moment_of_inertia_    = 4.0;
    // static constexpr double vehicle_radius_       = 0.2 * std::numbers::sqrt2;
    // static constexpr double wheel_radius_         = 0.055;
    // static constexpr double friction_coefficient_ = 0.6;

    static constexpr double mess_                 = 19.0;
    static constexpr double moment_of_inertia_    = 2.0;
    static constexpr double vehicle_radius_       = 0.24678;
    static constexpr double wheel_radius_         = 0.055;
    static constexpr double friction_coefficient_ = 0.6;

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

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<rmcs_description::YawLink::DirectionVector> chassis_control_velocity_;
    InputInterface<double> power_limit_;

    OutputInterface<double> left_front_steering_control_torque_;
    OutputInterface<double> left_back_steering_control_torque_;
    OutputInterface<double> right_back_steering_control_torque_;
    OutputInterface<double> right_front_steering_control_torque_;

    OutputInterface<double> left_front_wheel_control_torque_;
    OutputInterface<double> left_back_wheel_control_torque_;
    OutputInterface<double> right_back_wheel_control_torque_;
    OutputInterface<double> right_front_wheel_control_torque_;

    rmcs_description::YawLink::DirectionVector chassis_velocity_expected_;

    pid::MatrixPidCalculator<2> chassis_translational_velocity_pid_;
    pid::PidCalculator chassis_angular_velocity_pid_;

    const Eigen::Vector4d cos_varphi_, sin_varphi_;
    Eigen::Vector2d translational_control_velocity{0, 0};
    pid::MatrixPidCalculator<4> steering_velocity_pid_, steering_angle_pid_, wheel_velocity_pid_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::SteeringWheelControllerWithPlanner, rmcs_executor::Component)