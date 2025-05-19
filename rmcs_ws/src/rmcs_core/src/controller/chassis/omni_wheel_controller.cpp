#include <cmath>

#include <algorithm>

// #include <iterator>
#include <limits>
#include <numbers>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>

#include "controller/pid/matrix_pid_calculator.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class OmniWheelController
    : public rmcs_executor::Component
    , public rclcpp::Node { // OmniWheelController

public:
    OmniWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , translational_velocity_pid_calculator_(100.0, 0.0, 0.0)
        , translational_velocity_pid_calculator_.integral_split_min = 0.01,
        angular_velocity_pid_calculator_(100.0, 0.0, 0.0) {

        register_input("/chassis/left_front_wheel/max_torque", wheel_motor_max_control_torque_);
        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_);
        register_input("/chassis/control_velocity", control_velocity_);
        register_input("/chassis/control_power_limit", power_limit_);
        register_output(
            "/chassis/left_front_wheel/control_torque", left_front_control_torque_, nan_);
        register_output("/chassis/left_back_wheel/control_torque", left_back_control_torque_, nan_);
        register_output(
            "/chassis/right_back_wheel/control_torque", right_back_control_torque_, nan_);
        register_output(
            "/chassis/right_front_wheel/control_torque", right_front_control_torque_, nan_);
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(), "Max control torque of wheel motor: %.f",
            *wheel_motor_max_control_torque_);
    }

    void update() override {

        double wheel_velocities[] = {
            *left_front_velocity_, *left_back_velocity_, *right_back_velocity_,
            *right_front_velocity_, *gimbal_yaw_velocity_imu_ - *yaw_velocity_};

        auto [best_translational_control_torque, best_angular_control_torque, K_0, K_1] =
            calculate_best_control_torque(wheel_velocities);

        *left_front_control_torque_ =
            K_0 * (best_angular_control_torque + best_translational_control_torque.y());
        *left_back_control_torque_ =
            K_1 * (best_angular_control_torque - best_translational_control_torque.x());
        *right_back_control_torque_ =
            (2.0 - K_0) * (best_angular_control_torque - best_translational_control_torque.y());
        *right_front_control_torque_ =
            (2.0 - K_1) * (best_angular_control_torque + best_translational_control_torque.x());
        // RCLCPP_INFO(
        //     get_logger(), "Max control torque of wheel motor: %.2f",
        //     std::sqrt(
        //         (-wheel_velocities[0] + wheel_velocities[2])
        //             * (-wheel_velocities[0] + wheel_velocities[2])
        //         + (-wheel_velocities[1] + wheel_velocities[3])
        //               * (-wheel_velocities[1] + wheel_velocities[3]))
        //         * 0.075 / 2);

    }

private:
    auto calculate_best_control_torque(const double (&wheel_velocities)[5])
        -> std::tuple<Eigen::Vector2d, double, double, double> {

        auto [K_0, K_1] = calculate_K(wheel_velocities);
        auto next_angle = calculate_next_angle(wheel_velocities);

        auto [translational_control_torque_max, translational_control_direction] =
            calculate_translational_control_torque_max(wheel_velocities);
        auto direction_angle = translational_control_direction;
        translational_control_direction.x() =
            direction_angle.x() * next_angle.x() - direction_angle.y() * next_angle.y();
        translational_control_direction.y() =
            direction_angle.y() * next_angle.x() + direction_angle.x() * next_angle.y();

        auto angular_control_torque_max = calculate_angular_control_torque_max(wheel_velocities);

        bool angular_control_torque_positive = angular_control_torque_max > 0;

        if (!angular_control_torque_positive)
            angular_control_torque_max = -angular_control_torque_max;

        auto signed_affine_coefficient =
            angular_control_torque_positive ? affine_coefficient_ : -affine_coefficient_;

        auto polygon1 = calculate_polygon_constraints_co(
            K_0, K_1, translational_control_torque_max, 0, angular_control_torque_max,
            translational_control_direction);

        auto [circle_center, circle_radius, signed_affine, rotation_angle] =
            calculate_ellipse_parameters(
                wheel_velocities, translational_control_direction, K_0, K_1);
        rotation_angle.x() *= signed_affine_coefficient;
        circle_center.y() *= signed_affine_coefficient;
        auto polygon = calculate_polygon(polygon1, signed_affine, rotation_angle);

        auto best_point = calculate_best_point_within_constraints(
            polygon, circle_center, circle_radius, rotation_angle);
        best_point.x() = best_point.x() * rotation_angle.y() - best_point.y() * rotation_angle.x();
        best_point.y() = best_point.y() / signed_affine;
        best_point.y() = best_point.x() * rotation_angle.x() + best_point.y() * rotation_angle.y();

        return {
            best_point.x() * translational_control_direction,
            best_point.y() / signed_affine_coefficient, K_0, K_1};
    }
    static Eigen::Vector2d calculate_next_angle(const double (&wheel_velocities)[5]) {
        double v = std::sqrt(
                       (-wheel_velocities[0] + wheel_velocities[2])
                           * (-wheel_velocities[0] + wheel_velocities[2])
                       + (-wheel_velocities[1] + wheel_velocities[3])
                             * (-wheel_velocities[1] + wheel_velocities[3]))
                 * 0.07775 / 2;
        if (v < 0.1)
            return {1, 0};
        if (v < 1.0)
            v = 1.0;
        else if (v < 1.5)
            v = 1 + (v - 1);
        else
            v = 1.5 + 0.8 * (v - 1.5);
        double omega =
            (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3])
            * 0.07775 / 0.9;

        if (omega > 0) {
            omega = (20 - omega) * omega / 16;
        } else if (omega < 0) {
            omega = (20 + omega) * omega / 16;
        }
        // if (omega > 0) {
        //     omega = 0.2 * omega + std::pow(omega, 0.775);
        // } else if (omega < 0)
        //     omega = 0.2 * omega - std::pow(-omega, 0.775);

        double next_angle = v * omega * 0.065;
        return {std::cos(next_angle), std::sin(next_angle)};
    }

    static auto calculate_K(const double (&wheel_velocities)[5]) -> std::pair<double, double> {

        double K_0 = std::pow(
            K_dec_,
            -std::abs(
                ((wheel_velocities[0] + wheel_velocities[2]) * 0.07775 / 0.39404 + wheel_velocities[4]))
                / std::max(
                    std::max(
                        std::abs(wheel_velocities[4]),
                        std::abs(
                            (wheel_velocities[0] + wheel_velocities[2]) * 0.07775 / 0.39404)),
                    min_omega_));
        double K_1 = std::pow(
            K_dec_,
            -std::abs(
                ((wheel_velocities[1] + wheel_velocities[3]) * 0.07775 / 0.39404 + wheel_velocities[4]))
                / std::max(
                    std::max(
                        std::abs(wheel_velocities[4]),
                        std::abs((wheel_velocities[1] + wheel_velocities[3]) * 0.07775 / 0.39404)),
                    min_omega_));
        if (K_0 < 0.85)
            K_0 += 0.15;
        else
            K_0 = 1;
        if (K_1 < 0.85)
            K_1 += 0.15;
        else
            K_1 = 1;

        if (std::abs(wheel_velocities[0]) < std::abs(wheel_velocities[2])) {
            K_0 = 2.0 - K_0;
        }
        if (std::abs(wheel_velocities[1]) < std::abs(wheel_velocities[3])) {
            K_1 = 2.0 - K_1;
        }
        return {K_0, K_1};
    }

    auto calculate_translational_control_torque_max(const double (&wheel_velocities)[5])
        -> std::pair<double, Eigen::Vector2d> {
        Eigen::Vector2d translational_control_velocity = control_velocity_->vector.head<2>();

        std::pair<double, Eigen::Vector2d> result{0.0, Eigen::Vector2d::Zero()};
        auto& [translational_control_torque_max, translational_control_direction] = result;

        if (!std::isnan(translational_control_velocity[0])) {
            // Rotate the vector clockwise by 45 degrees to align it with the coordinate system

            // for the calculation.
            constexpr double sqrt_2_div_2  = std::numbers::sqrt2 / 2;
            translational_control_velocity = {
                sqrt_2_div_2
                    * (translational_control_velocity.x() + translational_control_velocity.y()),
                sqrt_2_div_2
                    * (-translational_control_velocity.x() + translational_control_velocity.y())};

            Eigen::Vector2d translational_velocity =
                wheel_radius_
                * Eigen::Vector2d{
                    (wheel_velocities[1] - wheel_velocities[3]) / 2,
                    (wheel_velocities[2] - wheel_velocities[0]) / 2};
            Eigen::Vector2d translational_control_torque_vector =
                -0.5 * wheel_radius_
                * translational_velocity_pid_calculator_.update(
                    translational_control_velocity - translational_velocity);
            translational_control_torque_max = translational_control_torque_vector.norm();
            if (translational_control_torque_max != 0)
                translational_control_direction =
                    translational_control_torque_vector / translational_control_torque_max;
        }

        return result;
    }

    double calculate_angular_control_torque_max(const double (&wheel_velocities)[5]) {
        double angular_control_velocity   = control_velocity_->vector[2];
        double angular_control_torque_max = 0.0;
        if (!std::isnan(angular_control_velocity)) {
            double angular_velocity = -0.5 * wheel_radius_ / chassis_radius_
                                    * (wheel_velocities[0] + wheel_velocities[1]
                                       + wheel_velocities[2] + wheel_velocities[3]);
            angular_control_torque_max = -0.25 * wheel_radius_ / chassis_radius_
                                       * angular_velocity_pid_calculator_.update(
                                           angular_control_velocity - angular_velocity);
        }

        return angular_control_torque_max;
    }

    /**
     * Calculates the feasible region of a linear program as a polygon. Returns a non-repeating
     list
     * of vertices in counter-clockwise order representing the polygon. The function assumes that
     * `rhombus_right` and `rhombus_top` are always positive values.
     */

    static std::vector<Eigen::Vector2d> calculate_polygon_constraints_co(
        double K_0, double K_1, double x_max, double y_min, double y_max,
        Eigen::Vector2d translational_control_direction) {

        std::vector<Eigen::Vector2d> polygon;
        do {
            if (y_min > y_max) [[unlikely]] {
                break;
            }

            if (y_min == y_max) [[unlikely]] {

                polygon.emplace_back(x_max, y_max);
                polygon.emplace_back(0.0, y_max);
                break;
            }

            polygon.emplace_back(0.0, y_max);
            polygon.emplace_back(x_max, y_max);
            polygon.emplace_back(x_max, y_min);
            polygon.emplace_back(0.0, y_min);

        } while (false);

        auto calculate_intersecting_point = [](Eigen::Vector2d c, Eigen::Vector2d p, double K,
                                               double k) -> Eigen::Vector2d {
            if (c.x() == p.x())
                return {c.x(), control_torque_max_ / K - k * c.x()};
            else {
                double t = (c.y() - p.y()) / (c.x() - p.x());
                double b = c.y() - t * c.x();
                return {
                    (control_torque_max_ / K - b) / (t + k),
                    ((control_torque_max_ / K - b) / (t + k)) * t + b};
            }
        };

        auto process_constraint = [&](double K, double dir_component, bool is_upper_limit,
                                      std::vector<Eigen::Vector2d>& polygon) {
            std::vector<Eigen::Vector2d> new_polygon;
            const double limit = is_upper_limit ? control_torque_max_ : -control_torque_max_;
                                        
            for (size_t i = 0; i < polygon.size(); i++) {
                const auto& curr = polygon[i];
                const auto& prev = polygon[i - 1];

                double curr_val = K * (curr.y() + dir_component * curr.x());
                double prev_val = K * (prev.y() + dir_component * prev.x());
                if (is_upper_limit) {

                    if (curr_val <= limit && prev_val <= limit) {
                        new_polygon.emplace_back(curr);
                    } else if (curr_val <= limit && prev_val > limit) {
                        new_polygon.emplace_back(
                            calculate_intersecting_point(curr, prev, K, dir_component));
                        new_polygon.emplace_back(curr);

                    } else if (curr_val > limit && prev_val <= limit) {
                        new_polygon.emplace_back(
                            calculate_intersecting_point(curr, prev, K, dir_component));
                    }
                }
                if (!is_upper_limit) {
                    if (curr_val >= limit && prev_val >= limit) {
                        new_polygon.emplace_back(curr);
                    } else if (curr_val >= limit && prev_val < limit) {
                        new_polygon.emplace_back(
                            calculate_intersecting_point(curr, prev, -K, dir_component));
                        new_polygon.emplace_back(curr);

                    } else if (curr_val < limit && prev_val >= limit) {
                        new_polygon.emplace_back(
                            calculate_intersecting_point(curr, prev, -K, dir_component));
                    }
                }
            }

            polygon = std::move(new_polygon);
        };

        process_constraint(K_0, translational_control_direction.x(), true, polygon);
        process_constraint(K_0, translational_control_direction.x(), false, polygon);
        process_constraint(2.0 - K_0, -translational_control_direction.x(), true, polygon);
        process_constraint(2.0 - K_0, -translational_control_direction.x(), false, polygon);
        process_constraint(2.0 - K_1, translational_control_direction.y(), true, polygon);
        process_constraint(2.0 - K_1, translational_control_direction.y(), false, polygon);
        process_constraint(K_1, -translational_control_direction.y(), true, polygon);
        process_constraint(K_1, -translational_control_direction.y(), false, polygon);

        std::vector<Eigen::Vector2d> result;
        for (size_t i = 0; i < polygon.size(); ++i) {
            if (i == 0 || polygon[i] != polygon[i - 1]) {
                result.emplace_back(polygon[i]);
            }
        }
        return result;
    }

    auto calculate_ellipse_parameters(
        const double (&wheel_velocities)[5], const Eigen::Vector2d& translational_control_direction,
        const double K_0, const double K_1) const
        -> std::tuple<Eigen::Vector2d, double, double, Eigen::Vector2d> {
        std::tuple<Eigen::Vector2d, double, double, Eigen::Vector2d> result;
        auto& [center, semi_major_axis, signed_affine, rotation_angle] = result;
        Eigen::Vector2d angle;
        angle.x() = translational_control_direction.y();
        angle.y() = translational_control_direction.x();

        double a = k1_
                 * ((K_0 * K_0 + (2.0 - K_0) * (2.0 - K_0)) * angle.x() * angle.x()
                    + (K_1 * K_1 + (2.0 - K_1) * (2.0 - K_1)) * angle.y() * angle.y());
        double b = 2 * k1_
                 * ((K_0 * K_0 - (2.0 - K_0) * (2.0 - K_0)) * angle.x()
                    + (-K_1 * K_1 + (2.0 - K_1) * (2.0 - K_1)) * angle.y());
        double c =
            k1_ * (K_0 * K_0 + (2.0 - K_0) * (2.0 - K_0) + K_1 * K_1 + (2.0 - K_1) * (2.0 - K_1));
        double d =
            ((wheel_velocities[0] * K_0 - wheel_velocities[2] * (2.0 - K_0)) * angle.x()
             - (wheel_velocities[1] * K_1 - wheel_velocities[3] * (2.0 - K_1)) * angle.y());
        double e = wheel_velocities[0] * K_0 + wheel_velocities[2] * (2.0 - K_0)
                 + wheel_velocities[1] * K_1 + wheel_velocities[3] * (2.0 - K_1);
        double f = k2_
                     * (wheel_velocities[0] * wheel_velocities[0]
                        + wheel_velocities[1] * wheel_velocities[1]
                        + wheel_velocities[2] * wheel_velocities[2]
                        + wheel_velocities[3] * wheel_velocities[3])
                 + no_load_power_ - *power_limit_;

        double h      = (b * e - 2 * c * d) / (4 * a * c - b * b);
        double k      = (b * d - 2 * a * e) / (4 * a * c - b * b);
        double lamb_1 = (a + c + std::sqrt((a - c) * (a - c) + b * b)) / 2;
        double lamb_2 = (a + c - std::sqrt((a - c) * (a - c) + b * b)) / 2;
        double f_1    = std::abs(a * h * h + b * h * k + c * k * k + d * h + e * k + f);

        double l = std::sqrt(f_1 / lamb_2);
        double s = std::sqrt(f_1 / lamb_1);

        rotation_angle = {
            std::sin(std::atan(b / (a - c)) / 2), std::cos(std::atan(b / (a - c)) / 2)};
        signed_affine = l / s;

        center = {
            h * rotation_angle.y() + k * rotation_angle.x(),
            signed_affine * (k * rotation_angle.y() - h * rotation_angle.x())};
        semi_major_axis = l;

        return result;
    }
    static std::vector<Eigen::Vector2d> calculate_polygon(
        const std::vector<Eigen::Vector2d>& polygon1, const double& signed_affine,
        const Eigen::Vector2d& rotation_angle) {
        std::vector<Eigen::Vector2d> polygon;
        for (int i = int(polygon1.size()) - 1; i > -1; i--) {
            auto x = polygon1[i].x();
            auto y = polygon1[i].y();
            polygon.emplace_back(
                x * rotation_angle.y() + y * rotation_angle.x(),
                signed_affine * (y * rotation_angle.y() - x * rotation_angle.x()));
        }

        return polygon;
    }

    static Eigen::Vector2d calculate_best_point_within_constraints(
        const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& circle_center,
        const double& circle_radius, const Eigen::Vector2d& rotation_angle) {
        if (std::isnan(circle_radius)) [[unlikely]]
            return Eigen::Vector2d::Zero();

        double sin = rotation_angle.x();
        double cos = rotation_angle.y();

        Eigen::Vector2d tangent_point =
            circle_center
            + circle_radius
                  * Eigen::Vector2d{
                      objective_angle_cos_ * (sin + cos), objective_angle_sin_ * (cos - sin)};
        if (is_point_inside_polygon(tangent_point, polygon))
            return tangent_point;

        bool is_in_circle[8];
        if (sizeof(is_in_circle) < polygon.size()) [[unlikely]]
            std::terminate(); // TODO: Remove this
        for (size_t i = 0; i < polygon.size(); i++)
            is_in_circle[i] = (polygon[i] - circle_center).norm() <= circle_radius;

        double max_z               = -inf_;
        Eigen::Vector2d best_point = Eigen::Vector2d::Zero();

        auto check_point = [&max_z, &best_point, &sin, &cos](Eigen::Vector2d point) {
            double z = objective_angle_cos_ * point.x() * (sin + cos)
                     + objective_angle_sin_ * point.y() * (cos - sin);
            if ((z > max_z)
                || (z == max_z && point.x() >= best_point.x() && point.y() >= best_point.y())) {
                max_z      = z;
                best_point = point;
            }
            return true;
        };

        for (size_t i = polygon.size() - 1, j = 0; j < polygon.size(); i = j++) {
            bool both_in_circle = true;
            both_in_circle &= is_in_circle[i] && check_point(polygon[i]);
            both_in_circle &= is_in_circle[j] && check_point(polygon[j]);

            if (!both_in_circle) {
                const auto &x0 = circle_center.x(), &y0 = circle_center.y();
                const auto &x1 = polygon[i].x(), &y1 = polygon[i].y();
                const auto &x2 = polygon[j].x(), &y2 = polygon[j].y();

                auto a = std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2);
                auto b = -2 * x0 * (x2 - x1) + 2 * x1 * (x2 - x1) - 2 * y0 * (y2 - y1)
                       + 2 * y1 * (y2 - y1);
                auto c = std::pow(x0, 2) - 2 * x0 * x1 + std::pow(x1, 2) + std::pow(y0, 2)
                       - 2 * y0 * y1 + std::pow(y1, 2) - std::pow(circle_radius, 2);
                auto delta = std::pow(b, 2) - 4 * a * c;
                if (delta < 0)
                    continue;
                delta = std::sqrt(delta);

                auto check_solution = [&](double t) {
                    if (0.0 < t && t < 1.0)
                        check_point({x1 + t * (x2 - x1), y1 + t * (y2 - y1)});
                };
                check_solution((-b + delta) / (2 * a));
                check_solution((-b - delta) / (2 * a));
            }
        }

        return best_point;
    }

    static bool is_point_inside_polygon(
        const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon) {
        // https://stackoverflow.com/a/63436180

        // Helper function to check if a point is between two values
        auto between = [](double p, double a, double b) {
            return (p >= a && p <= b) || (p <= a && p >= b);
        };

        bool inside = false;
        for (size_t i = polygon.size() - 1, j = 0; j < polygon.size(); i = j++) {
            const Eigen::Vector2d& a = polygon[i];
            const Eigen::Vector2d& b = polygon[j];

            // Corner cases
            if ((p.x() == a.x() && p.y() == a.y()) || (p.x() == b.x() && p.y() == b.y()))
                return true;                    // On edge
            if (a.y() == b.y() && p.y() == a.y() && between(p.x(), a.x(), b.x()))
                return true;
            if (a.x() == b.x() && p.x() == a.x() && between(p.y(), a.y(), b.y()))
                return true;                    // On edge

            if (between(p.y(), a.y(), b.y())) { // Check if P insides the vertical range
                // Filter out "ray pass vertex" problem by treating the line a little lower
                if ((p.y() == a.y() && b.y() >= a.y()) || (p.y() == b.y() && a.y() >= b.y()))
                    continue;

                // Calculate the cross product `PA X PB`, P lays on left side of AB if c > 0
                double c = (a.x() - p.x()) * (b.y() - p.y()) - (b.x() - p.x()) * (a.y() - p.y());
                if (c == 0)
                    return true; // On edge

                if ((a.y() < b.y()) == (c > 0))
                    inside = !inside;
            }
        }

        return inside;
    }

    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double k1_ = 2.958580e+00, k2_ = 3.082190e-03, no_load_power_ = 2.7;

    static constexpr double wheel_radius_   = 0.07;
    static constexpr double chassis_radius_ = 0.5;

    static constexpr double min_omega_ = 6.0;
    static constexpr double K_dec_     = 10.0;

    static constexpr double affine_coefficient_ = 1.0;

    // objective function: z = cos(angle) x + sin(angle) y
    static constexpr double objective_angle_        = std::numbers::pi / 4;
    static inline const double objective_angle_cos_ = std::cos(objective_angle_),
                               objective_angle_sin_ = std::sin(objective_angle_);

    static constexpr double control_torque_max_ = 3.5;

    InputInterface<double> wheel_motor_max_control_torque_;
    InputInterface<double> left_front_velocity_;

    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    InputInterface<double> yaw_velocity_;

    InputInterface<double> gimbal_yaw_velocity_imu_;

    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<double> power_limit_;

    pid::MatrixPidCalculator<2> translational_velocity_pid_calculator_;
    pid::PidCalculator angular_velocity_pid_calculator_;

    OutputInterface<double> left_front_control_torque_;
    OutputInterface<double> left_back_control_torque_;
    OutputInterface<double> right_back_control_torque_;
    OutputInterface<double> right_front_control_torque_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::OmniWheelController, rmcs_executor::Component)