#include <cmath>

#include <algorithm>
#include <iterator>
#include <limits>
#include <numbers>

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
    , public rclcpp::Node {
public:
    OmniWheelController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , translational_velocity_pid_calculator_(100.0, 0.0, 0.0)
        , angular_velocity_pid_calculator_(100.0, 0.0, 0.0) {

        register_input("/chassis/left_front_wheel/max_torque", wheel_motor_max_control_torque_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

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
            *right_front_velocity_};

        auto [best_translational_control_torque, best_angular_control_torque] =
            calculate_best_control_torque(wheel_velocities);

        *left_front_control_torque_ =
            best_angular_control_torque + best_translational_control_torque.y();
        *left_back_control_torque_ =
            best_angular_control_torque - best_translational_control_torque.x();
        *right_back_control_torque_ =
            best_angular_control_torque - best_translational_control_torque.y();
        *right_front_control_torque_ =
            best_angular_control_torque + best_translational_control_torque.x();
    }

private:
    auto calculate_best_control_torque(const double (&wheel_velocities)[4])
        -> std::pair<Eigen::Vector2d, double> {
        auto [translational_control_torque_max, translational_control_direction] =
            calculate_translational_control_torque_max(wheel_velocities);

        auto angular_control_torque_max = calculate_angular_control_torque_max(wheel_velocities);
        bool angular_control_torque_positive = angular_control_torque_max > 0;
        if (!angular_control_torque_positive)
            angular_control_torque_max = -angular_control_torque_max;
        auto signed_affine_coefficient =
            angular_control_torque_positive ? affine_coefficient_ : -affine_coefficient_;

        auto angular_constraint       = std::sqrt(control_torque_max_); // TODO: Cache this value
        auto translational_constraint = angular_constraint
                                      / std::max(
                                            std::abs(translational_control_direction.x()),
                                            std::abs(translational_control_direction.y()));

        auto polygon = calculate_polygon_constraints(
            translational_constraint, angular_constraint * affine_coefficient_,
            translational_control_torque_max, 0, angular_control_torque_max * affine_coefficient_);

        auto [circle_center, circle_radius] =
            calculate_ellipse_parameters(wheel_velocities, translational_control_direction);
        circle_center.y() *= signed_affine_coefficient;

        auto best_point =
            calculate_best_point_within_constraints(polygon, circle_center, circle_radius);
        return {
            best_point.x() * translational_control_direction,  // best translational control torque
            best_point.y() * (1 / signed_affine_coefficient)}; // best angular control torque
    }

    auto calculate_translational_control_torque_max(const double (&wheel_velocities)[4])
        -> std::pair<double, Eigen::Vector2d> {
        Eigen::Vector2d translational_control_velocity = control_velocity_->vector.head<2>();

        std::pair<double, Eigen::Vector2d> result{0.0, Eigen::Vector2d::Zero()};
        auto& [translational_control_torque_max, translational_control_direction] = result;

        if (!std::isnan(translational_control_velocity[0])) {
            // Rotate the vector clockwise by 45 degrees to align it with the coordinate system used
            // for the calculation.
            constexpr double sqrt_2_div_2  = std::numbers::sqrt2 / 2;
            translational_control_velocity = {
                sqrt_2_div_2
                    * (translational_control_velocity.x() + translational_control_velocity.y()),
                sqrt_2_div_2
                    * (-translational_control_velocity.x() + translational_control_velocity.y())};

            Eigen::Vector2d translational_velocity = wheel_radius_
                                                   * Eigen::Vector2d{
                                                       wheel_velocities[1] - wheel_velocities[3],
                                                       wheel_velocities[2] - wheel_velocities[0]};
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

    double calculate_angular_control_torque_max(const double (&wheel_velocities)[4]) {
        double angular_control_velocity   = control_velocity_->vector[2];
        double angular_control_torque_max = 0.0;
        if (!std::isnan(angular_control_velocity)) {
            double angular_velocity =
                -0.5 * wheel_radius_ / chassis_radius_
                * std::accumulate(std::begin(wheel_velocities), std::end(wheel_velocities), 0.0);
            angular_control_torque_max = -0.25 * wheel_radius_ / chassis_radius_
                                       * angular_velocity_pid_calculator_.update(
                                           angular_control_velocity - angular_velocity);
        }

        return angular_control_torque_max;
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

        std::vector<Eigen::Vector2d> polygon2;
        for (size_t i = polygon.size() - 1, j = 0; j < polygon.size(); i = j++) {
            auto& current_point = polygon[j];
            auto& prev_point    = polygon[i];

            auto calculate_intersecting_point = [&]() -> Eigen::Vector2d {
                if (current_point.y() == prev_point.y()) {
                    return {x_max, current_point.y()};
                } else {
                    if (current_point.y() > 0.0)
                        return {x_max, -(rhombus_top / rhombus_right) * x_max + rhombus_top};
                    else
                        return {x_max, (rhombus_top / rhombus_right) * x_max - rhombus_top};
                }
            };

            if (current_point.x() == x_max) {
                polygon2.emplace_back(current_point);
            } else if (current_point.x() < x_max) {
                if (prev_point.x() > x_max) {
                    polygon2.emplace_back(calculate_intersecting_point());
                }
                polygon2.emplace_back(current_point);
            } else if (prev_point.x() < x_max) {
                if (polygon.size() != 2) [[likely]] // Prevent point duplicates when 2 vertices
                    polygon2.emplace_back(calculate_intersecting_point());
            }
        }

        return polygon2;
    }

    auto calculate_ellipse_parameters(
        const double (&wheel_velocities)[4], const Eigen::Vector2d& translational_control_direction)
        const -> std::pair<Eigen::Vector2d, double> {
        std::pair<Eigen::Vector2d, double> result;
        auto& [center, semi_major_axis] = result;

        auto temp_value_0 =
            (wheel_velocities[0] - wheel_velocities[2]) * translational_control_direction.y()
            + (wheel_velocities[3] - wheel_velocities[1]) * translational_control_direction.x();
        auto temp_value_1 =
            std::accumulate(std::begin(wheel_velocities), std::end(wheel_velocities), 0.0);
        auto temp_value_2 = std::accumulate(
            std::begin(wheel_velocities), std::end(wheel_velocities), 0.0,
            [](double acc, double v) { return acc + std::pow(v, 2); });

        constexpr auto inv_k1    = 1 / k1_;
        constexpr auto inv_2_k1  = inv_k1 / 2;
        constexpr auto inv_4_k1  = inv_k1 / 4;
        constexpr auto inv_8_k1  = inv_k1 / 8;
        constexpr auto inv_16_k1 = inv_k1 / 16;

        center = {-inv_4_k1 * temp_value_0, -inv_8_k1 * temp_value_1};
        semi_major_axis =
            inv_2_k1
            * (inv_8_k1 * std::pow(temp_value_0, 2) + inv_16_k1 * std::pow(temp_value_1, 2)
               - k2_ * temp_value_2 + *power_limit_ - no_load_power_);
        semi_major_axis = semi_major_axis < 0 ? nan_ : std::sqrt(semi_major_axis);

        return result;
    }

    static Eigen::Vector2d calculate_best_point_within_constraints(
        const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& circle_center,
        const double& circle_radius) {
        if (std::isnan(circle_radius)) [[unlikely]]
            return Eigen::Vector2d::Zero();

        Eigen::Vector2d tangent_point =
            circle_center
            + circle_radius * Eigen::Vector2d{objective_angle_cos_, objective_angle_sin_};
        if (is_point_inside_polygon(tangent_point, polygon))
            return tangent_point;

        bool is_in_circle[6];
        if (sizeof(is_in_circle) < polygon.size()) [[unlikely]]
            std::terminate();                       // TODO: Remove this
        for (size_t i = 0; i < polygon.size(); i++)
            is_in_circle[i] = (polygon[i] - circle_center).norm() <= circle_radius;

        double max_z               = -inf_;
        Eigen::Vector2d best_point = Eigen::Vector2d::Zero();

        auto check_point = [&max_z, &best_point](Eigen::Vector2d point) {
            double z = objective_angle_cos_ * point.x() + objective_angle_sin_ * point.y();
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

    static constexpr double affine_coefficient_ = std::numbers::sqrt2;

    // objective function: z = cos(angle) x + sin(angle) y
    static constexpr double objective_angle_        = std::numbers::pi / 4;
    static inline const double objective_angle_cos_ = std::cos(objective_angle_),
                               objective_angle_sin_ = std::sin(objective_angle_);

    static constexpr double control_torque_max_ = 3.5;

    InputInterface<double> wheel_motor_max_control_torque_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

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