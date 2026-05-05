#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::controller::chassis {

class QcpSolver {
public:
    QcpSolver() = default;

    struct BoundaryConstraint {
        double x_max;
        double y_max;
    };

    struct RhombusConstraint {
        double right, top;
    };

    struct QuadraticConstraint {
        double a, b, c, d, e, f;
    };

    struct QuadraticConstraintMatrix {
        Eigen::Matrix2d q;
        Eigen::Vector2d p;
        double r;
    };

    static Eigen::Vector2d solve(
        const Eigen::Vector2d& objective, const BoundaryConstraint& boundary_constraint,
        const RhombusConstraint& rhombus_constraint,
        const QuadraticConstraint& quadratic_constraint) {

        auto linear_constraint = calculate_polygon_constraints(
            rhombus_constraint.right, rhombus_constraint.top, boundary_constraint.x_max,
            boundary_constraint.y_max);

        auto quadratic_constraint_matrixes =
            calculate_quadratic_constraint_matrixes(quadratic_constraint);
        auto quadratic_constraint_best_point =
            calculate_quadratic_constraint_best_point(objective, quadratic_constraint_matrixes);

        if (is_point_inside_polygon(quadratic_constraint_best_point, linear_constraint))
            return quadratic_constraint_best_point;
        else
            return calculate_best_value_at_intersections(
                objective, linear_constraint, quadratic_constraint_matrixes);
    }

private:
    /**
     * Calculates the feasible region of a linear program as a polygon. Returns a non-repeating list
     * of vertices in counter-clockwise order representing the polygon. The function assumes that
     * `rhombus_right` and `rhombus_top` are always positive values.
     */
    static std::vector<Eigen::Vector2d> calculate_polygon_constraints(
        double rhombus_right, double rhombus_top, double x_max, double y_max) {
        std::vector<Eigen::Vector2d> polygon;

        if (y_max >= rhombus_top) {
            polygon.emplace_back(rhombus_right, 0.0);
            polygon.emplace_back(0.0, rhombus_top);
            polygon.emplace_back(-rhombus_right, 0.0);
            polygon.emplace_back(0.0, -rhombus_top);
        } else if (y_max > 0) {
            double intersecting_x = (rhombus_right / rhombus_top) * (rhombus_top - y_max);
            polygon.emplace_back(rhombus_right, 0.0);
            polygon.emplace_back(intersecting_x, y_max);
            polygon.emplace_back(-intersecting_x, y_max);
            polygon.emplace_back(-rhombus_right, 0.0);
            polygon.emplace_back(0.0, -rhombus_top);
        } else if (y_max > -rhombus_top) {
            double intersecting_x = -(rhombus_right / rhombus_top) * (-rhombus_top - y_max);
            polygon.emplace_back(intersecting_x, y_max);
            polygon.emplace_back(-intersecting_x, y_max);
            polygon.emplace_back(0.0, -rhombus_top);
        } else if (y_max == -rhombus_top) {
            polygon.emplace_back(0.0, -rhombus_top);
        }

        sutherland_hodgman(polygon, {1, 0, -x_max});

        return polygon;
    }

    static void
        sutherland_hodgman(std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector3d& line) {
        sutherland_hodgman(
            polygon,
            [&line](const Eigen::Vector2d& point) { return line.head<2>().dot(point) + line.z(); },
            [&line](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) -> Eigen::Vector2d {
                Eigen::Vector3d line2 =
                    Eigen::Vector3d(p1.x(), p1.y(), 1).cross(Eigen::Vector3d(p2.x(), p2.y(), 1));
                Eigen::Matrix2d matrix{
                    { line.x(),  line.y()},
                    {line2.x(), line2.y()}
                };
                return matrix.inverse() * Eigen::Vector2d(-line.z(), -line2.z());
            });
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

    static QuadraticConstraintMatrix
        calculate_quadratic_constraint_matrixes(const QuadraticConstraint& quadratic_constraint) {
        QuadraticConstraintMatrix result{
            // clang-format off
            Eigen::Matrix2d{
                {2 * quadratic_constraint.a, quadratic_constraint.b},
                {quadratic_constraint.b, 2 * quadratic_constraint.c}
            },
            Eigen::Vector2d{
                quadratic_constraint.d, quadratic_constraint.e
            },
            quadratic_constraint.f
            // clang-format on
        };
        auto& [q, p, r] = result;
        if (q(0, 0) < 0)
            q = -q, p = -p, r = -r;
        return result;
    }

    static Eigen::Vector2d calculate_quadratic_constraint_best_point(
        const Eigen::Vector2d& objective, const QuadraticConstraintMatrix& constraint) {
        const auto& [q, p, r] = constraint;

        double det = q.determinant();
        Eigen::Matrix2d q_inv;
        if (det < 0)
            std::terminate();
        else if (det < epsilon_)
            q_inv = (q + epsilon_ * Eigen::Matrix2d::Identity()).inverse();
        else
            q_inv = q.inverse();
        auto q_inv_view = q_inv.selfadjointView<Eigen::Upper>();
        double lambda_inv =
            std::sqrt((p.dot(q_inv_view * p) - 2 * r) / objective.dot(q_inv_view * objective));

        return q_inv_view * (lambda_inv * objective - p);
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

    static bool is_point_inside_quadratic_constraint(
        const Eigen::Vector2d& point, const QuadraticConstraintMatrix& constraint) {
        const auto& [q, p, r] = constraint;
        return 0.5 * point.dot(q * point) + p.dot(point) + r <= 0;
    }

    static Eigen::Vector2d calculate_best_value_at_intersections(
        const Eigen::Vector2d& objective, const std::vector<Eigen::Vector2d>& linear_constraint,
        const QuadraticConstraintMatrix& quadratic_constraint) {
        const auto& [q, p, r] = quadratic_constraint;

        bool is_in_quadratic_constraint[20];
        if (sizeof(is_in_quadratic_constraint) < linear_constraint.size()) [[unlikely]]
            std::terminate();    // TODO: Remove this
        for (size_t i = 0; i < linear_constraint.size(); i++)
            is_in_quadratic_constraint[i] =
                is_point_inside_quadratic_constraint(linear_constraint[i], quadratic_constraint);

        double max_value           = -inf_;
        Eigen::Vector2d best_point = Eigen::Vector2d::Zero();

        auto check_point = [&](Eigen::Vector2d point) {
            double value = objective.dot(point);
            if ((value > max_value)
                || (value == max_value && point.x() >= best_point.x()
                    && point.y() >= best_point.y())) {
                max_value  = value;
                best_point = point;
            }
        };

        for (size_t i = linear_constraint.size() - 1, j = 0; j < linear_constraint.size();
             i = j++) {
            if (is_in_quadratic_constraint[i])
                check_point(linear_constraint[i]);
            if (!is_in_quadratic_constraint[i] || !is_in_quadratic_constraint[j]) {
                const Eigen::Vector2d &x1 = linear_constraint[i], &x2 = linear_constraint[j];
                Eigen::Vector2d dx = x2 - x1;

                double a     = 0.5 * dx.dot(q * dx);
                double b     = x1.dot(q * dx) + p.dot(dx);
                double c     = 0.5 * x1.dot(q * x1) + p.dot(x1) + r;
                double delta = b * b - 4 * a * c;

                if (delta < 0)
                    continue;

                auto check_solution = [&](double t) {
                    if (0.0 < t && t < 1.0)
                        check_point(x1 + t * dx);
                };
                check_solution((-b + sqrt(delta)) / (2 * a));
                check_solution((-b - sqrt(delta)) / (2 * a));
            }
        }

        return best_point;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double inf_ = std::numeric_limits<double>::infinity();

    static constexpr double epsilon_ = 1e-9;
};

} // namespace rmcs_core::controller::chassis