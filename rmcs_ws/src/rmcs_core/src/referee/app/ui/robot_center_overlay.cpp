#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {
using namespace rmcs_description;

class UiRobotCenterOverlay
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    UiRobotCenterOverlay()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , center_ring_{Shape::Color::GREEN, 2, 0, 0, 5, 5, false}
        , cross_top_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false}
        , cross_bottom_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false}
        , cross_left_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false}
        , cross_right_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false} {
        offset_x_ = get_parameter_or("offset_x", 0.000);
        offset_y_ = get_parameter_or("offset_y", 0.000);
        offset_z_ = get_parameter_or("offset_z", 0.202);

        register_input("/tf", tf_);
        register_input("/auto_aim/robot_center", robot_center_, false);
        register_input("/auto_aim/should_shoot", should_shoot_, false);
    }

    void update() override {
        using namespace Eigen;

        if (!robot_center_.ready()) {
            hide_all();
            return;
        }
        const auto& center = *robot_center_;
        if (!center.allFinite()) {
            hide_all();
            return;
        }

        const auto camera_pose = fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_);
        const Quaterniond q_aa{camera_pose.rotation()};
        const Vector3d t_aa = camera_pose.translation();

        const Vector3d t_ui = t_aa - q_aa * Vector3d{offset_x_, offset_y_, offset_z_};

        const Vector3d P_ros = q_aa.inverse() * (center - t_ui);

        const double x_cv = -P_ros.y();
        const double y_cv = P_ros.z();
        const double z_cv = P_ros.x();

        if (z_cv <= 1e-6) {
            hide_all();
            return;
        }

        const double x_norm = x_cv / z_cv;
        const double y_norm = y_cv / z_cv;

        const double r2 = x_norm * x_norm + y_norm * y_norm;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double radial = 1.0 + k1_ * r2 + k2_ * r4 + k3_ * r6;
        const double x_dist =
            x_norm * radial + 2.0 * p1_ * x_norm * y_norm + p2_ * (r2 + 2.0 * x_norm * x_norm);
        const double y_dist =
            y_norm * radial + p1_ * (r2 + 2.0 * y_norm * y_norm) + 2.0 * p2_ * x_norm * y_norm;

        const double u = fx_ * x_dist + cx_;
        const double v = fy_ * y_dist + cy_;

        if (u < 0.0 || u >= static_cast<double>(screen_width) || v < 0.0
            || v >= static_cast<double>(screen_height)) {
            hide_all();
            return;
        }

        const auto clamp_coord = [](long value, uint16_t upper) {
            return static_cast<uint16_t>(
                value < 0 ? 0 : (value >= upper ? upper - 1 : value));
        };
        const auto ui = clamp_coord(std::lround(u), screen_width);
        const auto vi = clamp_coord(std::lround(v), screen_height);

        const bool should_shoot = should_shoot_.ready() && *should_shoot_;
        const auto color = should_shoot ? Shape::Color::ORANGE : Shape::Color::GREEN;
        const uint16_t radius = should_shoot ? 10 : 15;

        center_ring_.set_color(color);
        center_ring_.set_x(ui);
        center_ring_.set_y(vi);
        center_ring_.set_r(radius);
        center_ring_.set_visible(true);

        if (should_shoot) {
            constexpr int cross_len = 16, cross_gap = 5;
            const auto clamp_x = [](int x) {
                return static_cast<uint16_t>(
                    x < 0 ? 0 : (x >= screen_width ? screen_width - 1 : x));
            };
            const auto clamp_y = [](int y) {
                return static_cast<uint16_t>(
                    y < 0 ? 0 : (y >= screen_height ? screen_height - 1 : y));
            };

            cross_top_.set_color(color);
            cross_top_.set_x(ui);
            cross_top_.set_y(clamp_y(vi - cross_len));
            cross_top_.set_x2(ui);
            cross_top_.set_y2(clamp_y(vi - cross_gap));
            cross_top_.set_visible(true);

            cross_bottom_.set_color(color);
            cross_bottom_.set_x(ui);
            cross_bottom_.set_y(clamp_y(vi + cross_gap));
            cross_bottom_.set_x2(ui);
            cross_bottom_.set_y2(clamp_y(vi + cross_len));
            cross_bottom_.set_visible(true);

            cross_left_.set_color(color);
            cross_left_.set_x(clamp_x(ui - cross_len));
            cross_left_.set_y(vi);
            cross_left_.set_x2(clamp_x(ui - cross_gap));
            cross_left_.set_y2(vi);
            cross_left_.set_visible(true);

            cross_right_.set_color(color);
            cross_right_.set_x(clamp_x(ui + cross_gap));
            cross_right_.set_y(vi);
            cross_right_.set_x2(clamp_x(ui + cross_len));
            cross_right_.set_y2(vi);
            cross_right_.set_visible(true);
        } else {
            cross_top_.set_visible(false);
            cross_bottom_.set_visible(false);
            cross_left_.set_visible(false);
            cross_right_.set_visible(false);
        }
    }

private:
    void hide_all() {
        center_ring_.set_visible(false);
        cross_top_.set_visible(false);
        cross_bottom_.set_visible(false);
        cross_left_.set_visible(false);
        cross_right_.set_visible(false);
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;

    static constexpr double fx_ = 730.7267062695;
    static constexpr double fy_ = 730.5886055073;
    static constexpr double cx_ = 961.8345772991;
    static constexpr double cy_ = 549.3357457590;

    static constexpr double k1_ = -0.1764932263;
    static constexpr double k2_ = 0.1582678597;
    static constexpr double k3_ = -0.1557993057;
    static constexpr double p1_ = -0.0000433367;
    static constexpr double p2_ = 0.0003937540;

    double offset_x_;
    double offset_y_;
    double offset_z_;

    InputInterface<Tf> tf_;
    InputInterface<Eigen::Vector3d> robot_center_;
    InputInterface<bool> should_shoot_;

    Circle center_ring_;
    Line cross_top_;
    Line cross_bottom_;
    Line cross_left_;
    Line cross_right_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::UiRobotCenterOverlay, rmcs_executor::Component)