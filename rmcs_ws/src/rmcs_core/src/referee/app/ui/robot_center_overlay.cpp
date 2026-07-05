#include <cmath>
#include <cstdint>

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
        , center_dot_{Shape::Color::GREEN, 3, 0, 0, 3, 3, false}
        , cross_h_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false}
        , cross_v_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false} {

        register_input("/tf", tf_);
        register_input("/auto_aim/robot_center", robot_center_, false);
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

        const auto ui = static_cast<uint16_t>(std::lround(u));
        const auto vi = static_cast<uint16_t>(std::lround(v));

        center_dot_.set_x(ui);
        center_dot_.set_y(vi);
        center_dot_.set_visible(true);

        cross_h_.set_x(ui >= 8 ? ui - 8 : 0);
        cross_h_.set_y(vi);
        cross_h_.set_x2(ui + 8);
        cross_h_.set_y2(vi);
        cross_h_.set_visible(true);

        cross_v_.set_x(ui);
        cross_v_.set_y(vi >= 8 ? vi - 8 : 0);
        cross_v_.set_x2(ui);
        cross_v_.set_y2(vi + 8);
        cross_v_.set_visible(true);
    }

private:
    void hide_all() {
        center_dot_.set_visible(false);
        cross_h_.set_visible(false);
        cross_v_.set_visible(false);
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

    static constexpr double offset_x_ = 0.000;
    static constexpr double offset_y_ = 0.072;
    static constexpr double offset_z_ = 0.071;

    InputInterface<Tf> tf_;
    InputInterface<Eigen::Vector3d> robot_center_;

    Circle center_dot_;
    Line cross_h_;
    Line cross_v_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::app::ui::UiRobotCenterOverlay, rmcs_executor::Component)
