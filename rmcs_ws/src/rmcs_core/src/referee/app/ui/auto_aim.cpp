#include <algorithm>
#include <cmath>
#include <format>
#include <iterator>
#include <limits>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {
using namespace rmcs_description;

class AutoAimUi
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimUi()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        offset_ = Eigen::Vector3d{
            get_parameter_or("offset_x", 0.),
            get_parameter_or("offset_y", 0.),
            get_parameter_or("offset_z", 0.),
        };

        // What's the point of an auto-aim UI if it doesn't have auto-aim?
        register_input("/tf", tf_, true);
        register_input("/auto_aim/robot_center", robot_center_, true);
        register_input("/auto_aim/should_shoot", should_shoot_, true);
        register_input("/auto_aim/single_shoot", single_shoot_, true);
    }

    void update() override {
        const auto type = *single_shoot_ ? "RUNE" : "ARMOR";

        if (!robot_center_->allFinite() || robot_center_->isZero()) {
            set_distance_text(type, std::nullopt);
            hide_all();
            return;
        }

        const auto point = reproject(*robot_center_);
        if (!point.allFinite()                        //
            || point.x() >= kScreenW || point.x() < 0 //
            || point.y() >= kScreenH || point.y() < 0 //
        ) {
            set_distance_text(type, std::nullopt);
            hide_all();
            return;
        }

        const auto x = std::clamp<std::uint16_t>(std::lround(point.x()), 0, kScreenW - 1);
        const auto y = std::clamp<std::uint16_t>(std::lround(point.y()), 0, kScreenH - 1);

        const auto color = *should_shoot_ ? Shape::Color::ORANGE : Shape::Color::GREEN;
        const auto radius = *should_shoot_ ? 10 : 15;

        {
            const auto distance = robot_center_->norm();
            if (!std::isfinite(distance)) {
                set_distance_text(type, std::nullopt);
                hide_all();
                return;
            }

            set_distance_text(type, distance);
        }

        center_ring_.set_color(color);
        center_ring_.set_x(x);
        center_ring_.set_y(y);
        center_ring_.set_r(radius);
        center_ring_.set_visible(true);

        if (*should_shoot_) {
            constexpr int kCrossLen = 16;
            constexpr int kCrossGap = 5;

            constexpr auto clamp_x = [](int x) {
                return std::clamp<std::uint16_t>(x, 0, kScreenW - 1);
            };
            constexpr auto clamp_y = [](int y) {
                return std::clamp<std::uint16_t>(y, 0, kScreenH - 1);
            };

            cross_top_.set_color(color);
            cross_top_.set_x(x);
            cross_top_.set_y(clamp_y(y - kCrossLen));
            cross_top_.set_x2(x);
            cross_top_.set_y2(clamp_y(y - kCrossGap));
            cross_top_.set_visible(true);

            cross_bottom_.set_color(color);
            cross_bottom_.set_x(x);
            cross_bottom_.set_y(clamp_y(y + kCrossGap));
            cross_bottom_.set_x2(x);
            cross_bottom_.set_y2(clamp_y(y + kCrossLen));
            cross_bottom_.set_visible(true);

            cross_left_.set_color(color);
            cross_left_.set_x(clamp_x(x - kCrossLen));
            cross_left_.set_y(y);
            cross_left_.set_x2(clamp_x(x - kCrossGap));
            cross_left_.set_y2(y);
            cross_left_.set_visible(true);

            cross_right_.set_color(color);
            cross_right_.set_x(clamp_x(x + kCrossGap));
            cross_right_.set_y(y);
            cross_right_.set_x2(clamp_x(x + kCrossLen));
            cross_right_.set_y2(y);
            cross_right_.set_visible(true);
        } else {
            cross_top_.set_visible(false);
            cross_bottom_.set_visible(false);
            cross_left_.set_visible(false);
            cross_right_.set_visible(false);
        }
    }

private:
    static constexpr std::uint16_t kScreenW = 1920;
    static constexpr std::uint16_t kScreenH = 1080;
    static constexpr size_t kMaxTextLength = 30;

    static constexpr double kFx = 730.7267062695;
    static constexpr double kFy = 730.5886055073;
    static constexpr double kCx = 961.8345772991;
    static constexpr double kCy = 549.3357457590;

    static constexpr double kK1 = -0.1764932263;
    static constexpr double kK2 = +0.1582678597;
    static constexpr double kK3 = -0.1557993057;
    static constexpr double kP1 = -0.0000433367;
    static constexpr double kP2 = +0.0003937540;

    Eigen::Vector3d offset_ = Eigen::Vector3d::Zero();

    InputInterface<Tf> tf_;
    InputInterface<Eigen::Vector3d> robot_center_;
    InputInterface<bool> should_shoot_;
    InputInterface<bool> single_shoot_;

    Circle center_ring_{Shape::Color::GREEN, 2, 0, 0, 5, 5, false};
    Line cross_top_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false};
    Line cross_bottom_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false};
    Line cross_left_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false};
    Line cross_right_{Shape::Color::GREEN, 2, 0, 0, 0, 0, false};

    std::string target_distance_text_{"unknown"};
    Text target_distance_indicator_{
        Shape::Color::GREEN, 15, 2, kScreenW / 2 + 34, kScreenH / 2 + 24, "unknown", false,
    };

    void hide_all() {
        center_ring_.set_visible(false);
        cross_top_.set_visible(false);
        cross_bottom_.set_visible(false);
        cross_left_.set_visible(false);
        cross_right_.set_visible(false);
    }

    void set_distance_text(const char* type, std::optional<double> distance) {
        auto& text = target_distance_text_;
        text.resize(kMaxTextLength);
        std::ranges::fill(text, ' ');

        if (distance)
            std::format_to(std::ranges::begin(text), "{} | {:.1f}m\0", type, *distance);
        else
            std::format_to(std::ranges::begin(text), "{} | NONE\0", type);

        target_distance_indicator_.set_value(text.data());
        target_distance_indicator_.set_visible(true);
    }

    Eigen::Vector2d reproject(const Eigen::Vector3d& center) const {
        const auto camera_pose = fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_);

        const auto q_aa = Eigen::Quaterniond{camera_pose.rotation()};
        const auto t_aa = Eigen::Vector3d{camera_pose.translation()};

        const auto t_ui = Eigen::Vector3d{t_aa - q_aa * offset_};

        const auto point_ros = Eigen::Vector3d{q_aa.inverse() * (center - t_ui)};

        const double x_cv = -point_ros.y();
        const double y_cv = +point_ros.z();
        const double z_cv = +point_ros.x();

        if (z_cv <= 1e-6)
            return Eigen::Vector2d{
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
            };

        const double x_norm = x_cv / z_cv;
        const double y_norm = y_cv / z_cv;

        const double r2 = x_norm * x_norm + y_norm * y_norm;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;

        const double radial = 1.0 + kK1 * r2 + kK2 * r4 + kK3 * r6;
        const double x_dist =
            x_norm * radial + 2.0 * kP1 * x_norm * y_norm + kP2 * (r2 + 2.0 * x_norm * x_norm);
        const double y_dist =
            y_norm * radial + kP1 * (r2 + 2.0 * y_norm * y_norm) + 2.0 * kP2 * x_norm * y_norm;

        return Eigen::Vector2d{kFx * x_dist + kCx, kFy * y_dist + kCy};
    }
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::AutoAimUi, rmcs_executor::Component)
