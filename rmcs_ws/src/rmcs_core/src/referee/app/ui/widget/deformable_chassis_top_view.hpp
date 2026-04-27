#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <utility>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class DeformableChassisTopView {
public:
    DeformableChassisTopView() {
        chassis_ring_.set_x(center_x_);
        chassis_ring_.set_y(center_y_);
        chassis_ring_.set_r(chassis_ring_radius_);
        chassis_ring_.set_width(chassis_ring_width_);
        chassis_ring_.set_color(Shape::Color::WHITE);
        chassis_ring_.set_visible(false);

        chassis_heading_.set_x(center_x_);
        chassis_heading_.set_y(center_y_);
        chassis_heading_.set_r(chassis_ring_radius_);
        chassis_heading_.set_width(chassis_heading_width_);
        chassis_heading_.set_color(Shape::Color::PINK);
        chassis_heading_.set_visible(false);

        center_dot_.set_x(center_x_);
        center_dot_.set_y(center_y_);
        center_dot_.set_r(center_dot_radius_);
        center_dot_.set_width(center_dot_width_);
        center_dot_.set_color(Shape::Color::WHITE);
        center_dot_.set_visible(false);

        for (auto& leg : legs_) {
            leg.set_width(leg_width_);
            leg.set_visible(false);
        }
    }

    void set_angle_range(double min_angle_deg, double max_angle_deg) {
        min_angle_rad_ = deg_to_rad_(min_angle_deg);
        max_angle_rad_ = deg_to_rad_(max_angle_deg);
        if (min_angle_rad_ > max_angle_rad_)
            std::swap(min_angle_rad_, max_angle_rad_);
    }

    void update(double chassis_angle, const std::array<double, 4>& leg_angles) {
        if (!std::isfinite(chassis_angle) || !valid_angle_range_()) {
            set_visible(false);
            return;
        }

        for (double leg_angle : leg_angles) {
            if (!std::isfinite(leg_angle)) {
                set_visible(false);
                return;
            }
        }

        set_visible(true);
        chassis_heading_.set_angle(to_referee_angle_(chassis_angle), chassis_heading_half_angle_);

        for (std::size_t i = 0; i < legs_.size(); ++i) {
            update_leg_(legs_[i], chassis_angle + leg_base_angles_[i], leg_angles[i]);
        }
    }

    void set_visible(bool value) {
        chassis_ring_.set_visible(value);
        chassis_heading_.set_visible(value);
        center_dot_.set_visible(value);

        for (auto& leg : legs_)
            leg.set_visible(value);
    }

private:
    static constexpr uint16_t screen_width_ = 1920;
    static constexpr uint16_t screen_height_ = 1080;

    static constexpr uint16_t center_x_ = 1650;
    static constexpr uint16_t center_y_ = 700;

    static constexpr uint16_t chassis_ring_radius_ = 50;
    static constexpr uint16_t chassis_ring_width_ = 3;
    static constexpr uint16_t chassis_heading_width_ = 8;
    static constexpr uint16_t chassis_heading_half_angle_ = 30;

    static constexpr uint16_t center_dot_radius_ = 1;
    static constexpr uint16_t center_dot_width_ = 8;

    static constexpr uint16_t leg_root_radius_ = 58;
    static constexpr uint16_t leg_min_length_ = 30;
    static constexpr uint16_t leg_max_length_ = 82;
    static constexpr uint16_t leg_width_ = 10;

    static constexpr std::array<double, 4> leg_base_angles_ = {
        std::numbers::pi_v<double> / 4.0,
        3.0 * std::numbers::pi_v<double> / 4.0,
        5.0 * std::numbers::pi_v<double> / 4.0,
        7.0 * std::numbers::pi_v<double> / 4.0,
    };

    static constexpr double default_min_angle_deg_ = 8.0;
    static constexpr double default_max_angle_deg_ = 58.0;

    static double deg_to_rad_(double degrees) {
        return degrees / 180.0 * std::numbers::pi_v<double>;
    }

    static uint16_t clamp_coordinate_(double value, uint16_t max_value) {
        const auto rounded = static_cast<int>(std::lround(value));
        return static_cast<uint16_t>(std::clamp(rounded, 0, static_cast<int>(max_value - 1)));
    }

    static uint16_t to_referee_angle_(double angle) {
        const auto degrees =
            static_cast<int>(std::lround((2.0 * std::numbers::pi_v<double> - angle)
                                         / std::numbers::pi_v<double> * 180.0));
        int wrapped = degrees % 360;
        if (wrapped < 0)
            wrapped += 360;
        return static_cast<uint16_t>(wrapped);
    }

    static std::pair<uint16_t, uint16_t> polar_to_screen_(double angle, double radius) {
        return {
            clamp_coordinate_(center_x_ + std::cos(angle) * radius, screen_width_),
            clamp_coordinate_(center_y_ - std::sin(angle) * radius, screen_height_),
        };
    }

    bool valid_angle_range_() const { return max_angle_rad_ - min_angle_rad_ > 1e-6; }

    double normalized_leg_extension_(double leg_angle) const {
        return std::clamp(
            (leg_angle - min_angle_rad_) / (max_angle_rad_ - min_angle_rad_), 0.0, 1.0);
    }

    static Shape::Color leg_color_(double normalized_extension) {
        if (normalized_extension < 1.0 / 3.0)
            return Shape::Color::PINK;
        if (normalized_extension < 2.0 / 3.0)
            return Shape::Color::YELLOW;
        return Shape::Color::GREEN;
    }

    void update_leg_(Line& leg, double body_angle, double leg_angle) const {
        const double normalized_extension = normalized_leg_extension_(leg_angle);
        const double leg_length = leg_min_length_
                                + normalized_extension * (leg_max_length_ - leg_min_length_);

        const auto [x1, y1] = polar_to_screen_(body_angle, leg_root_radius_);
        const auto [x2, y2] = polar_to_screen_(body_angle, leg_root_radius_ + leg_length);

        leg.set_color(leg_color_(normalized_extension));
        leg.set_x(x1);
        leg.set_y(y1);
        leg.set_x2(x2);
        leg.set_y2(y2);
    }

    double min_angle_rad_ = deg_to_rad_(default_min_angle_deg_);
    double max_angle_rad_ = deg_to_rad_(default_max_angle_deg_);

    Circle chassis_ring_;
    Arc chassis_heading_;
    Circle center_dot_;
    std::array<Line, 4> legs_;
};

} // namespace rmcs_core::referee::app::ui
