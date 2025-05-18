#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <bit>
#include <tuple>

#include <rmcs_msgs/robot_color.hpp>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class StatusRing {
public:
    StatusRing(
        double supercap_limit, double battery_limit, double friction_limit, int16_t bullet_limit) {
        supercap_status_.set_x(x_center);
        supercap_status_.set_y(y_center);
        supercap_status_.set_r(visible_radius - width_ring);
        supercap_status_.set_angle_start(275);
        supercap_status_.set_angle_end(275 + visible_angle);
        supercap_status_.set_width(width_ring);
        supercap_status_.set_color(Shape::Color::PINK);
        supercap_status_.set_visible(true);

        battery_status_.set_x(x_center);
        battery_status_.set_y(y_center);
        battery_status_.set_r(visible_radius - width_ring);
        battery_status_.set_angle_start(265 - visible_angle);
        battery_status_.set_angle_end(265);
        battery_status_.set_width(width_ring);
        battery_status_.set_color(Shape::Color::PINK);
        battery_status_.set_visible(true);

        friction_wheel_speed_.set_x(x_center);
        friction_wheel_speed_.set_y(y_center);
        friction_wheel_speed_.set_r(visible_radius - width_ring);
        friction_wheel_speed_.set_angle_start(85 - visible_angle);
        friction_wheel_speed_.set_angle_end(85);
        friction_wheel_speed_.set_width(width_ring);
        friction_wheel_speed_.set_color(Shape::Color::PINK);
        friction_wheel_speed_.set_visible(true);

        bullet_status_.set_x(x_center);
        bullet_status_.set_y(y_center);
        bullet_status_.set_r(visible_radius - width_ring);
        bullet_status_.set_angle_start(95);
        bullet_status_.set_angle_end(95 + visible_angle);
        bullet_status_.set_width(width_ring);
        bullet_status_.set_color(Shape::Color::PINK);
        bullet_status_.set_visible(true);

        // UI
        line_left_center_.set_x(x_center - visible_radius);
        line_left_center_.set_x2(x_center - visible_radius + width_ring + 10);
        line_left_center_.set_y(y_center);
        line_left_center_.set_y2(y_center);
        line_left_center_.set_width(40);
        line_left_center_.set_color(Shape::Color::WHITE);
        line_left_center_.set_visible(true);

        line_right_center_.set_x(x_center + visible_radius);
        line_right_center_.set_x2(x_center + visible_radius - width_ring - 10);
        line_right_center_.set_y(y_center);
        line_right_center_.set_y2(y_center);
        line_right_center_.set_width(40);
        line_right_center_.set_color(Shape::Color::WHITE);
        line_right_center_.set_visible(true);

        arc_left_up_.set_x(x_center);
        arc_left_up_.set_y(y_center);
        arc_left_up_.set_r(visible_radius - width_ring);
        arc_left_up_.set_angle_start(275 + visible_angle + 1);
        arc_left_up_.set_angle_end(275 + visible_angle + 3);
        arc_left_up_.set_width(width_ring + 10);
        arc_left_up_.set_color(Shape::Color::WHITE);
        arc_left_up_.set_visible(true);

        arc_left_down_.set_x(x_center);
        arc_left_down_.set_y(y_center);
        arc_left_down_.set_r(visible_radius - width_ring);
        arc_left_down_.set_angle_start(265 - visible_angle - 3);
        arc_left_down_.set_angle_end(265 - visible_angle - 1);
        arc_left_down_.set_width(width_ring + 10);
        arc_left_down_.set_color(Shape::Color::WHITE);
        arc_left_down_.set_visible(true);

        arc_right_up_.set_x(x_center);
        arc_right_up_.set_y(y_center);
        arc_right_up_.set_r(visible_radius - width_ring);
        arc_right_up_.set_angle_start(85 - visible_angle - 3);
        arc_right_up_.set_angle_end(85 - visible_angle - 1);
        arc_right_up_.set_width(width_ring + 10);
        arc_right_up_.set_color(Shape::Color::WHITE);
        arc_right_up_.set_visible(true);

        arc_right_down_.set_x(x_center);
        arc_right_down_.set_y(y_center);
        arc_right_down_.set_r(visible_radius - width_ring);
        arc_right_down_.set_angle_start(95 + visible_angle + 1);
        arc_right_down_.set_angle_end(95 + visible_angle + 3);
        arc_right_down_.set_width(width_ring + 10);
        arc_right_down_.set_color(Shape::Color::WHITE);
        arc_right_down_.set_visible(true);

        set_limits(supercap_limit, battery_limit, friction_limit, bullet_limit);
    }

    void set_visible(bool value) {
        // Dynamic
        supercap_status_.set_visible(value);
        battery_status_.set_visible(value);
        friction_wheel_speed_.set_visible(value);
        bullet_status_.set_visible(value);

        // Static
        line_left_center_.set_visible(value);
        line_right_center_.set_visible(value);
        arc_left_up_.set_visible(value);
        arc_left_down_.set_visible(value);
        arc_right_up_.set_visible(value);
        arc_right_down_.set_visible(value);

        for (auto& scale : bullet_scales_)
            scale.set_visible(value);
        for (auto& number : bullet_scales_number_)
            number.set_visible(value);
    }

    void update_static_parts(std::tuple<bool, bool> enable) {
        auto& [auto_aim_enable, precise_enable] = enable;
        auto static_enable                      = auto_aim_enable || precise_enable;

        static auto color{Shape::Color::WHITE};

        if (auto_aim_enable) {
            color = Shape::Color::GREEN;
        } else {
            if (precise_enable) {
                color = Shape::Color::CYAN;
            }
        }
        if (!static_enable) {
            color = Shape::Color::WHITE;
        }

        update_static_enable(static_enable, color);
    }

    void update_static_enable(bool enable, Shape::Color color) {
        static bool enable_last_{false};

        arc_left_up_.set_color(color);
        arc_left_down_.set_color(color);
        arc_right_up_.set_color(color);
        arc_right_down_.set_color(color);
        if (enable == enable_last_)
            return;

        if (enable) {
            arc_left_up_.set_width(width_ring + 50);
            arc_left_down_.set_width(width_ring + 50);
            arc_right_up_.set_width(width_ring + 50);
            arc_right_down_.set_width(width_ring + 50);

            arc_left_up_.set_angle_start(275 + visible_angle + 1);
            arc_left_up_.set_angle_end(275 + visible_angle + 2);
            arc_left_down_.set_angle_start(265 - visible_angle - 2);
            arc_left_down_.set_angle_end(265 - visible_angle - 1);
            arc_right_up_.set_angle_start(85 - visible_angle - 2);
            arc_right_up_.set_angle_end(85 - visible_angle - 1);
            arc_right_down_.set_angle_start(95 + visible_angle + 1);
            arc_right_down_.set_angle_end(95 + visible_angle + 2);

        } else {
            arc_left_up_.set_width(width_ring + 10);
            arc_left_down_.set_width(width_ring + 10);
            arc_right_up_.set_width(width_ring + 10);
            arc_right_down_.set_width(width_ring + 10);

            arc_left_up_.set_angle_start(275 + visible_angle + 1);
            arc_left_up_.set_angle_end(275 + visible_angle + 3);
            arc_left_down_.set_angle_start(265 - visible_angle - 3);
            arc_left_down_.set_angle_end(265 - visible_angle - 1);
            arc_right_up_.set_angle_start(85 - visible_angle - 3);
            arc_right_up_.set_angle_end(85 - visible_angle - 1);
            arc_right_down_.set_angle_start(95 + visible_angle + 1);
            arc_right_down_.set_angle_end(95 + visible_angle + 3);
        }

        enable_last_ = enable;
    }

    void update_auto_aim_enable(bool enable) {
        static bool enable_last_{false};

        if (enable == enable_last_)
            return;

        if (enable) {
            arc_left_up_.set_color(Shape::Color::GREEN);
            arc_left_down_.set_color(Shape::Color::GREEN);
            arc_right_up_.set_color(Shape::Color::GREEN);
            arc_right_down_.set_color(Shape::Color::GREEN);

            arc_left_up_.set_width(width_ring + 50);
            arc_left_down_.set_width(width_ring + 50);
            arc_right_up_.set_width(width_ring + 50);
            arc_right_down_.set_width(width_ring + 50);

            arc_left_up_.set_angle_start(275 + visible_angle + 1);
            arc_left_up_.set_angle_end(275 + visible_angle + 2);
            arc_left_down_.set_angle_start(265 - visible_angle - 2);
            arc_left_down_.set_angle_end(265 - visible_angle - 1);
            arc_right_up_.set_angle_start(85 - visible_angle - 2);
            arc_right_up_.set_angle_end(85 - visible_angle - 1);
            arc_right_down_.set_angle_start(95 + visible_angle + 1);
            arc_right_down_.set_angle_end(95 + visible_angle + 2);

        } else {
            arc_left_up_.set_color(Shape::Color::WHITE);
            arc_left_down_.set_color(Shape::Color::WHITE);
            arc_right_up_.set_color(Shape::Color::WHITE);
            arc_right_down_.set_color(Shape::Color::WHITE);

            arc_left_up_.set_width(width_ring + 10);
            arc_left_down_.set_width(width_ring + 10);
            arc_right_up_.set_width(width_ring + 10);
            arc_right_down_.set_width(width_ring + 10);

            arc_left_up_.set_angle_start(275 + visible_angle + 1);
            arc_left_up_.set_angle_end(275 + visible_angle + 3);
            arc_left_down_.set_angle_start(265 - visible_angle - 3);
            arc_left_down_.set_angle_end(265 - visible_angle - 1);
            arc_right_up_.set_angle_start(85 - visible_angle - 3);
            arc_right_up_.set_angle_end(85 - visible_angle - 1);
            arc_right_down_.set_angle_start(95 + visible_angle + 1);
            arc_right_down_.set_angle_end(95 + visible_angle + 3);
        }

        enable_last_ = enable;
    }

    void update_supercap(double value, bool enable) {
        auto angle = 275 + calculate_angle(value, 10.5, supercap_limit_) + 1;
        supercap_status_.set_angle_end(static_cast<uint16_t>(angle));

        if (value > 22.6) {
            supercap_status_.set_color(enable ? Shape::Color::CYAN : Shape::Color::GREEN);
        } else if (value > 13.5) {
            supercap_status_.set_color(enable ? Shape::Color::YELLOW : Shape::Color::ORANGE);
        } else {
            supercap_status_.set_color(enable ? Shape::Color::PURPLE : Shape::Color::PINK);
        }
    }

    void update_battery_power(double value) {
        auto angle = 265 - calculate_angle(value, 20, 25.7) - 1;
        battery_status_.set_angle_start(static_cast<uint16_t>(angle));

        if (value > 23.18) {
            battery_status_.set_color(Shape::Color::GREEN);
        } else if (value > 22.05) {
            battery_status_.set_color(Shape::Color::YELLOW);
        } else if (value > 21.40) {
            battery_status_.set_color(Shape::Color::ORANGE);
        } else {
            battery_status_.set_color(Shape::Color::PINK);
        }
    }

    void update_friction_wheel_speed(double value, bool enable) {
        auto angle = 85 - calculate_angle(value, 0, friction_limit_) - 1;
        friction_wheel_speed_.set_angle_start(static_cast<uint16_t>(angle));

        if (enable) {
            friction_wheel_speed_.set_color(Shape::Color::GREEN);
        } else {
            friction_wheel_speed_.set_color(Shape::Color::PINK);
        }
    }

    void update_bullet_allowance(uint16_t value) {
        auto allowance = std::bit_cast<int16_t>(value);

        // limit ring
        auto angle = 95 + calculate_angle(allowance, 0, bullet_limit_) + 1;
        bullet_status_.set_angle_end(static_cast<uint16_t>(angle));

        if (allowance < bullet_limit_ / 8) {
            bullet_status_.set_color(Shape::Color::PINK);
        } else if (allowance < bullet_limit_ / 4) {
            bullet_status_.set_color(Shape::Color::ORANGE);
        } else {
            bullet_status_.set_color(Shape::Color::GREEN);
        }
    }

private:
    static constexpr double calculate_angle(double value, double min, double max) {
        return visible_angle * std::clamp(value - min, 0.0, max - min) / (max - min);
    }

    void set_limits(
        double supercap_limit, double battery_limit, double friction_limit, int16_t bullet_limit) {
        supercap_limit_ = supercap_limit;
        battery_limit_  = battery_limit;
        friction_limit_ = friction_limit;
        bullet_limit_   = bullet_limit;

        int scale_angle = 5;
        for (auto& bullet_scale : bullet_scales_) {

            scale_angle += (visible_angle) / 4;

            bullet_scale.set_x(x_center);
            bullet_scale.set_y(y_center);
            bullet_scale.set_r(visible_radius - width_ring - 30);
            bullet_scale.set_angle_start(90 + scale_angle - 1);
            bullet_scale.set_angle_end(90 + scale_angle);
            bullet_scale.set_width(width_ring + 10);
            bullet_scale.set_color(Shape::Color::WHITE);
            bullet_scale.set_visible(true);
        }

        double value = 0;
        scale_angle  = 5;
        for (auto& number : bullet_scales_number_) {

            scale_angle += (visible_angle) / 4;
            value += static_cast<double>(bullet_limit_) / 4;

            const auto r     = visible_radius - width_ring + 30;
            const auto angle = static_cast<double>(-scale_angle) * std::numbers::pi / 180;

            number.set_x(x_center + static_cast<uint16_t>(r * std::cos(angle)));
            number.set_y(y_center + static_cast<uint16_t>(r * std::sin(angle)));
            number.set_color(Shape::Color::WHITE);
            number.set_font_size(15);
            number.set_width(2);
            number.set_value(static_cast<uint16_t>(value));
            number.set_visible(true);
        }
    }

    constexpr static uint16_t x_center       = 960;
    constexpr static uint16_t y_center       = 540;
    constexpr static uint16_t width_ring     = 15;
    constexpr static uint16_t visible_radius = 400;
    constexpr static uint16_t visible_angle  = 40;

    double supercap_limit_;
    double battery_limit_;
    double friction_limit_;
    int16_t bullet_limit_;

    // Dynamic part
    Arc supercap_status_;

    Arc battery_status_;

    Arc friction_wheel_speed_;

    Arc bullet_status_;

    // Static part
    Line line_left_center_;
    Line line_right_center_;
    Arc arc_left_up_;
    Arc arc_left_down_;
    Arc arc_right_up_;
    Arc arc_right_down_;
    Integer bullet_scales_number_[4];
    Arc bullet_scales_[4];
};

} // namespace rmcs_core::referee::app::ui