#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class PitchHud {
public:
    struct Config {
        uint16_t center_x = 1540;
        uint16_t center_y = y_center;
        uint16_t half_height_px = 180;
        double half_span_deg = 30.0;
        double tick_step_deg = 5.0;
    };

    PitchHud() { set_config(Config{}); }

    explicit PitchHud(Config config) { set_config(config); }

    void set_config(Config config) {
        config.tick_step_deg = std::max(config.tick_step_deg, 1.0);
        config.half_span_deg = std::max(config.half_span_deg, config.tick_step_deg);
        config.half_height_px = std::max<uint16_t>(config.half_height_px, 40);
        config_ = config;
        initialize_();
    }

    void update(double gimbal_pitch, double chassis_pitch) {
        if (std::isfinite(gimbal_pitch))
            update_gimbal_pitch_indicator_(pitch_to_hud_y_(gimbal_pitch));
        else
            set_indicator_visible_(gimbal_pitch_indicator_, false);

        if (std::isfinite(chassis_pitch))
            update_chassis_pitch_indicator_(pitch_to_hud_y_(chassis_pitch));
        else
            set_indicator_visible_(chassis_pitch_indicator_, false);
    }

private:
    static constexpr std::size_t tick_capacity_ = 25;

    void initialize_() {
        tick_count_ = std::clamp(
            static_cast<std::size_t>(
                std::floor(2.0 * config_.half_span_deg / config_.tick_step_deg + 1.0e-6))
                + 1,
            std::size_t{1}, tick_capacity_);

        axis_.set_color(Shape::Color::YELLOW);
        axis_.set_width(2);
        axis_.set_x(config_.center_x);
        axis_.set_y(axis_top_y_());
        axis_.set_x2(config_.center_x);
        axis_.set_y2(axis_bottom_y_());
        axis_.set_visible(true);

        for (std::size_t i = 0; i < ticks_.size(); ++i) {
            auto& tick = ticks_[i];
            if (i >= tick_count_) {
                tick.set_visible(false);
                continue;
            }

            const double tick_deg = -config_.half_span_deg + static_cast<double>(i) * config_.tick_step_deg;
            const int rounded_tick_deg = static_cast<int>(std::lround(tick_deg));
            if (rounded_tick_deg == 0) {
                tick.set_visible(false);
                continue;
            }

            const bool is_major = (std::abs(rounded_tick_deg) % 10 == 0);
            const uint16_t tick_length = is_major ? 18 : 10;
            const uint16_t tick_y = pitch_to_hud_y_(deg_to_rad_(tick_deg));

            tick.set_color(Shape::Color::YELLOW);
            tick.set_width(2);
            tick.set_x(config_.center_x);
            tick.set_y(tick_y);
            tick.set_x2(config_.center_x + tick_length);
            tick.set_y2(tick_y);
            tick.set_visible(true);
        }

        for (auto& line : gimbal_pitch_indicator_) {
            line.set_color(Shape::Color::YELLOW);
            line.set_width(2);
            line.set_visible(false);
        }
        for (auto& line : chassis_pitch_indicator_) {
            line.set_color(Shape::Color::YELLOW);
            line.set_width(2);
            line.set_visible(false);
        }
    }

    uint16_t axis_top_y_() const {
        return clamp_to_screen_y_(
            static_cast<double>(config_.center_y) - static_cast<double>(config_.half_height_px));
    }

    uint16_t axis_bottom_y_() const {
        return clamp_to_screen_y_(
            static_cast<double>(config_.center_y) + static_cast<double>(config_.half_height_px));
    }

    static constexpr double deg_to_rad_(double degrees) {
        return degrees * std::numbers::pi / 180.0;
    }

    static uint16_t clamp_to_screen_y_(double y) {
        return static_cast<uint16_t>(std::clamp(
            std::lround(y), 0l, static_cast<long>(screen_height - 1)));
    }

    uint16_t pitch_to_hud_y_(double pitch_rad) const {
        const double clamped_pitch =
            std::clamp(pitch_rad, -deg_to_rad_(config_.half_span_deg),
                deg_to_rad_(config_.half_span_deg));
        const double normalized = clamped_pitch / deg_to_rad_(config_.half_span_deg);
        return clamp_to_screen_y_(
            static_cast<double>(config_.center_y)
            + normalized * static_cast<double>(config_.half_height_px));
    }

    void update_gimbal_pitch_indicator_(uint16_t y) {
        const uint16_t tip_x = config_.center_x - 8;
        const uint16_t back_x = config_.center_x - 20;
        constexpr uint16_t half_height = 8;
        const uint16_t top_y = clamp_to_screen_y_(static_cast<double>(y) - half_height);
        const uint16_t bottom_y = clamp_to_screen_y_(static_cast<double>(y) + half_height);

        gimbal_pitch_indicator_[0].set_x(back_x);
        gimbal_pitch_indicator_[0].set_y(top_y);
        gimbal_pitch_indicator_[0].set_x2(tip_x);
        gimbal_pitch_indicator_[0].set_y2(y);

        gimbal_pitch_indicator_[1].set_x(back_x);
        gimbal_pitch_indicator_[1].set_y(bottom_y);
        gimbal_pitch_indicator_[1].set_x2(tip_x);
        gimbal_pitch_indicator_[1].set_y2(y);

        gimbal_pitch_indicator_[2].set_x(back_x);
        gimbal_pitch_indicator_[2].set_y(top_y);
        gimbal_pitch_indicator_[2].set_x2(back_x);
        gimbal_pitch_indicator_[2].set_y2(bottom_y);

        set_indicator_visible_(gimbal_pitch_indicator_, true);
    }

    void update_chassis_pitch_indicator_(uint16_t y) {
        const uint16_t front_x = config_.center_x - 20;
        const uint16_t rear_x = config_.center_x - 34;
        constexpr uint16_t front_half_height = 12;
        constexpr uint16_t rear_half_height = 8;
        const uint16_t front_top_y = clamp_to_screen_y_(static_cast<double>(y) - front_half_height);
        const uint16_t front_bottom_y =
            clamp_to_screen_y_(static_cast<double>(y) + front_half_height);
        const uint16_t rear_top_y = clamp_to_screen_y_(static_cast<double>(y) - rear_half_height);
        const uint16_t rear_bottom_y =
            clamp_to_screen_y_(static_cast<double>(y) + rear_half_height);

        chassis_pitch_indicator_[0].set_x(rear_x);
        chassis_pitch_indicator_[0].set_y(rear_top_y);
        chassis_pitch_indicator_[0].set_x2(front_x);
        chassis_pitch_indicator_[0].set_y2(front_top_y);

        chassis_pitch_indicator_[1].set_x(front_x);
        chassis_pitch_indicator_[1].set_y(front_top_y);
        chassis_pitch_indicator_[1].set_x2(front_x);
        chassis_pitch_indicator_[1].set_y2(front_bottom_y);

        chassis_pitch_indicator_[2].set_x(front_x);
        chassis_pitch_indicator_[2].set_y(front_bottom_y);
        chassis_pitch_indicator_[2].set_x2(rear_x);
        chassis_pitch_indicator_[2].set_y2(rear_bottom_y);

        chassis_pitch_indicator_[3].set_x(rear_x);
        chassis_pitch_indicator_[3].set_y(rear_bottom_y);
        chassis_pitch_indicator_[3].set_x2(rear_x);
        chassis_pitch_indicator_[3].set_y2(rear_top_y);

        set_indicator_visible_(chassis_pitch_indicator_, true);
    }

    template <std::size_t N>
    static void set_indicator_visible_(std::array<Line, N>& indicator, bool visible) {
        for (auto& line : indicator)
            line.set_visible(visible);
    }

    Config config_{};
    std::size_t tick_count_ = 0;
    Line axis_;
    std::array<Line, tick_capacity_> ticks_;
    std::array<Line, 3> gimbal_pitch_indicator_;
    std::array<Line, 4> chassis_pitch_indicator_;
};

} // namespace rmcs_core::referee::app::ui
