#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class PitchHud {
public:
    struct Config {
        uint16_t center_x = x_center;
        uint16_t center_y = y_center;
        uint16_t radius_px = 150;
        uint16_t width_px = 12;
        double half_span_deg = 22.0;
        double warning_pitch_deg = 5.0;
    };

    PitchHud() { set_config(Config{}); }

    explicit PitchHud(Config config) { set_config(config); }

    void set_config(Config config) {
        config.radius_px = std::max<uint16_t>(config.radius_px, 1);
        config.width_px = std::max<uint16_t>(config.width_px, 1);
        config.half_span_deg = std::max(config.half_span_deg, 1.0);
        config.warning_pitch_deg = std::max(config.warning_pitch_deg, 0.0);
        config_ = config;
        initialize_();
    }

    void set_visible(bool visible) {
        background_arc_.set_visible(visible);
        positive_fill_arc_.set_visible(visible && positive_fill_visible_);
        negative_fill_arc_.set_visible(visible && negative_fill_visible_);
    }

    void update(double pitch_rad, double reveal = 1.0) {
        reveal = std::clamp(reveal, 0.0, 1.0);
        if (!std::isfinite(pitch_rad) || reveal <= 1.0e-4) {
            positive_fill_visible_ = false;
            negative_fill_visible_ = false;
            set_visible(false);
            return;
        }

        apply_geometry_(reveal);

        const double visible_half_span_deg = config_.half_span_deg * reveal;
        const double pitch_deg =
            std::clamp(rad_to_deg_(pitch_rad), -visible_half_span_deg, visible_half_span_deg);
        const double fill_deg = std::abs(pitch_deg);

        const bool warning = pitch_deg > config_.warning_pitch_deg;
        const auto fill_color = warning ? Shape::Color::PINK : Shape::Color::YELLOW;
        positive_fill_arc_.set_color(fill_color);
        negative_fill_arc_.set_color(fill_color);

        positive_fill_visible_ = pitch_deg >= 0.0 && fill_deg > 1.0e-4;
        negative_fill_visible_ = pitch_deg < 0.0 && fill_deg > 1.0e-4;

        positive_fill_arc_.set_angle_start(to_referee_angle_(pitch_deg));
        positive_fill_arc_.set_angle_end(mid_angle_());

        negative_fill_arc_.set_angle_start(mid_angle_());
        negative_fill_arc_.set_angle_end(to_referee_angle_(pitch_deg));

        set_visible(true);
    }

private:
    void initialize_() {
        background_arc_.set_color(Shape::Color::ORANGE);
        positive_fill_arc_.set_color(Shape::Color::YELLOW);
        negative_fill_arc_.set_color(Shape::Color::YELLOW);
        positive_fill_visible_ = false;
        negative_fill_visible_ = false;
        set_visible(false);
    }

    void apply_geometry_(double reveal) {
        const uint16_t width = std::max<uint16_t>(1, static_cast<uint16_t>(std::lround(
            static_cast<double>(config_.width_px) * reveal)));
        const double half_span_deg = config_.half_span_deg * reveal;

        background_arc_.set_x(config_.center_x);
        background_arc_.set_y(config_.center_y);
        background_arc_.set_r(config_.radius_px);
        background_arc_.set_width(width);
        background_arc_.set_angle_start(to_referee_angle_(-half_span_deg));
        background_arc_.set_angle_end(to_referee_angle_(half_span_deg));

        positive_fill_arc_.set_x(config_.center_x);
        positive_fill_arc_.set_y(config_.center_y);
        positive_fill_arc_.set_r(config_.radius_px);
        positive_fill_arc_.set_width(width);

        negative_fill_arc_.set_x(config_.center_x);
        negative_fill_arc_.set_y(config_.center_y);
        negative_fill_arc_.set_r(config_.radius_px);
        negative_fill_arc_.set_width(width);
    }

    static constexpr double rad_to_deg_(double radians) {
        return radians * 180.0 / 3.14159265358979323846;
    }

    static uint16_t wrap_angle_(long angle) {
        angle %= 360;
        if (angle < 0)
            angle += 360;
        return static_cast<uint16_t>(angle);
    }

    static constexpr uint16_t mid_angle_() { return 180; }

    static uint16_t to_referee_angle_(double display_pitch_deg) {
        return wrap_angle_(std::lround(static_cast<double>(mid_angle_()) + display_pitch_deg));
    }

    Config config_{};
    Arc background_arc_{Shape::Color::WHITE, 1, 0, 0, 0, 0, 1, 1, false};
    Arc positive_fill_arc_{Shape::Color::YELLOW, 1, 0, 0, 0, 0, 1, 1, false};
    Arc negative_fill_arc_{Shape::Color::YELLOW, 1, 0, 0, 0, 0, 1, 1, false};
    bool positive_fill_visible_ = false;
    bool negative_fill_visible_ = false;
};

} // namespace rmcs_core::referee::app::ui
