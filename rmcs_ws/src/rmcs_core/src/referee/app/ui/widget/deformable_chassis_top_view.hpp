#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class DeformableChassisLegArcs {
public:
    DeformableChassisLegArcs() = default;

    void set_angle_range(double min_angle_deg, double max_angle_deg) {
        min_angle_rad_ = deg_to_rad_(min_angle_deg);
        max_angle_rad_ = deg_to_rad_(max_angle_deg);
        if (min_angle_rad_ > max_angle_rad_)
            std::swap(min_angle_rad_, max_angle_rad_);
    }

    void update(double chassis_angle, const std::array<double, 4>& leg_angles) {
        if (!valid_angle_range_()) {
            set_visible(false);
            return;
        }

        if (std::isfinite(chassis_angle))
            last_chassis_angle_ = chassis_angle;

        set_visible(true);
        for (std::size_t i = 0; i < legs_.size(); ++i) {
            if (std::isfinite(leg_angles[i]))
                last_leg_angles_[i] = leg_angles[i];
            update_leg_(
                legs_[i], last_chassis_angle_ + leg_base_mid_angles_[i], leg_radii_near_[i],
                leg_radii_far_[i], last_leg_angles_[i]);
        }
    }

    void set_visible(bool value) {
        for (auto& leg : legs_)
            leg.set_visible(value);
    }

private:
    static constexpr uint16_t center_x_ = 1920 / 2;
    static constexpr uint16_t center_y_ = 1080 / 2;

    static constexpr uint16_t front_leg_radius_near_ = 102;
    static constexpr uint16_t rear_leg_radius_near_ = 112;
    static constexpr uint16_t front_leg_radius_far_ = 132;
    static constexpr uint16_t rear_leg_radius_far_ = 142;

    static constexpr uint16_t prone_leg_width_ = 6;
    static constexpr uint16_t upright_leg_width_ = 16;

    static constexpr uint16_t upright_leg_half_angle_ = 10;
    static constexpr uint16_t prone_leg_half_angle_ = 6;

    static constexpr double front_pair_offset_deg_ = 28.0;
    static constexpr double rear_pair_offset_deg_ = 28.0;
    static constexpr double degrees_to_radians_ = std::numbers::pi_v<double> / 180.0;

    static constexpr double default_min_angle_deg_ = 8.0;
    static constexpr double default_max_angle_deg_ = 58.0;

    static constexpr double deg_to_rad_(double degrees) {
        return degrees * degrees_to_radians_;
    }

    static constexpr std::array<double, 4> leg_base_mid_angles_ = {
        front_pair_offset_deg_ * degrees_to_radians_,
        std::numbers::pi_v<double> - rear_pair_offset_deg_ * degrees_to_radians_,
        std::numbers::pi_v<double> + rear_pair_offset_deg_ * degrees_to_radians_,
        -front_pair_offset_deg_ * degrees_to_radians_,
    };

    static constexpr std::array<uint16_t, 4> leg_radii_near_ = {
        front_leg_radius_near_,
        rear_leg_radius_near_,
        rear_leg_radius_near_,
        front_leg_radius_near_,
    };

    static constexpr std::array<uint16_t, 4> leg_radii_far_ = {
        front_leg_radius_far_,
        rear_leg_radius_far_,
        rear_leg_radius_far_,
        front_leg_radius_far_,
    };

    static uint16_t to_referee_angle_(double angle) {
        const auto degrees =
            static_cast<int>(std::lround((2.0 * std::numbers::pi_v<double> - angle)
                                         / std::numbers::pi_v<double> * 180.0));
        int wrapped = degrees % 360;
        if (wrapped < 0)
            wrapped += 360;
        return static_cast<uint16_t>(wrapped);
    }

    bool valid_angle_range_() const { return max_angle_rad_ - min_angle_rad_ > 1e-6; }

    double normalized_leg_extension_(double leg_angle) const {
        return std::clamp(
            (leg_angle - min_angle_rad_) / (max_angle_rad_ - min_angle_rad_), 0.0, 1.0);
    }

    static Shape::Color leg_color_(double normalized_extension) {
        if (normalized_extension < 1.0 / 3.0)
            return Shape::Color::ORANGE;
        if (normalized_extension < 2.0 / 3.0)
            return Shape::Color::YELLOW;
        return Shape::Color::WHITE;
    }

    void update_leg_(
        Arc& leg, double body_angle, uint16_t near_radius, uint16_t far_radius,
        double leg_angle) const {
        const double normalized_extension = normalized_leg_extension_(leg_angle);
        // Min angle looks like a thin leg stretching radially outward from the center ring.
        const uint16_t radius = static_cast<uint16_t>(std::lround(
            far_radius - normalized_extension * (far_radius - near_radius)));
        const uint16_t half_angle = static_cast<uint16_t>(std::lround(
            prone_leg_half_angle_
            - normalized_extension * (prone_leg_half_angle_ - upright_leg_half_angle_)));
        const uint16_t width = static_cast<uint16_t>(std::lround(
            prone_leg_width_
            + normalized_extension * (upright_leg_width_ - prone_leg_width_)));

        leg.set_x(center_x_);
        leg.set_y(center_y_);
        leg.set_r(radius);
        leg.set_width(width);
        leg.set_color(leg_color_(normalized_extension));
        leg.set_angle(to_referee_angle_(body_angle), half_angle);
    }

    double min_angle_rad_ = deg_to_rad_(default_min_angle_deg_);
    double max_angle_rad_ = deg_to_rad_(default_max_angle_deg_);
    double last_chassis_angle_ = 0.0;
    std::array<double, 4> last_leg_angles_ = {
        deg_to_rad_(default_max_angle_deg_),
        deg_to_rad_(default_max_angle_deg_),
        deg_to_rad_(default_max_angle_deg_),
        deg_to_rad_(default_max_angle_deg_),
    };

    std::array<Arc, 4> legs_;
};

} // namespace rmcs_core::referee::app::ui
