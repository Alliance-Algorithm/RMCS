#pragma once

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class CrossHair {
public:
    CrossHair(Shape::Color color, uint16_t x, uint16_t y, bool visible = true)
        : guidelines_(
              {color, 2, (uint16_t)(x - r2), y, (uint16_t)(x - r1), y, visible},
              {color, 2, (uint16_t)(x + r1), y, (uint16_t)(x + r2), y, visible},
              {color, 2, x, (uint16_t)(y + r2), x, (uint16_t)(y + r1), visible},
              {color, 2, x, (uint16_t)(y - r1), x, (uint16_t)(y - r2), visible})
        , precise_lines_(
              {Shape::Color::CYAN, 2, (uint16_t)(x - r2), y, (uint16_t)(x + r2), y, false},
              {Shape::Color::CYAN, 2, x, (uint16_t)(y - r1 * 2), x, (uint16_t)(y + r1 * 2), false},
              {Shape::Color::CYAN, 2, (uint16_t)(x - r2), (uint16_t)(y - r1), (uint16_t)(x - r2),
               (uint16_t)(y + r1), false},
              {Shape::Color::CYAN, 2, (uint16_t)(x + r2), (uint16_t)(y - r1), (uint16_t)(x + r2),
               (uint16_t)(y + r1), false})
        , center_(color, 2, x, y, 1, 1) {}

    void set_visible(bool value) {
        for (auto& line : guidelines_)
            line.set_visible(value);
        center_.set_visible(value);
    }
    void enable_precise_crosshair(bool enable) {
        static bool enable_last{false};

        if (enable == enable_last)
            return;

        if (enable) {
            size_t index = 0;
            for (auto& line : precise_lines_) {
                line.set_visible(true);
                guidelines_[index++].set_visible(false);
            }
            center_.set_visible(false);
        } else {
            for (auto& line : precise_lines_) {
                line.set_visible(false);
            }
        }
        enable_last = enable;
    }

private:
    static constexpr uint16_t r1 = 8, r2 = 24;

    Line guidelines_[4];
    Line precise_lines_[4];
    Circle center_;
};

} // namespace rmcs_core::referee::app::ui