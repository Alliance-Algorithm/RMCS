#pragma once

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class CrossHairCircle {
public:
    CrossHairCircle(Shape::Color color, uint16_t x, uint16_t y, bool visible = true)
        : CrossHairCircle(color, x, y, 12, 2, visible) {}

    CrossHairCircle(
        Shape::Color color, uint16_t x, uint16_t y, uint16_t r, uint16_t width,
        bool visible = true)
        : circle_(color, width, x, y, r, r, visible) {}

    void set_visible(bool value) { circle_.set_visible(value); }
    void set_r(uint16_t r) { circle_.set_r(r); }
    void set_width(uint16_t width) { circle_.set_width(width); }

private:
    Circle circle_;
};

} // namespace rmcs_core::referee::app::ui
