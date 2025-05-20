#pragma once

#include "referee/app/ui/shape/shape.hpp"

namespace rmcs_core::referee::app::ui {

class Rangefinder {
public:
    Rangefinder() {
        set_horizontal_scales();
        set_vertical_scales();
    }

    void set_visible(bool value) {
        pitch_angle_.set_visible(value);
        bullet_allowance_.set_visible(false);

        for (auto& line : bullet_allowance_indicator_)
            line.set_visible(false);
    }

    void update_pitch_angle(double display_angle) { pitch_angle_.set_value(display_angle); }

    void update_vertical_rangefinder(uint16_t lift_height) {
        size_t index = 0;
        for (auto& scale : vertical_center_rangefinder_scales_) {
            auto parity = static_cast<uint16_t>(index % 2);
            scale.set_y(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + lift_height);
            scale.set_y2(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + lift_height);

            vertical_side_rangefinder_scales_[index].set_y(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + lift_height);
            vertical_side_rangefinder_scales_[index].set_y2(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + lift_height);

            if (parity == 1) {
                auto number_index = (index - 1) / 2;
                vertical_side_scale_numbers_[number_index].set_y(
                    y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + 3
                    + lift_height);
            }
            index++;
        }
    }

    void update_bullet_allowance(int bullet_allowance) {
        bullet_allowance_.set_value(bullet_allowance);
    }

private:
    void set_horizontal_scales() {
        size_t index = 0;
        for (auto& scale : horizontal_center_rangefinder_scales_) {
            auto parity = static_cast<uint16_t>(index % 2);
            if (index < 8) {
                scale.set_color(Shape::Color::BLACK);
                scale.set_x(
                    x_center_ - horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) + 1));
                scale.set_y(y_center_ + horizontal_unit_scale * (parity + 1));
                scale.set_x2(
                    x_center_ - horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) + 1));
                scale.set_y2(y_center_ - horizontal_unit_scale * (parity + 1));
                scale.set_width(1);
                scale.set_visible(true);

                if (parity == 1) {
                    auto number_index = (index - 1) / 2;
                    horizontal_center_scale_numbers_[number_index].set_color(Shape::Color::BLACK);
                    horizontal_center_scale_numbers_[number_index].set_x(
                        x_center_ - horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) + 1)
                        - 5);
                    horizontal_center_scale_numbers_[number_index].set_y(
                        y_center_ + horizontal_unit_scale * (parity + 1) + 20);
                    horizontal_center_scale_numbers_[number_index].set_font_size(10);
                    horizontal_center_scale_numbers_[number_index].set_width(1);
                    horizontal_center_scale_numbers_[number_index].set_value(
                        (static_cast<uint16_t>(number_index) + 1) * 8);
                    horizontal_center_scale_numbers_[number_index].set_visible(true);
                }
            } else {
                scale.set_color(Shape::Color::BLACK);
                scale.set_x(
                    x_center_ + horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) - 7));
                scale.set_y(y_center_ + horizontal_unit_scale * (parity + 1));
                scale.set_x2(
                    x_center_ + horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) - 7));
                scale.set_y2(y_center_ - horizontal_unit_scale * (parity + 1));
                scale.set_width(1);
                scale.set_visible(true);

                if (parity == 1) {
                    auto number_index = (index - 1) / 2;
                    horizontal_center_scale_numbers_[number_index].set_color(Shape::Color::BLACK);
                    horizontal_center_scale_numbers_[number_index].set_x(
                        x_center_ + horizontal_scale_accuracy_ * (static_cast<uint16_t>(index) - 7)
                        - 5);
                    horizontal_center_scale_numbers_[number_index].set_y(
                        y_center_ + horizontal_unit_scale * (parity + 1) + 20);
                    horizontal_center_scale_numbers_[number_index].set_font_size(10);
                    horizontal_center_scale_numbers_[number_index].set_width(1);
                    horizontal_center_scale_numbers_[number_index].set_value(
                        (static_cast<uint16_t>(number_index) - 3) * 8);
                    horizontal_center_scale_numbers_[number_index].set_visible(true);
                }
            }
            index++;
        }
    }

    void set_vertical_scales() {
        size_t index = 0;
        for (auto& scale : vertical_center_rangefinder_scales_) {
            auto parity = static_cast<uint16_t>(index % 2);
            scale.set_color(Shape::Color::BLACK);
            scale.set_x(x_center_ - vertical_unit_scale_ * (parity + 1));
            scale.set_y(y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index));
            scale.set_x2(x_center_ + vertical_unit_scale_ * (parity + 1));
            scale.set_y2(y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index));
            scale.set_width(1);
            scale.set_visible(true);

            vertical_side_rangefinder_scales_[index].set_color(Shape::Color::BLACK);
            vertical_side_rangefinder_scales_[index].set_x(
                x_side_center_ - vertical_side_unit_scale_ * (parity + 1));
            vertical_side_rangefinder_scales_[index].set_y(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index));
            vertical_side_rangefinder_scales_[index].set_x2(
                x_side_center_ + vertical_side_unit_scale_ * (parity + 1));
            vertical_side_rangefinder_scales_[index].set_y2(
                y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index));
            vertical_side_rangefinder_scales_[index].set_width(1);
            vertical_side_rangefinder_scales_[index].set_visible(true);

            if (parity == 1) {
                auto number_index = (index - 1) / 2;
                vertical_side_scale_numbers_[number_index].set_color(Shape::Color::BLACK);
                vertical_side_scale_numbers_[number_index].set_center_x(
                    x_side_center_ - vertical_side_unit_scale_ * (parity + 1) - 20);
                vertical_side_scale_numbers_[number_index].set_y(
                    y_center_ - vertical_scale_accuracy_ * static_cast<uint16_t>(index) + 3);
                vertical_side_scale_numbers_[number_index].set_font_size(10);
                vertical_side_scale_numbers_[number_index].set_width(1);
                vertical_side_scale_numbers_[number_index].set_value(
                    (static_cast<uint16_t>(number_index) + 1) * 4);
                vertical_side_scale_numbers_[number_index].set_visible(true);
            }
            index++;
        }
    }

private:
    constexpr static uint16_t x_center_      = 960;
    constexpr static uint16_t y_center_      = 540;
    constexpr static uint16_t x_side_center_ = x_center_ - 110;

    constexpr static uint16_t horizontal_unit_scale      = 15;
    constexpr static uint16_t horizontal_scale_accuracy_ = 40;

    constexpr static uint16_t vertical_unit_scale_      = 5;
    constexpr static uint16_t vertical_side_unit_scale_ = 15;
    constexpr static uint16_t vertical_scale_accuracy_  = 26;

    Float pitch_angle_{Shape::Color::BLACK, 17, 2, x_center_ - 100, y_center_ + 170, 0, false};

    Line vertical_center_line_   //
        {Shape::Color::BLACK, 1, x_center_, y_center_ + 530, x_center_, y_center_ - 530, true};
    Line horizontal_center_line_ //
        {Shape::Color::BLACK, 1, x_center_ - 450, y_center_, x_center_ + 450, y_center_, true};

    Line horizontal_center_rangefinder_scales_[16];
    Line vertical_center_rangefinder_scales_[18];
    Line vertical_side_rangefinder_scales_[18];

    Integer horizontal_center_scale_numbers_[8];
    Integer vertical_side_scale_numbers_[9];

    Integer bullet_allowance_    //
        {Shape::Color::WHITE, 12, 2, x_center_ - 100, y_center_ - 300, 0, false};
    Line bullet_allowance_indicator_[10];

    Arc friction_wheel_velocity_;
};
} // namespace rmcs_core::referee::app::ui
