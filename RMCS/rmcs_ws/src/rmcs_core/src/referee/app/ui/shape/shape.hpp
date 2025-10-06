#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <new>

#include "referee/app/ui/shape/cfs_scheduler.hpp"
#include "referee/app/ui/shape/remote_shape.hpp"
#include "referee/command/field.hpp"

namespace rmcs_core::referee {

namespace command::interaction {
class Ui;
}

namespace app::ui {

constexpr inline uint16_t screen_width = 1920, screen_height = 1080;
constexpr inline uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

class Shape
    : private CfsScheduler<Shape>::Entity
    , private RemoteShape<Shape>::Descriptor {
public:
    friend class CfsScheduler<Shape>;
    friend class RemoteShape<Shape>;
    friend class command::interaction::Ui;

    bool visible() const { return visible_; }
    void set_visible(bool value) {
        if (visible_ == value)
            return;

        visible_ = value;

        // Optimizations
        if (!visible_) {
            if (existence_confidence() == 0) {
                // Simply leave run_queue when shape was hidden and remote shape does not exist.
                leave_run_queue();
                return;
            } else {
                // Otherwise enable swapping
                enable_swapping();
            }
        } else {
            // Disable swapping when shape is visible
            disable_swapping();
        }

        sync_confidence_ = 0;
        enter_run_queue();
    }

    uint8_t priority() const { return priority_; }
    void set_priority(uint8_t value) {
        if (priority_ == value)
            return;
        priority_ = value;
        if (is_in_run_queue())
            enter_run_queue();
    }

    uint16_t width() const { return part2_.width; }
    void set_width(uint16_t width) {
        if (part2_.width == width)
            return;
        part2_.width = width;
        set_modified();
    }

    uint16_t x() const { return part2_.x; }
    void set_x(uint16_t x) {
        if (part2_.x == x)
            return;
        part2_.x = x;
        set_modified();
    }

    uint16_t y() const { return part2_.y; }
    void set_y(uint16_t y) {
        if (part2_.y == y)
            return;
        part2_.y = y;
        set_modified();
    }

    bool is_text_shape() const { return is_text_shape_; }

    enum class Operation : uint8_t {
        NO_OPERATION = 0,
        ADD          = 1,
        MODIFY       = 2,
        DELETE       = 3,
    };

    Operation predict_update() const {
        uint8_t predict_existence = existence_confidence();
        uint8_t predict_sync      = sync_confidence_;

        if (!has_id() && !predict_try_assign_id(predict_existence)) {
            return Operation::NO_OPERATION;
        }

        if (predict_existence == 0) {
            predict_sync = max_update_times;
        }

        command::Field field;
        if (visible_
            && (predict_existence <= predict_sync
                || (last_time_modified_ && predict_existence < max_update_times))) {
            return Operation::ADD;
        } else {
            return Operation::MODIFY;
        }
    }

    constexpr static inline command::Field no_operation_description() {
        return command::Field{[](std::byte* buffer) {
            auto& description                = *new (buffer) DescriptionField{};
            description.part1.operation_type = Operation::NO_OPERATION;
            return sizeof(DescriptionField);
        }};
    }

    enum class Color : uint8_t {
        SELF   = 0,
        YELLOW = 1,
        GREEN  = 2,
        ORANGE = 3,
        PURPLE = 4,
        PINK   = 5,
        CYAN   = 6,
        BLACK  = 7,
        WHITE  = 8,
    };

protected:
    enum class ShapeType : uint8_t {
        LINE      = 0,
        RECTANGLE = 1,
        CIRCLE    = 2,
        ELLIPSE   = 3,
        ARC       = 4,
        FLOAT     = 5,
        INTEGER   = 6,
        TEXT      = 7,
    };

    struct DescriptionField {
        uint8_t name[3];
        struct __attribute__((packed)) Part1 {
            Operation operation_type : 3;
            ShapeType shape_type     : 3;
            uint8_t layer            : 4;
            Color color              : 4;
            uint16_t details_a       : 9;
            uint16_t details_b       : 9;
        } part1;
        struct __attribute__((packed)) Part2 {
            uint16_t width : 10;
            uint16_t x     : 11;
            uint16_t y     : 11;
        } part2;
        struct __attribute__((packed)) Part3 {
            uint16_t details_c : 10;
            uint16_t details_d : 11;
            uint16_t details_e : 11;
        } part3;
    };

    void set_modified() {
        // Optimization: Assume the modification not exist when invisible.
        if (!visible_)
            return;

        sync_confidence_ = 0;
        enter_run_queue();
    }

    virtual size_t write_description_field(std::byte* buffer) = 0;

    DescriptionField::Part2 part2_ alignas(4);

private:
    void enter_run_queue() {
        uint8_t min_confidence     = std::min(existence_confidence(), sync_confidence_);
        uint16_t weighted_priority = (priority_ - 256) << (4 * min_confidence);
        CfsScheduler<Shape>::Entity::enter_run_queue(weighted_priority);
    }

    void id_revoked() {
        // This is a callback indicating that the remote id that this shape once had
        // is no longer associated with it.
        // Called by RemoteShape<Shape>::Descriptor.
        if (visible_) {
            // Re-enter the update queue to try to get a new id.
            set_modified();
        } else {
            // Leave run_queue when shape was hidden.
            leave_run_queue();
        }
    }

    command::Field update() {
        // This is a callback indicating that the shape is being updated.
        // Called by CfsScheduler<Shape>.

        if (!has_id() && !try_assign_id()) {
            // TODO: Print error message.
            sync_confidence_ = max_update_times;
            visible_         = false;
            // Do nothing when failed
            return no_operation_description();
        }

        if (existence_confidence() == 0) {
            // Optimization: Always consider it synchronized when remote shape does not exist.
            sync_confidence_ = max_update_times;
        }

        command::Field field;

        // Optimization1: Stop adding when shape is invisible.
        // Optimization2: Prevent continuous modification.
        if (visible_
            && (existence_confidence() <= sync_confidence_
                || (last_time_modified_ && existence_confidence() < max_update_times))) {
            // Send add packet
            last_time_modified_ = false;
            field               = command::Field{[this](std::byte* buffer) {
                return write_full_description_field(buffer, Operation::ADD);
            }};
            if (increase_existence_confidence() < max_update_times
                || sync_confidence_ < max_update_times)
                enter_run_queue();
        } else {
            // Send modify packet
            last_time_modified_ = true;
            field               = command::Field{[this](std::byte* buffer) {
                return write_full_description_field(buffer, Operation::MODIFY);
            }};
            // No need to compare existence_confidence here.
            // Because either the shape is not visible here, no need to send add packet.
            // Or existence_confidence > sync_confidence, only the min value needs to be considered.
            if (++sync_confidence_ < max_update_times)
                enter_run_queue();
        }

        return field;
    }

    size_t write_full_description_field(std::byte* buffer, Operation operation) {
        size_t written =
            visible_ ? write_description_field(buffer) : write_invisible_description_field(buffer);
        auto& description = *std::launder(reinterpret_cast<DescriptionField*>(buffer));

        // No special meaning, just to ensure no duplication
        description.name[0] = id();
        description.name[1] = 0xef;
        description.name[2] = 0xfe;

        // We only use layer 0
        description.part1.layer = 0;

        description.part1.operation_type = operation;

        return written;
    }

    static inline size_t write_invisible_description_field(std::byte* buffer) {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::LINE;
        description.part1.color      = Color::WHITE;

        description.part2.width = 0;
        description.part2.x     = 0;
        description.part2.y     = 0;

        description.part3.details_c = 0;
        description.part3.details_d = 0;
        description.part3.details_e = 0;

        return sizeof(DescriptionField);
    }

    static constexpr uint8_t max_update_times = 4;

    uint8_t priority_            = 15;
    uint8_t sync_confidence_ : 5 = max_update_times;
    bool is_text_shape_      : 1 = false;
    bool last_time_modified_ : 1 = false;
    bool visible_            : 1 = false;
};

class Line : public Shape {
public:
    Line() = default;
    Line(
        Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x2, uint16_t y2,
        bool visible = true) {
        part3_.color = color;
        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;
        part3_.x2    = x2;
        part3_.y2    = y2;

        set_visible(visible);
    }

    Color color() const { return part3_.color; }
    void set_color(Color color) {
        if (part3_.color == color)
            return;
        part3_.color = color;
        set_modified();
    }

    uint16_t x2() const { return part3_.x2; }
    void set_x2(uint16_t x2) {
        if (part3_.x2 == x2)
            return;
        part3_.x2 = x2;
        set_modified();
    }

    uint16_t y2() const { return part3_.y2; }
    void set_y2(uint16_t y2) {
        if (part3_.y2 == y2)
            return;
        part3_.y2 = y2;
        set_modified();
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::LINE;
        description.part1.color      = part3_.color;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(part3_);

        return sizeof(DescriptionField);
    }

private:
    struct __attribute__((packed)) {
        // Since details_c is invalid in lines, the memory is used to store color
        Color color         : 8;
        uint8_t placeholder : 2;

        uint16_t x2 : 11;
        uint16_t y2 : 11;
    } part3_ alignas(4);
};

class Circle : public Shape {
public:
    Circle() = default;
    Circle(
        Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t rx, uint16_t ry,
        bool visible = true) {
        part3_.color = color;
        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;
        part3_.rx    = rx;
        part3_.ry    = ry;

        set_visible(visible);
    }

    Color color() const { return part3_.color; }
    void set_color(Color color) {
        if (part3_.color == color)
            return;
        part3_.color = color;
        set_modified();
    }

    void set_r(uint16_t r) {
        set_rx(r);
        set_ry(r);
    }

    uint16_t rx() const { return part3_.rx; }
    void set_rx(uint16_t rx) {
        if (part3_.rx == rx)
            return;
        part3_.rx = rx;
        set_modified();
    }

    uint16_t ry() const { return part3_.ry; }
    void set_ry(uint16_t ry) {
        if (part3_.ry == ry)
            return;
        part3_.ry = ry;
        set_modified();
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::ELLIPSE;
        description.part1.color      = part3_.color;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(part3_);

        return sizeof(DescriptionField);
    }

    struct __attribute__((packed)) {
        Color color         : 8;
        uint8_t placeholder : 2;

        uint16_t rx : 11;
        uint16_t ry : 11;
    } part3_ alignas(4);
};

class Rectangle : public Shape {
public:
    Rectangle() = default;
    Rectangle(
        Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t x2, uint16_t y2,
        bool visible = true) {
        part3_.color = color;
        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;
        part3_.x2    = x2;
        part3_.y2    = y2;

        set_visible(visible);
    }

    Color color() const { return part3_.color; }
    void set_color(Color color) {
        if (part3_.color == color)
            return;
        part3_.color = color;
        set_modified();
    }

    uint16_t x2() const { return part3_.x2; }
    void set_x2(uint16_t x2) {
        if (part3_.x2 == x2)
            return;
        part3_.x2 = x2;
        set_modified();
    }

    uint16_t y2() const { return part3_.y2; }
    void set_y2(uint16_t y2) {
        if (part3_.y2 == y2)
            return;
        part3_.y2 = y2;
        set_modified();
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::RECTANGLE;
        description.part1.color      = part3_.color;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(part3_);

        return sizeof(DescriptionField);
    }

    struct __attribute__((packed)) {
        Color color         : 8;
        uint8_t placeholder : 2;

        uint16_t x2 : 11;
        uint16_t y2 : 11;
    } part3_ alignas(4);
};

class Arc : public Shape {
public:
    Arc() = default;
    Arc(Color color, uint16_t width, uint16_t x, uint16_t y, uint16_t angle_start,
        uint16_t angle_end, uint16_t rx, uint16_t ry, bool visible = true)
        : Arc() {
        angle_start_ = angle_start;
        angle_end_   = angle_end;

        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;

        part3_.color = color;
        part3_.rx    = rx;
        part3_.ry    = ry;

        set_visible(visible);
    }

    Color color() const { return part3_.color; }
    void set_color(Color color) {
        if (part3_.color == color)
            return;
        part3_.color = color;
        set_modified();
    }

    uint16_t angle_start() const { return angle_start_; }
    void set_angle_start(uint16_t angle_start) {
        if (angle_start_ == angle_start)
            return;
        angle_start_ = angle_start;
        set_modified();
    }

    uint16_t angle_end() const { return angle_end_; }
    void set_angle_end(uint16_t angle_end) {
        if (angle_end_ == angle_end)
            return;
        angle_end_ = angle_end;
        set_modified();
    }

    void set_angle(uint16_t midpoint, uint16_t half_central_angle) {
        int start = midpoint - half_central_angle;
        if (start < 0)
            start += 360;
        angle_start_ = start;

        int end = midpoint + half_central_angle;
        if (end >= 360)
            end -= 360;
        angle_end_ = end;

        set_modified();
    }

    uint16_t rx() const { return part3_.rx; }
    void set_rx(uint16_t rx) {
        if (part3_.rx == rx)
            return;
        part3_.rx = rx;
        set_modified();
    }

    uint16_t ry() const { return part3_.ry; }
    void set_ry(uint16_t ry) {
        if (part3_.ry == ry)
            return;
        part3_.ry = ry;
        set_modified();
    }

    void set_r(uint16_t r) {
        set_rx(r);
        set_ry(r);
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::ARC;
        description.part1.color      = part3_.color;
        description.part1.details_a  = angle_start_;
        description.part1.details_b  = angle_end_;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(part3_);

        return sizeof(DescriptionField);
    }

    uint16_t angle_start_, angle_end_;

    struct __attribute__((packed)) {
        Color color         : 8;
        uint8_t placeholder : 2;

        uint16_t rx : 11;
        uint16_t ry : 11;
    } part3_ alignas(4);
};

class Integer : public Shape {
public:
    Integer() = default;
    Integer(
        Color color, uint16_t font_size, uint16_t width, uint16_t x, uint16_t y, int32_t value,
        bool visible = true)
        : Integer() {
        color_     = color;
        font_size_ = font_size;

        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;

        value_ = value;

        set_visible(visible);
    }

    Color color() const { return color_; }
    void set_color(Color color) {
        if (color_ == color)
            return;
        color_ = color;
        set_modified();
    }

    void set_center_x(uint16_t x) {
        int value            = value_;
        int number_of_digits = value <= 0 ? 1 : 0;
        for (; value != 0; number_of_digits++)
            value /= 10;
        part2_.x = x - font_size_ * number_of_digits / 2 + font_size_ / 5;
        set_modified();
    }

    int32_t value() const { return value_; }
    void set_value(int32_t value) {
        if (value_ == value)
            return;
        value_ = value;
        set_modified();
    }

    uint16_t font_size() const { return font_size_; }
    void set_font_size(uint16_t font_size) {
        if (font_size_ == font_size)
            return;
        font_size_ = font_size;
        set_modified();
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::INTEGER;
        description.part1.color      = color_;
        description.part1.details_a  = font_size_;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(value_);

        return sizeof(DescriptionField);
    }

    uint16_t font_size_;
    Color color_;
    int32_t value_;
};

class Float : public Integer {
public:
    using Integer::Integer;

    void set_center_x(uint16_t x) {
        int value            = value_;
        int number_of_digits = value < 0 ? 1 : 0;

        int integer_part = value / 1000;
        if (integer_part == 0)
            ++number_of_digits;
        for (; integer_part != 0; number_of_digits++)
            integer_part /= 10;

        int decimal_part = value % 1000;
        number_of_digits +=
            2 * (decimal_part != 0) + (decimal_part % 100 != 0) + (decimal_part % 10 != 0);
        part2_.x = x - font_size_ * number_of_digits / 2 + font_size_ / 5;

        set_modified();
    }

    using Integer::set_value;
    void set_value(double value) { Integer::set_value(static_cast<int>(std::round(value * 1000))); }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::FLOAT;
        description.part1.color      = color_;
        description.part1.details_a  = font_size_;

        description.part2 = part2_;

        description.part3 = std::bit_cast<DescriptionField::Part3>(value_);

        return sizeof(DescriptionField);
    }
};

class Text : public Shape {
public:
    Text() { value_ = nullptr; };
    Text(
        Color color, uint16_t font_size, uint16_t width, uint16_t x, uint16_t y, const char* value,
        bool visible = true)
        : Text() {
        color_     = color;
        font_size_ = font_size;

        part2_.width = width;
        part2_.x     = x;
        part2_.y     = y;

        value_ = value;

        set_visible(visible);
    }

    Color color() const { return color_; }
    void set_color(Color color) {
        if (color_ == color)
            return;
        color_ = color;
        set_modified();
    }

    const char* value() const { return value_; }
    void set_value(const char* value) {
        if (value_ == value)
            return;
        value_ = value;
        set_modified();
    }

    uint16_t font_size() const { return font_size_; }
    void set_font_size(uint16_t font_size) {
        if (font_size_ == font_size)
            return;
        font_size_ = font_size;
        set_modified();
    }

protected:
    size_t write_description_field(std::byte* buffer) override {
        auto& description = *new (buffer) DescriptionField{};

        description.part1.shape_type = ShapeType::TEXT;
        description.part1.color      = color_;
        description.part1.details_a  = font_size_;

        description.part2 = part2_;

        constexpr size_t data_part_size = 30;
        auto str_length                 = std::min(strlen(value_), data_part_size);
        description.part1.details_b     = str_length;
        std::memcpy(buffer + sizeof(DescriptionField), value_, str_length);

        return sizeof(DescriptionField) + data_part_size;
    }

    uint16_t font_size_;
    Color color_;
    const char* value_;
};

} // namespace app::ui
} // namespace rmcs_core::referee