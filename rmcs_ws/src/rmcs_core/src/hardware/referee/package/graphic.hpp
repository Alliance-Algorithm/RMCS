#pragma once

#include "protocol.hpp"
#include "send.hpp"

#include <cstring>

namespace rmcs_core::hardware::referee::ui {

using namespace referee::package;
using namespace referee::package::send;

using Description = interact::ui::Description;

using DeletePackage = Package<interact::Data<interact::ui::DeleteLayer>>;
using DrawPackage1  = Package<interact::Data<interact::ui::Description>>;
using DrawPackage2  = Package<interact::Data<interact::ui::Description2>>;
using DrawPackage5  = Package<interact::Data<interact::ui::Description5>>;
using DrawPackage7  = Package<interact::Data<interact::ui::Description7>>;
using StringPackage = Package<interact::Data<interact::ui::String>>;

enum class GraphicType { LINE, RECTANGLE, CIRCLE, ELLIPSE, ARC, FLOAT, INTEGER, STRING };
enum class ColorType { SELF, YELLOW, GREEN, ORANGE, PURPLE_RED, PINK, BLUE, BLACK, WHITE };
enum class OperationType { EMPTY, ADD, MODIFY, DELETE };

class Graphic {
public:
    Graphic() = default;

    virtual Description generate_description() = 0;

    Description generate_basic_description() {
        auto description = Description();

        description.operate = static_cast<uint8_t>(this->operation);
        description.color   = static_cast<uint8_t>(color);
        description.name[0] = this->name[0];
        description.name[1] = this->name[1];
        description.name[2] = this->name[2];
        description.layer   = this->layer;
        description.width   = this->width;
        (void)description.graphic;
        (void)description.details_a;
        (void)description.details_b;
        (void)description.details_c;
        (void)description.details_d;
        (void)description.details_e;

        return description;
    }

    void set_name(const char str[3]) {
        std::strncpy(reinterpret_cast<char*>(name), str, sizeof(name));
    }

public:
    OperationType operation;
    ColorType color;
    uint8_t name[3];
    uint8_t layer;
    uint16_t width;
    uint16_t x;
    uint16_t y;
};

class Line : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::LINE);
        description.details_d = this->end_x;
        description.details_e = this->end_y;

        return description;
    }

public:
    uint16_t end_x;
    uint16_t end_y;
};

class Rectangle : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::RECTANGLE);
        description.details_d = this->other_x;
        description.details_e = this->other_y;

        return description;
    }

public:
    uint16_t other_x;
    uint16_t other_y;
};

class Circle : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::CIRCLE);
        description.details_c = radius;

        return description;
    }

public:
    uint16_t radius;
};

class Ellipse : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::ELLIPSE);
        description.details_d = radius_x;
        description.details_e = radius_y;

        return description;
    }

public:
    uint16_t radius_x;
    uint16_t radius_y;
};

class Arc : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::ARC);
        description.details_d = radius_x;
        description.details_e = radius_y;

        return description;
    }

public:
    uint16_t angle_start;
    uint16_t angle_end;
    uint16_t radius_x;
    uint16_t radius_y;
};

class FLoat : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::FLOAT);
        description.details_a = font_size;
        description.details_c = va_dot_lue >> 22;
        description.details_d = va_dot_lue >> 11;
        description.details_e = va_dot_lue >> 00;

        return description;
    }

public:
    uint16_t font_size;
    uint32_t va_dot_lue;
};

class Integer : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::INTEGER);
        description.details_a = font_size;
        description.details_c = value >> 22;
        description.details_d = value >> 11;
        description.details_e = value >> 00;

        return description;
    }

public:
    uint16_t font_size;
    uint32_t value;
};

class String : public Graphic {
public:
    Description generate_description() override {
        auto description = generate_basic_description();

        description.graphic   = static_cast<uint8_t>(GraphicType::STRING);
        description.details_a = font_size;
        description.details_b = length;

        return description;
    }

public:
    uint16_t font_size;
    uint16_t length;
};
} // namespace rmcs_core::hardware::referee::ui