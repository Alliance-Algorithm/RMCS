#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/crosshair_circle.hpp"
#include "referee/app/ui/widget/status_ring.hpp"

namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    static constexpr uint8_t kUiModeCombat      = 0;
    static constexpr uint8_t kUiModeOutpostOnly = 1;

    Flight()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_circle_(Shape::Color::WHITE, x_center - 2, y_center - 30, 8, 2)
        , status_ring_(
              26.5, 26.5, 600, 300, StatusRing::DynamicArcsVisibility{false, false})
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, 800, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, 200})
        , auto_aim_text_(Shape::Color::WHITE, 16, 2, 1320, 60, auto_aim_buffers_[0].data()) {

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);

        register_input("/gimbal/left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/auto_aim/ui_mode", auto_aim_mode_, false);

        register_input("/remote/mouse", mouse_);

        write_text(auto_aim_buffers_[0], "MODE:COMBAT");
    }

    void update() override {
        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_auto_aim_enable(mouse_->right == 1);

        update_auto_aim_text();
    }

private:
    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;
    static constexpr std::size_t text_capacity = 31;

    static auto to_mode_string(uint8_t mode) -> const char* {
        switch (mode) {
        case kUiModeCombat: return "COMBAT";
        case kUiModeOutpostOnly: return "OUTPOST";
        default: return "UNKNOWN";
        }
    }

    static auto write_text(
        std::array<char, text_capacity>& buffer, const char* format, const char* mode_value)
        -> void {
        std::snprintf(buffer.data(), buffer.size(), format, mode_value);
    }

    static auto write_text(std::array<char, text_capacity>& buffer, const char* value) -> void {
        std::snprintf(buffer.data(), buffer.size(), "%s", value);
    }

    auto update_text_shape(Text& text, std::array<char, text_capacity> (&buffers)[2],
        uint8_t& active_buffer, const char* format, const char* mode_value) -> void {
        auto next_buffer = static_cast<uint8_t>(1 - active_buffer);
        write_text(buffers[next_buffer], format, mode_value);
        if (std::string_view { buffers[next_buffer].data() }
            == std::string_view { buffers[active_buffer].data() }) {
            return;
        }

        active_buffer = next_buffer;
        text.set_value(buffers[active_buffer].data());
    }

    auto update_auto_aim_text() -> void {
        auto mode_value   = auto_aim_mode_.ready() ? *auto_aim_mode_ : kUiModeCombat;

        update_text_shape(
            auto_aim_text_, auto_aim_buffers_, active_auto_aim_buffer_, "MODE:%s",
            to_mode_string(mode_value));
    }

    InputInterface<uint16_t> robot_bullet_allowance_;

    InputInterface<double> left_friction_control_velocity_;
    InputInterface<double> left_friction_velocity_;
    InputInterface<double> right_friction_velocity_;

    InputInterface<uint8_t> auto_aim_mode_;

    InputInterface<rmcs_msgs::Mouse> mouse_;

    CrossHairCircle crosshair_circle_;
    StatusRing status_ring_;

    Line horizontal_center_guidelines_[2];
    Line vertical_center_guidelines_[2];

    std::array<char, text_capacity> auto_aim_buffers_[2] {};
    uint8_t active_auto_aim_buffer_ { 0 };

    Text auto_aim_text_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Flight, rmcs_executor::Component)
