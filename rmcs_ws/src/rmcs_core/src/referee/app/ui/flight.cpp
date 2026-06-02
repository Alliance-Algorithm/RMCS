#include <algorithm>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/crosshair_circle.hpp"
#include "referee/app/ui/widget/status_ring.hpp"

namespace rmcs_core::referee::app::ui {
class Flight
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    static constexpr uint8_t kUiModeCombat = 0;
    static constexpr uint8_t kUiModeOutpostOnly = 1;
    static constexpr uint8_t kUiModeEngineer = 2;

    Flight()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_circle_(Shape::Color::WHITE, x_center - 2, y_center - 30, 8, 2)
        , status_ring_(26.5, 26.5, 600, 300, StatusRing::DynamicArcsVisibility{false, false})
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, 800, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, 200})
        , mode_indicator_(
              Shape::Color::YELLOW, 30, x_center + 360, y_center + 220, x_center + 400,
              y_center + 220) {

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);

        register_input("/gimbal/left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/auto_aim/ui_mode", auto_aim_mode_, false);

        register_input("/remote/mouse", mouse_);
    }

    void update() override {
        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_auto_aim_enable(mouse_->right == 1);

        update_mode_indicator();
    }

private:
    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

    auto update_mode_indicator() -> void {
        auto mode_value = auto_aim_mode_.ready() ? *auto_aim_mode_ : kUiModeCombat;

        switch (mode_value) {
        case kUiModeOutpostOnly:
            mode_indicator_.set_color(Shape::Color::GREEN);
            break;
        case kUiModeEngineer:
            mode_indicator_.set_color(Shape::Color::PINK);
            break;
        case kUiModeCombat:
        default:
            mode_indicator_.set_color(Shape::Color::YELLOW);
            break;
        }
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

    Line mode_indicator_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Flight, rmcs_executor::Component)
