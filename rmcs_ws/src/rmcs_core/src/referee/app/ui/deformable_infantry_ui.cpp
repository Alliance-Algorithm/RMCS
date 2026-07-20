#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

#include <fmt/format.h>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/animated_toggle.hpp"
#include "referee/app/ui/widget/crosshair_circle.hpp"
#include "referee/app/ui/widget/deformable_chassis_top_view.hpp"
#include "referee/app/ui/widget/status_ring.hpp"

namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class DeformableInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_circle_(Shape::Color::WHITE, x_center - 2, y_center - 30, 8, 2)
        , status_ring_(24.0, 26.5, 600, 300)
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, 800, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, 200})
        , friction_wheel_speed_indicator_(
              Shape::Color::PINK, friction_wheel_speed_indicator_font_size_, 2, 0,
              friction_wheel_speed_indicator_y(), 0)
        , chassis_direction_indicator_(Shape::Color::PINK, 8, x_center, y_center, 0, 0, 84, 84)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false) {

        double deformable_leg_min_angle_deg = 8.0;
        double deformable_leg_max_angle_deg = 58.0;
        get_parameter_or(
            "deformable_leg_min_angle_deg", deformable_leg_min_angle_deg,
            deformable_leg_min_angle_deg);
        get_parameter_or(
            "deformable_leg_max_angle_deg", deformable_leg_max_angle_deg,
            deformable_leg_max_angle_deg);
        deformable_chassis_leg_arcs_.set_angle_range(
            deformable_leg_min_angle_deg, deformable_leg_max_angle_deg);

        register_input("/predefined/timestamp", timestamp_);
        register_input("/chassis/control_mode", chassis_mode_);
        register_input("/chassis/active_suspension/active", active_suspension_active_);

        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/enabled", supercap_enabled_);

        register_input("/chassis/voltage", chassis_voltage_);

        for (size_t i = 0; i < kJointCount; ++i) {
            register_input(
                fmt::format("/chassis/{}_joint/physical_angle", kJointName[i]),
                joint_physical_angle_[i], false);
        }

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);

        register_input("/gimbal/left_friction/working_velocity", left_friction_working_velocity_);
        register_input("/gimbal/left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/referee/game/stage", game_stage_);

        friction_wheel_speed_indicator_.set_center_x(friction_wheel_speed_indicator_center_x());

        crosshair_base_x_ = crosshair_circle_.x();
        crosshair_base_y_ = crosshair_circle_.y();
        ctrl_transition_.reset(false);
    }

    void update() override {
        update_chassis_direction_indicator();
        update_deformable_chassis_leg_arcs();
        update_ctrl_ui();

        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        const double friction_wheel_speed =
            std::min(*left_friction_velocity_, *right_friction_velocity_);
        const bool friction_wheel_enabled = *left_friction_control_velocity_ > 0;
        const auto friction_wheel_speed_value =
            static_cast<int32_t>(std::lround(*left_friction_working_velocity_));
        status_ring_.update_friction_wheel_speed(friction_wheel_speed, friction_wheel_enabled);
        if (friction_wheel_speed_indicator_.value() != friction_wheel_speed_value) {
            friction_wheel_speed_indicator_.set_value(friction_wheel_speed_value);
            friction_wheel_speed_indicator_.set_center_x(friction_wheel_speed_indicator_center_x());
        }
        friction_wheel_speed_indicator_.set_color(
            friction_wheel_enabled ? Shape::Color::GREEN : Shape::Color::PINK);
        status_ring_.update_supercap_energy(
            *supercap_voltage_, *supercap_enabled_, supercap_cutoff_voltage);
        status_ring_.update_battery_power(*chassis_voltage_);

        status_ring_.update_auto_aim_enable(mouse_->right == 1);
    }

private:
    void update_ctrl_ui() {
        const bool ctrl_active = keyboard_.ready() && keyboard_->ctrl;
        const double reveal = ctrl_transition_.update(*timestamp_, ctrl_active);

        crosshair_circle_.set_x(
            static_cast<uint16_t>(
                std::lround(static_cast<double>(crosshair_base_x_) + 45.0 * reveal)));
        crosshair_circle_.set_y(
            static_cast<uint16_t>(
                std::lround(static_cast<double>(crosshair_base_y_) + 20.0 * reveal)));
    }

    void update_time_reminder() {
        if (!game_stage_.ready())
            return;
    }

    void update_chassis_direction_indicator() {
        auto chassis_mode = *chassis_mode_;

        chassis_direction_indicator_.set_color(chassis_direction_indicator_color(chassis_mode));
        chassis_direction_indicator_.set_angle(0, 30);
    }

    static Shape::Color chassis_direction_indicator_color(rmcs_msgs::ChassisMode mode) {
        switch (mode) {
        case rmcs_msgs::ChassisMode::SPIN: return Shape::Color::GREEN;
        case rmcs_msgs::ChassisMode::AUTO: return Shape::Color::CYAN;
        case rmcs_msgs::ChassisMode::STEP_DOWN: return Shape::Color::PINK;
        case rmcs_msgs::ChassisMode::WIRELESS_CHARGING: return Shape::Color::PURPLE;
        default: return Shape::Color::WHITE;
        }
    }

    void update_deformable_chassis_leg_arcs() {
        if (!std::all_of(
                joint_physical_angle_.begin(), joint_physical_angle_.end(),
                [](const auto& j) { return j.ready(); })) {
            deformable_chassis_leg_arcs_.set_visible(false);
            return;
        }

        if (!chassis_angle_.ready()) {
            deformable_chassis_leg_arcs_.set_visible(false);
            return;
        }

        std::array<double, kJointCount> leg_angles;
        for (size_t i = 0; i < kJointCount; ++i)
            leg_angles[i] = *joint_physical_angle_[i];
        deformable_chassis_leg_arcs_.update(
            *chassis_angle_, leg_angles,
            active_suspension_active_.ready() && *active_suspension_active_);
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;
    static constexpr double friction_wheel_speed_indicator_radius_ = 430.0;
    static constexpr uint16_t friction_wheel_speed_indicator_font_size_ = 20;
    static constexpr double supercap_cutoff_voltage = 8.0;

    static uint16_t friction_wheel_speed_indicator_center_x() {
        return static_cast<uint16_t>(std::lround(
            static_cast<double>(x_center)
            + friction_wheel_speed_indicator_radius_ / std::numbers::sqrt2));
    }

    static uint16_t friction_wheel_speed_indicator_y() {
        return static_cast<uint16_t>(std::lround(
            static_cast<double>(y_center)
            + friction_wheel_speed_indicator_radius_ / std::numbers::sqrt2
            - static_cast<double>(friction_wheel_speed_indicator_font_size_) / 2.0));
    }

    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<bool> active_suspension_active_;
    InputInterface<double> chassis_angle_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_voltage_;

    static constexpr size_t kJointCount = 4;
    static constexpr const char* kJointName[] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    std::array<InputInterface<double>, kJointCount> joint_physical_angle_;

    InputInterface<uint16_t> robot_bullet_allowance_;

    InputInterface<double> left_friction_working_velocity_;
    InputInterface<double> left_friction_control_velocity_;
    InputInterface<double> left_friction_velocity_;
    InputInterface<double> right_friction_velocity_;

    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;

    CrossHairCircle crosshair_circle_;
    StatusRing status_ring_;

    Line horizontal_center_guidelines_[2];
    Line vertical_center_guidelines_[2];
    Integer friction_wheel_speed_indicator_;

    Arc chassis_direction_indicator_;
    DeformableChassisLegArcs deformable_chassis_leg_arcs_;

    AnimatedToggle ctrl_transition_{};
    uint16_t crosshair_base_x_ = 0;
    uint16_t crosshair_base_y_ = 0;

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::DeformableInfantry, rmcs_executor::Component)
