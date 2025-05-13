#include <algorithm>
#include <cmath>
#include <cstdint>

#include <game_stage.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>

#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/crosshair.hpp"
#include "referee/app/ui/widget/rangefinder.hpp"
#include "referee/app/ui/widget/status_ring.hpp"

namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class Hero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Hero()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_(Shape::Color::WHITE, x_center - 12, y_center - 37)
        , status_ring_(26.5, 26.5, 600, 40)
        , rangefinder_()
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, y_center + 300, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, y_center - 340})
        , chassis_power_number_(Shape::Color::WHITE, 15, 2, x_center - 340, y_center - 10, 0)
        , chassis_direction_indicator_(Shape::Color::PINK, 8, x_center, y_center, 0, 0, 84, 84)
        , chassis_control_power_limit_indicator_(
              Shape::Color::WHITE, 15, 2, x_center - 340, y_center + 25, 0)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false) {

        chassis_control_direction_indicator_.set_x(x_center);
        chassis_control_direction_indicator_.set_y(y_center);

        register_input("/chassis/control_mode", chassis_mode_);

        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/control_enable", supercap_control_enabled_);

        register_input("/chassis/voltage", chassis_voltage_);
        register_input("/chassis/power", chassis_power_);
        register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        register_input("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input("/referee/shooter/42mm_bullet_allowance", robot_bullet_allowance_);

        register_input(
            "/gimbal/first_left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/first_left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/first_right_friction/velocity", right_friction_velocity_);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/gimbal/auto_aim/laser_distance", laser_distance_);

        register_input("/gimbal/shooter/mode", shoot_mode_);
        register_input("/gimbal/scope/active", is_scope_active_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);

        // register_input("/auto_aim/ui_target", auto_aim_target_, false);
    }

    void update() override {
        if (*is_scope_active_) {
            set_normal_ui_visible(false);
            rangefinder_.set_visible(true);
        } else {
            set_normal_ui_visible(true);
            rangefinder_.set_visible(false);
        }

        update_normal_ui();
        update_sniper_ui();
    }

private:
    void set_normal_ui_visible(bool value) {
        crosshair_.set_visible(value);
        status_ring_.set_visible(value);

        for (auto& line : horizontal_center_guidelines_)
            line.set_visible(value);
        for (auto& line : vertical_center_guidelines_)
            line.set_visible(value);

        chassis_control_power_limit_indicator_.set_visible(value);
        chassis_power_number_.set_visible(value);

        chassis_direction_indicator_.set_visible(value);
        chassis_control_direction_indicator_.set_visible(value);
    }

    void update_normal_ui() {
        update_chassis_direction_indicator();

        chassis_control_power_limit_indicator_.set_value(*chassis_control_power_limit_);

        chassis_power_number_.set_value(*chassis_power_);

        chassis_control_power_limit_indicator_.set_color(
            *supercap_control_enabled_ ? Shape::Color::CYAN : Shape::Color::WHITE);

        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_supercap(*supercap_voltage_, *supercap_control_enabled_);
        status_ring_.update_battery_power(*chassis_voltage_);

        status_ring_.update_auto_aim_enable(mouse_->right == 1);

        status_ring_.update_precise_enable(*shoot_mode_ == rmcs_msgs::ShootMode::PRECISE);

        auto precise_enable = (*shoot_mode_ == rmcs_msgs::ShootMode::PRECISE && !*is_scope_active_);

        crosshair_.enable_precise_crosshair(precise_enable);
    }

    void update_sniper_ui() {
        auto display_angle = *gimbal_pitch_angle_ > std::numbers::pi / 2
                               ? *gimbal_pitch_angle_ - 2 * std::numbers::pi
                               : *gimbal_pitch_angle_;

        rangefinder_.update_pitch_angle(
            -display_angle, *shoot_mode_ == rmcs_msgs::ShootMode::PRECISE);

        // auto double_to_uint =
        //     [](double value, std::unsigned_integral auto min, std::unsigned_integral auto max) {
        //         auto span   = max - min;
        //         auto offset = min;
        //         return (std::unsigned_integral auto)value / (std::unsigned_integral auto)(-1) *
        //         span
        //              + offset;
        //     }

        // auto lift_height = double_to_uint(
        //     (display_angle / 0.7 * static_cast<double>(height_max)), height_min, height_max);
        double raw_height    = -display_angle / 0.7 * static_cast<double>(height_max);
        raw_height           = std::clamp(raw_height, 0.0, static_cast<double>(height_max));
        uint16_t lift_height = static_cast<uint16_t>(std::round(raw_height));

        lift_height = std::clamp(lift_height, height_min, height_max);
        rangefinder_.update_vertical_rangefinder(lift_height);
    }

    void update_time_reminder() {
        if (!game_stage_.ready())
            return;
    }

    void update_chassis_direction_indicator() {
        auto chassis_mode = *chassis_mode_;

        auto to_referee_angle = [](double angle) {
            return static_cast<int>(
                std::round((2 * std::numbers::pi - angle) / std::numbers::pi * 180));
        };
        chassis_direction_indicator_.set_color(
            chassis_mode == rmcs_msgs::ChassisMode::SPIN ? Shape::Color::GREEN
                                                         : Shape::Color::PINK);
        chassis_direction_indicator_.set_angle(to_referee_angle(*chassis_angle_), 30);

        bool chassis_control_direction_indicator_visible = false;
        if (!std::isnan(*chassis_control_angle_)) {
            if (chassis_mode == rmcs_msgs::ChassisMode::STEP_DOWN) {
                chassis_control_direction_indicator_visible = true;
                chassis_control_direction_indicator_.set_color(Shape::Color::CYAN);
                chassis_control_direction_indicator_.set_width(8);
                chassis_control_direction_indicator_.set_r(92);
                chassis_control_direction_indicator_.set_angle(
                    to_referee_angle(*chassis_control_angle_), 30);
            } else if (chassis_mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP) {
                chassis_control_direction_indicator_visible = true;
                chassis_control_direction_indicator_.set_color(Shape::Color::CYAN);
                chassis_control_direction_indicator_.set_width(28);
                chassis_control_direction_indicator_.set_r(102);
                chassis_control_direction_indicator_.set_angle(
                    to_referee_angle(*chassis_control_angle_), 4);
            }
        }
        chassis_control_direction_indicator_.set_visible(
            chassis_control_direction_indicator_visible);
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

    static constexpr uint16_t height_min = 0, height_max = 500;

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_, chassis_control_angle_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_control_enabled_;

    InputInterface<double> chassis_voltage_;
    InputInterface<double> chassis_power_;
    InputInterface<double> chassis_control_power_limit_;
    InputInterface<double> supercap_charge_power_limit_;

    InputInterface<double> left_front_velocity_, left_back_velocity_, right_back_velocity_,
        right_front_velocity_;

    InputInterface<uint16_t> robot_bullet_allowance_;

    InputInterface<double> left_friction_control_velocity_;
    InputInterface<double> left_friction_velocity_;
    InputInterface<double> right_friction_velocity_;

    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;

    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<double> gimbal_player_viewer_angle_;
    InputInterface<double> laser_distance_;

    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;
    InputInterface<bool> is_scope_active_;

    // InputInterface<std::pair<uint16_t, uint16_t>> auto_aim_target_;

    CrossHair crosshair_;
    StatusRing status_ring_;
    Rangefinder rangefinder_;

    Line horizontal_center_guidelines_[2];
    Line vertical_center_guidelines_[2];

    Float chassis_power_number_;

    Arc chassis_direction_indicator_, chassis_control_direction_indicator_;

    Float chassis_control_power_limit_indicator_;

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Hero, rmcs_executor::Component)