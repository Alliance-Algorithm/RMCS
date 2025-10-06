#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>

#include "referee/app/ui/shape/shape.hpp"
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
        , status_ring_(26.5, 26.5, 600, 40)
        , rangefinder_()
        , chassis_direction_indicator_(Shape::Color::PINK, 8, x_center, y_center, 0, 0, 84, 84)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false) {

        chassis_control_direction_indicator_.set_x(x_center);
        chassis_control_direction_indicator_.set_y(y_center);

        register_input("/chassis/control_mode", chassis_mode_);

        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);
        // register_input("/chassis/supercap/control_enable", supercap_control_enabled_);

        register_input("/chassis/voltage", chassis_voltage_);
        register_input("/chassis/power", chassis_power_);
        register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        register_input("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);

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
    }

    void update() override {
        update_normal_ui();
        update_sniper_ui();

        if (*is_scope_active_) {
            set_normal_ui_visible(false);
            rangefinder_.set_visible(true);
        } else {
            set_normal_ui_visible(true);
            rangefinder_.set_visible(false);
        }
    }

private:
    void set_normal_ui_visible(bool value) {
        status_ring_.set_visible(value);

        chassis_direction_indicator_.set_visible(value);
        chassis_control_direction_indicator_.set_visible(value);
    }

    void update_normal_ui() {
        update_chassis_direction_indicator();

        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_supercap(*supercap_voltage_, true);
        status_ring_.update_battery_power(*chassis_voltage_);
        update_static_status_ring();
    }

    void update_sniper_ui() {
        auto display_angle = *gimbal_pitch_angle_ > std::numbers::pi / 2
                               ? *gimbal_pitch_angle_ - 2 * std::numbers::pi
                               : *gimbal_pitch_angle_;

        rangefinder_.update_pitch_angle(-display_angle);

        double raw_height = -display_angle / 0.7 * static_cast<double>(height_max);
        raw_height = std::clamp(raw_height, 0.0, static_cast<double>(height_max));
        uint16_t lift_height = static_cast<uint16_t>(std::round(raw_height));

        lift_height = std::clamp(lift_height, height_min, height_max);
        rangefinder_.update_vertical_rangefinder(lift_height);
    }

    void update_time_reminder() {
        if (!game_stage_.ready())
            return;
    }

    void update_static_status_ring() {
        auto auto_aim_enable = mouse_->right == 1;
        auto precise_enable = *shoot_mode_ == rmcs_msgs::ShootMode::PRECISE;

        status_ring_.update_static_parts({auto_aim_enable, precise_enable});
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

    StatusRing status_ring_;
    Rangefinder rangefinder_;

    Arc chassis_direction_indicator_, chassis_control_direction_indicator_;

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Hero, rmcs_executor::Component)