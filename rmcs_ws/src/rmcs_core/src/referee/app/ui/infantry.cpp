#include <algorithm>
#include <cmath>
#include <cstdint>

#include <game_stage.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/mouse.hpp>

#include "librmcs/utility/logging.hpp"

#include "referee/app/ui/position.hpp"
#include "referee/app/ui/shape/shape.hpp"
#include "referee/app/ui/widget/crosshair.hpp"
#include "referee/app/ui/widget/status_ring.hpp"
#include "referee/status/field.hpp"

namespace rmcs_core::referee::app::ui {
using namespace std::chrono_literals;

class Infantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Infantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_(Shape::Color::WHITE, x_center - 12, y_center - 37)
        , status_ring_()
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, 800, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, 200})
        , chassis_power_number_(Shape::Color::WHITE, 20, 2, x_center - 40, 860, 0)
        , yaw_indicator_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 32, 830, x_center + 32, 830},
              {Shape::Color::WHITE, 2, x_center, 830, x_center, 820})
        , chassis_direction_indicator_(Shape::Color::PINK, 8, x_center, y_center, 0, 0, 84, 84)
        , chassis_control_power_limit_indicator_(Shape::Color::WHITE, 20, 2, x_center + 10, 820, 0)
        , supercap_control_power_limit_indicator_(Shape::Color::WHITE, 20, 2, x_center + 10, 790, 0)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false)
        , hero_bullet_allowance(
              Shape::Color::BLACK, 10, 5, x_center + 150, red_hero_x, robot_y, true)
        , infantry_III_bullet_allowance(
              Shape::Color::BLACK, 10, 5, x_center + 150, red_hero_x, robot_y, true)
        , infantry_IV_bullet_allowance(
              Shape::Color::BLACK, 10, 5, x_center + 150, red_hero_x, robot_y, true)
        , infantry_V_bullet_allowance(
              Shape::Color::BLACK, 10, 5, x_center + 150, red_hero_x, robot_y, true) {

        chassis_control_direction_indicator_.set_x(x_center);
        chassis_control_direction_indicator_.set_y(y_center);

        //for communicate
        register_input("/referee/id", robot_id_);
        register_input("/referee/communicate", communicate_data);

        register_input("/chassis/control_mode", chassis_mode_);

        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/enabled", supercap_enabled_);

        register_input("/chassis/voltage", chassis_voltage_);
        register_input("/chassis/power", chassis_power_);
        register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        register_input("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);

        register_input("/gimbal/left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);

        // register_input("/auto_aim/ui_target", auto_aim_target_, false);
    }

    void update() override {
        if (*robot_id_ >= rmcs_msgs::RobotId::RED_HERO
            && *robot_id_ <= rmcs_msgs::RobotId::RED_BASE) {
            switch (communicate_data->header.sender_id) {
            case rmcs_msgs::FullRobotId::RED_HERO:
                hero_bullet_allowance.set_x(red_hero_x);
                hero_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_ENGINEER:
                engineer_bullet_allowance.set_x(red_engineer_x);
                engineer_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_III:
                infantry_III_bullet_allowance.set_x(red_infantry_III_x);
                infantry_III_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_IV:
                infantry_IV_bullet_allowance.set_x(red_infantry_IV_x);
                infantry_IV_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_V:
                infantry_V_bullet_allowance.set_x(red_infantry_V_x);
                infantry_V_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_SENTRY:
                sentry_bullet_allowance.set_x(red_sentry_x);
                sentry_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            }
        } else {
            switch (communicate_data->header.sender_id) {
            case rmcs_msgs::FullRobotId::BLUE_HERO:
                hero_bullet_allowance.set_x(blue_hero_x);
                hero_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_ENGINEER:
                engineer_bullet_allowance.set_x(blue_engineer_x);
                engineer_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_III:
                infantry_III_bullet_allowance.set_x(blue_infantry_III_x);
                infantry_III_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_IV:
                infantry_IV_bullet_allowance.set_x(blue_infantry_IV_x);
                infantry_IV_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_V:
                infantry_V_bullet_allowance.set_x(blue_infantry_V_x);
                infantry_V_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_SENTRY:
                sentry_bullet_allowance.set_x(blue_sentry_x);
                sentry_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            }
        }

        update_chassis_direction_indicator();

        chassis_control_power_limit_indicator_.set_value(*chassis_control_power_limit_);
        supercap_control_power_limit_indicator_.set_value(*supercap_charge_power_limit_);

        chassis_power_number_.set_value(*chassis_power_);

        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_supercap(*supercap_voltage_, *supercap_enabled_);
        status_ring_.update_battery_power(*chassis_voltage_);

        if (*robot_id_ >= rmcs_msgs::RobotId::RED_HERO
            && *robot_id_ <= rmcs_msgs::RobotId::RED_BASE) {
            switch (communicate_data->header.sender_id) {
            case rmcs_msgs::FullRobotId::RED_HERO:
                hero_bullet_allowance.set_x(red_hero_x);
                hero_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_ENGINEER:
                engineer_bullet_allowance.set_x(red_engineer_x);
                engineer_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_III:
                infantry_III_bullet_allowance.set_x(red_infantry_III_x);
                infantry_III_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_IV:
                infantry_IV_bullet_allowance.set_x(red_infantry_IV_x);
                infantry_IV_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_INFANTRY_V:
                infantry_V_bullet_allowance.set_x(red_infantry_V_x);
                infantry_V_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::RED_SENTRY:
                sentry_bullet_allowance.set_x(red_sentry_x);
                sentry_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            }
        } else {
            switch (communicate_data->header.sender_id) {
            case rmcs_msgs::FullRobotId::BLUE_HERO:
                hero_bullet_allowance.set_x(blue_hero_x);
                hero_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_ENGINEER:
                engineer_bullet_allowance.set_x(blue_engineer_x);
                engineer_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_III:
                infantry_III_bullet_allowance.set_x(blue_infantry_III_x);
                infantry_III_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_IV:
                infantry_IV_bullet_allowance.set_x(blue_infantry_IV_x);
                infantry_IV_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_INFANTRY_V:
                infantry_V_bullet_allowance.set_x(blue_infantry_V_x);
                infantry_V_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            case rmcs_msgs::FullRobotId::BLUE_SENTRY:
                sentry_bullet_allowance.set_x(blue_sentry_x);
                sentry_bullet_allowance.set_value(communicate_data->data.bullet_allowance);
                break;
            }
        }

        status_ring_.update_auto_aim_enable(mouse_->right == 1);
    }

private:
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

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_, chassis_control_angle_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

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

    // InputInterface<std::pair<uint16_t, uint16_t>> auto_aim_target_;

    CrossHair crosshair_;
    StatusRing status_ring_;

    Line horizontal_center_guidelines_[2];
    Line vertical_center_guidelines_[2];

    Float chassis_power_number_;
    Line yaw_indicator_guidelines_[2];

    Arc chassis_direction_indicator_, chassis_control_direction_indicator_;

    Float chassis_control_power_limit_indicator_, supercap_control_power_limit_indicator_;

    Integer time_reminder_;

    // for communication
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<status::CommunicateDataWithHeader<status::CommunicateData>> communicate_data;
    Integer hero_bullet_allowance;
    Integer engineer_bullet_allowance;
    Integer infantry_III_bullet_allowance;
    Integer infantry_IV_bullet_allowance;
    Integer infantry_V_bullet_allowance;
    Integer sentry_bullet_allowance;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Infantry, rmcs_executor::Component)