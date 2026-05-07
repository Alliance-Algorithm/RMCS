#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/mouse.hpp>

#include "referee/app/ui/shape/shape.hpp"
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
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , crosshair_circle_(Shape::Color::WHITE, x_center - 2, y_center - 30, 8, 2)
        , status_ring_(26.5, 26.5, 600, 300)
        , horizontal_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center - 360, y_center, x_center - 110, y_center},
              {Shape::Color::WHITE, 2, x_center + 110, y_center, x_center + 360, y_center})
        , vertical_center_guidelines_(
              {Shape::Color::WHITE, 2, x_center, 800, x_center, y_center + 110},
              {Shape::Color::WHITE, 2, x_center, y_center - 110, x_center, 200})
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

        register_input("/chassis/control_mode", chassis_mode_);

        register_input("/chassis/angle", chassis_angle_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);
        register_input("/chassis/supercap/enabled", supercap_enabled_);

        register_input("/chassis/voltage", chassis_voltage_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_, false);
        register_input(
            "/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_, false);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_, false);

        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);

        register_input("/gimbal/left_friction/control_velocity", left_friction_control_velocity_);
        register_input("/gimbal/left_friction/velocity", left_friction_velocity_);
        register_input("/gimbal/right_friction/velocity", right_friction_velocity_);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);
    }

    void update() override {
        update_chassis_direction_indicator();
        update_deformable_chassis_leg_arcs();

        status_ring_.update_bullet_allowance(*robot_bullet_allowance_);
        status_ring_.update_friction_wheel_speed(
            std::min(*left_friction_velocity_, *right_friction_velocity_),
            *left_friction_control_velocity_ > 0);
        status_ring_.update_supercap(*supercap_voltage_, *supercap_enabled_);
        status_ring_.update_battery_power(*chassis_voltage_);

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
        chassis_direction_indicator_.set_color(chassis_direction_indicator_color(chassis_mode));
        chassis_direction_indicator_.set_angle(to_referee_angle(*chassis_angle_), 30);
    }

    static Shape::Color chassis_direction_indicator_color(rmcs_msgs::ChassisMode mode) {
        switch (mode) {
        case rmcs_msgs::ChassisMode::SPIN: return Shape::Color::GREEN;
        case rmcs_msgs::ChassisMode::AUTO: return Shape::Color::CYAN;
        case rmcs_msgs::ChassisMode::STEP_DOWN: return Shape::Color::WHITE;
        default: return Shape::Color::PINK;
        }
    }

    void update_deformable_chassis_leg_arcs() {
        if (!left_front_joint_physical_angle_.ready() || !left_back_joint_physical_angle_.ready()
            || !right_back_joint_physical_angle_.ready()
            || !right_front_joint_physical_angle_.ready()) {
            deformable_chassis_leg_arcs_.set_visible(false);
            return;
        }

        const std::array<double, 4> leg_angles = {
            *left_front_joint_physical_angle_,
            *left_back_joint_physical_angle_,
            *right_back_joint_physical_angle_,
            *right_front_joint_physical_angle_,
        };
        deformable_chassis_leg_arcs_.update(*chassis_angle_, leg_angles);
    }

    static constexpr uint16_t screen_width = 1920, screen_height = 1080;
    static constexpr uint16_t x_center = screen_width / 2, y_center = screen_height / 2;

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_enabled_;

    InputInterface<double> chassis_voltage_;

    InputInterface<double> left_front_velocity_, left_back_velocity_, right_back_velocity_,
        right_front_velocity_;
    InputInterface<double> left_front_joint_physical_angle_, left_back_joint_physical_angle_,
        right_back_joint_physical_angle_, right_front_joint_physical_angle_;

    InputInterface<uint16_t> robot_bullet_allowance_;

    InputInterface<double> left_friction_control_velocity_;
    InputInterface<double> left_friction_velocity_;
    InputInterface<double> right_friction_velocity_;

    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;

    CrossHairCircle crosshair_circle_;
    StatusRing status_ring_;

    Line horizontal_center_guidelines_[2];
    Line vertical_center_guidelines_[2];

    Arc chassis_direction_indicator_;
    DeformableChassisLegArcs deformable_chassis_leg_arcs_;

    Integer time_reminder_;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::DeformableInfantry, rmcs_executor::Component)
