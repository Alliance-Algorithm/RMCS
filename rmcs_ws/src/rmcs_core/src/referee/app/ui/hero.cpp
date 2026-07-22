#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_condiction.hpp>
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
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , status_ring_(26.5, 26.5, 600, 40)
        // , chassis_direction_indicator_(Shape::Color::PINK, 8, x_center, y_center, 0, 0, 84, 84)
        , chassis_left_wheel_indicator_(
              Shape::Color::WHITE, wheel_indicator_width, x_center, y_center, 0, 0,
              wheel_indicator_radius, wheel_indicator_radius)
        , chassis_right_wheel_indicator_(
              Shape::Color::WHITE, wheel_indicator_width, x_center, y_center, 0, 0,
              wheel_indicator_radius, wheel_indicator_radius)
        , pitch_angle_number_(
              Shape::Color::YELLOW, 20, 5, x_center + 270, y_center - 35, 0.0, false)
        , bottom_yaw_angle_number_(
              Shape::Color::YELLOW, 20, 5, x_center + 270, y_center - 65, 0.0, false)
        , time_reminder_(Shape::Color::PINK, 50, 5, x_center + 150, y_center + 65, 0, false)
        , bullet_allowance_number_(
              Shape::Color::YELLOW, 20, 5, x_center - 220, y_center + 270, 0, false)
        , friction_profile_number_(
              Shape::Color::GREEN, friction_profile_number_font_size, 5, 0, 0, 12, false)
        , friction_profile_indicator_{
              Line(Shape::Color::WHITE, friction_profile_box_line_width, 0, 0, 0, 0, false),
              Line(Shape::Color::WHITE, friction_profile_box_line_width, 0, 0, 0, 0, false),
              Line(Shape::Color::WHITE, friction_profile_box_line_width, 0, 0, 0, 0, false),
              Line(Shape::Color::WHITE, friction_profile_box_line_width, 0, 0, 0, 0, true)} {

        chassis_control_direction_indicator_.set_x(x_center);
        chassis_control_direction_indicator_.set_y(y_center);

        register_input("/gimbal/mode", gimbal_mode_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/chassis/control_mode", chassis_mode_);

        register_input("/chassis/angle", chassis_angle_);
        register_input("/chassis/control_angle", chassis_control_angle_);

        register_input("/chassis/climber/left_front_motor/velocity", left_track_velocity_);
        register_input("/chassis/climber/right_front_motor/velocity", right_track_velocity_);

        register_input("/chassis/supercap/voltage", supercap_voltage_);

        register_input("/chassis/voltage", chassis_voltage_);
        register_input("/chassis/power", chassis_power_);
        register_input("/chassis/control_power_limit", chassis_control_power_limit_);
        register_input("/chassis/supercap/charge_power_limit", supercap_charge_power_limit_);
        register_input("/gimbal/control_bullet_allowance/limited_by_heat", robot_bullet_allowance_);

        register_input(
            "/gimbal/first_back_friction/control_velocity", back_friction_control_velocity_);
        register_input("/gimbal/first_back_friction/velocity", back_friction_velocity_);
        register_input("/gimbal/first_front_friction/velocity", front_friction_velocity_);
        register_input("/gimbal/friction_profile_1_active", friction_profile_1_active_, false);

        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);
        register_input("/gimbal/pitch/raw_angle", gimbal_pitch_raw_angle_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/raw_angle", bottom_yaw_raw_angle_);

        register_input("/gimbal/shooter/preloaded_ready", shooter_preloaded_ready_, false);

        register_input("/remote/mouse", mouse_);

        register_input("/referee/game/stage", game_stage_);
    }

    void update() override {
        update_normal_ui();
        // update_sniper_ui();

        set_normal_ui_visible(true);
    }

private:
    static uint16_t count_digits(int32_t value) {
        uint16_t digits = value <= 0 ? 1 : 0;
        while (value != 0) {
            value /= 10;
            ++digits;
        }
        return digits;
    }
    void set_normal_ui_visible(bool value) {
        status_ring_.set_visible(value);

        chassis_left_wheel_indicator_.set_visible(value);
        chassis_right_wheel_indicator_.set_visible(value);
        pitch_angle_number_.set_visible(value);
        bottom_yaw_angle_number_.set_visible(value);
        bullet_allowance_number_.set_visible(value);
        friction_profile_number_.set_visible(value);
        const bool show_friction_profile_box =
            value && friction_profile_1_active_.ready() && *friction_profile_1_active_;
        for (auto& line : friction_profile_indicator_)
            line.set_visible(show_friction_profile_box);
        if (!value)
            chassis_control_direction_indicator_.set_visible(false);
    }

    void update_normal_ui() {
        update_chassis_direction_indicator();
        pitch_angle_number_.set_value(static_cast<double>(*gimbal_pitch_raw_angle_));
        update_pitch_raw_angle_color();
        bottom_yaw_angle_number_.set_value(static_cast<double>(*bottom_yaw_raw_angle_));
        // update_bottom_yaw_tracking_lines();
        const int32_t bullet_allowance =
            static_cast<int32_t>(std::max<int64_t>(0, *robot_bullet_allowance_));
        bullet_allowance_number_.set_value(bullet_allowance);
        const bool shooter_preloaded_ready =
            shooter_preloaded_ready_.ready() && *shooter_preloaded_ready_;
        bullet_allowance_number_.set_color(
            shooter_preloaded_ready ? Shape::Color::GREEN : Shape::Color::PINK);

        const uint16_t pitch_right =
            pitch_raw_angle_x
            + count_digits(static_cast<int32_t>(*gimbal_pitch_raw_angle_)) * raw_angle_font_size;
        const uint16_t box_left = pitch_right + friction_profile_box_gap;
        const uint16_t box_right = box_left + friction_profile_box_visual_width;

        const uint16_t box_top = friction_profile_box_top_y;
        const uint16_t box_bottom = pitch_raw_angle_y;

        const bool friction_profile_1_active =
            friction_profile_1_active_.ready() && *friction_profile_1_active_;
        const uint16_t box_center_x = (box_left + box_right) / 2;
        const uint16_t profile_number_y = box_top + friction_profile_number_gap;

        for (auto& line : friction_profile_indicator_)
            line.set_color(Shape::Color::GREEN);

        friction_profile_number_.set_value(friction_profile_1_active ? 16 : 12);
        friction_profile_number_.set_color(
            friction_profile_1_active ? Shape::Color::PINK : Shape::Color::GREEN);
        friction_profile_number_.set_font_size(friction_profile_number_font_size);
        friction_profile_number_.set_xy(box_left, profile_number_y);
        friction_profile_number_.set_center_x(box_center_x);

        friction_profile_indicator_[0].set_x(box_left);
        friction_profile_indicator_[0].set_y(box_top);
        friction_profile_indicator_[0].set_x2(box_right);
        friction_profile_indicator_[0].set_y2(box_top);

        friction_profile_indicator_[1].set_x(box_right);
        friction_profile_indicator_[1].set_y(box_top);
        friction_profile_indicator_[1].set_x2(box_right);
        friction_profile_indicator_[1].set_y2(box_bottom);

        friction_profile_indicator_[2].set_x(box_left);
        friction_profile_indicator_[2].set_y(box_bottom);
        friction_profile_indicator_[2].set_x2(box_right);
        friction_profile_indicator_[2].set_y2(box_bottom);

        friction_profile_indicator_[3].set_x(box_left);
        friction_profile_indicator_[3].set_y(box_top);
        friction_profile_indicator_[3].set_x2(box_left);
        friction_profile_indicator_[3].set_y2(box_bottom);
        status_ring_.update_friction_wheel_speed(
            std::min(*back_friction_velocity_, *front_friction_velocity_),
            *back_friction_control_velocity_ > 0);
        status_ring_.update_supercap(*supercap_voltage_, true);
        status_ring_.update_battery_power(*chassis_voltage_);
        last_keyboard_ = *keyboard_;
    }

    void update_pitch_raw_angle_color() {
        if (keyboard_.ready()) {
            if (!last_keyboard_.e && keyboard_->e) {
                last_e_triggered_with_ctrl_ = keyboard_->ctrl;
            }
            last_keyboard_ = *keyboard_;
        }

        if (!gimbal_mode_.ready()) {
            pitch_angle_number_.set_color(Shape::Color::YELLOW);
            return;
        }

        const bool is_encoder = *gimbal_mode_ == rmcs_msgs::GimbalMode::ENCODER;
        const bool entering_encoder =
            last_gimbal_mode_ != rmcs_msgs::GimbalMode::ENCODER && is_encoder;
        const bool leaving_encoder =
            last_gimbal_mode_ == rmcs_msgs::GimbalMode::ENCODER && !is_encoder;

        if (entering_encoder) {
            pitch_encoder_by_ctrl_e_ = last_e_triggered_with_ctrl_;
        }

        if (leaving_encoder) {
            pitch_encoder_by_ctrl_e_ = false;
        }

        if (!is_encoder) {
            pitch_angle_number_.set_color(Shape::Color::YELLOW);
        } else if (pitch_encoder_by_ctrl_e_) {
            pitch_angle_number_.set_color(Shape::Color::PINK);
        } else {
            pitch_angle_number_.set_color(Shape::Color::GREEN);
        }

        last_gimbal_mode_ = *gimbal_mode_;
    }

    void update_sniper_ui() {
        auto display_angle = *gimbal_pitch_angle_ > std::numbers::pi / 2
                               ? *gimbal_pitch_angle_ - 2 * std::numbers::pi
                               : *gimbal_pitch_angle_;

        rangefinder_.update_pitch_angle(static_cast<double>(*gimbal_pitch_raw_angle_));

        double raw_height = -display_angle / 0.7 * static_cast<double>(height_max);
        raw_height = std::clamp(raw_height, 0.0, static_cast<double>(height_max));
        uint16_t lift_height = static_cast<uint16_t>(std::round(raw_height));

        lift_height = std::clamp(lift_height, height_min, height_max);
        rangefinder_.update_vertical_rangefinder(lift_height);
        pitch_angle_number_.set_value(static_cast<double>(*gimbal_pitch_raw_angle_));
    }

    void update_bottom_yaw_tracking_lines() {
        const bool ctrl_e_pressed = keyboard_->ctrl && keyboard_->e;
        if (ctrl_e_pressed) {
            bottom_yaw_tracking_enabled_ = !bottom_yaw_tracking_enabled_;
            bottom_yaw_anchor_angle_rad_ = *bottom_yaw_angle_;
        }

        const double delta_bottom_yaw_rad = *bottom_yaw_angle_ - bottom_yaw_anchor_angle_rad_;

        const int min_center_x = static_cast<int>(pink_line_half_length);
        const int max_center_x =
            static_cast<int>(screen_width) - static_cast<int>(pink_line_half_length);

        const int pink_center_x = std::clamp(
            static_cast<int>(std::lround(
                static_cast<double>(x_center)
                + delta_bottom_yaw_rad * pink_line_pixels_per_radian)),
            min_center_x, max_center_x);

        tracking_pink_line_.set_x(
            static_cast<uint16_t>(pink_center_x - static_cast<int>(pink_line_half_length)));
        tracking_pink_line_.set_y(y_center + pink_line_offset_y);
        tracking_pink_line_.set_x2(
            static_cast<uint16_t>(pink_center_x + static_cast<int>(pink_line_half_length)));
        tracking_pink_line_.set_y2(y_center + pink_line_offset_y);
        tracking_pink_line_.set_visible(true);
    }

    void update_time_reminder() {
        if (!game_stage_.ready())
            return;
    }

    void update_chassis_direction_indicator() {
        auto chassis_mode = *chassis_mode_;

        auto to_referee_angle = [](double angle) {
            int degrees = static_cast<int>(
                std::lround((2.0 * std::numbers::pi - angle) / std::numbers::pi * 180.0));
            degrees %= 360;
            if (degrees < 0)
                degrees += 360;
            return static_cast<uint16_t>(degrees);
        };
        const bool left_track_active =
            std::abs(*left_track_velocity_) > track_velocity_active_threshold;
        const bool right_track_active =
            std::abs(*right_track_velocity_) > track_velocity_active_threshold;

        chassis_left_wheel_indicator_.set_color(
            left_track_active ? Shape::Color::GREEN : Shape::Color::WHITE);
        chassis_right_wheel_indicator_.set_color(
            right_track_active ? Shape::Color::GREEN : Shape::Color::WHITE);

        const double wheel_offset = wheel_indicator_offset_deg * std::numbers::pi / 180.0;
        const double left_wheel_angle = *chassis_angle_ + wheel_offset;
        const double right_wheel_angle = *chassis_angle_ - wheel_offset;

        chassis_left_wheel_indicator_.set_angle(
            to_referee_angle(left_wheel_angle), wheel_indicator_half_angle);
        chassis_right_wheel_indicator_.set_angle(
            to_referee_angle(right_wheel_angle), wheel_indicator_half_angle);

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

    static constexpr uint16_t raw_angle_font_size = 20;

    static constexpr uint16_t pitch_raw_angle_x = x_center + 270;
    static constexpr uint16_t pitch_raw_angle_y = y_center - 65;

    static constexpr uint16_t friction_profile_box_top_y = y_center + 85;
    static constexpr uint16_t friction_profile_box_gap = 18;
    static constexpr uint16_t friction_profile_box_visual_width = 105;
    static constexpr uint16_t friction_profile_box_line_width = 6;
    static constexpr uint16_t friction_profile_number_font_size = 24;
    static constexpr uint16_t friction_profile_number_gap = 12;

    static constexpr uint16_t height_min = 0, height_max = 500;

    static constexpr uint16_t wheel_indicator_radius = 110;
    static constexpr uint16_t wheel_indicator_width = 12;
    static constexpr uint16_t wheel_indicator_half_angle = 10;
    static constexpr double wheel_indicator_offset_deg = 28.0;
    static constexpr double track_velocity_active_threshold = 1.0;
    static constexpr uint16_t green_line_half_length = 8;
    static constexpr uint16_t green_line_width = 40;
    static constexpr uint16_t green_line_offset_y = 0;

    static constexpr uint16_t pink_line_half_length = 40;
    static constexpr uint16_t pink_line_width = 8;
    static constexpr uint16_t pink_line_offset_y = 0;

    static constexpr double pink_line_pixels_per_radian = 400.0;

    InputInterface<rmcs_msgs::GimbalMode> gimbal_mode_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();
    rmcs_msgs::GimbalMode last_gimbal_mode_ = rmcs_msgs::GimbalMode::IMU;

    bool last_e_triggered_with_ctrl_ = false;
    bool pitch_encoder_by_ctrl_e_ = false;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<double> chassis_angle_, chassis_control_angle_;
    InputInterface<double> left_track_velocity_, right_track_velocity_;

    InputInterface<double> supercap_voltage_;
    InputInterface<bool> supercap_control_enabled_;

    InputInterface<double> chassis_voltage_;
    InputInterface<double> chassis_power_;
    InputInterface<double> chassis_control_power_limit_;
    InputInterface<double> supercap_charge_power_limit_;

    InputInterface<int64_t> robot_bullet_allowance_;

    InputInterface<double> back_friction_control_velocity_;
    InputInterface<double> back_friction_velocity_;
    InputInterface<double> front_friction_velocity_;
    InputInterface<bool> friction_profile_1_active_;

    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;

    InputInterface<double> gimbal_yaw_angle_;
    InputInterface<double> gimbal_pitch_angle_;
    InputInterface<int64_t> gimbal_pitch_raw_angle_;
    InputInterface<int64_t> bottom_yaw_raw_angle_;
    InputInterface<double> bottom_yaw_angle_;

    InputInterface<bool> shooter_preloaded_ready_;

    StatusRing status_ring_;
    Rangefinder rangefinder_;

    Arc chassis_left_wheel_indicator_;
    Arc chassis_right_wheel_indicator_;
    Arc chassis_control_direction_indicator_;

    Float pitch_angle_number_;
    Float bottom_yaw_angle_number_;

    Text state_word_;
    Integer time_reminder_;

    Integer bullet_allowance_number_;
    Integer friction_profile_number_;
    Line friction_profile_indicator_[4];
    Line center_green_line_;
    Line tracking_pink_line_;

    bool bottom_yaw_tracking_enabled_ = false;
    double bottom_yaw_anchor_angle_rad_ = 0.0;
};

} // namespace rmcs_core::referee::app::ui

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::app::ui::Hero, rmcs_executor::Component)
