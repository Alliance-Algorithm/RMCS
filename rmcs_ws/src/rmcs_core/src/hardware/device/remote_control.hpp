#pragma once

#include <cstring>

#include <eigen3/Eigen/Dense>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/dr16.hpp"
#include "hardware/device/vt13.hpp"

namespace rmcs_core::hardware::device {

/*
遥控输入仲裁：
- vt13 valid S挡：vt13主控 | 比赛用
- vt13 valid C挡：等同于dr16双下 | 疯车救车
- 其他情况：dr16主控；dr16无效则进入空安全态
- 旋钮始终来自 dr16，dr16 无效则清零
*/

class RemoteControl {
public:
    explicit RemoteControl(rmcs_executor::Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_output_, Eigen::Vector2d::Zero());
        component.register_output(
            "/remote/joystick/left", joystick_left_output_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/right", switch_right_output_, rmcs_msgs::Switch::UNKNOWN);
        component.register_output(
            "/remote/switch/left", switch_left_output_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output("/remote/rotary_knob", rotary_knob_output_, 0.0);
        component.register_output(
            "/remote/rotary_knob_switch", rotary_knob_switch_output_, rmcs_msgs::Switch::UNKNOWN);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_output_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_output_, 0.0);

        component.register_output("/remote/mouse", mouse_output_, rmcs_msgs::Mouse::zero());
        component.register_output(
            "/remote/keyboard", keyboard_output_, rmcs_msgs::Keyboard::zero());
    }

    void register_dr16(Dr16* dr16) { dr16_ = dr16; }
    void register_vt13(Vt13* vt13) { vt13_ = vt13; }

    void update() {
        update_timeout_interlock();

        const auto control_source = select_control_source();
        const auto snapshot = build_snapshot(control_source);

        *joystick_right_output_ = snapshot.joystick_right;
        *joystick_left_output_ = snapshot.joystick_left;

        *switch_right_output_ = snapshot.switch_right;
        *switch_left_output_ = snapshot.switch_left;

        *mouse_velocity_output_ = snapshot.mouse_velocity;
        *mouse_wheel_output_ = snapshot.mouse_wheel;

        *mouse_output_ = snapshot.mouse;
        *keyboard_output_ = snapshot.keyboard;

        if (dr16_ && dr16_->valid()) {
            *rotary_knob_output_ = dr16_->rotary_knob();
            *rotary_knob_switch_output_ = dr16_->rotary_knob_switch();
        } else {
            *rotary_knob_output_ = 0.0;
            *rotary_knob_switch_output_ = rmcs_msgs::Switch::UNKNOWN;
        }
    }

private:
    enum class ControlSource {
        kDr16,
        kVt13Sport,
        kCineSafe,
        kInvalidSafe,
    };

    struct Snapshot {
        Eigen::Vector2d joystick_right = Eigen::Vector2d::Zero();
        Eigen::Vector2d joystick_left = Eigen::Vector2d::Zero();

        rmcs_msgs::Switch switch_right = rmcs_msgs::Switch::UNKNOWN;
        rmcs_msgs::Switch switch_left = rmcs_msgs::Switch::UNKNOWN;

        Eigen::Vector2d mouse_velocity = Eigen::Vector2d::Zero();
        double mouse_wheel = 0.0;

        rmcs_msgs::Mouse mouse = rmcs_msgs::Mouse::zero();
        rmcs_msgs::Keyboard keyboard = rmcs_msgs::Keyboard::zero();
    };

    // 超时互锁：仅当对方 valid 时本设备才允许超时失效，保证至少一路不失效
    auto update_timeout_interlock() const -> void {
        const auto dr16_ok = dr16_ && dr16_->valid();
        const auto vt13_ok = vt13_ && vt13_->valid();
        if (dr16_)
            dr16_->set_timeout_enabled(vt13_ok);
        if (vt13_)
            vt13_->set_timeout_enabled(dr16_ok);
    }

    ControlSource select_control_source() const {
        if (vt13_ && vt13_->valid()) {
            switch (vt13_->mode_switch()) {
            case Vt13::ModeSwitch::kSport: return ControlSource::kVt13Sport;
            case Vt13::ModeSwitch::kCine: return ControlSource::kCineSafe;
            case Vt13::ModeSwitch::kNormal:
            case Vt13::ModeSwitch::kUnknown: break;
            }
        }

        return (dr16_ && dr16_->valid()) ? ControlSource::kDr16 : ControlSource::kInvalidSafe;
    }

    Snapshot build_snapshot(ControlSource source) const {
        Snapshot snapshot{};
        switch (source) {
        case ControlSource::kDr16:
            snapshot.joystick_right = dr16_->joystick_right();
            snapshot.joystick_left = dr16_->joystick_left();
            snapshot.switch_right = dr16_->switch_right();
            snapshot.switch_left = dr16_->switch_left();
            snapshot.mouse_velocity = dr16_->mouse_velocity();
            snapshot.mouse_wheel = dr16_->mouse_wheel();
            snapshot.mouse = dr16_->mouse();
            snapshot.keyboard = dr16_->keyboard();
            break;
        case ControlSource::kVt13Sport:
            snapshot.joystick_right = vt13_->joystick_right();
            snapshot.joystick_left = vt13_->joystick_left();
            snapshot.switch_right = rmcs_msgs::Switch::MIDDLE;
            snapshot.switch_left = rmcs_msgs::Switch::MIDDLE;
            snapshot.mouse_velocity = vt13_->mouse_velocity();
            snapshot.mouse_wheel = vt13_->mouse_wheel();
            snapshot.mouse = vt13_->mouse();
            snapshot.keyboard = vt13_->keyboard();
            break;
        case ControlSource::kCineSafe:
            snapshot.switch_right = rmcs_msgs::Switch::DOWN;
            snapshot.switch_left = rmcs_msgs::Switch::DOWN;
            break;
        case ControlSource::kInvalidSafe: break;
        }

        return snapshot;
    }

    Dr16* dr16_{nullptr};
    Vt13* vt13_{nullptr};

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_output_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_right_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> switch_left_output_;

    rmcs_executor::Component::OutputInterface<double> rotary_knob_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Switch> rotary_knob_switch_output_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_output_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_output_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_output_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_output_;
};

} // namespace rmcs_core::hardware::device
