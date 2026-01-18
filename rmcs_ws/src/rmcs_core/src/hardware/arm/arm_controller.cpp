#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/endian_promise.hpp"
#include "hardware/fsm/FSM.hpp"
#include "hardware/fsm/FSM_gold_l.hpp"
#include "hardware/fsm/FSM_gold_m.hpp"
#include "hardware/fsm/FSM_gold_r.hpp"
#include "hardware/fsm/FSM_sliver.hpp"
#include "hardware/fsm/FSM_storage_lb.hpp"
#include "hardware/fsm/FSM_storage_rb.hpp"
#include "hardware/fsm/FSM_up_stairs.hpp"
#include "hardware/fsm/FSM_walk.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::hardware::arm {
class ArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        // register_input("/referee/vision/custom", custom_controller);

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);
        register_output("/arm/Joint6/target_theta", target_theta[5], nan);
        register_output("/arm/Joint5/target_theta", target_theta[4], nan);
        register_output("/arm/Joint4/target_theta", target_theta[3], nan);
        register_output("/arm/Joint3/target_theta", target_theta[2], nan);
        register_output("/arm/Joint2/target_theta", target_theta[1], nan);
        register_output("/arm/Joint1/target_theta", target_theta[0], nan);

        register_output("/arm/Joint6/control_torque", control_torque[5], nan);
        register_output("/arm/Joint5/control_torque", control_torque[4], nan);
        register_output("/arm/Joint4/control_torque", control_torque[3], nan);
        register_output("/arm/Joint3/control_torque", control_torque[2], nan);
        register_output("/arm/Joint2/control_torque", control_torque[1], nan);
        register_output("/arm/Joint1/control_torque", control_torque[0], nan);

        register_input("/arm/Joint1/raw_angle", joint1_raw_angle);
        register_output("/arm/Joint1/zero_point", joint1_zero_point, 14381);

        register_output("/arm/enable_flag", is_arm_enable, true);
        register_output("/arm/pump/target_vel", arm_pump_target_vel, NAN);
        register_output("/mine/pump/target_vel", mine_pump_target_vel, NAN);
        register_output("/arm/pump/relay/CH", arm_pump_relay, 0b00000000);
        publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("/engineer/joint/measure", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/engineer/joint/control", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 6) {
                    *target_theta[0] = static_cast<double>(msg->data[0]);
                    *target_theta[1] = -static_cast<double>(msg->data[1]);
                    *target_theta[2] = -static_cast<double>(msg->data[2]) + std::numbers::pi / 2;
                    *target_theta[3] = static_cast<double>(msg->data[3]);
                    *target_theta[4] = static_cast<double>(msg->data[4]);
                    *target_theta[5] = static_cast<double>(msg->data[5]);
                }
            });
        *arm_mode = rmcs_msgs::ArmMode::None;
        register_output("/arm/mode", arm_mode, rmcs_msgs::ArmMode::None);
    }
    void update() override {
        // auto switch_right = *switch_right_;
        // auto switch_left  = *switch_left_;
        // auto mouse        = *mouse_;
        // auto keyboard     = *keyboard_;
        // auto msg          = std_msgs::msg::Float32MultiArray();
        // msg.data          = {
        //     static_cast<float>(*theta[0]),
        //     -static_cast<float>(*theta[1]),
        //     -(static_cast<float>(*theta[2] - std::numbers::pi / 2)),
        //     static_cast<float>(*theta[3]),
        //     static_cast<float>(*theta[4]),
        //     static_cast<float>(*theta[5])};
        // publisher_->publish(msg);
        // if (keyboard.e) {
        //     if (keyboard.ctrl && keyboard.shift) {
        //         *joint1_zero_point = *joint1_raw_angle;
        //         *target_theta[0]   = 0.0;
        //     }
        // }
        // using namespace rmcs_msgs;
        // if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
        //     || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {

        //     reset_motors();
        // } else {
        //     *is_arm_enable = true;

        //     if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP) {
        //         *arm_mode = rmcs_msgs::ArmMode::DT7_Control_Position;
        //     } else if (
        //         switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::MIDDLE) {
        //         *arm_mode = rmcs_msgs::ArmMode::DT7_Control_Orientation;
        //     } else if (
        //         switch_left == rmcs_msgs::Switch::DOWN
        //         && switch_right == rmcs_msgs::Switch::MIDDLE) {
        //         is_arm_pump_on  = true;
        //         is_mine_pump_on = true;
        //     } else if (
        //         switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP) {
        //         if (keyboard.a) {
        //             if (!keyboard.ctrl && !keyboard.shift) {
        //                 is_arm_pump_on = true;

        //             } else if (keyboard.shift && !keyboard.ctrl) {
        //                 is_arm_pump_on = false;
        //             }
        //         }
        //         if (keyboard.s) {
        //             if (!keyboard.ctrl && !keyboard.shift) {
        //                 is_mine_pump_on = true;

        //             } else if (keyboard.shift && !keyboard.ctrl) {
        //                 is_mine_pump_on = false;
        //             }
        //         }
        //         if (keyboard.z) {
        //             is_arm_pump_on = true;
        //             if (!keyboard.ctrl && !keyboard.shift) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Auto_Gold_Left;
        //                 fsm_gold_l.reset();
        //             } else if (keyboard.shift && !keyboard.ctrl) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Auto_Gold_Mid;
        //                 fsm_gold_m.reset();
        //             } else if (keyboard.ctrl && !keyboard.shift) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Auto_Gold_Right;
        //                 fsm_gold_r.reset();
        //             }
        //         }

        //         if (keyboard.x) {
        //             is_arm_pump_on = true;
        //             *arm_mode      = rmcs_msgs::ArmMode::Auto_Sliver;
        //             fsm_sliver.reset();
        //         }
        //         if (keyboard.g) {
        //             *arm_mode = rmcs_msgs::ArmMode::Auto_Walk;
        //             fsm_walk.reset();
        //         }
        //         if (keyboard.d) {
        //             *arm_mode = rmcs_msgs::ArmMode::Auto_Spin;
        //             fsm_walk.reset();
        //         }
        //         if (keyboard.b) {
        //             *arm_mode = rmcs_msgs::ArmMode::Auto_Up_Stairs;
        //             fsm_up_stairs.reset();
        //         }
        //         if (keyboard.f) {
        //             is_arm_pump_on  = true;
        //             is_mine_pump_on = true;
        //             if (keyboard.shift && !keyboard.ctrl) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Auto_Storage_LB;
        //                 fsm_storage_lb.reset();
        //             } else if (keyboard.ctrl && !keyboard.shift) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Auto_Storage_RB;
        //                 fsm_storage_rb.reset();
        //             }
        //         }

        //         if (keyboard.r) {
        //             if (!keyboard.ctrl && !keyboard.shift) {
        //                 *arm_mode = rmcs_msgs::ArmMode::Customer;
        //             };
        //         }

        //     } else {
        //         *arm_mode       = rmcs_msgs::ArmMode::None;
        //         is_arm_pump_on  = false;
        //         is_mine_pump_on = false;
        //     }
        //     switch (*arm_mode) {
        //         using namespace rmcs_msgs;
        //     case ArmMode::Auto_Gold_Left: execute_gold(fsm_gold_l); break;
        //     case ArmMode::Auto_Gold_Mid: execute_gold(fsm_gold_m); break;
        //     case ArmMode::Auto_Gold_Right: execute_gold(fsm_gold_r); break;
        //     case ArmMode::Auto_Sliver: execute_sliver(fsm_sliver); break;
        //     case ArmMode::DT7_Control_Position: execute_dt7_position(); break;
        //     case ArmMode::DT7_Control_Orientation: execute_dt7_orientation(); break;
        //     // case ArmMode::Customer: execute_customer(); break;
        //     case ArmMode::Auto_Up_Stairs: execute_up_stairs(); break;
        //     case ArmMode::Auto_Storage_LB: execute_storage(fsm_storage_lb); break;
        //     case ArmMode::Auto_Storage_RB: execute_storage(fsm_storage_rb); break;
        //     case ArmMode::Auto_Spin:
        //     case ArmMode::Auto_Walk: execute_walk(); break;

        //     default: break;
        //     }
        //     pump_control();
        // }
    }

private:
    void execute_dt7_orientation() {
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[5] += 0.003 * joystick_left_->y();
            *target_theta[5] = std::clamp(*target_theta[5], -3.1415926, 3.1415926);
        }
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[4] += 0.003 * joystick_left_->x();
            *target_theta[4] = std::clamp(*target_theta[4], -1.83532, 1.83532);
        }
        if (fabs(joystick_right_->y()) > 0.01) {
            *target_theta[3] += 0.003 * joystick_right_->y();
            *target_theta[3] = std::clamp(*target_theta[3], -3.1415926, 3.1415926);
        }
    }
    void execute_dt7_position() {
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[2] += 0.001 * joystick_left_->x();
            *target_theta[2] = std::clamp(*target_theta[2], -1.0472, 0.8727);
        }
        if (fabs(joystick_right_->x()) > 0.01) {
            *target_theta[1] += 0.001 * joystick_right_->x();
            *target_theta[1] = std::clamp(*target_theta[1], -1.308, 1.16719);
        }
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[0] += 0.001 * joystick_left_->y();
            *target_theta[0] = std::clamp(*target_theta[0], -3.1415926, 3.1415926);
        }
    }
    template <typename T>
    void execute_gold(T& fsm_gold) {
        auto keyboard = *keyboard_;
        auto mouse    = *mouse_;

        static auto last_state = Auto_Gold_State::Set_initial;

        if (fsm_gold.fsm_direction == initial_enter || mouse.right) {
            fsm_gold.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }
        if (mouse.left || fsm_gold.fsm_direction == initial_enter) {
            fsm_gold.fsm_direction = up;
        } else if (mouse.right) {
            fsm_gold.fsm_direction = down;
        }
        if (fsm_gold.fsm_direction == up) {
            fsm_gold.processEvent(Auto_Gold_Event::Up);
        } else if (fsm_gold.fsm_direction == down) {
            fsm_gold.processEvent(Auto_Gold_Event::Down);
        }
        if (fsm_gold.getState() != last_state) {
            fsm_gold.fsm_direction = 0;
        }
        last_state       = fsm_gold.getState();
        *target_theta[5] = fsm_gold.get_result()[5];
        *target_theta[4] = fsm_gold.get_result()[4];
        *target_theta[3] = fsm_gold.get_result()[3];
        *target_theta[2] = fsm_gold.get_result()[2];
        *target_theta[1] = fsm_gold.get_result()[1];
        *target_theta[0] = fsm_gold.get_result()[0];
    }
    template <typename T>
    void execute_sliver(T& fsm_sliver) {
        auto keyboard          = *keyboard_;
        auto mouse             = *mouse_;
        static auto last_state = Auto_Sliver_State::Set_initial;

        if (fsm_sliver.fsm_direction == initial_enter || mouse.right) {
            fsm_sliver.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }
        if (mouse.left || fsm_sliver.fsm_direction == initial_enter)
            fsm_sliver.fsm_direction = up;
        else if (mouse.right)
            fsm_sliver.fsm_direction = down;

        if (fsm_sliver.fsm_direction == up) {
            fsm_sliver.processEvent(Auto_Sliver_Event::Up);
        } else if (fsm_sliver.fsm_direction == down) {
            fsm_sliver.processEvent(Auto_Sliver_Event::Down);
        }

        if (fsm_sliver.getState() != last_state) {
            fsm_sliver.fsm_direction = 0;
        }
        last_state       = fsm_sliver.getState();
        *target_theta[5] = fsm_sliver.get_result()[5];
        *target_theta[4] = fsm_sliver.get_result()[4];
        *target_theta[3] = fsm_sliver.get_result()[3];
        *target_theta[2] = fsm_sliver.get_result()[2];
        *target_theta[1] = fsm_sliver.get_result()[1];
        *target_theta[0] = fsm_sliver.get_result()[0];
    }
    void execute_walk() {
        auto keyboard = *keyboard_;
        if (fsm_walk.fsm_direction == initial_enter) {
            fsm_walk.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
            fsm_walk.fsm_direction = up;
        }
        if (fsm_walk.fsm_direction == up) {
            fsm_walk.processEvent(Auto_Walk_Event::Up);
        }
        *target_theta[5] = fsm_walk.get_result()[5];
        *target_theta[4] = fsm_walk.get_result()[4];
        *target_theta[3] = fsm_walk.get_result()[3];
        *target_theta[2] = fsm_walk.get_result()[2];
        *target_theta[1] = fsm_walk.get_result()[1];
        *target_theta[0] = fsm_walk.get_result()[0];
    }

    void execute_customer() {

        // std::memcpy(customer_theta, custom_controller->begin() + 1, sizeof(customer_theta));
        double customer_[6];
        for (int i = 0; i < 6; ++i) {
            customer_[i] = customer_theta[i];
        }
        *target_theta[5] = customer_[5];
        *target_theta[4] = customer_[4];
        *target_theta[3] = customer_[3];
        *target_theta[2] = customer_[2];
        *target_theta[1] = customer_[1];
        *target_theta[0] = customer_[0];
    }
    void execute_up_stairs() {
        auto keyboard = *keyboard_;
        auto mouse    = *mouse_;

        static auto last_state = Auto_Up_Stairs_State::Leg_Initial;

        if (fsm_up_stairs.fsm_direction == initial_enter
            || (keyboard.ctrl && fsm_up_stairs.getState() == Auto_Up_Stairs_State::Leg_Lift)) {
            fsm_up_stairs.fsm_direction = up;
            fsm_up_stairs.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }

        if (fsm_up_stairs.fsm_direction == up) {
            fsm_up_stairs.processEvent(Auto_Up_Stairs_Event::Up);
        } else if (fsm_up_stairs.fsm_direction == down) {
            fsm_up_stairs.processEvent(Auto_Up_Stairs_Event::Down);
        }
        if (fsm_up_stairs.getState() != last_state) {
            fsm_up_stairs.fsm_direction = 0;
        }
        last_state       = fsm_up_stairs.getState();
        *target_theta[5] = fsm_up_stairs.get_result()[5];
        *target_theta[4] = fsm_up_stairs.get_result()[4];
        *target_theta[3] = fsm_up_stairs.get_result()[3];
        *target_theta[2] = fsm_up_stairs.get_result()[2];
        *target_theta[1] = fsm_up_stairs.get_result()[1];
        *target_theta[0] = fsm_up_stairs.get_result()[0];
    }
    template <typename T>
    void execute_storage(T& fsm) {
        auto keyboard = *keyboard_;
        if (fsm.fsm_direction == initial_enter) {
            fsm.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
            fsm.fsm_direction = up;
        }
        if (fsm.fsm_direction == up) {
            fsm.processEvent(Auto_Storage_Event::Up);
        }
        if (fsm.get_first_trajectory_result()) {
            is_arm_pump_on  = false;
            is_mine_pump_on = true;
        }
        *target_theta[5] = fsm.get_result()[5];
        *target_theta[4] = fsm.get_result()[4];
        *target_theta[3] = fsm.get_result()[3];
        *target_theta[2] = fsm.get_result()[2];
        *target_theta[1] = fsm.get_result()[1];
        *target_theta[0] = fsm.get_result()[0];
    }

    void pump_control() {
        if (is_arm_pump_on) {
            *arm_pump_target_vel = 5300 * std::numbers::pi / 30.0;
            *arm_pump_relay      = 0b00000000;
        } else {
            *arm_pump_target_vel = 0.0;
            *arm_pump_relay      = 0b11111111;
        }
        if (is_mine_pump_on) {
            *mine_pump_target_vel = 5300 * std::numbers::pi / 30.0;

        } else {
            *mine_pump_target_vel = 0.0;
        }
    }
    void reset_motors() {
        *is_arm_enable        = false;
        *target_theta[5]      = *theta[5];
        *target_theta[4]      = *theta[4];
        *target_theta[3]      = *theta[3];
        *target_theta[2]      = *theta[2];
        *target_theta[1]      = *theta[1];
        *target_theta[0]      = *theta[0];
        *arm_pump_target_vel  = NAN;
        *mine_pump_target_vel = NAN;
    }
    bool is_auto_exchange = false;
    // auto_mode_Fsm
    Auto_Gold_Left fsm_gold_l;
    Auto_Gold_Mid fsm_gold_m;
    Auto_Gold_Right fsm_gold_r;
    Auto_Sliver fsm_sliver;
    Auto_Set_Walk_Arm fsm_walk;
    Auto_Up_Stairs fsm_up_stairs;
    Auto_Storage_LB fsm_storage_lb;
    Auto_Storage_RB fsm_storage_rb;

    OutputInterface<rmcs_msgs::ArmMode> arm_mode;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    float customer_theta[6];
    // InputInterface<std::array<uint8_t, 30>> custom_controller;

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    OutputInterface<double> control_torque[6];

    InputInterface<int> joint1_raw_angle;
    OutputInterface<double> joint1_zero_point;

    OutputInterface<double> arm_pump_target_vel;
    OutputInterface<double> mine_pump_target_vel;
    OutputInterface<uint8_t> arm_pump_relay;

    bool is_arm_pump_on  = false;
    bool is_mine_pump_on = false;
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::arm::ArmController, rmcs_executor::Component)