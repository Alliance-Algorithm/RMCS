#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/fsm/FSM.hpp"
#include "hardware/fsm/FSM_gold_l.hpp"
#include "hardware/fsm/FSM_gold_m.hpp"
#include "hardware/fsm/FSM_gold_r.hpp"
#include "hardware/fsm/FSM_sliver.hpp"
#include "hardware/fsm/FSM_walk.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::arm {
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

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);

        register_input("/arm/Joint1/vision", vision_theta1);
        register_input("/arm/Joint2/vision", vision_theta2);
        register_input("/arm/Joint3/vision", vision_theta3);
        register_input("/arm/Joint4/vision", vision_theta4);
        register_input("/arm/Joint5/vision", vision_theta5);
        register_input("/arm/Joint6/vision", vision_theta6);

        register_output("/arm/Joint6/target_theta", target_theta[5], nan);
        register_output("/arm/Joint5/target_theta", target_theta[4], nan);
        register_output("/arm/Joint4/target_theta", target_theta[3], nan);
        register_output("/arm/Joint3/target_theta", target_theta[2], nan);
        register_output("/arm/Joint2/target_theta", target_theta[1], nan);
        register_output("/arm/Joint1/target_theta", target_theta[0], nan);

        register_output("/arm/enable_flag", is_arm_enable, true);

        // std::array<double, 6> *target_theta_inital_value = {nan, nan, nan, nan, nan, nan};
        //  register_output("/arm*target_theta",*target_theta, *target_theta_inital_value);
        //  auto exchange
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
    }
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        auto keyboard     = *keyboard_;

        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {
            static_cast<float>(*theta[0]),
            -static_cast<float>(*theta[1]),
            -(static_cast<float>(*theta[2] - std::numbers::pi / 2)),
            static_cast<float>(*theta[3]),
            static_cast<float>(*theta[4]),
            static_cast<float>(*theta[5])};
        publisher_->publish(msg);

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            *is_arm_enable   = false;
            *target_theta[5] = *theta[5];
            *target_theta[4] = *theta[4];
            *target_theta[3] = *theta[3];
            *target_theta[2] = *theta[2];
            *target_theta[1] = *theta[1];
            *target_theta[0] = *theta[0];

        } else {
            *is_arm_enable = true;
            if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP) {
                keyboard_mode_selection();
                switch (mode) {
                case Mode::Auto_Gold_Left: execute_gold(fsm_gold_l); break;
                case Mode::Auto_Gold_Mid: execute_gold(fsm_gold_m); break;
                case Mode::Auto_Gold_right: execute_gold(fsm_gold_r); break;
                case Mode::Auto_Sliver: execute_sliver(fsm_sliver); break;
                case Mode::Auto_Walk: execute_walk();break;
                default: break;
                }
            }
        }
    }

private:
    void update_dr16_control_theta() {

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto mouse        = *mouse_;
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::MIDDLE) {
            if (fabs(joystick_left_->y()) > 0.01)
                *target_theta[5] += 0.003 * joystick_left_->y();
            if (fabs(joystick_left_->x()) > 0.01)
                *target_theta[4] += 0.003 * joystick_left_->x();
            if (fabs(joystick_right_->y()) > 0.01)
                *target_theta[3] += 0.003 * joystick_right_->y();
        }
        if (switch_left == rmcs_msgs::Switch::UP && switch_right == rmcs_msgs::Switch::UP) {
            if (fabs(joystick_left_->x()) > 0.01)
                *target_theta[2] += 0.001 * joystick_left_->x();
            if (fabs(joystick_right_->x()) > 0.01)
                *target_theta[1] += 0.001 * joystick_right_->x();
            if (fabs(joystick_left_->y()) > 0.01)
                *target_theta[0] += 0.001 * joystick_left_->y();
        }
    };
    void keyboard_mode_selection() {
        auto keyboard = *keyboard_;

        if (keyboard.z && !keyboard.ctrl && !keyboard.shift) {
            mode = Mode::Auto_Gold_Left;
            fsm_gold_l.reset();
        }
        if (keyboard.z && !keyboard.ctrl && keyboard.shift) {
            mode = Mode::Auto_Gold_Mid;
            fsm_gold_m.reset();
        }
        if (keyboard.z && keyboard.ctrl && !keyboard.shift) {
            mode = Mode::Auto_Gold_right;
            fsm_gold_r.reset();
        }
        if (keyboard.x) {
            mode = Mode::Auto_Sliver;
            fsm_sliver.reset();
        }
        if (keyboard.g) {
            mode = Mode::Auto_Walk;
            fsm_walk.reset();
        }
    }
    template <typename T>
    void execute_gold(T& fsm_gold) {
        auto keyboard          = *keyboard_;
        static auto last_state = Auto_Gold_State::Set_initial;

        if (fsm_gold.fsm_direction == initial_enter || keyboard.s) {
            fsm_gold.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }
        if (keyboard.w || fsm_gold.fsm_direction == initial_enter)
            fsm_gold.fsm_direction = up;
        else if (keyboard.s)
            fsm_gold.fsm_direction = down;

        if (fsm_gold.fsm_direction == up) {
            fsm_gold.processEvent(Auto_Gold_Event::Up);
        }
        if (fsm_gold.fsm_direction == down) {
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
        // RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f",
        //         fsm_gold.get_result()[0],
        //         fsm_gold.get_result()[1],
        //         fsm_gold.get_result()[2],
        //         fsm_gold.get_result()[3],
        //         fsm_gold.get_result()[4],
        //         fsm_gold.get_result()[5]);
    }
    template <typename T>
    void execute_sliver(T& fsm_sliver) {
        auto keyboard          = *keyboard_;
        static auto last_state = Auto_Sliver_State::Set_initial;

        if (fsm_sliver.fsm_direction == initial_enter || keyboard.s) {
            fsm_sliver.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
        }
        if (keyboard.w || fsm_sliver.fsm_direction == initial_enter)
            fsm_sliver.fsm_direction = up;
        else if (keyboard.s)
            fsm_sliver.fsm_direction = down;

        if (fsm_sliver.fsm_direction == up) {
            fsm_sliver.processEvent(Auto_Sliver_Event::Up);
        }
        if (fsm_sliver.fsm_direction == down) {
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
    void execute_walk(){
        auto keyboard          = *keyboard_;
        if(fsm_walk.fsm_direction == initial_enter){
            fsm_walk.get_current_theta(
                {*theta[0], *theta[1], *theta[2], *theta[3], *theta[4], *theta[5]});
            fsm_walk.fsm_direction = up;
        }
        if(fsm_walk.fsm_direction == up){
            fsm_walk.processEvent(Auto_Walk_Event::Up);
        }
        *target_theta[5] = fsm_walk.get_result()[5];
        *target_theta[4] = fsm_walk.get_result()[4];
        *target_theta[3] = fsm_walk.get_result()[3];
        *target_theta[2] = fsm_walk.get_result()[2];
        *target_theta[1] = fsm_walk.get_result()[1];
        *target_theta[0] = fsm_walk.get_result()[0];

    }
    bool is_auto_exchange = false;
    //auto_mode_Fsm
    Auto_Gold_Left fsm_gold_l;
    Auto_Gold_Mid fsm_gold_m;
    Auto_Gold_Right fsm_gold_r;
    Auto_Sliver fsm_sliver;
    Auto_Set_Walk_Arm fsm_walk;
    //

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

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
    enum class Mode {
        Auto_Gold_Left,
        Auto_Gold_right,
        Auto_Gold_Mid,
        Auto_Sliver,
        Auto_Walk,
        None
    } mode = Mode::None;

    // test
    //  hardware::device::Trajectory<hardware::device::LineTrajectoryType> line;
    rmcs_core::hardware::device::Trajectory<rmcs_core::hardware::device::TrajectoryType::BEZIER>
        bezier;

    InputInterface<double> vision_theta1;
    InputInterface<double> vision_theta2;
    InputInterface<double> vision_theta3;
    InputInterface<double> vision_theta4;
    InputInterface<double> vision_theta5;
    InputInterface<double> vision_theta6;
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)