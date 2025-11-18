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

namespace rmcs_core::controller::arm {
class Arm
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Arm()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
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
        register_output("/arm/enable_flag", is_arm_enable, false);
    };
    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_motors();
        } 
        else {
        *is_arm_enable = true;
        }
        if (switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
            execute_dt7_orientation();
            
        } else if (switch_left == Switch::UP && switch_right == Switch::UP) {
            execute_dt7_position();
        }
        RCLCPP_INFO(get_logger(),"%f",*theta[1]);
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
            *target_theta[2] = std::clamp(*target_theta[2], -0.80, 0.9227);
        }
        if (fabs(joystick_right_->x()) > 0.01) {
            *target_theta[1] += 0.001 * joystick_right_->x();
            *target_theta[1] = std::clamp(*target_theta[1], -1.0108, 1.09719);
        }
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[0] += 0.001 * joystick_left_->y();
            *target_theta[0] = std::clamp(*target_theta[0], -2.841592, 2.841592);
        }
    }
    void reset_motors() {
        *is_arm_enable   = false;
        *target_theta[5] = *theta[5];
        *target_theta[4] = *theta[4];
        *target_theta[3] = *theta[3];
        *target_theta[2] = *theta[2];
        *target_theta[1] = *theta[1];
        *target_theta[0] = *theta[0];
    }
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
};
} // namespace rmcs_core::controller::arm
  // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::Arm, rmcs_executor::Component)