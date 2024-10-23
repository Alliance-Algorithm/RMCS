#pragma once

#include "controller/chassis/steering_wheel/chassis_status.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <rmcs_executor/component.hpp>
#include <switch.hpp>

namespace rmcs_core::controller::chassis::steering_wheel {
using rmcs_executor::Component;
using rmcs_msgs::Switch;

class JoystickLogic {
public:
  JoystickLogic(Component &chassis) {
    chassis.register_input("/remote/switch/right", switch_right_);
    chassis.register_input("/remote/switch/left", switch_left_);
  }

  ChassisStatus get_status() {
    auto switch_right = *switch_right_;
    auto switch_left = *switch_left_;

    do {
      if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN) ||
          (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {

        spinning_ = false;
        following_ = false;

        break;
      }

      if (switch_left != Switch::DOWN) {
        if ((last_switch_right_ == Switch::MIDDLE &&
             switch_right == Switch::DOWN)) {
          following_ = spinning_;
          spinning_ = !spinning_;
          if (spinning_)
            last_spinning_ = !last_spinning_;
        }
      }

      if (spinning_)
        return ChassisStatus::Spin;
      if (following_)
        return ChassisStatus::Follow;

      return ChassisStatus::Free;

    } while (false);

    last_switch_right_ = switch_right;
    last_switch_left_ = switch_left;

    return ChassisStatus::Safe;
  }

private:
  Component::InputInterface<rmcs_msgs::Switch> switch_right_;
  Component::InputInterface<rmcs_msgs::Switch> switch_left_;

  rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
  rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;

  bool spinning_ = false, following_ = false, last_spinning_ = true;
};
} // namespace rmcs_core::controller::chassis::steering_wheel
