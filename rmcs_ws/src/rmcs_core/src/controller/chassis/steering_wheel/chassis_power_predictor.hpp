#pragma once

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>
#include <switch.hpp>
#include <vector>

#include "motor.hpp"

namespace rmcs_core::controller::chassis::steering_wheel {
using rmcs_executor::Component;

class ChassisPowerPredictor {
public:
  ChassisPowerPredictor(Component &chassis) {
    chassis.register_input("/referee/chassis_power", chassis_power_referee_);
    chassis.register_input("/referee/buffer_energy",
                           chassis_buffer_energy_referee_);
    chassis.register_input("/referee/chassis_power_limit",
                           chassis_power_limit_referee_);
  };

  void update_power_max() {
    constexpr double buffer_energy_control_line = 50;
    double power_reduction_factor = std::min(
        1.0, *chassis_buffer_energy_referee_ / buffer_energy_control_line);
    power_max_ = *chassis_power_limit_referee_;
    power_max_ *= power_reduction_factor;
  }

  template <typename... MotorVectors>
  double power_ratio_predict(const MotorVectors &...motorVectors) {
    static_assert(
        (std::is_same_v<MotorVectors, std::vector<std::unique_ptr<Motor>>> &&
         ...),
        "All parameters must be std::vector<std::unique_ptr<Motor>>");

    double total_power = 0.0;
    auto accumulate_power =
        [&total_power](const std::vector<std::unique_ptr<Motor>> &motors) {
          for (const auto &motor_ptr : motors) {
            total_power += motor_ptr->predict_power();
          }
        };
    (accumulate_power(motorVectors), ...);

    return total_power / power_max_;
  }

  template <typename... MotorVectors>
  void motor_current_update(const MotorVectors &...motorVectors) {
    static_assert(
        (std::is_same_v<MotorVectors, std::vector<std::unique_ptr<Motor>>> &&
         ...),
        "All parameters must be std::vector<std::unique_ptr<Motor>>");

    double total_power = 0.0;
    auto accumulate_power =
        [&total_power](const std::vector<std::unique_ptr<Motor>> &motors) {
          for (const auto &motor_ptr : motors) {
            total_power += motor_ptr->predict_power();
          }
        };
    double a = 0, b = 0, c = 0;
    auto get_params_sum =
        [&a, &b, &c](const std::vector<std::unique_ptr<Motor>> &motors) {
          for (const auto &motor_ptr : motors) {
            auto &[a_, b_, c_] = motor_ptr->get_param();

            a += a_;
            b += b_;
            c += c_;
          }
        };
    double k;
    auto update_current =
        [&k](const std::vector<std::unique_ptr<Motor>> &motors) {
          for (auto &motor : motors) {
            motor->update_control_current(k);
          }
        };

    (accumulate_power(motorVectors), ...);

    (get_params_sum(motorVectors), ...);

    if (total_power <= power_max_) {
      k = 1;
    } else {
      c -= power_max_;
      k = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
      if (std::isnan(k))
        k = 0;
      else
        k = std::clamp(k, 0.0, 1.0);
    }
    (update_current(motorVectors), ...);
  }

  void shrink_total_power(double ratio) { power_max_ *= ratio; }

  Component::InputInterface<double> chassis_power_referee_;
  Component::InputInterface<double> chassis_buffer_energy_referee_;
  Component::InputInterface<double> chassis_power_limit_referee_;

  double power_max_ = 0;
};

} // namespace rmcs_core::controller::chassis::steering_wheel
