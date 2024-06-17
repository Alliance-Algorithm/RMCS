#pragma once

#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>

#include "controller/mpc/objective_function/objective_function_base.hpp"
#include "models/vehicle_dynamics_model/four_wheel_steering_model.hpp"

namespace rmcs_core::controller::mpc::utils {
class FourWheelCost {
public:
  explicit FourWheelCost(const std::vector<double> &lambdas,
                         const std::vector<double> &omegas,
                         double time_interval) {
    lambdas_ = {lambdas[0], lambdas[1], lambdas[2]};
    omegas_ = omegas;
    time_interval_ = time_interval;
  };

  [[nodiscard]] double calculate_cost(
      const std::vector<double> &control_vector,
      const Eigen::Matrix3d &target_matrix,
      model::car::four_wheel_steering_dynamics_model &car_model_) const {
    car_model_.update(
        std::vector<double>(control_vector.begin(), control_vector.begin() + 4),
        std::vector<double>(control_vector.begin() + 4,
                            control_vector.begin() + 8),
        yaw_angle_, current_velocity_, angular_velocity_);
    auto angular_acceleration = car_model_.angular_acceleration();
    auto acceleration = car_model_.acceleration();
    auto velocity = current_velocity_ + acceleration * time_interval_;
    auto position =
        position_ + (velocity + current_velocity_) / 2 * time_interval_;
    auto angular_velocity =
        angular_velocity_ + angular_acceleration * time_interval_;

    Eigen::Matrix3d calc_matrix = Eigen::Matrix3d::Zero();
    calc_matrix.block(0, 0, 1, 2) = position;
    calc_matrix.block(1, 0, 1, 2) = velocity;
    calc_matrix(2, 2) = angular_velocity;
    return (lambdas_ *
            (target_matrix - calc_matrix).cwiseAbs().rowwise().sum())(0, 0);
  }

  void set_parameters(double yaw_angle, const Eigen::Vector2d &linear_velocity,
                      const Eigen::Vector2d &position,
                      double angular_velocity) {
    yaw_angle_ = yaw_angle;
    current_velocity_ = linear_velocity;
    angular_velocity_ = angular_velocity;
    position_ = position;
  }

private:
  Eigen::Vector3d lambdas_;
  double time_interval_;
  double yaw_angle_;
  std::vector<double> omegas_;
  Eigen::Vector2d current_velocity_;
  Eigen::Vector2d position_;
  double angular_velocity_;
};
} // namespace rmcs_core::controller::mpc::utils