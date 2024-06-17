
#include <algorithm>
#include <ceres/problem.h>
#include <memory>
#include <type_traits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include <Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <ceres/ceres.h>

#include "../../models/vehicle_dynamics_model/four_wheel_steering_model.hpp"
#include "./cost_function/four_wheel_cost.hpp"
#include "./objective_function/b_spline_calculator.hpp"
#include "models/vehicle_dynamics_model/car_model_base.hpp"

namespace rmcs_core::controller::mpc {

class MpcController : public rmcs_executor::Component, public rclcpp::Node {
public:
  MpcController()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)),
        car_model_(), bspline_(),
        cost_func_(
            get_parameter("cost_lambdas").as_double_array(),
            get_parameter("time_interval").as_double(),
            std::make_shared<model::car::CarDynamicModelBase>(car_model_)),
        options_() {
    options_.linear_solver_type = ceres::DENSE_QR;
    options_.minimizer_progress_to_stdout = true;
    cost_function = std::make_shared<
        ceres::AutoDiffCostFunction<decltype(cost_func_), 1, 1>>((cost_func_));

    register_input("measured_steering_angles", current_steering_angles_);
    register_input("measured_slippage_rates", current_slippage_rates_);
    register_output("expected_steering_angles", target_steering_angles_);
    register_output("expected_slippage_rates", target_slippage_rates_);
  }

  void update() override {
    auto problem_ = ceres::Problem();

    std::copy_n(*current_steering_angles_, 4, inputs_);
    std::copy_n(*current_slippage_rates_, 4, inputs_ + 4);

    problem_.AddResidualBlock(cost_function.get(), nullptr, inputs_);

    std::copy_n(inputs_, 4, *target_steering_angles_);
    std::copy_n(inputs_ + 4, 4, *target_slippage_rates_);
  }

private:
  model::car::four_wheel_steering_dynamics_model car_model_;
  utils::BSplineCalculator<3, 4, 3> bspline_;
  utils::FourWheelCost cost_func_;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::CostFunction> cost_function;
  double inputs_[8];

  InputInterface<double[4]> current_steering_angles_;
  InputInterface<double[4]> current_slippage_rates_;

  OutputInterface<double[4]> target_steering_angles_;
  OutputInterface<double[4]> target_slippage_rates_;
};
} // namespace rmcs_core::controller::mpc