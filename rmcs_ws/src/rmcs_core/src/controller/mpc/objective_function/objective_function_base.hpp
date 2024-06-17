#pragma once

#include <cmath>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>

namespace rmcs_core::controller::mpc::utils {

template <int dimension, int ret_dimension> class ObjectiveFunctionBase {
public:
  ObjectiveFunctionBase<dimension, ret_dimension>() = default;

  virtual Eigen::Matrix<double, ret_dimension, dimension>
  calculate(double t) = 0;

  void set_control_points(
      Eigen::Matrix<double, Eigen::Dynamic, dimension> &&control_points) {
    control_points_ = std::make_shared(control_points);
  }
  void set_knot_timeline(std::vector<double> &&knot_timeline) {
    knot_timeline_ = std::make_shared<std::vector<double>>(knot_timeline);
  }
  void set_timeline_control_points(
      Eigen::Matrix<double, Eigen::Dynamic, 1> &&timeline_control_points) {
    timeline_control_points_ =
        std::make_shared<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
            timeline_control_points);
  }

protected:
  std::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, dimension>>
      control_points_;
  std::shared_ptr<std::vector<double>> knot_timeline_;
  std::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, 1>>
      timeline_control_points_;
};

template <int dimension, int k, int ret_dimension>
concept BSplineConcept = requires() {
  ret_dimension < k &&k < 5;
};

} // namespace rmcs_core::controller::mpc::utils