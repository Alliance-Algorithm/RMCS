#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>

namespace rmcs_core::controller::mpc::utils {
template <int dimension, int k, int ret_dimension>
concept BSplineConcept = std::isless<ret_dimension, k>() && std::isless<k, 5>();

template <int dimension, int ret_dimension> class ObjectiveFunctionBase {
public:
  ObjectiveFunctionBase<dimension, ret_dimension>() = default;

  virtual Eigen::Matrix<double, ret_dimension, dimension>
  calculate(float t) = 0;

protected:
  Eigen::Matrix<double, Eigen::Dynamic, dimension> control_points_;
  std::vector<double> knot_timeline_;
  Eigen::Matrix<double, Eigen::Dynamic, 1> timeline_control_points_;
};
} // namespace rmcs_core::controller::mpc::utils