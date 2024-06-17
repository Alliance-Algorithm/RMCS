#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/MatrixBase.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>

namespace rmcs_core::controller::mpc::utils {
template <int dimension> class ObjectiveFunctionBase {
public:
  ObjectiveFunctionBase() = default;

private:
  Eigen::Matrix<double, Eigen::Dynamic, dimension> parameters_;
  Eigen::Matrix<double, Eigen::Dynamic, dimension> control_points_;
  std::vector<double> knot_timeline_;
  Eigen::Matrix<double, Eigen::Dynamic, dimension> timeline_parameters_;
  Eigen::Matrix<double, Eigen::Dynamic, dimension> timeline_control_points_;
};
} // namespace rmcs_core::controller::mpc::utils