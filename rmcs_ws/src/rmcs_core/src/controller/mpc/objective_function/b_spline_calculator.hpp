#pragma once

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include "./objective_function_base.hpp"

namespace rmcs_core::controller::mpc::utils {

template <int dimension, int k, int ret_dimension>
requires BSplineConcept<dimension, k, ret_dimension>
class BSplineCalculator
    : public ObjectiveFunctionBase<dimension, ret_dimension> {
  BSplineCalculator() : ObjectiveFunctionBase<dimension, ret_dimension>() {
#if k == 3
    m_k_ = {0.5, 0.5, 0, -1, 1, 0, 0.5, -1, 0.5};
#else
    m_k_ = {1 / 6.0,  4 / 6.0, 1 / 6.0,  0,        -3 / 6.0, 0,
            3 / 6.0,  0,       3 / 6.0,  -6 / 6.0, 3 / 6.0,  0,
            -1 / 6.0, 3 / 6.0, -3 / 6.0, 1 / 6.0};
#endif
    m_tk_ = {0.5, 0.5, 0, -1, 1, 0, 0.5, -1, 0.5};
  }

  Eigen::Matrix<double, ret_dimension, dimension> calculate(double t) override {
    t = std::min(std::max(t, 0.0), this->knot_timeline_.back());
    auto i = 0;
    while (this->knot_timeline_[i + 1] < t)
      i++;
    auto u = this->knot_timeline_[i + 1] - this->knot_timeline_[i];
    u_time_density = {0, 1, 2 * u};

    double time_density = u_time_density * m_tk_ *
                          this->timeline_control_points_.block(i, 0, 3, 1);

    Eigen::Matrix<double, 5, 1> time_density_vec = {
        1, time_density, time_density * time_density,
        time_density * time_density * time_density,
        time_density * time_density * time_density * time_density};

    u_traj = {1, u, u * u, u * u * u, u * u * u * u,
              0, 1, 2 * u, 3 * u * u, 4 * u * u * u,
              0, 0, 2 * 1, 2 * 3 * u, 4 * 3 * u * u,
              0, 0, 2 * 0, 2 * 3 * 1, 4 * 3 * 2 * u,
              0, 0, 2 * 0, 2 * 3 * 0, 4 * 3 * 2 * 1};
    u_traj.array().colwise() /= time_density_vec.array().col(0);

    auto size = k - ret_dimension == k ? 0 : 1;
    Eigen::Matrix<double, ret_dimension, dimension> target_matrix =
        u_traj.block(0, 0, ret_dimension, size) * m_k_.block(0, 0, size, size) *
        this->control_points_.block(i, 0, size, dimension);

    return target_matrix;
  }

private:
  Eigen::Matrix<double, k, k> m_k_;
  Eigen::Matrix<double, 5, 5> u_traj;
  Eigen::Matrix<double, 3, 3> m_tk_;
  Eigen::Matrix<double, 1, 3> u_time_density;
};
} // namespace rmcs_core::controller::mpc::utils