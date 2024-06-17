#pragma once

#include "./objective_function_base.hpp"

namespace rmcs_core::controller::mpc::utils {
template <int dimension>
class BSplineCalculator : public ObjectiveFunctionBase<dimension> {
  BSplineCalculator() : ObjectiveFunctionBase<dimension>() {}
};
} // namespace rmcs_core::controller::mpc::utils