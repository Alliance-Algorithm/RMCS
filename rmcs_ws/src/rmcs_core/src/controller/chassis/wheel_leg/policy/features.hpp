#pragma once

#include <array>

namespace rmcs_core::controller::chassis {

template <typename Scalar>
struct BasicWheelLegFeatures {
    std::array<Scalar, 3> command{}, gyro{}, gravity{};
    Scalar height = 0;
    std::array<Scalar, 6> position{}, velocity{}, previous_action{};
    std::array<Scalar, 7> auxiliary{1, 0, 0, 0, 0, 0, 0};
};

using WheelLegFeatures = BasicWheelLegFeatures<double>;
using StandingReferenceFeatures = BasicWheelLegFeatures<float>;

} // namespace rmcs_core::controller::chassis
