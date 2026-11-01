#include <array>
#include <cassert>
#include <cmath>
#include <limits>

#include "controller/wheelleg/wheelleg_gas_spring_v202.hpp"

int main() {
    namespace gas = rmcs_core::wheelleg::gas_spring_v202;
    const std::array sign{-1., -1., -1., -1.};
    const std::array offset{1.6, 1.6+gas::left.back().delta,
                          -1.6, -1.6-gas::right.back().delta};
    auto q = gas::from_encoders({}, sign, offset);
    auto output = gas::evaluate(q);
    assert(output.finite && output.geometry_valid[0] && output.geometry_valid[1]);
    assert(std::abs(output.beta_deg[0]-110.)<1e-10);
    assert(std::abs(output.beta_deg[1]-110.)<1e-10);
    assert(std::abs(output.torque[0])>0.01 && std::abs(output.torque[3])>0.01);
    const auto zero = gas::evaluate(q,0.);
    for (double t : zero.torque) assert(t==0.);
    const auto turns = gas::from_encoders({2.*std::numbers::pi,0.,0.,-2.*std::numbers::pi},sign,offset);
    const auto wrapped = gas::evaluate(turns);
    for (std::size_t i=0;i<4;++i) assert(std::abs(wrapped.torque[i]-output.torque[i])<1e-10);
    q[0] = std::numeric_limits<double>::quiet_NaN();
    output = gas::evaluate(q);
    assert(!output.finite);
    for (double t : output.torque) assert(t==0.);
    q = {0.,1.428,0.,-1.428};
    const auto protected_effort = gas::protect({-100.,100.,100.,-100.},q);
    for (double t : protected_effort) assert(std::abs(t)<=40.);
    assert(protected_effort[1]-protected_effort[0]<=1e-10);
    assert(protected_effort[2]-protected_effort[3]<=1e-10);
}
