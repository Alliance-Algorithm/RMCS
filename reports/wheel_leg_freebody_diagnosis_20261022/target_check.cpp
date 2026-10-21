#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"
#include <algorithm>
using namespace rmcs_core::controller::chassis;
extern "C" int target_check(const char* geometry, const double* q, const double* g, double* output) {
    RecoveryTuckController tuck{geometry, "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997", {}};
    const Eigen::Vector3d gravity=Eigen::Map<const Eigen::Vector3d>{g};
    const auto current=tuck.measure(Eigen::Map<const Eigen::Vector4d>{q},gravity);
    const auto state=tuck.update(Eigen::Map<const Eigen::Vector4d>{q},gravity,.001,false);
    const auto target=tuck.measure(state.reference,gravity);
    std::copy_n(state.reference.data(),4,output);
    std::copy_n(target.knee_deg.data(),2,output+4);
    std::copy_n(target.sagittal_error_m.data(),2,output+6);
    for (int side=0;side<2;++side)
        output[8+side]=gravity.dot(target.wheels_b_m[side]-target.com_b_m);
    output[10]=target.valid;output[11]=target.ready;output[12]=current.valid;output[13]=state.valid;
    return state.valid;
}
