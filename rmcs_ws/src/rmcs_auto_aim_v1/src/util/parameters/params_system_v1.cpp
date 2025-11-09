
#include "parameters/params_system_v1.hpp"
#include <memory>
#include <string>

namespace world_exe::parameters {
struct ParamsForSystemV1::Impl {
public:
    std::string model_path = "/workspaces/src/alliance_ros_auto_aim/alliance_auto_aim/models/"
                             "szu_identify_model.onnx";
    std::string device     = "AUTO";
    double control_delay_in_second = 0.05;
    double velocity_begin          = 26;
    double gravity                 = 9.81;
};
void ParamsForSystemV1::set_device(std::string device) {
    if (impl_ == nullptr) impl_ = std::make_unique<ParamsForSystemV1::Impl>();
    impl_->device = std::move(device);
}
void ParamsForSystemV1::set_gravity(double gravity) {
    if (impl_ == nullptr) impl_ = std::make_unique<ParamsForSystemV1::Impl>();
    impl_->gravity = std::move(gravity);
}
void ParamsForSystemV1::set_control_delay_in_second(double control_delay_in_second) {
    if (impl_ == nullptr) impl_ = std::make_unique<ParamsForSystemV1::Impl>();
    impl_->control_delay_in_second = std::move(control_delay_in_second);
}
void ParamsForSystemV1::set_szu_model_path(std::string model_path) {
    if (impl_ == nullptr) impl_ = std::make_unique<ParamsForSystemV1::Impl>();
    impl_->model_path = std::move(model_path);
}
void ParamsForSystemV1::set_velocity_begin(double velocity_begin) {
    if (impl_ == nullptr) impl_ = std::make_unique<ParamsForSystemV1::Impl>();
    impl_->velocity_begin = std::move(velocity_begin);
}
std::string ParamsForSystemV1::szu_model_path() {
    if (impl_ == nullptr) impl_ = std::make_unique<Impl>();
    return impl_->model_path;
}
std::string ParamsForSystemV1::device() {
    if (impl_ == nullptr) impl_ = std::make_unique<Impl>();
    return impl_->device;
}
double ParamsForSystemV1::control_delay_in_second() {
    if (impl_ == nullptr) impl_ = std::make_unique<Impl>();
    return impl_->control_delay_in_second;
}
double ParamsForSystemV1::velocity_begin() {
    if (impl_ == nullptr) impl_ = std::make_unique<Impl>();
    return impl_->velocity_begin;
}
double ParamsForSystemV1::gravity() {
    if (impl_ == nullptr) impl_ = std::make_unique<Impl>();
    return impl_->gravity;
}

std::unique_ptr<ParamsForSystemV1::Impl> ParamsForSystemV1::impl_ = nullptr;
}