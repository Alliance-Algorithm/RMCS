#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::model::wheel {
class ruder_wheel_model : public rmcs_executor::Component, public rclcpp::Node {
public:
  ruder_wheel_model()
      : Node(get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)) {
    fx_c_ = get_parameter("c").as_double();
    fx_b_ = get_parameter("b").as_double();
    fx_d_ = get_parameter("d").as_double();
    fx_e_ = get_parameter("e").as_double();
    fx_s_v_ = get_parameter("s_v").as_double();

    fz_c_ = get_parameter("c").as_double();
    fz_b_ = get_parameter("b").as_double();
    fz_d_ = get_parameter("d").as_double();
    fz_e_ = get_parameter("e").as_double();
    fz_s_v_ = get_parameter("s_v").as_double();

    my_c_ = get_parameter("c").as_double();
    my_b_ = get_parameter("b").as_double();
    my_d_ = get_parameter("d").as_double();
    my_e_ = get_parameter("e").as_double();
    my_s_v_ = get_parameter("s_v").as_double();
    identify_ = get_parameter("wheel_id").as_string();

    register_input("/chassis/" + identify_ + "_wheel/alpha", alpha_);
    register_input("/chassis/" + identify_ + "_wheel/slip_rate", slip_rate_);
  }

  void update() override {
    auto alpha = *alpha_;
    auto slip_rate = *slip_rate_;

    auto longitudinal_force =
        fx_d_ * sin(fx_c_ * atan(fx_b_ * slip_rate -
                                 fx_e_ * (fx_b_ * slip_rate -
                                          atan(fx_b_ * slip_rate)))) +
        fx_s_v_;
    auto aligning_torque =
        my_d_ *
            sin(my_c_ * atan(my_b_ * alpha -
                             my_e_ * (my_b_ * alpha - atan(my_b_ * alpha)))) +
        my_s_v_;
    auto lateral_force =
        fz_d_ *
            sin(fz_c_ * atan(fz_b_ * alpha -
                             fz_e_ * (fz_b_ * alpha - atan(fz_b_ * alpha)))) +
        fz_s_v_;

    *longitudinal_force_ = longitudinal_force;
    *aligning_torque_ = aligning_torque;
    *lateral_force_ = lateral_force;
  }

private:
  double fx_c_;
  double fx_b_;
  double fx_d_;
  double fx_e_;
  double fx_s_v_;

  double fz_c_;
  double fz_b_;
  double fz_d_;
  double fz_e_;
  double fz_s_v_;

  double my_c_;
  double my_b_;
  double my_d_;
  double my_e_;
  double my_s_v_;

  // double fx_s_h_;
  // double fz_s_h_;
  // double my_s_h_;
  // because we assume that F_z is a constant,so delta s_h is zero,
  // param s_h is useless

  std::string identify_;

  InputInterface<double> alpha_;     // in rad
  InputInterface<double> slip_rate_; // in rad

  OutputInterface<double> longitudinal_force_;
  OutputInterface<double> aligning_torque_;
  OutputInterface<double> lateral_force_;
};
} // namespace rmcs_core::controller::model::wheel