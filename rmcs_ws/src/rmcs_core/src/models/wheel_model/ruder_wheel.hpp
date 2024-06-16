#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rmcs_executor/component.hpp>

#include "wheel_model_base.hpp"

namespace rmcs_core::controller::model::wheel {
class RuderWheelModel : public WheelBase {
public:
  RuderWheelModel() : WheelBase("ruder_wheel") {
    fx_c_ = get_parameter("fx_c").as_double();
    fx_b_ = get_parameter("fx_b").as_double();
    fx_d_ = get_parameter("fx_d").as_double();
    fx_e_ = get_parameter("fx_e").as_double();
    fx_s_v_ = get_parameter("fx_s_v").as_double();

    fz_c_ = get_parameter("fz_c").as_double();
    fz_b_ = get_parameter("fz_b").as_double();
    fz_d_ = get_parameter("fz_d").as_double();
    fz_e_ = get_parameter("fz_e").as_double();
    fz_s_v_ = get_parameter("fz_s_v").as_double();

    my_c_ = get_parameter("my_c").as_double();
    my_b_ = get_parameter("my_b").as_double();
    my_d_ = get_parameter("my_d").as_double();
    my_e_ = get_parameter("my_e").as_double();
    my_s_v_ = get_parameter("my_s_v").as_double();
  }

  void claculate(double alpha, double slip_rate) override {

    longitudinal_force_ =
        fx_d_ * sin(fx_c_ * atan(fx_b_ * slip_rate -
                                 fx_e_ * (fx_b_ * slip_rate -
                                          atan(fx_b_ * slip_rate)))) +
        fx_s_v_;
    aligning_torque_ =
        my_d_ *
            sin(my_c_ * atan(my_b_ * alpha -
                             my_e_ * (my_b_ * alpha - atan(my_b_ * alpha)))) +
        my_s_v_;
    lateral_force_ =
        fz_d_ *
            sin(fz_c_ * atan(fz_b_ * alpha -
                             fz_e_ * (fz_b_ * alpha - atan(fz_b_ * alpha)))) +
        fz_s_v_;
  }

  // properties
  double longitudinal_force() const override { return longitudinal_force_; };
  double aligning_torque() const override { return aligning_torque_; };
  double lateral_force() const override { return lateral_force_; };
  double mixed_force() const override {
    return sqrt(lateral_force_ * lateral_force_ +
                longitudinal_force_ * longitudinal_force_);
  };

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

  double longitudinal_force_;
  double aligning_torque_;
  double lateral_force_;
};
} // namespace rmcs_core::controller::model::wheel

#include <pluginlib/class_list_macros.hpp>
