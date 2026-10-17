#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::filter {

// Generic first-order low-pass filter component. Insert it between any two
// scalar topics in the YAML graph:
//
//   estimator pitch -> LowPassFilterController -> balance LQR pitch input
//
// The coefficient is either given directly (`alpha`) or derived from the
// cutoff: alpha = 1 - exp(-2*pi*cutoff_hz*dt). `cutoff_hz <= 0` (and no alpha)
// is a pass-through. On the first valid input the filter is seeded with the
// input (no startup transient); invalid input publishes NaN so downstream
// safety logic can react instead of acting on a stale value.
class LowPassFilterController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    using node = rmcs_utility::NodeMixin;

    LowPassFilterController()
        : Node{get_component_name(), node::options()} {
        const std::string input = node::param_or("input", std::string{});
        const std::string output = node::param_or("output", std::string{});
        if (input.empty() || output.empty())
            throw std::runtime_error(
                "LowPassFilterController requires 'input' and 'output'");

        cutoff_hz_ = get_parameter_or("cutoff_hz", 30.0);
        dt_ = std::max(1e-6, get_parameter_or("dt", 0.001));
        const double alpha_param = get_parameter_or("alpha", -1.0);
        if (alpha_param > 0.0) {
            alpha_ = std::clamp(alpha_param, 0.0, 1.0);
        } else if (cutoff_hz_ > 0.0) {
            alpha_ = 1.0 - std::exp(-kTwoPi * cutoff_hz_ * dt_);
        } else {
            alpha_ = 1.0;
        }
        lpf_ = std::make_unique<::rmcs_core::filter::LowPassFilter<1>>(alpha_);

        register_input(input, input_);
        register_output(output, output_, kNaN);

        info(
            "LowPassFilterController {} -> {}: cutoff={:.2f} Hz dt={:.6f} s alpha={:.5f}",
            input, output, cutoff_hz_, dt_, alpha_);
    }

    void update() override {
        if (!input_.ready() || !std::isfinite(*input_)) {
            lpf_->reset();
            *output_ = kNaN;
            return;
        }
        *output_ = lpf_->update(*input_);
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kTwoPi = 6.283185307179586476925286766559;

    double cutoff_hz_ = 30.0;
    double dt_ = 0.001;
    double alpha_ = 1.0;

    InputInterface<double> input_;
    OutputInterface<double> output_;
    std::unique_ptr<::rmcs_core::filter::LowPassFilter<1>> lpf_;
};

} // namespace rmcs_core::controller::filter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::filter::LowPassFilterController, rmcs_executor::Component)
