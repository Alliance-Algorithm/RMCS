#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Commissioning signal source for the wheel-leg leg loops. Emits square waves
// (offset + amplitude * sign(sin)) on
//   /wheelleg/test/leg_length_ref   (equivalent-leg length reference, m)
//   /wheelleg/test/d_ref            (CoM offset reference, m)
// so the task-space loops can be exercised with live steps while watching the
// MuJoCo viewer. Set amplitude = 0 for a constant reference.
class WheelLegTestSignal
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegTestSignal()
        : Node{get_component_name(), node::options()} {
        dt_ = node::param_or("dt", 0.001);

        leg_length_ = load_channel_("leg_length");
        d_ = load_channel_("d");

        register_output("/wheelleg/test/leg_length_ref_l", leg_length_ref_l_, kNaN);
        register_output("/wheelleg/test/leg_length_ref_r", leg_length_ref_r_, kNaN);
        register_output("/wheelleg/test/d_ref", d_ref_, kNaN);
    }

    void update() override {
        elapsed_ += dt_;
        const double leg_length_ref = channel_value_(leg_length_);
        *leg_length_ref_l_ = leg_length_ref;
        *leg_length_ref_r_ = leg_length_ref;
        *d_ref_ = channel_value_(d_);
    }

private:
    struct Channel {
        double offset = 0.0;
        double amplitude = 0.0;
        double period = 0.0;
        double start = 0.0;
    };

    Channel load_channel_(const std::string& prefix) {
        Channel c;
        c.offset = node::param_or(prefix + "_offset", 0.0);
        c.amplitude = node::param_or(prefix + "_amplitude", 0.0);
        c.period = node::param_or(prefix + "_period", 0.0);
        c.start = node::param_or(prefix + "_start", 0.0);
        return c;
    }

    double channel_value_(const Channel& c) const {
        if (c.period <= 0.0 || c.amplitude == 0.0 || elapsed_ < c.start)
            return c.offset;
        const double phase = std::fmod(elapsed_ - c.start, c.period) / c.period;
        const double sign = (phase < 0.5) ? 1.0 : -1.0;
        return c.offset + c.amplitude * sign;
    }

    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    double dt_ = 0.001;
    double elapsed_ = 0.0;
    Channel leg_length_;
    Channel d_;

    OutputInterface<double> leg_length_ref_l_;
    OutputInterface<double> leg_length_ref_r_;
    OutputInterface<double> d_ref_;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegTestSignal, rmcs_executor::Component)
