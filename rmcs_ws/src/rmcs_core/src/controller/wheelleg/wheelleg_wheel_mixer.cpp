#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

namespace rmcs_core::controller::wheelleg {

// Combines the common-mode wheel torque (balance / velocity) with the
// differential torque produced by the yaw-rate loop into the per-wheel commands
// consumed by the hardware bridge:
//
//   tau_L = common + yaw_sign * diff
//   tau_R = common - yaw_sign * diff
//
// Keeping the mix separate from the simulation bridge makes it reusable on real
// hardware, where the same common+differential split drives the two wheel
// motors. When the yaw loop is inactive (or disabled) diff is zero and both
// wheels receive the common torque, so turning is purely additive.
class WheelLegWheelMixer
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegWheelMixer()
        : Node{get_component_name(), node::options()} {
        register_input("/wheelleg/control/wheel_torque", common_, false);
        register_input("/wheelleg/control/wheel_torque_diff", diff_, false);

        register_output("/wheelleg/control/wheel_torque_l", wheel_torque_l_, kNaN);
        register_output("/wheelleg/control/wheel_torque_r", wheel_torque_r_, kNaN);

        yaw_sign_ = node::param_or("yaw_sign", 1.0);
        torque_limit_ = node::param_or("wheel_torque_limit", 16.13);
    }

    void update() override {
        // Without a finite common torque the balance loop is inactive: forward
        // NaN so the bridge falls back to its (zero) torque path, exactly as it
        // did before the mixer existed.
        if (!common_.ready() || !std::isfinite(*common_)) {
            *wheel_torque_l_ = kNaN;
            *wheel_torque_r_ = kNaN;
            return;
        }

        // Priority allocation. The common-mode torque (balance, plus any
        // forward feedforward it carries) is safety-critical, so it is
        // allocated first and preserved up to the wheel limit. The differential
        // (yaw) torque is the lowest priority and only receives the remaining
        // per-wheel torque headroom -- if a turn would otherwise saturate one
        // wheel, the yaw demand is trimmed instead of the balance authority.
        const double common = std::clamp(*common_, -torque_limit_, torque_limit_);
        const double headroom = torque_limit_ - std::abs(common);

        double diff = (diff_.ready() && std::isfinite(*diff_)) ? yaw_sign_ * *diff_ : 0.0;
        diff = std::clamp(diff, -headroom, headroom);

        *wheel_torque_l_ = common + diff;
        *wheel_torque_r_ = common - diff;
    }

private:
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> common_;
    InputInterface<double> diff_;

    OutputInterface<double> wheel_torque_l_;
    OutputInterface<double> wheel_torque_r_;

    double yaw_sign_ = 1.0;
    double torque_limit_ = 16.13;
};

} // namespace rmcs_core::controller::wheelleg

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::wheelleg::WheelLegWheelMixer, rmcs_executor::Component)
