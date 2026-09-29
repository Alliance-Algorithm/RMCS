#include <pluginlib/class_list_macros.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::identification {

// Standalone observation graph: keep the hardware's required enable request
// explicitly false, regardless of the current remote-switch positions.
class WheelLegPassiveRecorderGate final : public rmcs_executor::Component {
public:
    WheelLegPassiveRecorderGate() {
        register_output("/wheel_leg/enable_request", enable_request_, false);
    }

    void update() override { *enable_request_ = false; }

private:
    OutputInterface<bool> enable_request_;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegPassiveRecorderGate, rmcs_executor::Component)
