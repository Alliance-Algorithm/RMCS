#include "concexpt_chassis_controller.hpp"

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis::steering_wheel
{
using rmcs_executor::Component;

// static constexpr double inf = std::numeric_limits<double>::infinity();

class Supercap
{
public:
    Supercap(DerivedFromBoth auto* component) {
        component->register_output("/chassis/supercap/control_enable", supercap_control_enabled_, false);
        component->register_output("/chassis/supercap/control_power_limit",
                                   supercap_control_power_limit_,
                                   0.0);
    }

private:
    Component::OutputInterface<bool>   supercap_control_enabled_;
    Component::OutputInterface<double> supercap_control_power_limit_;
};
} // namespace rmcs_core::controller::chassis::steering_wheel