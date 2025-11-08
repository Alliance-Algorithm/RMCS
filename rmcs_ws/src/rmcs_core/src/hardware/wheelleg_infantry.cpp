#include <rclcpp/node.hpp>

#include "librmcs/client/cboard.hpp"
#include "rmcs_executor/component.hpp"

namespace rmcs_core::hardware {
class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    WheelLegInfantry()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())} {}
};
} // namespace rmcs_core::hardware