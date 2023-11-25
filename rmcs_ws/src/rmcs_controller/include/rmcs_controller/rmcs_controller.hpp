// Copyright (c) 2023, Alliance
// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMCS_CONTROLLER__RMCS_CONTROLLER_HPP_
#define RMCS_CONTROLLER__RMCS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "rmcs_controller/visibility_control.h"
#include "rmcs_controller_parameters.hpp"
#include "std_srvs/srv/set_bool.hpp"

///// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

#include "pid.hpp"

namespace rmcs_controller {
// name constants for state interfaces
// static constexpr size_t STATE_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_ITFS = 0;

///// TODO(anyone: setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t {
    VELOCITY_CTRL = 0,
    POSITION_CTRL = 1,
};

class RMCS_Controller : public controller_interface::ControllerInterface {
public:
    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    RMCS_Controller();

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& previous_state) override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state) override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    RMCS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::return_type
        update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    ///// TODO(anyone): replace the state and command message types
    using ControllerReferenceMsg = control_msgs::msg::JointJog;
    using ControllerModeSrvType  = std_srvs::srv::SetBool;
    using ControllerStateMsg     = control_msgs::msg::JointJog;

protected:
    std::shared_ptr<rmcs_controller::ParamListener> param_listener_;
    rmcs_controller::Params params_;

    std::vector<std::string> state_joints_;

    // Command subscribers and Controller State publisher
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

    rclcpp::Service<ControllerModeSrvType>::SharedPtr set_control_mode_service_;
    realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    PID pid_controller;

private:
    // callback for topic interface
    RMCS_CONTROLLER__VISIBILITY_LOCAL
    void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
};

} // namespace rmcs_controller

#endif // RMCS_CONTROLLER__RMCS_CONTROLLER_HPP_
