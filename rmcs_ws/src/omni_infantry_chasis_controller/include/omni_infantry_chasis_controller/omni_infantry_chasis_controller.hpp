// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef OMNI_INFANTRY_CHASIS_CONTROLLER__OMNI_INFANTRY_CHASIS_CONTROLLER_HPP_
#define OMNI_INFANTRY_CHASIS_CONTROLLER__OMNI_INFANTRY_CHASIS_CONTROLLER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "omni_infantry_chasis_controller/visibility_control.h"
#include "omni_infantry_chasis_controller_parameters.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/joint_jog.hpp"

// 自定义函数
#include "chasiscontroll.hpp"

// 自定义消息
#include "rmcs_msgs/msg/dbus.hpp"

namespace omni_infantry_chasis_controller {
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

// TODO(anyone: example setup for control mode (usually you will use some enums defined in messages)
enum class control_mode_type : std::uint8_t {
    FAST = 0,
    SLOW = 1,
};

class OmniInfantryChasisController : public controller_interface::ControllerInterface {
public:
    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    OmniInfantryChasisController();

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& previous_state) override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state) override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    OMNI_INFANTRY_CHASIS_CONTROLLER__VISIBILITY_PUBLIC
    controller_interface::return_type
        update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::CallbackReturn
        configure_chasis_motor(const std::vector<std::string>& chasis_names_vector);

    controller_interface::CallbackReturn configure_chasis_motor_pid();
    // TODO(anyone): replace the state and command message types
    using ControllerReferenceMsg = control_msgs::msg::JointJog;
    using ControllerModeSrvType  = std_srvs::srv::SetBool;
    using ControllerStateMsg     = control_msgs::msg::JointControllerState;
    using Dbus                   = rmcs_msgs::msg::Dbus;

protected:
    struct CmdData {
        CmdData()
            : ChasisVx_(std::numeric_limits<double>::quiet_NaN())
            , ChasisVy_(std::numeric_limits<double>::quiet_NaN())
            , ChasisVw_(std::numeric_limits<double>::quiet_NaN()) {}
        CmdData(double ChasisVx, double ChasisVy, double ChasisVw)
            : ChasisVx_(ChasisVx)
            , ChasisVy_(ChasisVy)
            , ChasisVw_(ChasisVw) {}
        double ChasisVx_;
        double ChasisVy_;
        double ChasisVw_;
        /* data */
    };

    std::shared_ptr<omni_infantry_chasis_controller::ParamListener> param_listener_;
    omni_infantry_chasis_controller::Params params_;

    std::vector<std::string> state_joints_;

    // Command subscribers and Controller State publisher
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdData>> input_ref_;

    rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
    realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

    using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    ChasisController::ChasisControll ChasisData_;

    rclcpp::Subscription<Dbus>::SharedPtr RemoteCmdSubscriber_;

private:
    void RemoteCmdCallback(const Dbus& msg);
    // callback for topic interface
};

} // namespace omni_infantry_chasis_controller

#endif // OMNI_INFANTRY_CHASIS_CONTROLLER__OMNI_INFANTRY_CHASIS_CONTROLLER_HPP_
