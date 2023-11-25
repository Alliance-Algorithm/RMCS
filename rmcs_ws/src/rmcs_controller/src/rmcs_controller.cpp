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

#include "rmcs_controller/rmcs_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace { // utility

///// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    1, // message queue depth
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

using ControllerReferenceMsg = rmcs_controller::RMCS_Controller::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
    std::shared_ptr<ControllerReferenceMsg>& msg, const std::vector<std::string>& joint_names) {
    msg->joint_names = joint_names;
    msg->displacements.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    msg->velocities.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
    msg->duration = std::numeric_limits<double>::quiet_NaN();
}

} // namespace

namespace rmcs_controller {
RMCS_Controller::RMCS_Controller()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn RMCS_Controller::on_init() {
    control_mode_.initRT(control_mode_type::VELOCITY_CTRL);

    try {
        param_listener_ = std::make_shared<rmcs_controller::ParamListener>(get_node());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
    RMCS_Controller::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    params_ = param_listener_->get_params();

    if (params_.command_interfaces.empty() || params_.state_interfaces.empty()) {
        RCLCPP_FATAL(
            get_node()->get_logger(),
            "Param \"command_interfaces\" and \"state_interfaces\" can not be empty!");
        return CallbackReturn::FAILURE;
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&RMCS_Controller::reference_callback, this, std::placeholders::_1));

    std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, params_.joints);
    input_ref_.writeFromNonRT(msg);

    auto set_mode_service_callback =
        [&](const std::shared_ptr<ControllerModeSrvType::Request> request,
            std::shared_ptr<ControllerModeSrvType::Response> response) {
            if (request->data) {
                control_mode_.writeFromNonRT(control_mode_type::POSITION_CTRL);
            } else {
                control_mode_.writeFromNonRT(control_mode_type::VELOCITY_CTRL);
            }
            response->success = true;
        };

    set_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
        "~/set_control_mode", set_mode_service_callback, rmw_qos_profile_services_hist_keep_all);

    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/state", rclcpp::SystemDefaultsQoS());
        state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
    } catch (const std::exception& e) {
        fprintf(
            stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // TODO(anyone): Reserve memory in state publisher depending on the message type
    // ?(KalecKKK) WTF does it mean?
    state_publisher_->lock();
    state_publisher_->msg_.header.frame_id = params_.joints[0];
    state_publisher_->unlock();

    pid_controller.setKp(params_.pid.kp, params_.pid.kp_limit);
    pid_controller.setKi(params_.pid.ki, params_.pid.ki_limit);
    pid_controller.setKd(params_.pid.kd, params_.pid.kd_limit);

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

void RMCS_Controller::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg) {
    if (msg->joint_names.size() == params_.joints.size()) {
        input_ref_.writeFromNonRT(msg);
    } else {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Received %zu , but expected %zu joints in command. Ignoring message.",
            msg->joint_names.size(), params_.joints.size());
    }
}

controller_interface::InterfaceConfiguration
    RMCS_Controller::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(
        params_.joints.size() * params_.command_interfaces.size());
    for (const auto& joint : params_.joints) {
        for (const auto& command : params_.command_interfaces) {
            command_interfaces_config.names.push_back(joint + "/" + command);
        }
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
    RMCS_Controller::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(params_.joints.size() * params_.state_interfaces.size());
    for (const auto& joint : params_.joints) {
        for (const auto& state : params_.state_interfaces) {
            state_interfaces_config.names.push_back(joint + "/" + state);
        }
    }

    return state_interfaces_config;
}

controller_interface::CallbackReturn
    RMCS_Controller::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for examplary use of
    // `controller_interface::get_ordered_interfaces` helper function
    // ?(KalecKKK) WTF does this mean?

    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT)(), params_.joints);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
    RMCS_Controller::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    ///// TODO(anyone): depending on number of interfaces, use definitions instead of a loop
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
    RMCS_Controller::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    auto current_ref = input_ref_.readFromRT();

    if (*(control_mode_.readFromRT()) == control_mode_type::POSITION_CTRL) {
        if (!std::isnan((*current_ref)->displacements[CMD_ITFS])
            && (*current_ref)->displacements[CMD_ITFS] != pid_controller.setPoint())
            pid_controller.setPoint((*current_ref)->displacements[CMD_ITFS]);

        command_interfaces_[CMD_ITFS].set_value(
            pid_controller.update(state_interfaces_[0].get_value()));
    } else { // control_mode_type::VELOCITY_CTRL
        if (!std::isnan((*current_ref)->velocities[CMD_ITFS])
            && (*current_ref)->velocities[CMD_ITFS] != pid_controller.setPoint())
            pid_controller.setPoint((*current_ref)->velocities[CMD_ITFS]);

        command_interfaces_[CMD_ITFS].set_value(
            pid_controller.update(state_interfaces_[1].get_value()));
    }

    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.joint_names.clear();
        state_publisher_->msg_.displacements.clear();
        state_publisher_->msg_.velocities.clear();

        state_publisher_->msg_.header.stamp = time;
        for (const auto& state : state_interfaces_) {
            state_publisher_->msg_.joint_names.push_back(state.get_name());
            if (state.get_interface_name() == "velocity") {
                state_publisher_->msg_.velocities.push_back(state.get_value());
            } else {
                state_publisher_->msg_.displacements.push_back(state.get_value());
            }
        }
        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace rmcs_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rmcs_controller::RMCS_Controller, controller_interface::ControllerInterface)
