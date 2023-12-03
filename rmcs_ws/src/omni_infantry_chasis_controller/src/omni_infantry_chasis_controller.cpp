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

#include "omni_infantry_chasis_controller/omni_infantry_chasis_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace { // utility

std::vector<std::string> Chasis_List;

// TODO(destogl): remove this when merged upstream
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

using ControllerReferenceMsg =
    omni_infantry_chasis_controller::OmniInfantryChasisController::ControllerReferenceMsg;

} // namespace

namespace omni_infantry_chasis_controller {
OmniInfantryChasisController::OmniInfantryChasisController()
    : controller_interface::ControllerInterface()
    , ChasisData_(4) {}

controller_interface::CallbackReturn OmniInfantryChasisController::on_init() {
    control_mode_.initRT(control_mode_type::FAST);

    try {
        param_listener_ =
            std::make_shared<omni_infantry_chasis_controller::ParamListener>(get_node());
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    Chasis_List.reserve(4);
    Chasis_List.emplace_back("front_left_wheel");
    Chasis_List.emplace_back("front_right_wheel");
    Chasis_List.emplace_back("back_left_wheel");
    Chasis_List.emplace_back("back_right_wheel");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
    OmniInfantryChasisController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {

    ChasisData_.BindNode(get_node());
    params_ = param_listener_->get_params();

    state_joints_ = params_.joints;

    configure_chasis_motor_pid(); // pid数据获取

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    try {

        // State publisher
        RemoteCmdSubscriber_ = get_node()->create_subscription<Dbus>(
            "remotecmd", rclcpp::SystemDefaultsQoS(),
            std::bind(
                &OmniInfantryChasisController::RemoteCmdCallback, this, std::placeholders::_1));

    } catch (const std::exception& e) {
        fprintf(
            stderr,
            "Exception thrown during subscriber creation at configure stage with message : %s \n",
            e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

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
    state_publisher_->lock();
    state_publisher_->msg_.header.frame_id = params_.joints[0];
    state_publisher_->unlock();

    std::shared_ptr<CmdData> NewCmdData(new CmdData);

    input_ref_.initRT(NewCmdData);

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
    OmniInfantryChasisController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(
        params_.joints.size() * params_.cmd_interface_name.size());
    for (const auto& joint : params_.joints) {
        for (const auto& cmd_interface_name : params_.cmd_interface_name) {
            command_interfaces_config.names.push_back(joint + "/" + cmd_interface_name);
            RCLCPP_INFO(
                get_node()->get_logger(), "configure %s",
                (joint + "/" + cmd_interface_name).c_str());
            // command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
            // RCLCPP_INFO(
            //     get_node()->get_logger(), "configure %s", (joint + "/" + HW_IF_EFFORT).c_str());
        }
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
    OmniInfantryChasisController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(
        state_joints_.size() * params_.state_interface_name.size());
    for (const auto& joint : params_.joints) {
        for (const auto& state_interface_name : params_.state_interface_name) {
            state_interfaces_config.names.push_back(joint + "/" + state_interface_name);
            RCLCPP_INFO(
                get_node()->get_logger(), "configure %s",
                (joint + "/" + state_interface_name).c_str());
            // state_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
            // state_interfaces_config.names.push_back(joint + "/" + HW_IF_POSITION);
            // RCLCPP_INFO(
            //     get_node()->get_logger(), "configure %s", (joint + "/" +
            //     HW_IF_VELOCITY).c_str());
            // RCLCPP_INFO(
            //     get_node()->get_logger(), "configure %s", (joint + "/" +
            //     HW_IF_POSITION).c_str());
        }
    }
    return state_interfaces_config;
}

controller_interface::CallbackReturn
    OmniInfantryChasisController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): if you have to manage multiple interfaces that need to be sorted check
    // `on_activate` method in `JointTrajectoryController` for examplary use of
    // `controller_interface::get_ordered_interfaces` helper function
    configure_chasis_motor(Chasis_List);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
    OmniInfantryChasisController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniInfantryChasisController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    auto current_ref = input_ref_.readFromRT();

    if (!(std::isnan(current_ref->get()->ChasisVw_) || std::isnan(current_ref->get()->ChasisVx_)
          || std::isnan(current_ref->get()->ChasisVy_))) {
        ChasisData_.Update(
            current_ref->get()->ChasisVx_, current_ref->get()->ChasisVy_,
            current_ref->get()->ChasisVw_);
    }
    // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
    // instead of a loop

    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;
        state_publisher_->msg_.set_point    = command_interfaces_[CMD_MY_ITFS].get_value();
        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn OmniInfantryChasisController::configure_chasis_motor(
    const std::vector<std::string>& chasis_names_vector) {
    auto logger = get_node()->get_logger();

    if (chasis_names_vector.empty()) {
        RCLCPP_ERROR(logger, "Gimbal is empty\n");
        return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    for (const auto& chasis_names : chasis_names_vector) {

        const auto velocity_state_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&chasis_names](const auto& interface) {
                return interface.get_prefix_name() == chasis_names
                    && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
            });

        if (velocity_state_handle == state_interfaces_.cend()) {
            RCLCPP_ERROR(
                logger, "Unable to obtain velocity joint state handle for %s",
                chasis_names.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            logger, "%s 's velocity state name is %s name is %s ", chasis_names.c_str(),
            velocity_state_handle->get_interface_name().c_str(),
            velocity_state_handle->get_full_name().c_str());

        const auto effort_command_handle = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [&chasis_names](const auto& interface) {
                return interface.get_prefix_name() == chasis_names
                    && interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
            });

        if (effort_command_handle == command_interfaces_.end()) {
            RCLCPP_ERROR(
                logger, "Unable to obtain joint command handle for %s", chasis_names.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            logger, "%s 's effort cmd name is %s name is %s ", chasis_names.c_str(),
            effort_command_handle->get_interface_name().c_str(),
            effort_command_handle->get_full_name().c_str());

        ChasisData_.AddChasisHandle(*velocity_state_handle, *effort_command_handle);
    }

    if (ChasisData_.GetFeedbackDataSize() != 4) {
        RCLCPP_ERROR(
            logger, "ChasisData FeedbackDataSize= % ld Too Less ",
            ChasisData_.GetFeedbackDataSize());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniInfantryChasisController::configure_chasis_motor_pid() {
    double LfP = params_.front_left_wheel.pid_data.P;
    double LfI = params_.front_left_wheel.pid_data.I;
    double LfD = params_.front_left_wheel.pid_data.D;

    PID::PID LfPid(LfP, LfI, LfD, 10000);

    ChasisData_.AddChasisPid(LfPid);

    double LrP = params_.front_left_wheel.pid_data.P;
    double LrI = params_.front_left_wheel.pid_data.I;
    double LrD = params_.front_left_wheel.pid_data.D;

    PID::PID LrPid(LrP, LrI, LrD, 10000);

    ChasisData_.AddChasisPid(LrPid);

    double BfP = params_.front_left_wheel.pid_data.P;
    double BfI = params_.front_left_wheel.pid_data.I;
    double BfD = params_.front_left_wheel.pid_data.D;

    PID::PID BfPid(BfP, BfI, BfD, 10000);

    ChasisData_.AddChasisPid(BfPid);

    double BrP = params_.front_left_wheel.pid_data.P;
    double BrI = params_.front_left_wheel.pid_data.I;
    double BrD = params_.front_left_wheel.pid_data.D;

    PID::PID BrPid(BrP, BrI, BrD, 10000);

    ChasisData_.AddChasisPid(BrPid);
}

void OmniInfantryChasisController::RemoteCmdCallback(const rmcs_msgs::msg::Dbus& msg) {
    if (msg.sw1 == 0 && msg.sw2 == 0) {
        double vx = static_cast<double>(msg.ch4) / 660.0 * 1.0;
        double vy = static_cast<double>(msg.ch2) / 660.0 * 1.0;
        double vw = static_cast<double>(msg.ch1) / 660.0 * 1.0;

        input_ref_.writeFromNonRT(std::shared_ptr<CmdData>(new CmdData(vx, vy, vw)));
    }
}

} // namespace omni_infantry_chasis_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    omni_infantry_chasis_controller::OmniInfantryChasisController,
    controller_interface::ControllerInterface)
