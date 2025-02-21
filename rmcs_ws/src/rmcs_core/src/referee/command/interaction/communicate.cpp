#include <algorithm>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/full_robot_id.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "referee/command/field.hpp"
#include "referee/command/interaction/header.hpp"
#include "referee/status/field.hpp"

namespace rmcs_core::referee::command::interaction {
class CommunicateForInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CommunicateForInfantry()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/referee/id", robot_id_);
        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);
        register_output("/referee/command/interaction/communicate", communicate_field_);
    }

    void update() override {
        if (*robot_id_ == rmcs_msgs::RobotId::UNKNOWN) {
            *communicate_field_ = Field{};
            return;
        }

        // TODO:重置和编解码可能的错误处理

        *communicate_field_ = Field{[this](std::byte* buffer) { return write_field(buffer); }};
    };

private:
    size_t write_field(std::byte* buffer) {
        size_t written     = 0;
        auto& header       = *new (buffer + written) Header{};
        header.command_id  = 0x0200;
        auto full_robot_id = rmcs_msgs::FullRobotId{*robot_id_};
        header.sender_id   = full_robot_id;
        header.receiver_id = rmcs_msgs::FullRobotId::RED_HERO;
        written += sizeof(Header);

        auto& command = *new (buffer + written) status::CommunicateBulletAllowance{};

        // command.bullet_allowance = *robot_bullet_allowance_;
        // for debug:
        command.bullet_allowance = 20;
        written += sizeof(status::CommunicateBulletAllowance);

        return written;
    }
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint16_t> robot_bullet_allowance_;
    OutputInterface<Field> communicate_field_;
};

class CommunicateForHero
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CommunicateForHero()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/referee/id", robot_id_);
        register_input("/referee/shooter/bullet_allowance", robot_bullet_allowance_);
        register_output("/referee/command/interaction/communicate", communicate_field_);
    }

    void update() override {
        if (*robot_id_ == rmcs_msgs::RobotId::UNKNOWN) {
            *communicate_field_ = Field{};
            return;
        }

        // TODO:重置和编解码可能的错误处理

        *communicate_field_ = Field{[this](std::byte* buffer) { return write_field(buffer); }};
    };

private:
    size_t write_field(std::byte* buffer) {
        size_t written     = 0;
        auto& header       = *new (buffer + written) Header{};
        header.command_id  = 0x0200;
        auto full_robot_id = rmcs_msgs::FullRobotId{*robot_id_};
        header.sender_id   = full_robot_id;
        header.receiver_id = rmcs_msgs::FullRobotId::RED_INFANTRY_III;
        written += sizeof(Header);

        auto& command = *new (buffer + written) status::CommunicateBulletAllowance;

        // command.robot_bullet_allowance = *robot_bullet_allowance_;
        // for debug:
        command.bullet_allowance = 30;
        written += sizeof(status::CommunicateBulletAllowance);

        return written;
    }
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint16_t> robot_bullet_allowance_;
    OutputInterface<Field> communicate_field_;
};
}; // namespace rmcs_core::referee::command::interaction

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::command::interaction::CommunicateForHero, rmcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::command::interaction::CommunicateForInfantry, rmcs_executor::Component)