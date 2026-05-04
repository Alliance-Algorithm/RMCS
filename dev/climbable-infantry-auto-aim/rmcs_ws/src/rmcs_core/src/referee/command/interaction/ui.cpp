#include <algorithm>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/full_robot_id.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "referee/app/ui/shape/cfs_scheduler.hpp"
#include "referee/app/ui/shape/shape.hpp"
#include "referee/command/interaction/header.hpp"

namespace rmcs_core::referee::command::interaction {
using namespace app::ui;

class Ui
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Ui()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/referee/id", robot_id_);
        register_input("/referee/game/stage", game_stage_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/referee/command/interaction/ui", ui_field_);
    }

    void update() override {
        if (*robot_id_ == rmcs_msgs::RobotId::UNKNOWN) {
            *ui_field_ = Field{};
            return;
        }

        if ((last_game_stage_ == rmcs_msgs::GameStage::UNKNOWN
             && *game_stage_ != rmcs_msgs::GameStage::UNKNOWN)
            || (last_game_stage_ != rmcs_msgs::GameStage::PREPARATION
                && *game_stage_ == rmcs_msgs::GameStage::PREPARATION)
            || (!last_keyboard_.r && keyboard_->r)) {
            RemoteShape<Shape>::force_revoke_all_id();
            resetting_ = 4;
        }
        last_game_stage_ = *game_stage_;
        last_keyboard_   = *keyboard_;

        if (resetting_) {
            *ui_field_ = Field{[this](std::byte* buffer) {
                --resetting_;
                return write_resetting_field(buffer);
            }};
            return;
        }

        if (CfsScheduler<Shape>::empty()) {
            *ui_field_ = Field{};
            return;
        }

        *ui_field_ = Field{[this](std::byte* buffer) { return write_updating_field(buffer); }};
    }

private:
    size_t write_resetting_field(std::byte* buffer) const {
        size_t written = 0;

        auto& header       = *new (buffer + written) Header{};
        header.command_id  = 0x0100; // Clear shapes
        auto full_robot_id = rmcs_msgs::FullRobotId{*robot_id_};
        header.sender_id   = full_robot_id;
        header.receiver_id = full_robot_id.client();
        written += sizeof(Header);

        struct Command {
            uint8_t type;
            uint8_t layer;
        };
        auto& command = *new (buffer + written) Command{};
        command.type  = 2;           // Clear all layers
        command.layer = 0;
        written += sizeof(Command);

        return written;
    }

    size_t write_updating_field(std::byte* buffer) const {
        size_t written = 0;

        auto& header       = *new (buffer + written) Header{};
        auto full_robot_id = rmcs_msgs::FullRobotId{*robot_id_};
        header.sender_id   = full_robot_id;
        header.receiver_id = full_robot_id.client();
        written += sizeof(Header);

        int slot = 0;
        intptr_t updated[7];
        for (auto it = CfsScheduler<Shape>::get_update_iterator(); it && slot < 7;) {
            // Ignore text shape unless it is the first.
            if (it->is_text_shape()) {
                if (slot == 0) {
                    header.command_id = 0x0110; // Draw text shape
                    return written + it.update().write(buffer + written);
                } else {
                    it.ignore();
                    continue;
                }
            }

            auto operation = it->predict_update();
            if (operation == Shape::Operation::NO_OPERATION) {
                it.ignore();
                continue;
            }

            // Shapes are always aligned, so the last bits can be used to store information.
            auto identification =
                reinterpret_cast<intptr_t>(it.get()) | (operation == Shape::Operation::ADD);
            // Ignore identical shapes that operate identically.
            if (std::find(updated, updated + slot, identification) != updated + slot) {
                it.ignore();
                continue;
            }

            written += it.update().write(buffer + written);

            updated[slot++] = identification;
        }

        constexpr std::pair<int, uint16_t> optional_packet[4] = {
            {1, 0x0101}, // Draw 1 shape
            {2, 0x0102}, // Draw 2 shapes
            {5, 0x0103}, // Draw 5 shapes
            {7, 0x0104}, // Draw 7 shapes
        };
        for (const auto& [shape_count, command_id] : optional_packet) {
            if (slot <= shape_count) {
                for (; slot < shape_count; ++slot) {
                    written += Shape::no_operation_description().write(buffer + written);
                }
                header.command_id = command_id;
                break;
            }
        }

        return written;
    }

    InputInterface<rmcs_msgs::RobotId> robot_id_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;
    rmcs_msgs::GameStage last_game_stage_ = rmcs_msgs::GameStage::UNKNOWN;

    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    int resetting_ = 0;

    OutputInterface<Field> ui_field_;
};

} // namespace rmcs_core::referee::command::interaction

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::command::interaction::Ui, rmcs_executor::Component)