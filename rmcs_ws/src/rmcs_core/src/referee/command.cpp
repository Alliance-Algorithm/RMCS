#include <chrono>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>

#include "referee/command/field.hpp"
#include "referee/frame.hpp"

namespace rmcs_core::referee {
using namespace command;

class Command
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Command()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , next_sent_(std::chrono::steady_clock::time_point::min())
        , interaction_next_sent_(std::chrono::steady_clock::time_point::min())
        , map_marker_next_sent_(std::chrono::steady_clock::time_point::min())
        , text_display_next_sent_(std::chrono::steady_clock::time_point::min()) {

        register_input("/referee/serial", serial_, false);

        register_input("/referee/command/interaction", interaction_field_, false);
        register_input("/referee/command/map_marker", map_marker_field_, false);
        register_input("/referee/command/text_display", text_display_field_, false);
    }

    void before_updating() override {
        if (!interaction_field_.ready())
            interaction_field_.bind_directly(empty_field_);
        if (!map_marker_field_.ready())
            map_marker_field_.bind_directly(empty_field_);
        if (!text_display_field_.ready())
            text_display_field_.bind_directly(empty_field_);
    }

    void update() override {
        if (!serial_.ready())
            return;

        using namespace std::chrono_literals;
        auto now = std::chrono::steady_clock::now();
        auto& serial = const_cast<rmcs_msgs::SerialInterface&>(*serial_);

        if (now < next_sent_)
            return;

        constexpr auto one_second = std::chrono::steady_clock::duration(1s);
        size_t data_length;
        if (now >= interaction_next_sent_ && !interaction_field_->empty()) {
            interaction_next_sent_ = now + (one_second / 25); // 25hz max to reduce packet loss
            frame_.body.command_id = 0x0301;
            data_length = interaction_field_->write(frame_.body.data);
        } else if (now >= map_marker_next_sent_ && !map_marker_field_->empty()) {
            map_marker_next_sent_ = now + (one_second / 1);   // 1hz max
            frame_.body.command_id = 0x0307;
            data_length = map_marker_field_->write(frame_.body.data);
        } else if (now >= text_display_next_sent_ && !text_display_field_->empty()) {
            text_display_next_sent_ = now + (one_second / 3); // 3hz max
            frame_.body.command_id = 0x0308;
            data_length = text_display_field_->write(frame_.body.data);
        } else {
            return;
        }

        // TODO(qzh): Assert data length.

        frame_.header.sof = sof_value;
        frame_.header.data_length = data_length;
        frame_.header.sequence = 0;
        rmcs_utility::dji_crc::append_crc8(frame_.header);

        auto frame_size =
            sizeof(frame_.header) + sizeof(frame_.body.command_id) + data_length + sizeof(uint16_t);
        rmcs_utility::dji_crc::append_crc16(&frame_, frame_size);

        serial.write(reinterpret_cast<std::byte*>(&frame_), frame_size);
        next_sent_ = now + (one_second / 3720 * frame_size);
    }

private:
    InputInterface<rmcs_msgs::SerialInterface> serial_;
    Frame frame_;

    Field empty_field_;
    std::chrono::steady_clock::time_point next_sent_;

    InputInterface<Field> interaction_field_;
    std::chrono::steady_clock::time_point interaction_next_sent_;

    InputInterface<Field> map_marker_field_;
    std::chrono::steady_clock::time_point map_marker_next_sent_;

    InputInterface<Field> text_display_field_;
    std::chrono::steady_clock::time_point text_display_next_sent_;
};

} // namespace rmcs_core::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::Command, rmcs_executor::Component)