#include <algorithm>
#include <chrono>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_id.hpp>
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
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , next_sent_(std::chrono::steady_clock::time_point::min())
        , interaction_next_sent_(std::chrono::steady_clock::time_point::min())
        , map_marker_next_sent_(std::chrono::steady_clock::time_point::min())
        , text_display_next_sent_(std::chrono::steady_clock::time_point::min()) {

        register_input("/referee/serial", serial_, false);

        register_input("/referee/command/interaction", interaction_field_, false);
        register_input("/referee/command/map_marker", map_marker_field_, false);
        register_input("/referee/command/text_display", text_display_field_, false);
        register_input("/referee/id", robot_id_, false);
        register_input("/referee/ally/base/hp", ally_base_hp_, false);

        bandwidth_limit_ = get_parameter_or("bandwidth_limit", 0.0);
        base_hp_max_ = static_cast<uint16_t>(get_parameter_or("base_hp_max", 0));
    }

    void before_updating() override {
        if (!interaction_field_.ready())
            interaction_field_.bind_directly(empty_field_);
        if (!map_marker_field_.ready())
            map_marker_field_.bind_directly(empty_field_);
        if (!text_display_field_.ready())
            text_display_field_.bind_directly(empty_field_);
        if (!robot_id_.ready())
            robot_id_.make_and_bind_directly(rmcs_msgs::RobotId{});
        if (!ally_base_hp_.ready())
            ally_base_hp_.make_and_bind_directly(uint16_t{0});
    }

    void update() override {
        if (!serial_.ready())
            return;

        using namespace std::chrono_literals;
        auto now = std::chrono::steady_clock::now();
        auto& serial = const_cast<rmcs_msgs::SerialInterface&>(*serial_);

        const auto robot_id = *robot_id_;
        const auto is_sentry =
            robot_id == rmcs_msgs::RobotId::RED_SENTRY
            || robot_id == rmcs_msgs::RobotId::BLUE_SENTRY;

        const auto current_base_hp = *ally_base_hp_;
        if (current_base_hp > observed_max_base_hp_)
            observed_max_base_hp_ = current_base_hp;

        const auto max_base_hp = base_hp_max_ > 0 ? base_hp_max_ : observed_max_base_hp_;
        auto lambda = 1.0;
        if (max_base_hp > 0 && current_base_hp <= max_base_hp) {
            lambda = std::clamp(
                1.0 - static_cast<double>(max_base_hp - current_base_hp) / 20000.0, 0.0, 1.0);
        }

        auto bandwidth_limit = bandwidth_limit_;
        if (bandwidth_limit <= 0.0)
            bandwidth_limit = is_sentry ? 5120.0 : 3720.0;
        bandwidth_limit *= lambda;

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

        const auto interval = std::chrono::duration<double>(
            static_cast<double>(frame_size) / std::max(bandwidth_limit, 1.0));
        next_sent_ =
            now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval);
    }

private:
    InputInterface<rmcs_msgs::SerialInterface> serial_;
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint16_t> ally_base_hp_;
    double bandwidth_limit_ = 0.0;
    uint16_t base_hp_max_ = 0;
    uint16_t observed_max_base_hp_ = 0;
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