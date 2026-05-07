#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/full_robot_id.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "referee/command/field.hpp"
#include "referee/command/interaction/header.hpp"

namespace rmcs_core::referee::command::interaction {

class SentryDecision
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SentryDecision()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/referee/id", robot_id_);
        register_input("/referee/sentry/mode", sentry_mode_);

        register_input("/referee/sentry/decision/enabled", decision_enabled_, false);
        register_input("/referee/sentry/decision/confirm_revive", confirm_revive_, false);
        register_input(
            "/referee/sentry/decision/exchange_instant_revive", exchange_instant_revive_, false);
        register_input(
            "/referee/sentry/decision/bullet_exchange_value", bullet_exchange_value_, false);
        register_input(
            "/referee/sentry/decision/remote_bullet_exchange_count",
            requested_remote_bullet_exchange_count_, false);
        register_input(
            "/referee/sentry/decision/remote_hp_exchange_count",
            requested_remote_hp_exchange_count_, false);
        register_input("/referee/sentry/decision/mode", requested_mode_, false);
        register_input(
            "/referee/sentry/decision/activate_energy_mechanism", activate_energy_mechanism_,
            false);

        register_output("/referee/command/interaction/sentry_decision", sentry_decision_field_);

        double resend_rate = 5.0;
        if (get_parameter("resend_rate", resend_rate) && resend_rate > 0.0) {
            resend_interval_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>{1.0 / resend_rate});
        }

        int64_t repeat_count = repeat_count_;
        if (get_parameter("repeat_count", repeat_count))
            repeat_count_ = std::max<int64_t>(repeat_count, 0);
    }

    void before_updating() override {
        decision_enabled_ready_ = decision_enabled_.ready();
        confirm_revive_ready_ = confirm_revive_.ready();
        exchange_instant_revive_ready_ = exchange_instant_revive_.ready();
        bullet_exchange_value_ready_ = bullet_exchange_value_.ready();
        requested_remote_bullet_exchange_count_ready_ =
            requested_remote_bullet_exchange_count_.ready();
        requested_remote_hp_exchange_count_ready_ = requested_remote_hp_exchange_count_.ready();
        requested_mode_ready_ = requested_mode_.ready();
        activate_energy_mechanism_ready_ = activate_energy_mechanism_.ready();

        if (!decision_enabled_.ready())
            decision_enabled_.bind_directly(default_false_);
        if (!confirm_revive_.ready())
            confirm_revive_.bind_directly(default_false_);
        if (!exchange_instant_revive_.ready())
            exchange_instant_revive_.bind_directly(default_false_);
        if (!bullet_exchange_value_.ready())
            bullet_exchange_value_.bind_directly(default_u16_);
        if (!requested_remote_bullet_exchange_count_.ready())
            requested_remote_bullet_exchange_count_.bind_directly(default_u8_);
        if (!requested_remote_hp_exchange_count_.ready())
            requested_remote_hp_exchange_count_.bind_directly(default_u8_);
        if (!requested_mode_.ready())
            requested_mode_.bind_directly(default_mode_);
        if (!activate_energy_mechanism_.ready())
            activate_energy_mechanism_.bind_directly(default_false_);
    }

    void update() override {
        *sentry_decision_field_ = Field{};

        if (!*decision_enabled_ || !has_decision_request() || !is_sentry_robot()) {
            reset_pending_command();
            return;
        }

        const uint32_t command = make_command();
        if (!last_command_ || command != *last_command_) {
            last_command_ = command;
            remaining_repeats_ = repeat_count_;
            next_sent_ = std::chrono::steady_clock::time_point::min();
        }

        const auto now = std::chrono::steady_clock::now();
        if (remaining_repeats_ == 0 || now < next_sent_)
            return;

        auto full_robot_id = rmcs_msgs::FullRobotId{*robot_id_};
        outgoing_header_.command_id = 0x0120;
        outgoing_header_.sender_id = full_robot_id;
        outgoing_header_.receiver_id = rmcs_msgs::FullRobotId::REFEREE_SERVER;
        outgoing_command_ = command;

        *sentry_decision_field_ =
            Field{[this](std::byte* buffer) { return write_sentry_decision(buffer); }};

        --remaining_repeats_;
        next_sent_ = now + resend_interval_;
    }

private:
    bool is_sentry_robot() const {
        return *robot_id_ == rmcs_msgs::RobotId::RED_SENTRY
            || *robot_id_ == rmcs_msgs::RobotId::BLUE_SENTRY;
    }

    bool has_decision_request() const {
        return confirm_revive_ready_ || exchange_instant_revive_ready_
            || bullet_exchange_value_ready_ || requested_remote_bullet_exchange_count_ready_
            || requested_remote_hp_exchange_count_ready_ || requested_mode_ready_
            || activate_energy_mechanism_ready_;
    }

    static uint8_t normalized_mode(uint8_t requested, uint8_t current) {
        current = current >= 1 && current <= 3 ? current : default_mode_value;
        return requested >= 1 && requested <= 3 ? requested : current;
    }

    uint32_t make_command() const {
        const uint16_t requested_bullet =
            bullet_exchange_value_ready_ ? *bullet_exchange_value_ : 0;
        const uint16_t bullet_exchange_value =
            std::min<uint16_t>(requested_bullet, max_bullet_exchange_value);

        const uint8_t remote_bullet_count = std::min<uint8_t>(
            requested_remote_bullet_exchange_count_ready_ ? *requested_remote_bullet_exchange_count_
                                                          : 0,
            max_count);
        const uint8_t remote_hp_count = std::min<uint8_t>(
            requested_remote_hp_exchange_count_ready_ ? *requested_remote_hp_exchange_count_
                                                      : 0,
            max_count);
        const uint8_t mode =
            normalized_mode(requested_mode_ready_ ? *requested_mode_ : *sentry_mode_, *sentry_mode_);

        uint32_t command = 0;
        command |= *confirm_revive_ ? 1u << 0 : 0;
        command |= *exchange_instant_revive_ ? 1u << 1 : 0;
        command |= static_cast<uint32_t>(bullet_exchange_value & 0x07ff) << 2;
        command |= static_cast<uint32_t>(remote_bullet_count & 0x0f) << 13;
        command |= static_cast<uint32_t>(remote_hp_count & 0x0f) << 17;
        command |= static_cast<uint32_t>(mode & 0x03) << 21;
        command |= *activate_energy_mechanism_ ? 1u << 23 : 0;
        return command;
    }

    size_t write_sentry_decision(std::byte* buffer) const {
        return write_field(buffer, outgoing_header_, outgoing_command_);
    }

    void reset_pending_command() {
        last_command_.reset();
        remaining_repeats_ = 0;
        next_sent_ = std::chrono::steady_clock::time_point::min();
    }

    static constexpr uint16_t max_bullet_exchange_value = 0x07ff;
    static constexpr uint8_t max_count = 0x0f;
    static constexpr uint8_t default_mode_value = 3;

    bool default_false_ = false;
    uint16_t default_u16_ = 0;
    uint8_t default_u8_ = 0;
    uint8_t default_mode_ = default_mode_value;

    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<uint8_t> sentry_mode_;

    InputInterface<bool> decision_enabled_;
    InputInterface<bool> confirm_revive_;
    InputInterface<bool> exchange_instant_revive_;
    InputInterface<uint16_t> bullet_exchange_value_;
    InputInterface<uint8_t> requested_remote_bullet_exchange_count_;
    InputInterface<uint8_t> requested_remote_hp_exchange_count_;
    InputInterface<uint8_t> requested_mode_;
    InputInterface<bool> activate_energy_mechanism_;

    bool decision_enabled_ready_ = false;
    bool confirm_revive_ready_ = false;
    bool exchange_instant_revive_ready_ = false;
    bool bullet_exchange_value_ready_ = false;
    bool requested_remote_bullet_exchange_count_ready_ = false;
    bool requested_remote_hp_exchange_count_ready_ = false;
    bool requested_mode_ready_ = false;
    bool activate_energy_mechanism_ready_ = false;

    OutputInterface<Field> sentry_decision_field_;

    Header outgoing_header_{};
    uint32_t outgoing_command_ = 0;

    std::optional<uint32_t> last_command_;
    int64_t repeat_count_ = 3;
    int64_t remaining_repeats_ = 0;
    std::chrono::steady_clock::duration resend_interval_ = std::chrono::milliseconds{200};
    std::chrono::steady_clock::time_point next_sent_ =
        std::chrono::steady_clock::time_point::min();
};

} // namespace rmcs_core::referee::command::interaction

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::command::interaction::SentryDecision, rmcs_executor::Component)
