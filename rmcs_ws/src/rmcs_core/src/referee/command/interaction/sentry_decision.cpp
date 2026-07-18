#include "referee/command/field.hpp"
#include "referee/command/interaction/header.hpp"
#include "referee/status/field.hpp"

#include <array>
#include <cstdint>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/full_robot_id.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/sentry_event.hpp>
#include <unordered_map>
#include <unordered_set>

namespace rmcs_core::referee::command::interaction {

class SentryDecision
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    using Command = status::SentryCommand;
    using Posture = Command::Posture;
    using SentryEvent = rmcs_msgs::SentryEvent;
    using EventCounts = std::unordered_map<SentryEvent, std::uint16_t>;
    using Clock = std::chrono::steady_clock;

    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<EventCounts> sentry_events_;
    InputInterface<std::uint8_t> sentry_posture_fb_;
    InputInterface<std::uint16_t> robot_hp_fb_;
    InputInterface<std::uint8_t> energy_core_status_;
    InputInterface<bool> can_rebirth_free_;

    OutputInterface<Field> sentry_decision_field_;

    Header header_{};
    Command command_{};

    EventCounts cached_events_;
    std::unordered_set<SentryEvent> requests_;
    std::unordered_map<SentryEvent, Posture> pose_targets_;
    std::unordered_set<SentryEvent> logged_events_;
    std::uint8_t last_fb_posture_ = 3;
    bool last_can_rebirth_free_ = false;

    Clock::time_point last_sent_{Clock::now()};
    Clock::time_point last_status_log_{Clock::now()};

    static const std::unordered_set<SentryEvent>& kPoseEvents() {
        static const auto s = std::unordered_set<SentryEvent>{
            SentryEvent::SWITCH_POSE_ATTACK,         SentryEvent::SWITCH_POSE_DEFENSE,
            SentryEvent::SWITCH_POSE_MOVE,           SentryEvent::SWITCH_POSE_POWERED_ATTACK,
            SentryEvent::SWITCH_POSE_POWERED_DEFENSE, SentryEvent::SWITCH_POSE_POWERED_MOVE,
        };
        return s;
    }

    static auto to_posture(SentryEvent event) -> Posture {
        switch (event) {
        case SentryEvent::SWITCH_POSE_ATTACK: return Posture::ATTACK;
        case SentryEvent::SWITCH_POSE_DEFENSE: return Posture::DEFENSE;
        case SentryEvent::SWITCH_POSE_MOVE: return Posture::MOVE;
        case SentryEvent::SWITCH_POSE_POWERED_ATTACK: return Posture::POWERED_ATTACK;
        case SentryEvent::SWITCH_POSE_POWERED_DEFENSE: return Posture::POWERED_DEFENSE;
        case SentryEvent::SWITCH_POSE_POWERED_MOVE: return Posture::POWERED_MOVE;
        default: return Posture::MOVE;
        }
    }

    static constexpr auto kEventPriority = std::array{
        SentryEvent::CONFIRM_REBIRTH,            SentryEvent::CONFIRM_INSTANT_REBIRTH,
        SentryEvent::SWITCH_POSE_ATTACK,         SentryEvent::SWITCH_POSE_DEFENSE,
        SentryEvent::SWITCH_POSE_MOVE,           SentryEvent::SWITCH_POSE_POWERED_ATTACK,
        SentryEvent::SWITCH_POSE_POWERED_DEFENSE, SentryEvent::SWITCH_POSE_POWERED_MOVE,
        SentryEvent::EXCHANGE_AMMO_SUPPLY_POINT,  SentryEvent::EXCHANGE_AMMO_REMOTE,
        SentryEvent::EXCHANGE_HP_REMOTE,          SentryEvent::ACTIVATE_ENERGY_CORE,
    };

    SentryDecision()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/referee/id", robot_id_);
        register_input("/rmcs_navigation/sentry_events", sentry_events_, false);
        register_input("/referee/sentry/posture", sentry_posture_fb_, false);
        register_input("/referee/current_hp", robot_hp_fb_, false);
        register_input(
            "/referee/event/ally_big_energy_activation_status", energy_core_status_, false);
        register_input("/referee/sentry/can_rebirth_free", can_rebirth_free_, false);

        register_output("/referee/command/interaction/sentry_decision", sentry_decision_field_);
    }

    auto before_updating() -> void override {
        if (!sentry_events_.ready())
            sentry_events_.make_and_bind_directly();
        if (!sentry_posture_fb_.ready())
            sentry_posture_fb_.make_and_bind_directly(uint8_t{3});
        if (!robot_hp_fb_.ready())
            robot_hp_fb_.make_and_bind_directly(uint16_t{0});
        if (!energy_core_status_.ready())
            energy_core_status_.make_and_bind_directly(uint8_t{0});
        if (!can_rebirth_free_.ready())
            can_rebirth_free_.make_and_bind_directly(false);
    }

    auto update() -> void override {
        using namespace std::chrono_literals;

        if (*robot_id_ == rmcs_msgs::RobotId::UNKNOWN) {
            *sentry_decision_field_ = Field{};
            return;
        }

        const auto now = Clock::now();

        if (now - last_status_log_ > 1s) {
            RCLCPP_INFO(get_logger(), "Sentry posture: %d", *sentry_posture_fb_);
            last_status_log_ = now;
        }

        detect_new_events();

        const auto can_rebirth_free = *can_rebirth_free_;
        if (can_rebirth_free && !last_can_rebirth_free_) {
            requests_.insert(SentryEvent::CONFIRM_REBIRTH);
        }
        last_can_rebirth_free_ = can_rebirth_free;

        consume_one_event(now);
        verify_feedback();
    }

private:
    auto detect_new_events() -> void {
        const auto& input = *sentry_events_;

        for (const auto event : kEventPriority) {
            auto input_it = input.find(event);
            auto input_count = (input_it != input.end()) ? input_it->second : uint16_t{0};
            auto cache_count = cached_events_[event];

            if (cache_count != input_count) {
                if (kPoseEvents().contains(event)) {
                    for (const auto rm : kPoseEvents())
                        requests_.erase(rm);
                }
                requests_.insert(event);
                cached_events_[event] = input_count;
            }
        }
    }

    auto consume_one_event(Clock::time_point now) -> void {
        using namespace std::chrono_literals;

        const auto id = rmcs_msgs::FullRobotId{*robot_id_};
        header_.command_id = 0x0120;
        header_.sender_id = id;
        header_.receiver_id = rmcs_msgs::FullRobotId::REFEREE_SERVER;

        for (const auto event : kEventPriority) {
            if (!requests_.contains(event))
                continue;

            command_ = Command{};

            if (kPoseEvents().contains(event)) {
                command_.posture = to_posture(event);
                pose_targets_[event] = to_posture(event);
            } else if (event == SentryEvent::CONFIRM_REBIRTH) {
                command_.rebirth_confirm = 1;
            } else if (event == SentryEvent::CONFIRM_INSTANT_REBIRTH) {
                command_.instant_rebirth_confirm = 1;
            } else if (event == SentryEvent::EXCHANGE_AMMO_SUPPLY_POINT) {
                command_.ammo_exchange = 1;
            } else if (event == SentryEvent::EXCHANGE_AMMO_REMOTE) {
                command_.remote_ammo_request = 1;
            } else if (event == SentryEvent::EXCHANGE_HP_REMOTE) {
                command_.remote_hp_request = 1;
            } else if (event == SentryEvent::ACTIVATE_ENERGY_CORE) {
                command_.energy_core_confirm = 1;
            }

            *sentry_decision_field_ = MAKE_FIELD(header_, command_);
            last_sent_ = now;

            if (kPoseEvents().contains(event)) {
                if (!logged_events_.contains(event)) {
                    RCLCPP_INFO(get_logger(), "Sentry pose command: %d",
                                std::to_underlying(command_.posture));
                    logged_events_.insert(event);
                }
            }

            break;
        }
    }

    auto verify_feedback() -> void {
        const auto fb_posture_id = *sentry_posture_fb_;
        const auto fb_hp = *robot_hp_fb_;

        if (fb_posture_id != last_fb_posture_) {
            RCLCPP_INFO(
                get_logger(), "Sentry posture feedback: %d → %d", last_fb_posture_, fb_posture_id);
            last_fb_posture_ = fb_posture_id;
        }

        auto to_erase = std::vector<SentryEvent>{};
        for (const auto event : requests_) {
            if (kPoseEvents().contains(event)) {
                auto it = pose_targets_.find(event);
                if (it != pose_targets_.end()
                    && static_cast<uint8_t>(it->second) == fb_posture_id) {
                    to_erase.push_back(event);
                    pose_targets_.erase(it);
                    logged_events_.erase(event);
                }
            } else if (event == SentryEvent::CONFIRM_REBIRTH
                       || event == SentryEvent::CONFIRM_INSTANT_REBIRTH) {
                if (fb_hp > 0) {
                    to_erase.push_back(event);
                }
            } else {
                to_erase.push_back(event);
            }
        }

        for (const auto event : to_erase)
            requests_.erase(event);
    }
};

} // namespace rmcs_core::referee::command::interaction

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::command::interaction::SentryDecision, rmcs_executor::Component)
