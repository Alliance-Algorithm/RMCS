#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/full_robot_id.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "referee/command/field.hpp"
#include "referee/command/interaction/header.hpp"
#include "referee/status/field.hpp"

namespace rmcs_core::referee::command::interaction {

class SentryDecision
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    using Command = status::SentryCommand;
    using Clock = std::chrono::steady_clock;

    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<bool> auto_aim_should_control_;

    OutputInterface<Field> sentry_decision_field_;

    Header header_{};
    Command command_{};

    Clock::time_point last_sent_{Clock::now()};

    Command::Posture last_posture_ = Command::Posture::MOVE;
    Clock::time_point last_change_{Clock::now()};
    Clock::time_point last_attack_{Clock::now()};

    SentryDecision()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input("/auto_aim/should_control", auto_aim_should_control_, false);
        register_input("/referee/id", robot_id_);

        register_output("/referee/command/interaction/sentry_decision", sentry_decision_field_);
    }

    auto before_updating() -> void override {
        if (auto_aim_should_control_.ready() == false) {
            auto_aim_should_control_.make_and_bind_directly(false);
        }
    }

    auto update() -> void override {
        using namespace std::chrono_literals;
        using Command = status::SentryCommand;

        if (*robot_id_ == rmcs_msgs::RobotId::UNKNOWN) {
            *sentry_decision_field_ = Field{};
            return;
        }

        auto posture = Command::Posture::MOVE;

        const auto now = Clock::now();
        if (*auto_aim_should_control_) {
            last_attack_ = now;
        }
        if (now - last_attack_ < 2s) {
            posture = Command::Posture::ATTACK;
        }

        const auto id = rmcs_msgs::FullRobotId{*robot_id_};
        header_.command_id = 0x0120;
        header_.sender_id = id;
        header_.receiver_id = rmcs_msgs::FullRobotId::REFEREE_SERVER;

        command_ = Command{};
        command_.rebirth_confirmed = 1;
        command_.posture = posture;

        const auto need_change = (posture != last_posture_) && (now - last_change_ > 5s);
        const auto reach_delay = (now - last_sent_ > 5s);
        if (reach_delay || need_change) {
            *sentry_decision_field_ = MAKE_FIELD(header_, command_);
            if (need_change) {
                RCLCPP_INFO(
                    get_logger(), "Sentry posture changes to %d", std::to_underlying(posture));
                last_change_ = now;
            }
            last_posture_ = posture;
            last_sent_ = now;
        }
    }
};

} // namespace rmcs_core::referee::command::interaction

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::referee::command::interaction::SentryDecision, rmcs_executor::Component)
