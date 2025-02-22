#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "referee/command/field.hpp"

namespace rmcs_core::referee::command {

class Interaction
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Interaction()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        register_input(
            "/referee/command/interaction/sentry_decision", sentry_decision_field_, false);
        register_input("/referee/command/interaction/communicate", communicate_field_, false);
        register_input("/referee/command/interaction/ui", ui_field_, false);

        register_output("/referee/command/interaction", interaction_field_);
    }

    void before_updating() override {
        if (!sentry_decision_field_.ready())
            sentry_decision_field_.bind_directly(empty_field_);
        if (!communicate_field_.ready())
            communicate_field_.bind_directly(empty_field_);
        if (!ui_field_.ready())
            ui_field_.bind_directly(empty_field_);
    }

    void update() override {
        // if (*sentry_decision_field_)
        //     *interaction_field_ = *sentry_decision_field_;
        // else if (*ui_field_)
        //     *interaction_field_ = *ui_field_;
        // else
        //     *interaction_field_ = Field{};

        const auto phase_index = update_num % total;
        if (phase_index < ui_proportion) {
            *interaction_field_ = *ui_field_;
        } else if (phase_index < communicate_proportion + ui_proportion) {
            *interaction_field_ = *communicate_field_;
        } else {
            *interaction_field_ = Field{};
        }

        ++update_num;
    }

private:
    uint update_num = 0;
    Field empty_field_;

    // uint ui_proportion{static_cast<uint>(get_parameter("ui").as_int())};
    // uint communicate_proportion{static_cast<uint>(get_parameter("communicate").as_int())};
    // uint
    // sentry_decision_proportion{static_cast<uint>(get_parameter("sentry_decision").as_int())};
    // uint total = sentry_decision_proportion + ui_proportion + communicate_proportion;

    // for debug
    uint ui_proportion{9};
    uint communicate_proportion{1};
    uint sentry_decision_proportion{0};
    uint total = sentry_decision_proportion + ui_proportion + communicate_proportion;

    InputInterface<Field> sentry_decision_field_;
    InputInterface<Field> communicate_field_;
    InputInterface<Field> ui_field_;

    struct __attribute__((packed)) InteractionHeader {
        uint16_t command_id;
        uint16_t sender_id;
        uint16_t receiver_id;
    };

    OutputInterface<Field> interaction_field_;
};

} // namespace rmcs_core::referee::command

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::command::Interaction, rmcs_executor::Component)