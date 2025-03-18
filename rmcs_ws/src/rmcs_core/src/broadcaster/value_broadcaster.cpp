#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>

namespace rmcs_core::broadcaster {

class ValueBroadcaster
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ValueBroadcaster()
        : Node{get_component_name()} {
        declare_parameter<std::vector<std::string>>("forward_list", std::vector<std::string>{});
        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);
        parameter_callback_   = parameter_subscriber_->add_parameter_callback(
            "forward_list",
            [this](const rclcpp::Parameter& para) { update_forward_list(para.as_string_array()); });
    }

    void before_pairing(
        const std::map<std::string, const std::type_info&>& output_map) override {
        for (const auto& [name, type] : output_map) {
            if (type == typeid(double)) {
                forward_units_.emplace(
                    name,
                    std::make_unique<ForwardUnit<double, std_msgs::msg::Float64>>(this, name));
            }
        }
        std::vector<std::string> forward_list;
        if (get_parameter("forward_list", forward_list))
            update_forward_list(forward_list);
    }

    void update() override {
        for (auto& [name, unit] : forward_units_)
            unit->update();
    }

private:
    void update_forward_list(const std::vector<std::string>& forward_list) {
        for (auto& [name, unit] : forward_units_)
            unit->active = false;

        for (auto& name : forward_list) {
            auto iter = forward_units_.find(name);
            if (iter == forward_units_.end()) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Unable to find corresponding output of '%s', maybe the type of output is "
                    "unsupported or the output does not exist.",
                    name.c_str());
            } else {
                auto& unit   = iter->second;
                unit->active = true;
            }
        }

        for (auto& [name, unit] : forward_units_) {
            if (unit->active)
                unit->activate(this);
            else
                unit->deactivate();
        }
    }

    class BasicForwarderUnit {
    public:
        virtual ~BasicForwarderUnit() {}

        virtual void activate(rclcpp::Node* node) = 0;
        virtual void update()                     = 0;
        virtual void deactivate()                 = 0;

        bool active;
    };

    template <typename StdT, typename RosT>
    // using StdT = double;
    // using RosT = std_msgs::msg::Float64;
    class ForwardUnit : public BasicForwarderUnit {
    public:
        ForwardUnit(Component* component, const std::string& name)
            : name_(name) {
            component->register_input(name, input_);
        }

        void activate(rclcpp::Node* node) override {
            if (!publisher_)
                publisher_ = node->create_publisher<RosT>(name_, rclcpp::QoS{5}.reliable());
        }

        void update() override {
            if (!publisher_)
                return;

            RosT msg;
            msg.data = *input_;
            publisher_->publish(msg);
        }

        void deactivate() override { publisher_ = nullptr; }

    private:
        std::string name_;
        InputInterface<StdT> input_;
        rclcpp::Publisher<RosT>::SharedPtr publisher_;
    };

    std::map<std::string, std::unique_ptr<BasicForwarderUnit>> forward_units_;

    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::ValueBroadcaster, rmcs_executor::Component)