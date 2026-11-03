#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rmcs_core::broadcaster {

class ValueBroadcaster
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ValueBroadcaster()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        double publish_rate;
        get_parameter_or("publish_rate", publish_rate, 0.0);
        if (!std::isfinite(publish_rate) || publish_rate < 0.0)
            throw std::invalid_argument("publish_rate must be finite and nonnegative");
        if (publish_rate > 0.0) {
            publish_period_ = std::chrono::duration<double>{1.0 / publish_rate};
            if (!std::isfinite(publish_period_.count()))
                throw std::invalid_argument("publish_rate is too small");
        }

        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);
        parameter_callback_   = parameter_subscriber_->add_parameter_callback(
            "forward_list", [this](const rclcpp::Parameter& para) {
                // ROS callbacks only enqueue changes; publishers belong to the update thread.
                std::lock_guard lock(forward_list_mutex_);
                pending_forward_list_ = para.as_string_array();
                forward_list_pending_.store(true, std::memory_order_release);
            });
    }

    void before_pairing(
        const std::map<std::string, const std::type_info&>& output_map) override {
        for (const auto& [name, type] : output_map) {
            if (type == typeid(double)) {
                forward_units_.emplace(
                    name,
                    std::make_unique<ForwardUnit<double, std_msgs::msg::Float64>>(this, name));
            } else if (type == typeid(int)) {
                forward_units_.emplace(
                    name, std::make_unique<ForwardUnit<int, std_msgs::msg::Int32>>(this, name));
            } else if (type == typeid(int64_t)) {
                forward_units_.emplace(
                    name, std::make_unique<ForwardUnit<int64_t, std_msgs::msg::Int64>>(this, name));
            } else if (type == typeid(bool)) {
                forward_units_.emplace(
                    name, std::make_unique<ForwardUnit<bool, std_msgs::msg::Bool>>(this, name));
            } else if (type == typeid(rmcs_msgs::Switch)) {
                forward_units_.emplace(
                    name,
                    std::make_unique<ForwardUnit<rmcs_msgs::Switch, std_msgs::msg::UInt8>>(
                        this, name));
            } else if (type == typeid(std::string)) {
                forward_units_.emplace(
                    name,
                    std::make_unique<ForwardUnit<std::string, std_msgs::msg::String>>(this, name));
            }
        }
        std::vector<std::string> forward_list;
        if (get_parameter("forward_list", forward_list))
            update_forward_list(forward_list);
    }

    void update() override {
        if (forward_list_pending_.load(std::memory_order_acquire)) {
            std::vector<std::string> forward_list;
            bool apply = false;
            {
                // Never wait for a parameter callback in the control loop.
                std::unique_lock lock(forward_list_mutex_, std::try_to_lock);
                if (lock.owns_lock()) {
                    forward_list.swap(pending_forward_list_);
                    forward_list_pending_.store(false, std::memory_order_release);
                    apply = true;
                }
            }
            if (apply)
                update_forward_list(forward_list);
        }

        if (publish_period_.count() > 0.0) {
            const auto now = std::chrono::steady_clock::now();
            if (published_ && now - last_publish_ < publish_period_)
                return;
            last_publish_ = now;
            published_    = true;
        }

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

        bool active = false;
    };

    template <typename StdT, typename RosT>
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
            msg.data = static_cast<typename RosT::_data_type>(*input_);
            publisher_->publish(msg);
        }

        void deactivate() override { publisher_ = nullptr; }

    private:
        std::string name_;
        InputInterface<StdT> input_;
        rclcpp::Publisher<RosT>::SharedPtr publisher_;
    };

    std::map<std::string, std::unique_ptr<BasicForwarderUnit>> forward_units_;

    // Floating-point seconds avoid conversion overflow for very low publication rates.
    std::chrono::duration<double> publish_period_{0.0};
    std::chrono::steady_clock::time_point last_publish_{};
    bool published_ = false;

    std::mutex forward_list_mutex_;
    std::vector<std::string> pending_forward_list_;
    std::atomic<bool> forward_list_pending_{false};

    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_;
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::ValueBroadcaster, rmcs_executor::Component)
