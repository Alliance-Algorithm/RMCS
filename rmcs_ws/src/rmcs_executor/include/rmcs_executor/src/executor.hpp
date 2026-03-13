#pragma once

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include "predefined_msg_provider.hpp"
#include "rmcs_executor/component.hpp"

namespace rmcs_executor {

class Executor final : public rclcpp::Node {
public:
    explicit Executor(
        const std::string& node_name, rclcpp::executors::SingleThreadedExecutor& rcl_executor)
        : Node{node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)}
        , rcl_executor_(rcl_executor) {
        Component::initializing_component_name = "predefined_msg_provider";
        predefined_msg_provider_               = std::make_shared<PredefinedMsgProvider>();
        add_component(predefined_msg_provider_);
    }
    ~Executor() {
        if (thread_.joinable())
            thread_.join();
    };

    void add_component(std::shared_ptr<Component> component) {
        component_list_.emplace_back(component);
        if (auto node = std::dynamic_pointer_cast<rclcpp::Node>(component))
            rcl_executor_.add_node(node);
        for (auto& partner_component : component->partner_component_list_)
            add_component(partner_component);
    }

    void start() {
        init();

        for (auto& component : component_list_)
            component->before_updating();

        double update_rate;
        if (!get_parameter("update_rate", update_rate))
            throw std::runtime_error{"Unable to get parameter update_rate<double>"};
        predefined_msg_provider_->set_update_rate(update_rate);

        thread_ = std::thread{[update_rate, this]() {
            const auto period = std::chrono::nanoseconds(
                static_cast<long>(std::round(1'000'000'000.0 / update_rate)));
            auto next_iteration_time = std::chrono::steady_clock::now();
            while (rclcpp::ok()) {
                predefined_msg_provider_->set_timestamp(next_iteration_time);
                next_iteration_time += period;
                for (const auto& component : updating_order_) {
                    component->update();
                }
                std::this_thread::sleep_until(next_iteration_time);
            }
        }};
    }

private:
    void init() {
        updating_order_.clear();

        auto output_map      = std::unordered_map<std::string, Component::OutputDeclaration*>{};
        auto user_output_map = std::map<std::string, const std::type_info&>{};
        for (const auto& component : component_list_) {
            component->dependency_count_ = 0;
            component->wanted_by_.clear();
            for (auto& output : component->output_list_) {
                if (!output_map.emplace(output.name, &output).second)
                    throw std::runtime_error{"Duplicate names of output"};
                user_output_map.emplace(output.name, output.type);
            }
        }

        for (auto& component : component_list_) {
            component->before_pairing(user_output_map);
        }

        for (const auto& component : component_list_) {
            for (const auto& input : component->input_list_) {
                auto output_iter = output_map.find(input.name);
                if (output_iter == output_map.end()) {
                    if (!input.required)
                        continue;

                    RCLCPP_FATAL(
                        get_logger(),
                        "Cannot find the corresponding output of input \"%s\" declared by "
                        "component [%s]",
                        input.name.c_str(), component->get_component_name().c_str());
                    throw std::runtime_error{"Cannot find the corresponding output"};
                }

                const auto& output = *output_iter->second;
                if (input.type != output.type) {
                    RCLCPP_FATAL(get_logger(), "With message \"%s\":", input.name.c_str());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] declared the output with type \"%s\"",
                        output.component->get_component_name().c_str(), output.type.name());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] requested the input with type \"%s\"",
                        component->get_component_name().c_str(), input.type.name());
                    RCLCPP_FATAL(get_logger(), "Type not match.");
                    throw std::runtime_error{"Type not match"};
                }

                if (output.component->wanted_by_.emplace(component.get()).second)
                    component->dependency_count_++;

                *input.pointer_to_data_pointer = output.data_pointer;
            }
        }

        RCLCPP_INFO(get_logger(), "Calculating component dependencies");
        std::vector<Component*> independent_list;
        for (const auto& component : component_list_) {
            if (component->dependency_count_ == 0)
                independent_list.emplace_back(component.get());
        }
        for (const auto& component : independent_list) {
            append_updating_order(component);
        }

        if (updating_order_.size() < component_list_.size()) {
            RCLCPP_FATAL(get_logger(), "Circular dependency found:");
            for (const auto& component : component_list_) {
                if (component->dependency_count_ == 0)
                    continue;

                RCLCPP_FATAL(
                    get_logger(), "Component [%s]:", component->get_component_name().c_str());
                for (const auto& input : component->input_list_) {
                    const auto& output = output_map[input.name];
                    if (output->component->dependency_count_ == 0)
                        continue;
                    RCLCPP_FATAL(
                        get_logger(), "    Depends on [%s] because requesting \"%s\"",
                        output->component->component_name_.c_str(), output->name.c_str());
                }
            }
            throw std::runtime_error{"Circular dependency found"};
        }
    }

    void append_updating_order(Component* updatable_component) {
        std::string space = "- ";
        for (size_t i = dependency_recursive_level_; i-- > 0;)
            space.append("    ");
        RCLCPP_INFO(
            get_logger(), "%s%s", space.c_str(), updatable_component->get_component_name().c_str());
        updating_order_.emplace_back(updatable_component);

        for (auto& component : component_list_) {
            if (updatable_component->wanted_by_.contains(component.get())) {
                if (--component->dependency_count_ == 0) {
                    dependency_recursive_level_++;
                    append_updating_order(component.get());
                    dependency_recursive_level_--;
                }
            }
        }
    };

    rclcpp::executors::SingleThreadedExecutor& rcl_executor_;

    std::thread thread_;

    std::shared_ptr<PredefinedMsgProvider> predefined_msg_provider_;
    std::vector<std::shared_ptr<Component>> component_list_;

    std::vector<Component*> updating_order_;
    size_t dependency_recursive_level_ = 0;
};

}; // namespace rmcs_executor