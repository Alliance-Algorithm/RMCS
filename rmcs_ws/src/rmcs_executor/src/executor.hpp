#pragma once

#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
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
    ~Executor() override {
        disable_event_outputs();
        wait_event_outputs_idle();
        if (thread_.joinable())
            thread_.join();
    };

    void add_component(const std::shared_ptr<Component>& component) {
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
        enable_event_outputs();

        thread_ = std::thread{[update_rate, this]() {
            const auto period = std::chrono::nanoseconds(
                static_cast<long>(std::round(1'000'000'000.0 / update_rate)));
            auto next_iteration_time = std::chrono::steady_clock::now();
            try {
                while (rclcpp::ok()) {
                    predefined_msg_provider_->set_timestamp(next_iteration_time);
                    next_iteration_time += period;
                    for (const auto& component : updating_order_) {
                        component->update();
                    }
                    std::this_thread::sleep_until(next_iteration_time);
                }
            } catch (const std::exception& exception) {
                RCLCPP_FATAL(
                    get_logger(), "Executor update thread terminated by exception: %s",
                    exception.what());
                rclcpp::shutdown();
            } catch (...) {
                RCLCPP_FATAL(get_logger(), "Executor update thread terminated by unknown exception");
                rclcpp::shutdown();
            }
        }};
    }

private:
    void enable_event_outputs() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.enable == nullptr)
                    continue;
                output.enable(output.binding);
            }
        }
    }

    void disable_event_outputs() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.disable == nullptr)
                    continue;
                output.disable(output.binding);
            }
        }
    }

    void wait_event_outputs_idle() {
        for (const auto& component : component_list_) {
            for (const auto& output : component->output_list_) {
                if (output.wait_idle == nullptr)
                    continue;
                output.wait_idle(output.binding);
            }
        }
    }

    struct InterfaceKey {
        std::string name;
        InterfaceKind kind;

        bool operator==(const InterfaceKey& other) const {
            return name == other.name && kind == other.kind;
        }
    };

    struct InterfaceKeyHash {
        std::size_t operator()(const InterfaceKey& key) const {
            return std::hash<std::string>{}(key.name)
                ^ (std::hash<int>{}(static_cast<int>(key.kind)) << 1U);
        }
    };

    struct OutputRecord {
        Component::OutputDeclaration* declaration;
    };

    struct NameKindRecord {
        InterfaceKind kind;
        std::string component_name;
        const char* direction;
    };

    static std::string describe_output(
        const std::string& name, InterfaceKind kind, const std::type_info& type,
        const std::string& component_name) {
        std::ostringstream stream;
        stream << "Component [" << component_name << "] registered " << interface_kind_name(kind)
               << " output \"" << name << "\" with type \"" << type.name() << "\"";
        return stream.str();
    }

    static std::string describe_declaration(
        const std::string& name, InterfaceKind kind, const std::string& component_name,
        const char* direction) {
        std::ostringstream stream;
        stream << "Component [" << component_name << "] registered " << interface_kind_name(kind)
               << ' ' << direction << " \"" << name << "\"";
        return stream.str();
    }

    void init() {
        updating_order_.clear();

        auto output_map = std::unordered_map<InterfaceKey, OutputRecord, InterfaceKeyHash>{};
        auto output_kind_map = std::unordered_map<std::string, InterfaceKind>{};
        auto declaration_kind_map = std::unordered_map<std::string, NameKindRecord>{};
        auto user_output_map = Component::OutputInfoMap{};
        for (const auto& component : component_list_) {
            component->dependency_count_ = 0;
            component->wanted_by_.clear();
            for (auto& output : component->output_list_) {
                auto declaration_iter = declaration_kind_map.find(output.name);
                if (
                    declaration_iter != declaration_kind_map.end()
                    && declaration_iter->second.kind != output.kind) {
                    const auto message = std::string{"Conflicting interface kinds for name \""}
                        + output.name + "\": "
                        + describe_declaration(
                            output.name, declaration_iter->second.kind,
                            declaration_iter->second.component_name,
                            declaration_iter->second.direction)
                        + "; "
                        + describe_declaration(
                            output.name, output.kind, component->get_component_name(), "output")
                        + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }
                declaration_kind_map.emplace(
                    output.name,
                    NameKindRecord{output.kind, component->get_component_name(), "output"});

                auto kind_iter = output_kind_map.find(output.name);
                if (kind_iter != output_kind_map.end() && kind_iter->second != output.kind) {
                    const auto message = std::string{"Conflicting output kinds for name \""}
                        + output.name + "\": " + interface_kind_name(kind_iter->second)
                        + " and " + interface_kind_name(output.kind)
                        + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                output_kind_map.emplace(output.name, output.kind);

                const auto [output_iter, inserted] = output_map.emplace(
                    InterfaceKey{output.name, output.kind}, OutputRecord{&output});
                if (!inserted) {
                    const auto message = std::string{"Duplicate "}
                        + interface_kind_name(output.kind) + " output name \"" + output.name
                        + "\". "
                        + describe_output(
                            output_iter->second.declaration->name,
                            output_iter->second.declaration->kind,
                            output_iter->second.declaration->type,
                            output_iter->second.declaration->component->get_component_name())
                        + "; "
                        + describe_output(
                            output.name, output.kind, output.type, component->get_component_name());
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                user_output_map.emplace(
                    output.name, Component::OutputInfo{std::cref(output.type), output.kind});
            }

            for (const auto& input : component->input_list_) {
                auto declaration_iter = declaration_kind_map.find(input.name);
                if (
                    declaration_iter != declaration_kind_map.end()
                    && declaration_iter->second.kind != input.kind) {
                    const auto message = std::string{"Conflicting interface kinds for name \""}
                        + input.name + "\": "
                        + describe_declaration(
                            input.name, declaration_iter->second.kind,
                            declaration_iter->second.component_name,
                            declaration_iter->second.direction)
                        + "; "
                        + describe_declaration(
                            input.name, input.kind, component->get_component_name(), "input")
                        + ". A name can only belong to one interface kind.";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }
                declaration_kind_map.emplace(
                    input.name,
                    NameKindRecord{input.kind, component->get_component_name(), "input"});
            }
        }

        for (auto& component : component_list_) {
            component->before_pairing(user_output_map);
        }

        for (const auto& component : component_list_) {
            for (const auto& input : component->input_list_) {
                auto output_iter = output_map.find(InterfaceKey{input.name, input.kind});
                if (output_iter == output_map.end()) {
                    auto kind_iter = output_kind_map.find(input.name);
                    if (kind_iter != output_kind_map.end()) {
                        const auto message = std::string{"Component ["}
                            + component->get_component_name() + "] requested "
                            + interface_kind_name(input.kind) + " input \"" + input.name
                            + "\", but the available output \"" + input.name
                            + "\" is registered as "
                            + interface_kind_name(kind_iter->second) + ".";

                        RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                        throw std::runtime_error{message};
                    }

                    if (!input.required)
                        continue;

                    const auto message = std::string{"Cannot find the corresponding "}
                        + interface_kind_name(input.kind) + " output of input \"" + input.name
                        + "\" declared by component [" + component->get_component_name() + "]";
                    RCLCPP_FATAL(get_logger(), "%s", message.c_str());
                    throw std::runtime_error{message};
                }

                const auto& output = *output_iter->second.declaration;
                if (input.type != output.type) {
                    const auto message = std::string{"Type mismatch for "}
                        + interface_kind_name(input.kind) + " interface \"" + input.name
                        + "\": component [" + output.component->get_component_name()
                        + "] declared output type \"" + output.type.name()
                        + "\", but component [" + component->get_component_name()
                        + "] requested input type \"" + input.type.name() + "\".";
                    RCLCPP_FATAL(
                        get_logger(), "With %s interface \"%s\":",
                        interface_kind_name(input.kind), input.name.c_str());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] declared the output with type \"%s\"",
                        output.component->get_component_name().c_str(), output.type.name());
                    RCLCPP_FATAL(
                        get_logger(), "    Component [%s] requested the input with type \"%s\"",
                        component->get_component_name().c_str(), input.type.name());
                    RCLCPP_FATAL(get_logger(), "Type not match.");
                    throw std::runtime_error{message};
                }

                if (output.component->wanted_by_.emplace(component.get()).second)
                    component->dependency_count_++;

                input.bind(input.binding, output.binding);
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
                    const auto output_iter = output_map.find(InterfaceKey{input.name, input.kind});
                    if (output_iter == output_map.end())
                        continue;

                    const auto* output = output_iter->second.declaration;
                    if (output->component->dependency_count_ == 0)
                        continue;
                    RCLCPP_FATAL(
                        get_logger(), "    Depends on [%s] because requesting %s interface \"%s\"",
                        output->component->component_name_.c_str(),
                        interface_kind_name(input.kind), output->name.c_str());
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
