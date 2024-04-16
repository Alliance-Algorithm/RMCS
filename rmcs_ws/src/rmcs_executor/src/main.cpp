#include <regex>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors.hpp>

#include "executor.hpp"
#include "rmcs_executor/component.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<rmcs_executor::Component> component_loader(
        "rmcs_executor", "rmcs_executor::Component");

    auto executor = std::make_shared<rmcs_executor::Executor>("rmcs_executor");
    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    rcl_executor.add_node(executor);

    std::vector<std::string> component_descriptions;
    if (!executor->get_parameter("components", component_descriptions))
        throw std::runtime_error("para");

    std::regex regex(R"(\s*(\S+)\s*->\s*(\S+)\s*)");
    for (const auto& component_description : component_descriptions) {
        std::smatch matches;
        std::string plugin_name, component_name;

        if (std::regex_search(component_description, matches, regex)) {
            if (matches.size() != 3)
                throw std::runtime_error("In regex matching: unexpected number of matches");

            plugin_name    = matches[1].str();
            component_name = matches[2].str();
        } else {
            plugin_name = component_name = component_description;
        }

        rmcs_executor::Component::initializing_component_name = component_name.c_str();
        auto component = component_loader.createSharedInstance(plugin_name);
        executor->add_component(component);
        if (auto node = std::dynamic_pointer_cast<rclcpp::Node>(component))
            rcl_executor.add_node(node);
    }

    executor->start();
    rcl_executor.spin();

    rclcpp::shutdown();
}