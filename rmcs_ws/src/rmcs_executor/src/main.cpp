#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <execinfo.h>
#include <unistd.h>

#include <regex>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors.hpp>

#include "executor.hpp"
#include "rmcs_executor/component.hpp"

void segmentation_fault_handler(int) {
    void* array[100];

    int size = backtrace(array, 100);
    fprintf(stderr, "[Fatal] Segmentation fault\n>>> STACK TRACE BEGIN\n");

    // Print the stack trace to stderr.
    if (size >= 2)
        // Remove the stack trace used to call this function.
        backtrace_symbols_fd(array + 2, size - 2, STDERR_FILENO);
    else
        backtrace_symbols_fd(array, size, STDERR_FILENO);

    fprintf(stderr, "<<< STACK TRACE END\n");

    exit(1);
}

int main(int argc, char** argv) {
    std::signal(SIGSEGV, segmentation_fault_handler);

    rclcpp::init(argc, argv);

    pluginlib::ClassLoader<rmcs_executor::Component> component_loader(
        "rmcs_executor", "rmcs_executor::Component");

    rclcpp::executors::SingleThreadedExecutor rcl_executor;
    auto executor = std::make_shared<rmcs_executor::Executor>("rmcs_executor", rcl_executor);
    rcl_executor.add_node(executor);

    std::vector<std::string> component_descriptions;
    if (!executor->get_parameter("components", component_descriptions))
        throw std::runtime_error("Missing parameter 'components' or config is not found");

    std::regex regex(R"(\s*(\S+)\s*->\s*(\S+)\s*)");
    for (const auto& component_description : component_descriptions) {
        std::smatch matches;
        std::string plugin_name, component_name;

        if (std::regex_search(component_description, matches, regex)) {
            if (matches.size() != 3)
                throw std::runtime_error("In regex matching: unexpected number of matches");

            plugin_name = matches[1].str();
            component_name = matches[2].str();
        } else {
            plugin_name = component_name = component_description;
        }

        rmcs_executor::Component::initializing_component_name = component_name.c_str();
        auto component = component_loader.createSharedInstance(plugin_name);
        executor->add_component(component);
    }

    executor->start();
    rcl_executor.spin();

    rclcpp::shutdown();
}