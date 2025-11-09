#include "kernel/capturer.hpp"
#include "kernel/visualization.hpp"

#include "modules/debug/framerate.hpp"
#include "utility/rclcpp/configuration.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/singleton/running.hpp"

#include <csignal>
#include <yaml-cpp/yaml.h>

auto main() -> int {
    using namespace std::chrono_literals;
    using namespace rmcs;

    std::signal(SIGINT, [](int) { util::set_running(false); });

    ///
    /// Runtime
    ///

    auto rclcpp_node = util::RclcppNode { "AutoAim" };

    auto framerate = FramerateCounter {};
    framerate.set_intetval(5s);

    auto capturer      = kernel::Capturer {};
    auto visualization = kernel::Visualization {};

    ///
    /// Configure
    ///
    auto configuration     = util::configuration();
    auto use_visualization = configuration["use_visualization"].as<bool>();

    auto handle_result = [&](auto runtime_name, const auto& result) {
        auto initialized = true;
        if (!result.has_value()) {
            rclcpp_node.error("Failed to init <{}>", runtime_name);
            rclcpp_node.error("  e: {}", result.error());
            initialized = false;
        }
        return initialized;
    };

    // CAPTURER
    {
        auto config = configuration["capturer"];
        auto result = capturer.initialize(config);
        if (!handle_result("capturer", result)) {
            return -1;
        }
    }
    // VISUALIZATION
    if (use_visualization) {
        auto config = configuration["visualization"];
        auto result = visualization.initialize(config);
        if (!handle_result("visualization", result)) {
            return -1;
        }
    }

    for (;;) {
        if (!util::get_running()) [[unlikely]]
            break;

        if (auto image = capturer.fetch_image()) {
            if (framerate.tick()) {
                rclcpp_node.info("Framerate: {}hz", framerate.fps());
            }
            if (visualization.initialized()) {
                visualization.send_image(*image);
            }

            // Publish task
        }

        rclcpp_node.spin_once();
    }

    return 0;
}
