#include "rmcs_utility/rclcpp/node.hpp"

auto main() -> int {
    auto rclcpp = rmcs_util::RclcppNode{"Name"};
    rclcpp.info("Hello World!");

    rclcpp.shutdown();
}
