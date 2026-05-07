#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  int i = 0;
  while (!exiter.exit()) {
    auto x = ros2.subscribe_enemy_status();
    // tools::logger()->info("invincible enemy ids size is{}", x.size());
    for (const auto & id : x) {
      tools::logger()->info("id:{}", id);
    }
    // i++;

    std::this_thread::sleep_for(std::chrono::microseconds(500));
    // if (i > 1000) break;
  }
  return 0;
}
