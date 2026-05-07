#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;
  rclcpp::Clock clock;
  auto string_publisher =
    ros2.create_publisher<sp_msgs::msg::EnemyStatusMsg>("temp_node", "enemy_status", 10);

  int i = 0;
  while (!exiter.exit()) {
    sp_msgs::msg::EnemyStatusMsg msg;
    msg.invincible_enemy_ids = {1, 2, 3};
    msg.timestamp = clock.now();
    string_publisher->publish(msg);
    RCLCPP_INFO(
      rclcpp::get_logger("msg send timestamp is"), "msg.timestamp: %d.%09u", msg.timestamp.sec,
      msg.timestamp.nanosec);

    i++;
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    if (i % 3 == 0) {
      auto x = ros2.subscribe_enemy_status();
      // tools::logger()->info("invincible enemy ids size is{}", x.size());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }

  return 0;
}
