#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp" 

using namespace std::chrono_literals;

class AngleWaiter : public rclcpp::Node
{
public:
  AngleWaiter()
  : Node("angel_waiter")
  {
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("Waiting_angle", 10);

    input_thread_ = std::thread(&AngleWaiter::handle_user_input, this);
        
    RCLCPP_INFO(this->get_logger(), "角度触发发布器已启动");
  }

  ~AngleWaiter(){
      // 确保输入线程正确退出
      if (input_thread_.joinable()) {
        input_thread_.join();
      }
  }

private:
  void handle_user_input()
  {
    double input_angle;
    while (rclcpp::ok()) {
      std::cout << "请输入目标角度 (度): ";
      if (std::cin >> input_angle) {
      // 输入有效，更新目标角度并重置触发状态
        target_angle_ = input_angle;
        triggered_ = false;

        auto message = std_msgs::msg::Float64(); 
        message.data = target_angle_;
        angle_publisher_->publish(message); 

        RCLCPP_INFO(this->get_logger(), "已更新目标角度为: %.2f 度", target_angle_);
      } else {
      // 处理输入错误
        std::cin.clear();
        std::string invalid_input;
        std::cin >> invalid_input;
        RCLCPP_WARN(this->get_logger(), "无效输入: %s,请输入数字", invalid_input.c_str());
      }
    }
  }  

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;

  double target_angle_;        // 目标角度
  bool triggered_;             // 是否已触发发布

  std::thread input_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleWaiter>());
  rclcpp::shutdown();
  return 0;
}
