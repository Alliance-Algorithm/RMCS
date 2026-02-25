#include <chrono>
#include <cmath>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::gimbal::test {

// --- 独立的目标追踪信号生成器 (方便配置各种测试信号) ---
class DualYawTargetGenerator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawTargetGenerator()
        : rclcpp::Node(
              "dual_yaw_target_generator",
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // Inputs: 接收物理当前状态以计算误差
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/pitch/angle", pitch_angle_);

        // Outputs: 发送给 Solver 和 PID 的误差控制量
        register_output("/gimbal/yaw/control_angle_error", control_angle_error_, 0.0);
        register_output("/gimbal/yaw/control_angle_shift", control_angle_shift_, 0.0);
        register_output("/gimbal/pitch/control_angle_error", pitch_control_angle_error_, 0.0);
        register_output("/gimbal/pitch/control_angle", pitch_control_angle_, 0.0);

        // Publishers: 发送给 Simulator 用于 RViz 渲染目标 Marker
        vis_yaw_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/vis_target_yaw", 10);
        vis_pitch_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/vis_target_pitch", 10);
    }

    void update() override {
        static auto start_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start_time).count();

        // 测试信号
        double target_yaw = 1 * std::sin(0.5 * M_PI * 0.5 * t);
        double target_pitch = 0;
        // double target_yaw = 0.5 *std::sin(2.0 * M_PI * 0.5 * t); double target_pitch = 0.2 *
        // std::sin(2.0 * M_PI * 0.3 * t);

        double current_yaw = *top_yaw_angle_ + *bottom_yaw_angle_;
        *control_angle_error_ = target_yaw - current_yaw;
        *control_angle_shift_ = 0.0;

        *pitch_control_angle_error_ = target_pitch - *pitch_angle_;
        *pitch_control_angle_ = 0.0;

        // 发布纯渲染用信号
        std_msgs::msg::Float64 msg_yaw, msg_pitch;
        msg_yaw.data = target_yaw;
        msg_pitch.data = target_pitch;
        vis_yaw_pub_->publish(msg_yaw);
        vis_pitch_pub_->publish(msg_pitch);
    }

private:
    InputInterface<double> top_yaw_angle_, bottom_yaw_angle_, pitch_angle_;
    OutputInterface<double> control_angle_error_, control_angle_shift_;
    OutputInterface<double> pitch_control_angle_error_, pitch_control_angle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vis_yaw_pub_, vis_pitch_pub_;
};

} // namespace rmcs_core::controller::gimbal::test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::test::DualYawTargetGenerator, rmcs_executor::Component)