#include <chrono>
#include <cmath>

#include <atomic>
#include <random>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/empty.hpp>
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
        // Fix 5: 新增目标角速度前馈输出，供 Solver 分发给 Controller
        register_output("/gimbal/yaw/control_angle_velocity", control_angle_velocity_, 0.0);
        register_output("/gimbal/pitch/control_angle_error", pitch_control_angle_error_, 0.0);
        register_output("/gimbal/pitch/control_angle", pitch_control_angle_, 0.0);

        // Subscribers: 接收用于触发随机阶跃目标的 ROS 2 消息
        trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/sim_cmd/trigger_random_target", 10,
            [this](const std_msgs::msg::Empty::SharedPtr /*msg*/) {
                std::uniform_real_distribution<double> dist(-M_PI, M_PI);
                double new_target = dist(rng_);
                target_yaw_.store(new_target, std::memory_order_relaxed);
                RCLCPP_INFO(
                    this->get_logger(), "Triggered! New random target_yaw: %.2f rad", new_target);
            });

        // Publishers: 发送给 Simulator 用于 RViz 渲染目标 Marker
        vis_yaw_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/vis_target_yaw", 10);
        vis_pitch_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/vis_target_pitch", 10);
    }

    void update() override {
        // // --- 正弦波连续追踪测试信号 ---
        // static auto start_time = std::chrono::steady_clock::now();
        // auto now = std::chrono::steady_clock::now();
        // double t = std::chrono::duration<double>(now - start_time).count();

        // // 测试信号：正弦波 target_yaw = A * sin(2π * f * t)
        // constexpr double A = 0.2; // 幅值 [rad]
        // constexpr double f = 2.0; // 频率 [Hz]
        // double target_yaw = A * std::sin(2.0 * M_PI * f * t);
        // // Fix 5: 目标角速度 = 对 target_yaw 求时间导数
        // double target_yaw_velocity = A * 2.0 * M_PI * f * std::cos(2.0 * M_PI * f * t);

        // --- 随机阶跃响应测试信号（已屏蔽）---
        // 目标位置由外部 ROS2 消息离散触发，产生位置突变，瞬时导数恒为 0
        double target_yaw = target_yaw_.load(std::memory_order_relaxed);
        double target_yaw_velocity = 0.0;

        double target_pitch = 0.0;

        double current_yaw = *top_yaw_angle_ + *bottom_yaw_angle_;

        // Fix 5: 角度误差加 remainder 归一化，防止越过 ±π 边界时产生跳变
        *control_angle_error_ = std::remainder(target_yaw - current_yaw, 2.0 * M_PI);
        *control_angle_velocity_ = target_yaw_velocity;
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
    // 随机信号触发器与状态
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    std::atomic<double> target_yaw_{0.0};
    std::mt19937 rng_{std::random_device{}()};

    InputInterface<double> top_yaw_angle_, bottom_yaw_angle_, pitch_angle_;
    OutputInterface<double> control_angle_error_, control_angle_shift_;
    // Fix 5: 目标角速度前馈
    OutputInterface<double> control_angle_velocity_;
    OutputInterface<double> pitch_control_angle_error_, pitch_control_angle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vis_yaw_pub_, vis_pitch_pub_;
};

} // namespace rmcs_core::controller::gimbal::test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::test::DualYawTargetGenerator, rmcs_executor::Component)