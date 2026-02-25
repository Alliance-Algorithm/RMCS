#include <array>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace rmcs_core::controller::arm {

class ArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_output("/arm/enable_flag", is_arm_enable_, false);

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(prefix + "/theta", theta_[i]);
            register_output(prefix + "/target_theta", target_theta_[i], NAN);
            register_output(prefix + "/control_angle_error", angle_error_[i], NAN);
        }

        // ── 订阅 /arm/target_joints
        target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm/target_joints",
            rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                std::lock_guard lock(mutex_);
                for (std::size_t i = 0; i < 6; ++i)
                    pending_target_[i] = msg->data[i];
                has_pending_ = true;
            });

        // ── 订阅 /arm/enable_cmd
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arm/enable_cmd",
            rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                pending_enable_.store(msg->data);
            });

    }

    ~ArmController() override {
    }

    void update() override {
        // 写入 enable
        *is_arm_enable_ = pending_enable_.load();

        // 写入目标角度到共享内存
        {
            std::lock_guard lock(mutex_);
            if (has_pending_) {
                for (std::size_t i = 0; i < 6; ++i)
                    *target_theta_[i] = pending_target_[i];
                has_pending_ = false;
            }
        }

        //计算角度误差
        for (std::size_t i = 0; i < 6; ++i) {
            const double theta  = *theta_[i];
            const double target = *target_theta_[i];
            if (!std::isnan(theta) && !std::isnan(target))
                *angle_error_[i] = target - theta;
            else
                *angle_error_[i] = NAN;
        }
    }

private:
    OutputInterface<double> angle_error_[6];

    OutputInterface<bool>   is_arm_enable_;
    InputInterface<double>  theta_[6];
    OutputInterface<double> target_theta_[6];

    // 跨线程缓冲
    std::mutex            mutex_;
    std::array<double, 6> pending_target_{};
    bool                  has_pending_{false};
    std::atomic<bool>     pending_enable_{false};

    // ROS 通信
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              enable_sub_;

    // spin 线程
    // int               pub_counter_{0};
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)