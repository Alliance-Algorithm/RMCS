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
            register_input(prefix + "/lower_limit", lower_limit_[i]);
            register_input(prefix + "/upper_limit", upper_limit_[i]);
            register_output(prefix + "/target_theta", target_theta_[i], NAN);
            register_output(prefix + "/control_angle_error", angle_error_[i], NAN);
        }
        register_input("urdf_loaded", urdf_loaded);

        // ── 订阅 /arm/target_joints
        target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm/target_joints",
            rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                auto cmd = std::make_shared<TargetCmd>();
                for (std::size_t i = 0; i < 6; ++i)
                    cmd->angles[i] = msg->data[i];
                pending_target_.store(cmd, std::memory_order_release);

                last_target_time_.store(
                    std::chrono::steady_clock::now().time_since_epoch().count(),
                    std::memory_order_release);
            });

        // ── 订阅 /arm/enable_cmd
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arm/enable_cmd",
            rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                pending_enable_.store(msg->data, std::memory_order_release);
            });

    }

    ~ArmController() override = default;

    void update() override {
        if (!*urdf_loaded || std::isnan(*theta_[0])) {
            for (std::size_t i = 0; i < 6; ++i)
                *angle_error_[i] = 0.0;
            return;
        }
        auto normalize = [](double a) -> double {
            while (a >  M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;
            return a;
        };

        auto clamp_target = [this](std::size_t i, double target) -> double {
            const double lo = *lower_limit_[i];
            const double hi = *upper_limit_[i];
            return std::clamp(target, lo, hi);
        };

        if (target_initialized_) {
            const auto now_ns  = std::chrono::steady_clock::now().time_since_epoch().count();
            const auto last_ns = last_target_time_.load(std::memory_order_acquire);
            constexpr int64_t TIMEOUT_NS = 500'000'000LL;  // 500ms
            if (now_ns - last_ns > TIMEOUT_NS) {
                target_initialized_ = false;
                RCLCPP_WARN(
                    get_logger(),
                    "[ArmController] ：超过 500ms 未收到目标，回退到保持当前位置");
            }
        }
        
        *is_arm_enable_ = pending_enable_.load(std::memory_order_acquire);

        // ── 读取目标角度
        const auto cmd = pending_target_.load(std::memory_order_acquire);
        if (cmd) {
            for (std::size_t i = 0; i < 6; ++i)
                *target_theta_[i] = cmd->angles[i];
            target_initialized_ = true;
        }
        // ── 计算角度误差 
        for (std::size_t i = 0; i < 6; ++i) {
            const double theta = *theta_[i];

            if (!target_initialized_) {
                *target_theta_[i] = theta;
                *angle_error_[i] = 0.0;
                continue;
            }
            if (std::isnan(*target_theta_[i])) {
                *angle_error_[i] = NAN;
                continue;
            }
            const double target = clamp_target(i, *target_theta_[i]);
            *target_theta_[i]   = target;
            *angle_error_[i]    = normalize(target - theta);
        }
    }

private:
    struct TargetCmd {
            std::array<double, 6> angles{};
        };
    std::atomic<std::shared_ptr<const TargetCmd>> pending_target_{nullptr};
    std::atomic<bool>    pending_enable_{false};

    std::atomic<int64_t> last_target_time_{0};

    // 只在 update() 线程访问，不需要原子
    bool target_initialized_{false};

    OutputInterface<double> angle_error_[6];
    OutputInterface<bool>   is_arm_enable_;
    InputInterface<double>  theta_[6];
    OutputInterface<double> target_theta_[6];
    InputInterface<double>  lower_limit_[6];
    InputInterface<double>  upper_limit_[6];
    InputInterface<bool> urdf_loaded;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              enable_sub_;

};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)