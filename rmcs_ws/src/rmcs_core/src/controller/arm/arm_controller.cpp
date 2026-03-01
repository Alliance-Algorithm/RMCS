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

        target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm/target_joints",
            rclcpp::QoS(10).best_effort(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                auto cmd = std::make_shared<TargetCmd>();
                for (std::size_t i = 0; i < 6; ++i)
                    cmd->angles[i] = msg->data[i];
                cmd->recv_time = std::chrono::steady_clock::now();
                pending_target_.store(cmd, std::memory_order_release);
            });

        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/arm/enable_cmd",
            rclcpp::QoS(10).best_effort(),
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

        *is_arm_enable_ = pending_enable_.load(std::memory_order_acquire);
        const auto cmd = pending_target_.load(std::memory_order_acquire);
        if (target_initialized_) {
            constexpr int64_t TIMEOUT_NS = 500'000'000LL;  // 500ms
            const bool timed_out = !cmd ||
                (std::chrono::steady_clock::now() - cmd->recv_time).count() > TIMEOUT_NS;
            if (timed_out) {
                target_initialized_ = false;
                RCLCPP_WARN(
                    get_logger(),
                    "[ArmController] 看门狗触发：超过 500ms 未收到目标，回退到保持当前位置");
            } else {
                for (std::size_t i = 0; i < 6; ++i)
                    *target_theta_[i] = cmd->angles[i];
            }
        } else {
            if (cmd) {
                for (std::size_t i = 0; i < 6; ++i)
                    *target_theta_[i] = cmd->angles[i];
                target_initialized_ = true;
            }
        }

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
        std::chrono::steady_clock::time_point recv_time{};
    };
    std::atomic<std::shared_ptr<const TargetCmd>> pending_target_{nullptr};
    std::atomic<bool>    pending_enable_{false};

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