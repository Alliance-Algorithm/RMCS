#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rmcs_core::controller::gimbal::test {

class DualYawSimulator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSimulator()
        : rclcpp::Node(
              "dual_yaw_simulator",
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // --- Inputs: 为了打破 rmcs_executor 的循环依赖，我们不再使用 register_input ---
        // 而是通过 ROS2 订阅器接收控制力矩 (由 Relay 组件发出)
        // BUG-1 Fix: 回调写入使用 atomic store，彻底消除与 physics_loop 的数据竞争
        top_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/top_yaw_torque", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                cmd_top_torque_.store(msg->data, std::memory_order_relaxed);
            });
        bottom_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/bottom_yaw_torque", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                cmd_bot_torque_.store(msg->data, std::memory_order_relaxed);
            });
        pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/pitch_torque", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                cmd_pitch_torque_.store(msg->data, std::memory_order_relaxed);
            });

        // --- Outputs: 模拟传感器和云台物理状态给到 Controller & Solver ---
        register_output("/gimbal/top_yaw/angle", top_yaw_angle_, 0.0);
        register_output("/gimbal/top_yaw/velocity", top_yaw_velocity_, 0.0);
        register_output("/gimbal/bottom_yaw/angle", bottom_yaw_angle_, 0.0);
        register_output("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_, 0.0);
        register_output("/gimbal/pitch/angle", pitch_angle_, 0.0);
        register_output("/gimbal/pitch/velocity", pitch_velocity_, 0.0);

        register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_imu_, 0.0);
        register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0.0);

        // --- ROS2 Subscribers: 接收用于在 RViz 中渲染的纯视觉目标角度 ---
        // (由单独的 TargetGenerator 组件发送以防止循环依赖)
        // BUG-2 Fix: 回调写入使用 atomic store，消除与 update() 线程的数据竞争
        vis_target_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/vis_target_yaw", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                vis_target_yaw_.store(msg->data, std::memory_order_relaxed);
            });
        vis_target_pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/vis_target_pitch", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                vis_target_pitch_.store(msg->data, std::memory_order_relaxed);
            });

        // 可视化 Marker 发布器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/dual_yaw_simulator/markers", 10);

        // 启动高频物理模拟线程
        running_ = true;
        physics_thread_ = std::thread(&DualYawSimulator::physics_loop, this);
    }

    ~DualYawSimulator() override {
        running_ = false;
        if (physics_thread_.joinable()) {
            physics_thread_.join();
        }
    }

    // 此函数由 Executor 调用，通常频率为 1kHz（模拟数据回报）
    void update() override {
        // 问题-7 Fix: 在锁内只拷贝状态快照，锁外再做 ROS2 I/O（publish_markers），
        //             避免持锁期间触发网络操作而阻塞 physics_loop 线程造成时序抖动
        double snap_top_angle, snap_top_vel;
        double snap_bot_angle, snap_bot_vel;
        double snap_pitch_angle, snap_pitch_vel;
        double snap_chassis_vel;
        double snap_target_yaw, snap_target_pitch;
        bool do_publish = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            snap_top_angle = sim_top_angle_;
            snap_top_vel = sim_top_vel_;
            snap_bot_angle = sim_bot_angle_;
            snap_bot_vel = sim_bot_vel_;
            snap_pitch_angle = sim_pitch_angle_;
            snap_pitch_vel = sim_pitch_vel_;
            snap_chassis_vel = sim_chassis_vel_;

            // BUG-2 Fix: vis_target_* 已为 atomic，load 无需持锁
            snap_target_yaw = vis_target_yaw_.load(std::memory_order_relaxed);
            snap_target_pitch = vis_target_pitch_.load(std::memory_order_relaxed);

            publish_count_++;
            if (publish_count_ >= 33) {
                do_publish = true;
                publish_count_ = 0;
            }
        } // ← 锁在此释放，后续所有操作均在锁外执行

        // 将快照状态刷新至输出接口
        *top_yaw_angle_ = snap_top_angle;
        *top_yaw_velocity_ = snap_top_vel;
        *bottom_yaw_angle_ = snap_bot_angle;
        *bottom_yaw_velocity_ = snap_bot_vel;
        *pitch_angle_ = snap_pitch_angle;
        *pitch_velocity_ = snap_pitch_vel;

        *chassis_yaw_velocity_imu_ = snap_chassis_vel;
        // 云台绝对角速度 = 底盘角速度 + 下Yaw相对角速度 + 上Yaw相对角速度
        *gimbal_yaw_velocity_imu_ = snap_chassis_vel + snap_bot_vel + snap_top_vel;

        // 问题-7 Fix: 在锁外发布 Marker，不再阻塞 physics_loop
        if (do_publish)
            publish_markers(
                snap_bot_angle, snap_top_angle, snap_pitch_angle, snap_target_yaw,
                snap_target_pitch);
    }

private:
    // 高频物理模拟主循环
    // BUG-3 Fix: 循环周期 20µs（50kHz），每周期积分 1 步，dt 与睡眠时长一致
    void physics_loop() {
        // dt = 20µs，对应 50kHz 积分频率
        constexpr double dt = 0.00002;
        constexpr int LOOP_US = 20; // 睡眠时长 [µs]，与 dt 严格对齐

        // 预设物理参数（可根据真实模型微调）
        constexpr double J_top = 0.005;   // 上云台自身转动惯量 [kg·m²]
        constexpr double J_bot = 0.08;    // 下云台自身转动惯量 [kg·m²]
        constexpr double J_pitch = 0.005; // Pitch 转动惯量 [kg·m²]
        constexpr double B_top = 0.005;   // 上云台相对阻尼系数
        constexpr double B_bot = 0.02;    // 下云台相对阻尼系数
        constexpr double B_pitch = 0.002; // Pitch 阻尼系数
        // 问题-5 Fix: 下云台旋转时需带动上云台，有效惯量 = J_bot + J_top（串联结构）
        constexpr double J_bot_eff = J_bot + J_top;

        constexpr double MAX_TORQUE = 5.0;           // 电机力矩硬限幅 [N·m]
        constexpr double TOP_YAW_LIMIT = M_PI / 3.0; // 上Yaw相对限位 ±60°

        while (running_) {
            auto next_time = std::chrono::steady_clock::now() + std::chrono::microseconds(LOOP_US);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                // BUG-1 Fix: 用 atomic load 读取控制命令，不依赖 state_mutex_ 保护
                const double actual_top_torque = std::clamp(
                    cmd_top_torque_.load(std::memory_order_relaxed), -MAX_TORQUE, MAX_TORQUE);
                const double actual_bot_torque = std::clamp(
                    cmd_bot_torque_.load(std::memory_order_relaxed), -MAX_TORQUE, MAX_TORQUE);
                const double actual_pitch_torque = std::clamp(
                    cmd_pitch_torque_.load(std::memory_order_relaxed), -MAX_TORQUE, MAX_TORQUE);

                // --- 串联刚体动力学（绝对空间求解）---
                // sim_bot_vel_：下云台相对底盘的角速度（底盘静止时即绝对角速度）
                // sim_top_vel_：上云台相对下云台的角速度
                double abs_bot_vel = sim_bot_vel_;
                double abs_top_vel = sim_bot_vel_ + sim_top_vel_;

                // 黏性摩擦（线性阻尼）
                double friction_bot = B_bot * abs_bot_vel;  // 下云台相对底盘
                double friction_top = B_top * sim_top_vel_; // 上云台相对下云台

                // 问题-5 Fix: 下云台有效惯量使用 J_bot_eff = J_bot + J_top
                // 下云台绝对角加速度：驱动力矩 - 上云台反作用 - 自身摩擦 + 上云台摩擦反作用
                double abs_alpha_bot =
                    (actual_bot_torque - actual_top_torque - friction_bot + friction_top)
                    / J_bot_eff;
                // 上云台绝对角加速度：驱动力矩 - 相对摩擦
                double abs_alpha_top = (actual_top_torque - friction_top) / J_top;

                // Pitch 独立轴（简化：不耦合水平惯量）
                double alpha_pitch = (actual_pitch_torque - B_pitch * sim_pitch_vel_) / J_pitch;

                // 欧拉积分：更新绝对速度
                abs_bot_vel += abs_alpha_bot * dt;
                abs_top_vel += abs_alpha_top * dt;

                // 转回相对速度，再积分位置
                sim_bot_vel_ = abs_bot_vel;
                sim_top_vel_ = abs_top_vel - abs_bot_vel;

                sim_bot_angle_ += sim_bot_vel_ * dt;
                sim_top_angle_ += sim_top_vel_ * dt;

                // 上Yaw物理硬限位 ±60°，撞墙后速度归零
                if (sim_top_angle_ > TOP_YAW_LIMIT) {
                    sim_top_angle_ = TOP_YAW_LIMIT;
                    if (sim_top_vel_ > 0.0)
                        sim_top_vel_ = 0.0;
                } else if (sim_top_angle_ < -TOP_YAW_LIMIT) {
                    sim_top_angle_ = -TOP_YAW_LIMIT;
                    if (sim_top_vel_ < 0.0)
                        sim_top_vel_ = 0.0;
                }

                sim_pitch_vel_ += alpha_pitch * dt;
                sim_pitch_angle_ += sim_pitch_vel_ * dt;
            }

            // 精确睡眠至下一个 20µs 时刻，保证 50kHz 循环频率
            std::this_thread::sleep_until(next_time);
        }
    }

    // ROS2 原生订阅器 (接收由 Relay 组件发送的力矩，打破循环依赖)
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr top_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bottom_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;

    // 执行器输出接口
    OutputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    OutputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;
    OutputInterface<double> pitch_angle_, pitch_velocity_;
    OutputInterface<double> gimbal_yaw_velocity_imu_, chassis_yaw_velocity_imu_;
    // 用于可视化的异步变量
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vis_target_yaw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vis_target_pitch_sub_;
    // BUG-2 Fix: 改为 atomic，消除回调线程与 update() 线程的数据竞争
    std::atomic<double> vis_target_yaw_{0.0};
    std::atomic<double> vis_target_pitch_{0.0};

    // 物理仿真相关
    std::thread physics_thread_;
    std::atomic<bool> running_{false};
    std::mutex state_mutex_;

    // 内部物理状态（50kHz 更新，1kHz 采样）
    double sim_top_angle_ = 0.0;
    double sim_top_vel_ = 0.0;
    double sim_bot_angle_ = 0.0;
    double sim_bot_vel_ = 0.0;
    double sim_pitch_angle_ = 0.0;
    double sim_pitch_vel_ = 0.0;
    double sim_chassis_vel_ = 0.0;

    // 当前生效的控制命令
    // BUG-1 Fix: 改为 atomic，消除 ROS2 回调线程与 physics_loop 线程的数据竞争
    std::atomic<double> cmd_top_torque_{0.0};
    std::atomic<double> cmd_bot_torque_{0.0};
    std::atomic<double> cmd_pitch_torque_{0.0};

    // 可视化相关
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int publish_count_ = 0;

    // 问题-7 Fix: 接收状态快照参数，在锁外调用，避免持锁期间做 ROS2 I/O
    void publish_markers(
        double bot_angle, double top_angle, double pitch_angle, double target_yaw,
        double target_pitch) {
        visualization_msgs::msg::MarkerArray msg;
        auto now_time = this->now();

        // 通用 marker 创建助手
        auto create_marker = [&](int id, int type, double z_offset, double pitch, double yaw,
                                 double scale_x, double scale_y, double scale_z, float r, float g,
                                 float b, const std::string& ns) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now_time;
            marker.ns = ns;
            marker.id = id;
            marker.type = type;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = 0.0;
            marker.pose.position.y = 0.0;
            marker.pose.position.z = z_offset;

            tf2::Quaternion q;
            q.setRPY(0, pitch, yaw);
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            marker.scale.x = scale_x;
            marker.scale.y = scale_y;
            marker.scale.z = scale_z;

            marker.color.a = 0.8;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            return marker;
        };

        // 使用快照计算绝对角度
        double chassis_yaw = 0.0;
        double bot_yaw_abs = chassis_yaw + bot_angle;
        double top_yaw_abs = bot_yaw_abs + top_angle;

        // 1. 底盘: 正方形 CUBE
        msg.markers.push_back(create_marker(
            0, visualization_msgs::msg::Marker::CUBE, 0.0, 0.0, chassis_yaw, 0.4, 0.4, 0.1, 0.5,
            0.5, 0.5, "chassis"));

        // 2. 下云台 (大 Yaw): 圆柱底座
        msg.markers.push_back(create_marker(
            1, visualization_msgs::msg::Marker::CYLINDER, 0.1, 0.0, bot_yaw_abs, 0.3, 0.3, 0.1, 0.0,
            0.5, 1.0, "bottom_yaw_base"));

        // 2.1 下云台朝向指示: 蓝色箭头 (水平指向大 Yaw 的零点正前方向)
        msg.markers.push_back(create_marker(
            2, visualization_msgs::msg::Marker::ARROW, 0.15, 0.0, bot_yaw_abs, 0.4, 0.04, 0.04, 0.0,
            0.5, 1.0, "bottom_yaw_arrow"));

        // 3. 上云台: 圆柱形 CYLINDER
        msg.markers.push_back(create_marker(
            3, visualization_msgs::msg::Marker::CYLINDER, 0.2, 0.0, top_yaw_abs, 0.2, 0.2, 0.1, 0.0,
            1.0, 0.0, "top_yaw_base"));

        // 4. Pitch 轴 / 上云台朝向: 箭头 ARROW (搭载在上云台上，与 Top Yaw 和 Pitch 同向)
        // 在双级零点时 (bot_yaw_abs == 0, sim_top_angle_ == 0, sim_pitch_angle_ == 0)
        // 它会与上方的蓝色 bottom_yaw_arrow 完全重合指向同一处水平方向
        msg.markers.push_back(create_marker(
            4, visualization_msgs::msg::Marker::ARROW, 0.25, pitch_angle, top_yaw_abs, 0.5, 0.05,
            0.05, 0.0, 1.0, 1.0, "pitch_arrow"));

        // 5. 目标角度指示: 红色半透明箭头 ARROW
        auto target_marker = create_marker(
            5, visualization_msgs::msg::Marker::ARROW, 0.25, target_pitch, target_yaw, 0.6, 0.02,
            0.02, 1.0, 0.0, 0.0, "target_arrow");
        target_marker.color.a = 0.5;
        msg.markers.push_back(target_marker);

        marker_pub_->publish(msg);
    }
};

// --- 新增一个 Relay 组件用于打破执行器的循环依赖 ---
class DualYawSimulatorRelay
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawSimulatorRelay()
        : rclcpp::Node(
              "dual_yaw_simulator_relay",
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // 使用 register_input 接收控制器的力矩输出，从而被置于依赖图的最末端
        register_input("/gimbal/top_yaw/control_torque", top_yaw_torque_, false);
        register_input("/gimbal/bottom_yaw/control_torque", bottom_yaw_torque_, false);
        register_input("/gimbal/pitch/control_torque", pitch_torque_, false);

        // 创建 ROS2 发布器，发布给 Simulator 订阅
        top_yaw_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/top_yaw_torque", 10);
        bottom_yaw_pub_ =
            this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/bottom_yaw_torque", 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Float64>("/sim_cmd/pitch_torque", 10);
    }

    void update() override {
        // 问题-6 Fix: NaN 或未就绪时发零力矩，防止上一帧力矩残留驱动仿真失控
        auto publish_or_zero = [](const InputInterface<double>& input,
                                  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub) {
            std_msgs::msg::Float64 msg;
            msg.data = (input.ready() && !std::isnan(*input)) ? *input : 0.0;
            pub->publish(msg);
        };

        publish_or_zero(top_yaw_torque_, top_yaw_pub_);
        publish_or_zero(bottom_yaw_torque_, bottom_yaw_pub_);
        publish_or_zero(pitch_torque_, pitch_pub_);
    }

private:
    InputInterface<double> top_yaw_torque_, bottom_yaw_torque_, pitch_torque_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr top_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bottom_yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_pub_;
};

} // namespace rmcs_core::controller::gimbal::test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::test::DualYawSimulator, rmcs_executor::Component)
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::test::DualYawSimulatorRelay, rmcs_executor::Component)
