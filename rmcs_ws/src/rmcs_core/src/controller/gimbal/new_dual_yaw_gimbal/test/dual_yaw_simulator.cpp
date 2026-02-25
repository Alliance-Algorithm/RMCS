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
        top_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/top_yaw_torque", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { cmd_top_torque_ = msg->data; });
        bottom_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/bottom_yaw_torque", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { cmd_bot_torque_ = msg->data; });
        pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/pitch_torque", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { cmd_pitch_torque_ = msg->data; });

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
        vis_target_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/vis_target_yaw", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { vis_target_yaw_ = msg->data; });
        vis_target_pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sim_cmd/vis_target_pitch", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { vis_target_pitch_ = msg->data; });

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

    // 此函数由 Executor 调用，通常频率为 1kHz (模拟数据回报)
    void update() override {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 1. 输入力矩现已由 ROS2 原生回调函数异步更新到 cmd_*_torque_ 中

        // 2. 将模拟线程计算的物理状态刷新至输出接口
        *top_yaw_angle_ = sim_top_angle_;
        *top_yaw_velocity_ = sim_top_vel_;
        *bottom_yaw_angle_ = sim_bot_angle_;
        *bottom_yaw_velocity_ = sim_bot_vel_;
        *pitch_angle_ = sim_pitch_angle_;
        *pitch_velocity_ = sim_pitch_vel_;

        *chassis_yaw_velocity_imu_ = sim_chassis_vel_;
        // 简化的绝对角速度关系：云台绝对角速度 = 底盘角速度 + 下yaw角速度 + 上yaw角速度
        *gimbal_yaw_velocity_imu_ = sim_chassis_vel_ + sim_bot_vel_ + sim_top_vel_;

        // 3. 目标误差的生成已转移到 DualYawTargetGenerator 组件中。
        // 此处直接取回异步获取的可视化目标角度用于 RViz 渲染。
        double target_absolute_angle = vis_target_yaw_;
        double target_pitch_angle = vis_target_pitch_;

        // 4. 定期发布可视化 Marker (通过降频，比如每 33 次 update 发布一次 ~30Hz)
        publish_count_++;
        if (publish_count_ >= 33) {
            publish_markers(target_absolute_angle, target_pitch_angle);
            publish_count_ = 0;
        }
    }

private:
    // 高频物理模拟主循环 (模拟 50kHz 物理效果)
    void physics_loop() {
        const double dt = 0.00002;    // 100 microseconds
                                      // 预设的物理参数 (可根据真实模型进行微调)
        const double J_top = 0.01;    // 上云台转动惯量
        const double J_bot = 0.05;    // 下云台转动惯量
        const double J_pitch = 0.005; // Pitch转动惯量
        const double B_top = 0.005;   // 上云台摩擦阻尼系数
        const double B_bot = 0.01;    // 下云台摩擦阻尼系数
        const double B_pitch = 0.002; // Pitch阻尼系数

        while (running_) {
            auto next_time = std::chrono::steady_clock::now() + std::chrono::microseconds(100);

            {
                std::lock_guard<std::mutex> lock(state_mutex_);

                // --- 电机扭矩物理限幅 (动力学约束) ---
                const double MAX_TORQUE = 5.0; // 5 Nm
                double actual_top_torque = std::clamp(cmd_top_torque_, -MAX_TORQUE, MAX_TORQUE);
                double actual_bot_torque = std::clamp(cmd_bot_torque_, -MAX_TORQUE, MAX_TORQUE);
                double actual_pitch_torque = std::clamp(cmd_pitch_torque_, -MAX_TORQUE, MAX_TORQUE);

                // --- 串联刚体动力学 (在绝对空间内求解) ---
                // 设 theta_1 为下云台绝对角，theta_2 为上云台绝对角
                // 当前相对状态:
                // sim_top_vel_ = d(theta_2 - theta_1)/dt
                // sim_bot_vel_ = d(theta_1)/dt

                double abs_bot_vel = sim_bot_vel_;
                double abs_top_vel = sim_bot_vel_ + sim_top_vel_;

                // 摩擦力 (相对摩擦)
                double friction_bot = B_bot * abs_bot_vel;  // 相对底盘(假设底盘速度0)
                double friction_top = B_top * sim_top_vel_; // 相对下云台

                // 绝对角加速度 (牛顿第二定律)
                // 下云台受力：底盘给的电机力 - 上云台给的电机力反作用 - 自身摩擦 + 上云台摩擦反作用
                double abs_alpha_bot =
                    (actual_bot_torque - actual_top_torque - friction_bot + friction_top) / J_bot;
                // 上云台受力：下云台给的电机力 - 相对摩擦力
                double abs_alpha_top = (actual_top_torque - friction_top) / J_top;

                // Pitch 是相对独立的一轴，不影响水平向惯量(简化)
                double alpha_pitch = (actual_pitch_torque - B_pitch * sim_pitch_vel_) / J_pitch;

                // 积分更新绝对速度
                abs_bot_vel += abs_alpha_bot * dt;
                abs_top_vel += abs_alpha_top * dt;

                // 更新相对速度和相对角度
                sim_bot_vel_ = abs_bot_vel;
                sim_top_vel_ = abs_top_vel - abs_bot_vel;

                sim_bot_angle_ += sim_bot_vel_ * dt;
                sim_top_angle_ += sim_top_vel_ * dt;

                // --- 模拟 Top Yaw 相对 Bottom Yaw 的物理硬限位: [-60°, +60°] ---
                const double TOP_YAW_LIMIT = M_PI / 3.0; // 60 度
                if (sim_top_angle_ > TOP_YAW_LIMIT) {
                    sim_top_angle_ = TOP_YAW_LIMIT;
                    // 撞击限位时，相对速度强制归零
                    if (sim_top_vel_ > 0)
                        sim_top_vel_ = 0.0;
                } else if (sim_top_angle_ < -TOP_YAW_LIMIT) {
                    sim_top_angle_ = -TOP_YAW_LIMIT;
                    if (sim_top_vel_ < 0)
                        sim_top_vel_ = 0.0;
                }

                sim_pitch_vel_ += alpha_pitch * dt;
                sim_pitch_angle_ += sim_pitch_vel_ * dt;
            }

            // 精确睡眠至下一个 100us 时刻
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
    double vis_target_yaw_ = 0.0;
    double vis_target_pitch_ = 0.0;

    // 物理仿真相关
    std::thread physics_thread_;
    std::atomic<bool> running_{false};
    std::mutex state_mutex_;

    // 内部物理状态 (10kHz 更新，1kHz 采样)
    double sim_top_angle_ = 0.0;
    double sim_top_vel_ = 0.0;
    double sim_bot_angle_ = 0.0;
    double sim_bot_vel_ = 0.0;
    double sim_pitch_angle_ = 0.0;
    double sim_pitch_vel_ = 0.0;
    double sim_chassis_vel_ = 0.0;

    // 当前生效的控制命令
    double cmd_top_torque_ = 0.0;
    double cmd_bot_torque_ = 0.0;
    double cmd_pitch_torque_ = 0.0;

    // 可视化相关
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int publish_count_ = 0;

    void publish_markers(double target_yaw, double target_pitch) {
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

            // 使用 TF2 欧拉角转四元数 (Roll=0, Pitch, Yaw)
            // 注意：ROS 里通常 Pitch 是绕 Y 轴转，Yaw 是绕 Z 轴转。
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

        // 绝对角度计算
        double chassis_yaw = 0.0;
        double bot_yaw_abs = chassis_yaw + sim_bot_angle_;
        double top_yaw_abs = bot_yaw_abs + sim_top_angle_;

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
            4, visualization_msgs::msg::Marker::ARROW, 0.25, sim_pitch_angle_, top_yaw_abs, 0.5,
            0.05, 0.05, 0.0, 1.0, 1.0, "pitch_arrow"));

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
        auto publish_torque = [](const InputInterface<double>& input,
                                 rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub) {
            if (input.ready() && !std::isnan(*input)) {
                std_msgs::msg::Float64 msg;
                msg.data = *input;
                pub->publish(msg);
            }
        };

        publish_torque(top_yaw_torque_, top_yaw_pub_);
        publish_torque(bottom_yaw_torque_, bottom_yaw_pub_);
        publish_torque(pitch_torque_, pitch_pub_);
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
