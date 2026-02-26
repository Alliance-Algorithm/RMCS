#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

static constexpr const char* PLANNING_GROUP = "alliance_arm";

class ArmMoveitNode : public rclcpp::Node {
public:
    explicit ArmMoveitNode()
        : Node("arm_moveit_process")  
    {
        target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm/target_joints",
            rclcpp::QoS(1).best_effort());

        enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/arm/enable_cmd",
            rclcpp::QoS(1).best_effort());

        exec_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { execute_trajectory_step(); });

        RCLCPP_INFO(get_logger(), "[ArmMoveitNode] 节点启动，等待初始化 MoveGroupInterface...");
    }

    ~ArmMoveitNode() override {

        planning_running_.store(false);
        if (planning_thread_.joinable())
            planning_thread_.join();

        RCLCPP_INFO(get_logger(), "[ArmMoveitNode] 节点已停止");
    }

    void request_plan(const geometry_msgs::msg::Pose& target_pose) {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_pose_   = target_pose;
        should_plan_.store(true);
        RCLCPP_INFO(get_logger(),
            "[ArmMoveitNode] 收到新目标: x=%.3f y=%.3f z=%.3f",
            target_pose.position.x, target_pose.position.y, target_pose.position.z);
    }

private:
    void init_moveit() {
        if (moveit_initialized_) return;

        RCLCPP_INFO(get_logger(), "[ArmMoveitNode] 正在连接 move_group ...");

        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(),   
            PLANNING_GROUP);

        move_group_->startStateMonitor(10.0);

        planning_running_.store(true);
        planning_thread_ = std::thread([this]() {
            while (planning_running_.load()) {
                do_planning();  
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });

        moveit_initialized_ = true;
        RCLCPP_INFO(get_logger(), "[ArmMoveitNode] MoveGroupInterface 连接成功！");

        demo_trigger_timer_ = create_wall_timer(
            std::chrono::seconds(3),
            [this]() {
                geometry_msgs::msg::Pose pose;
                pose.position.x    = 0.30;
                pose.position.y    = 0.0;
                pose.position.z    = 0.25;
                pose.orientation.w = 1.0;  
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                request_plan(pose);
                demo_trigger_timer_.reset();  // 只触发一次
            });
            RCLCPP_INFO(get_logger(), "[ArmMoveitNode] 初始化完成");
    }

    void do_planning() {
        if (!should_plan_.exchange(false)) return;

        geometry_msgs::msg::Pose pose;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            pose = target_pose_;
        }

        RCLCPP_INFO(get_logger(), "[规划] 开始规划到 (%.3f, %.3f, %.3f)...",
            pose.position.x, pose.position.y, pose.position.z);

        // 设置规划参数
        move_group_->setMaxVelocityScalingFactor(0.05);      
        move_group_->setMaxAccelerationScalingFactor(0.05);  
        move_group_->setPlanningTime(5.0);                 
        move_group_->setPoseTarget(pose);                  

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN(get_logger(), "[规划] 失败！目标可能不可达或超出关节限制");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            traj_points_      = plan.trajectory.joint_trajectory.points;
            traj_start_time_  = now();    
            trajectory_ready_ = true;
        }

        RCLCPP_INFO(get_logger(), "[规划] 成功！共 %zu 个路点，开始执行",
            plan.trajectory.joint_trajectory.points.size());
    }

    void execute_trajectory_step() {
        if (!moveit_initialized_) {
            init_moveit();
            return;  // 初始化后本次不执行轨迹
        }

        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> pts;
        rclcpp::Time start;
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            if (!trajectory_ready_) {
                if (!last_positions_.empty())
                    publish_joint_target(last_positions_);
                return;
            }
            pts   = traj_points_;
            start = traj_start_time_;
        }

        // 计算已过去的时间（秒）
        double elapsed = (now() - start).seconds();

        // 整条轨迹的总时长 = 最后一个点的 time_from_start
        double total = rclcpp::Duration(pts.back().time_from_start).seconds();

        if (elapsed >= total) {
            publish_joint_target(pts.back().positions);
            {
                std::lock_guard<std::mutex> lock(traj_mutex_);
                last_positions_   = pts.back().positions;  // 保存末端位置
                trajectory_ready_ = false;
            }
            RCLCPP_INFO(get_logger(), "[执行] 轨迹执行完成");
            return;
        }

        for (int i = static_cast<int>(pts.size()) - 1; i >= 0; --i) {
            double t = rclcpp::Duration(pts[i].time_from_start).seconds();
            if (elapsed >= t) {
                publish_joint_target(pts[i].positions);
                return;
            }
        }
    }

    void publish_joint_target(const std::vector<double>& q) {
        // 发布目标关节角度
        std_msgs::msg::Float64MultiArray target_msg;
        target_msg.data.resize(6, 0.0);
        for (std::size_t i = 0; i < 5 && i < q.size(); ++i)
            target_msg.data[i] = q[i];
        target_msg.data[5] = 0.0;
        target_pub_->publish(target_msg);

        // 同时发布使能指令（让电机开始跟随）
        std_msgs::msg::Bool enable_msg;
        enable_msg.data = true;
        enable_pub_->publish(enable_msg);
    }

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    bool moveit_initialized_{false};

    std::thread          planning_thread_;
    std::atomic<bool>    planning_running_{false};
    std::atomic<bool>    should_plan_{false};
    std::mutex           target_mutex_;
    geometry_msgs::msg::Pose target_pose_;
    std::vector<double> last_positions_;

    // ── 轨迹缓冲（exec_timer_ 和 planning_thread_ 共用，需要锁） ─
    std::mutex                                               traj_mutex_;
    bool                                                     trajectory_ready_{false};
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint>  traj_points_;
    rclcpp::Time                                             traj_start_time_;

    rclcpp::TimerBase::SharedPtr exec_timer_;
    rclcpp::TimerBase::SharedPtr demo_trigger_timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               enable_pub_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArmMoveitNode>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2);
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "[main] 开始 spin,Ctrl+C 退出");
    executor.spin();  
    rclcpp::shutdown();
    return 0;
}