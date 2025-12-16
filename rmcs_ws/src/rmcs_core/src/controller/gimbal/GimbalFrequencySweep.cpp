// #include <chrono>
// #include <fstream>
// #include <vector>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <fast_tf/rcl.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include <ctime>      // 用于时间格式化
#include <fstream>    // 添加文件流支持
#include <iomanip>    // 添加格式化支持
#include <sys/stat.h> // 用于创建目录

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class GimbalFrequencySweep
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalFrequencySweep()
        : Node("gimbal_frequency_sweep") {
        // 创建数据存储目录
        create_data_directory();
        // 初始化CSV文件
        init_csv_file();

        register_input("/remote/switch/right", switch_right_);

        register_input("/remote/switch/left", switch_left_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/yaw/max_torque", yaw_max_torque_);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_);

        // 参数配置
    }

    ~GimbalFrequencySweep() {
        if (csv_file_.is_open()) {
            csv_file_.close();
            RCLCPP_INFO(get_logger(), "CSV file closed: %s", filename_.c_str());
        }
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;

        do {
            if (switch_left == Switch::DOWN && switch_right == Switch::MIDDLE) {

                Now_time = this->now();
                if (f_now > F_end) {
                    break;
                }
                if (last_time <= 0.0002) {
                    last_time = Now_time.seconds();
                }
                now_time = Now_time.seconds();

                current = Top * std::sin(2 * Pi * f_now * (now_time - last_time));
                double torque_command = current * k;
                *yaw_control_torque_ = torque_command;
                if ((now_time - last_time) > ((1 / f_now) * Repeat_time)) {
                    if (f_now < 24) {
                        f_now += 0.5;
                    } else if (f_now >= 24 && f_now <= 120) {
                        f_now += 2;
                    } else {
                        f_now += 4;
                    }
                    last_time = Now_time.seconds();
                }
                record_data(); // gimbal_yaw_motor_.control_torque()
            }
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                reset_status();
            }

        } while (false);
    }

private:
    void create_data_directory() {
        // 获取当前用户的主目录
        const char* home_dir = std::getenv("HOME");
        if (!home_dir) {
            RCLCPP_ERROR(get_logger(), "Failed to get home directory");
            return;
        }

        // 创建数据目录
        data_dir_ = std::string(home_dir) + "/gimbal_sweep_data";
        if (mkdir(data_dir_.c_str(), 0777) == -1) {
            if (errno != EEXIST) {
                RCLCPP_ERROR(get_logger(), "Failed to create data directory: %s", strerror(errno));
            } else {
                RCLCPP_INFO(get_logger(), "Using existing data directory: %s", data_dir_.c_str());
            }
        } else {
            RCLCPP_INFO(get_logger(), "Created data directory: %s", data_dir_.c_str());
        }
    }

    void init_csv_file() {
        // 生成带时间戳的文件名
        auto now       = std::time(nullptr);
        std::tm now_tm = *std::localtime(&now);

        std::ostringstream oss;
        oss << data_dir_ << "/sweep_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << ".csv";
        filename_ = oss.str();

        csv_file_.open(filename_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", filename_.c_str());
            return;
        }

        // 写入CSV表头
        csv_file_ << "actual_velocity control_torque\n";
        csv_file_.flush();

        RCLCPP_INFO(get_logger(), "Data recording started. File: %s", filename_.c_str());
    }

    void record_data() {
        if (!csv_file_.is_open())
            return;

        // 获取当前时间
        auto now = this->now();
        // double elapsed = (last_time > 0) ? now.seconds() - last_time : 0.0;

        // 写入数据
        csv_file_ << *yaw_velocity_ << " " << current << "\n";
        RCLCPP_INFO(get_logger(), "%d", i++);
        // 定期刷新缓冲区以确保
        static int count = 0;
        if (++count % 100 == 0) {
            csv_file_.flush();
        }
    }

    void reset_status() {
        f_now                = 1.0;
        *yaw_control_torque_ = 0.0;
    }

    rclcpp::Time Now_time;

    double now_time;
    double last_time = 0.0;
    double current;

    float f_now = 5.0;
    int i       = 0;

    static constexpr int F_start     = 5;
    static constexpr int F_end       = 50;
    static constexpr int Repeat_time = 20;
    static constexpr double Pi       = 3.1415926;
    static constexpr double Top      = 0.999;
    static constexpr double k        = 0.1;
    // ROS接口

    rclcpp::TimerBase::SharedPtr control_timer_;

    // 数据接口
    InputInterface<double> yaw_velocity_;
    InputInterface<double> yaw_max_torque_;
    OutputInterface<double> yaw_current_command_;
    OutputInterface<double> yaw_control_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;

    std::string data_dir_;
    std::string filename_;
    std::ofstream csv_file_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::GimbalFrequencySweep, rmcs_executor::Component)