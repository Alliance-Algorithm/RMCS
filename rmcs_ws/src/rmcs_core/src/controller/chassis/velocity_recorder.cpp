#include <fmt/format.h>
#include <fstream>

#include <rmcs_msgs/chassis_mode.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <string>

namespace rmcs_core::controller::chassis {
class VelocityRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    VelocityRecorder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/chassis/control_mode", chassis_mode_);
        register_input("/chassis/supercap/control_enable", supercap_enabled_);

        register_input("/chassis/left_front_wheel/velocity", left_front_velocity_);
        register_input("/chassis/left_back_wheel/velocity", left_back_velocity_);
        register_input("/chassis/right_back_wheel/velocity", right_back_velocity_);
        register_input("/chassis/right_front_wheel/velocity", right_front_velocity_);

        using namespace std::chrono;
        auto now = high_resolution_clock::now();
        auto ms  = duration_cast<milliseconds>(now.time_since_epoch()).count();

        auto filepath = fmt::format("/wheels_velocity/{}.log", ms);
        log_stream_.open(filepath);
    }

    ~VelocityRecorder() { log_stream_.close(); }

    void update() override {
        auto log_msgs      = std::string{};
        auto log_timestamp = get_timestamp();

        auto mode    = *chassis_mode_;
        auto enabled = *supercap_enabled_;

        if (mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP && enabled == true) {
            log_msgs = fmt::format(
                "{} {},{},{},{}", log_timestamp, *left_front_velocity_, *left_back_velocity_,
                *right_back_velocity_, *right_front_velocity_);
        }
    }

private:
    static  std::string get_timestamp() {
        auto now             = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        std::tm local_tm = *std::localtime(&now_time);
        std::ostringstream time_stream;
        time_stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S");

        auto now_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        time_stream << '.' << std::setw(3) << std::setfill('0') << now_ms.count();

        return time_stream.str();
    }

    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;

    InputInterface<double> left_front_velocity_;
    InputInterface<double> left_back_velocity_;
    InputInterface<double> right_back_velocity_;
    InputInterface<double> right_front_velocity_;

    InputInterface<bool> supercap_enabled_;

    std::ostringstream filename_stream_;
    std::ofstream log_stream_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::VelocityRecorder, rmcs_executor::Component)