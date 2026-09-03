#include <fmt/format.h>
#include <fstream>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::shooting {

class ShootingRecorder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ShootingRecorder(
        const rclcpp::NodeOptions& option =
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        : Node(get_component_name(), option) {

        friction_wheel_count_ = get_parameter("friction_wheel_count").as_int();

        log_mode_ = static_cast<LogMode>(get_parameter("log_mode").as_int());

        register_input("/referee/shooter/initial_speed", initial_speed_);
        register_input("/referee/shooter/shoot_timestamp", shoot_timestamp_);
        register_input("/friction_wheels/temperature", fractional_temperature_);

        if (friction_wheel_count_ == 2) {
            const auto topic = std::array{
                "/gimbal/left_friction/control_velocity",
                "/gimbal/right_friction/control_velocity",
            };
            for (int i = 0; i < 2; i++)
                register_input(topic[i], friction_wheels_velocity_[i]);
        } else if (friction_wheel_count_ == 4) {
            const auto topic = std::array{
                "/gimbal/first_left_friction/control_velocity",
                "/gimbal/first_right_friction/control_velocity",
                "/gimbal/second_left_friction/control_velocity",
                "/gimbal/second_right_friction/control_velocity",
            };
            for (int i = 0; i < 4; i++)
                register_input(topic[i], friction_wheels_velocity_[i]);
        }

        using namespace std::chrono;
        auto now = high_resolution_clock::now();
        auto ms  = duration_cast<milliseconds>(now.time_since_epoch()).count();

        auto file = fmt::format("/robot_shoot/{}.log", ms);
        log_stream_.open(file);
    }

    ~ShootingRecorder() { log_stream_.close(); }

    void update() override {

        switch (log_mode_) {
        case LogMode::TRIGGER:
            // It will be triggered by shooting action
            if (*shoot_timestamp_ == last_shoot_timestamp_)
                return;
            break;
        case LogMode::TIMING:
            // 10Hz to log
            if (log_count_++ % 100)
                return;
            break;
        }

        auto log_text  = std::string{};
        auto timestamp = timestamp_to_string(*shoot_timestamp_);

        if (friction_wheel_count_ == 2) {
            log_text = fmt::format(
                "{},{},{},{},{}", timestamp, *initial_speed_,       //
                *friction_wheels_velocity_[0], *friction_wheels_velocity_[1],
                *fractional_temperature_);
        } else if (friction_wheel_count_ == 4) {
            log_text = fmt::format(
                "{},{},{},{},{},{},{}", timestamp, *initial_speed_, //
                *friction_wheels_velocity_[0], *friction_wheels_velocity_[1],
                *friction_wheels_velocity_[2], *friction_wheels_velocity_[3],
                *fractional_temperature_);
        }

        log_stream_ << log_text << std::endl;
        RCLCPP_INFO(get_logger(), "%s", log_text.c_str());

        last_shoot_timestamp_ = *shoot_timestamp_;
    }

private:
    /// @brief Component interface
    InputInterface<float> initial_speed_;
    InputInterface<double> shoot_timestamp_;

    InputInterface<double> fractional_temperature_;

    std::size_t friction_wheel_count_ = 2;
    std::array<InputInterface<double>, 4> friction_wheels_velocity_;

    /// @brief For log
    enum class LogMode { TRIGGER = 1, TIMING = 2 };
    LogMode log_mode_ = LogMode::TRIGGER;

    double last_shoot_timestamp_ = 0;
    std::ofstream log_stream_;

    std::size_t log_count_ = 0;

private:
    static std::string timestamp_to_string(double timestamp) {
        auto time       = static_cast<std::time_t>(timestamp);
        auto local_time = std::localtime(&time);

        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local_time);

        double fractional_seconds = timestamp - std::floor(timestamp);
        int milliseconds          = static_cast<int>(fractional_seconds * 1000);

        char result[150];
        std::snprintf(result, sizeof(result), "%s.%03d", buffer, milliseconds);

        return result;
    }
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::ShootingRecorder, rmcs_executor::Component)