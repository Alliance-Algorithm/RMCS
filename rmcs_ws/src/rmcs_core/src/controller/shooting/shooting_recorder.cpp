#include <algorithm>
#include <array>
#include <fmt/format.h>
#include <fstream>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

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

        aim_velocity = get_parameter("aim_velocity").as_double();

        register_input("/referee/shooter/initial_speed", initial_speed_);
        register_input("/referee/shooter/shoot_timestamp", shoot_timestamp_);

        if (friction_wheel_count_ == 2) {
            const auto topic = std::array{
                "/gimbal/first_left_friction/control_velocity",
                "/gimbal/first_right_friction/control_velocity",
            };
            for (int i = 0; i < 2; i++)
                register_input(topic[i], friction_wheels_velocity_[i]);
            register_input("/gimbal/first_left_friction/velocity", friction_velocities_[0]);
            register_input("/gimbal/first_right_friction/velocity", friction_velocities_[1]);
        }

        using namespace std::chrono;
        auto now = high_resolution_clock::now();
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()).count();

        auto file = fmt::format("/robot_shoot/{}.log", ms);
        log_stream_.open(file);

        std::ofstream outFile("shoot_recorder");
        RCLCPP_INFO(get_logger(), "ShootingRecorder initialized, log file: %s", file.c_str());
    }

    ~ShootingRecorder() { log_stream_.close(); }

    void update() override {
        // if (*friction_velocities_[0] <= 366.0 && *friction_velocities_[1] <= 366.0)
        //     return;
        // if (start_time_ == 0.0) {
        //     start_time_ = GetTime();
        // }

        // int flag = 0;

        // if (GetTime() - start_time_ > 10.0) {
        //     if (flag == 0) {
        //         RCLCPP_INFO(get_logger(), "vv = %f", vv);
        //         flag = 1;
        //     }
        //     return;
        // }

        // vv = std::max(vv, abs(*friction_velocities_[0] + *friction_velocities_[1]));

        // auto log_text = std::string{};
        // log_text = fmt::format(
        //     "{:.3f}, {:.3f}, {:.3f}", *friction_velocities_[0], *friction_velocities_[1],
        //     (*friction_velocities_[0] + *friction_velocities_[1]));

        // log_stream_ << log_text << std::endl;
        // RCLCPP_INFO(get_logger(), "%s", log_text.c_str());

        // std::ofstream outFile("shoot_recorder", std::ios::app);
        // if (outFile.is_open()) {
        //     outFile << log_text << std::endl;
        //     outFile.close();
        // }
        switch (log_mode_) {
        case LogMode::TRIGGER:
            // It will be triggered by shooting action
            if (*shoot_timestamp_ == last_shoot_timestamp_ || v == *shoot_timestamp_)
                return;
            break;
        case LogMode::TIMING:
            // 10Hz to log
            if (log_count_++ % 100)
                return;
            break;
        }
        v = *shoot_timestamp_;

        velocities.push_back(*initial_speed_);

        analysis3();

        auto log_text = std::string{};
        auto timestamp = timestamp_to_string(*shoot_timestamp_);

        if (friction_wheel_count_ == 2) {
            log_text = fmt::format(
                "{},{},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}", *initial_speed_,
                (int)velocities.size(), //
                velocity_, excellence_rate_, pass_rate_, range_, range2_, velocity_max,
                velocity_min);
        }

        log_stream_ << log_text << std::endl;
        RCLCPP_INFO(get_logger(), "%s", log_text.c_str());

        log_velocity = fmt::format("{:.3f}", *initial_speed_);
        std::ofstream outFile("shoot_recorder", std::ios::app);
        if (outFile.is_open()) {
            outFile << log_velocity << std::endl;
            outFile.close();
        }

        last_shoot_timestamp_ = *shoot_timestamp_;
    }

private:
    /// @brief Component interface
    std::array<InputInterface<double>, 2> friction_velocities_;

    InputInterface<float> initial_speed_;
    InputInterface<double> shoot_timestamp_;

    std::size_t friction_wheel_count_ = 2;
    std::array<InputInterface<double>, 2> friction_wheels_velocity_;

    /// @brief For log
    enum class LogMode { TRIGGER = 1, TIMING = 2 };
    LogMode log_mode_ = LogMode::TRIGGER;

    double last_shoot_timestamp_ = 0;
    std::ofstream log_stream_;
    std::string log_velocity;

    std::size_t log_count_ = 0;

    std::vector<double> velocities;

    double velocity_;

    double excellence_rate_;
    double pass_rate_;

    double range_;
    double range2_;

    double velocity_min;
    double velocity_max;

    double v;
    double aim_velocity;

    double start_time_ = 0.0;
    double vv = 0.0;

private:
    static std::string timestamp_to_string(double timestamp) {
        auto time = static_cast<std::time_t>(timestamp);
        auto local_time = std::localtime(&time);

        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local_time);

        double fractional_seconds = timestamp - std::floor(timestamp);
        int milliseconds = static_cast<int>(fractional_seconds * 1000);

        char result[150];
        std::snprintf(result, sizeof(result), "%s.%03d", buffer, milliseconds);

        return result;
    }
    void analysis1() {
        double sum = 0.0;
        for (const auto& v : velocities) {
            sum += v;
        }
        velocity_ = sum / double(velocities.size());

        sort(velocities.begin(), velocities.end());

        range_ = velocities.back() - velocities.front();
        range2_ = velocities[int(velocities.size() - 2)] - velocities[1];

        velocity_max = velocities.back();
        velocity_min = velocities.front();

        int excellence_count = 0;
        int pass_count = 0;
        for (int i = 0; i < int(velocities.size()); i++) {
            if (velocities[i] >= velocity_ - 0.1 && velocities[i] <= velocity_ + 0.1) {
                pass_count += 1;
            }
            if (velocities[i] >= velocity_ - 0.05 && velocities[i] <= velocity_ + 0.05) {
                excellence_count += 1;
            }
        }
        excellence_rate_ = double(excellence_count) / double(velocities.size());
        pass_rate_ = double(pass_count) / double(velocities.size());
    }

    void analysis2() {
        double sum = 0.0;
        for (const auto& v : velocities) {
            sum += v;
        }

        sort(velocities.begin(), velocities.end());

        velocity_max = velocities.back();
        velocity_min = velocities.front();

        int n_adjust = std::max(1, int((int)velocities.size() * 0.05));

        for (int i = 0; i < n_adjust; i++) {
            sum -= velocities[i];
            sum -= velocities[velocities.size() - 1 - i];
        }

        velocity_ = sum / double(velocities.size() - 2 * n_adjust);

        range_ = velocities.back() - velocities.front();
        range2_ = velocities[int(velocities.size() - 2)] - velocities[1];

        int excellence_count = 0;
        int pass_count = 0;
        for (int i = 0; i < int(velocities.size()); i++) {
            if (velocities[i] >= velocity_ - 0.1 && velocities[i] <= velocity_ + 0.1) {
                pass_count += 1;
            }
            if (velocities[i] >= velocity_ - 0.05 && velocities[i] <= velocity_ + 0.05) {
                excellence_count += 1;
            }
        }
        excellence_rate_ = double(excellence_count) / double(velocities.size());
        pass_rate_ = double(pass_count) / double(velocities.size());
    }

    void analysis3() {
        double sum = 0.0;
        for (const auto& v : velocities) {
            sum += v;
        }
        velocity_ = sum / double(velocities.size());

        int excellence_count = 0;
        int pass_count = 0;

        for (const auto& v : velocities) {
            if (v >= aim_velocity - 0.05 && v <= aim_velocity + 0.05) {
                excellence_count += 1;
            }
            if (v >= aim_velocity - 0.1 && v <= aim_velocity + 0.1) {
                pass_count += 1;
            }
        }
        excellence_rate_ = double(excellence_count) / double(velocities.size());
        pass_rate_ = double(pass_count) / double(velocities.size());

        sort(velocities.begin(), velocities.end());
        velocity_max = velocities.back();
        velocity_min = velocities.front();

        range_ = velocities.back() - velocities.front();
        range2_ = velocities[int(velocities.size() - 2)] - velocities[1];
    }

    static double GetTime() {
        using namespace std::chrono;
        static auto start = high_resolution_clock::now();
        auto now = high_resolution_clock::now();
        duration<double> elapsed = now - start;
        return elapsed.count();
    }
};

} // namespace rmcs_core::controller::shooting

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::shooting::ShootingRecorder, rmcs_executor::Component)