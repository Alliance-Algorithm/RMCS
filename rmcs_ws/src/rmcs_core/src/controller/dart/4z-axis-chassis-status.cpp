#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/four_z_chassis_command.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class FourZAxisChassisStatus
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using FourZCommand = rmcs_dart_guidance::msg::FourZChassisCommand;

public:
    FourZAxisChassisStatus()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/chassis/4z/command", command_, false);

        for (size_t i = 0; i < kAxisCount; ++i) {
            register_input(kMotorPrefixes[i] + "/angle", motor_angle_[i], false);
            register_input(kMotorPrefixes[i] + "/velocity", motor_velocity_[i], false);
            register_input(kMotorPrefixes[i] + "/bottom_limit_switch", bottom_limit_switch_[i],
                           false);

            register_output(kMotorPrefixes[i] + "/height", axis_height_[i], kNaN);
            register_output(kMotorPrefixes[i] + "/height_velocity", axis_height_velocity_[i], kNaN);
            register_output(kMotorPrefixes[i] + "/stroke_angle", axis_stroke_angle_[i], kNaN);
            register_output(kMotorPrefixes[i] + "/bottom_zero_valid", bottom_zero_valid_output_[i],
                            false);
        }

        register_input("/dart/chassis/imu/pitch", imu_pitch_, false);
        register_input("/dart/chassis/imu/roll", imu_roll_, false);
        register_input("/dart/chassis/imu/pitch_rate", imu_pitch_rate_, false);
        register_input("/dart/chassis/imu/roll_rate", imu_roll_rate_, false);

        register_output("/dart/chassis/pitch", pitch_, 0.0);
        register_output("/dart/chassis/roll", roll_, 0.0);
        register_output("/dart/chassis/height", height_, 0.0);
        register_output("/dart/chassis/height_velocity", height_velocity_, 0.0);
        register_output("/dart/chassis/pitch_rate", pitch_rate_, 0.0);
        register_output("/dart/chassis/roll_rate", roll_rate_, 0.0);
        register_output("/dart/chassis/height_calibrated", height_calibrated_, false);
        register_output("/dart/chassis/motor_estimated_pitch", motor_estimated_pitch_, kNaN);
        register_output("/dart/chassis/motor_estimated_roll", motor_estimated_roll_, kNaN);

        get_parameter_or("axis_x", axis_x_, 0.20);
        get_parameter_or("axis_y", axis_y_, 0.15);
        get_parameter_or(
            "clear_bottom_zero_on_calibrate_start", clear_bottom_zero_on_calibrate_start_, false);

        for (size_t i = 0; i < kAxisCount; ++i) {
            get_parameter_or(
                kHeightPerMotorRadParameters[i], height_per_motor_rad_[i],
                height_per_motor_rad_[i]);
            get_parameter_or(
                kHeightDirectionParameters[i], height_direction_[i], height_direction_[i]);
        }
    }

    void update() override {
        const auto command = command_.ready() ? *command_ : FourZCommand::IDLE;
        if (command == FourZCommand::CALIBRATE_BOTTOM
            && last_command_ != FourZCommand::CALIBRATE_BOTTOM
            && clear_bottom_zero_on_calibrate_start_) {
            bottom_zero_valid_.fill(false);
        }
        last_command_ = command;

        std::array<double, kAxisCount> z{};
        std::array<double, kAxisCount> v{};
        std::array<double, kAxisCount> stroke{};
        stroke.fill(kNaN);

        for (size_t i = 0; i < kAxisCount; ++i) {
            const double angle = motor_angle_[i].ready() ? *motor_angle_[i] : 0.0;
            const bool bottom = bottom_limit_switch_[i].ready() && *bottom_limit_switch_[i];

            if (!startup_zero_valid_[i] && motor_angle_[i].ready()) {
                startup_zero_angle_[i] = angle;
                startup_zero_valid_[i] = true;
            }

            if (bottom && !bottom_zero_valid_[i]) {
                bottom_zero_angle_[i] = angle;
                bottom_zero_valid_[i] = true;
            }
        }

        const bool calibrated = std::ranges::all_of(bottom_zero_valid_, [](bool valid) {
            return valid;
        });

        for (size_t i = 0; i < kAxisCount; ++i) {
            const double angle = motor_angle_[i].ready() ? *motor_angle_[i] : 0.0;
            const double velocity = motor_velocity_[i].ready() ? *motor_velocity_[i] : 0.0;
            const double reference_zero =
                calibrated ? bottom_zero_angle_[i] : startup_zero_angle_[i];

            const double height_direction = height_direction_[i];
            z[i] = height_per_motor_rad_[i] * height_direction * (angle - reference_zero);
            v[i] = height_per_motor_rad_[i] * height_direction * velocity;

            if (bottom_zero_valid_[i])
                stroke[i] = height_direction * (angle - bottom_zero_angle_[i]);

            *axis_height_[i] = z[i];
            *axis_height_velocity_[i] = v[i];
            *axis_stroke_angle_[i] = stroke[i];
            *bottom_zero_valid_output_[i] = bottom_zero_valid_[i];
        }

        const double current_height = average(z);
        const double current_height_velocity = average(v);
        const double current_pitch = imu_pitch_.ready() ? *imu_pitch_ : 0.0;
        const double current_roll = imu_roll_.ready() ? *imu_roll_ : 0.0;
        const double current_pitch_rate = imu_pitch_rate_.ready() ? *imu_pitch_rate_ : 0.0;
        const double current_roll_rate = imu_roll_rate_.ready() ? *imu_roll_rate_ : 0.0;

        *height_ = current_height;
        *height_velocity_ = current_height_velocity;
        *pitch_ = current_pitch;
        *roll_ = current_roll;
        *pitch_rate_ = current_pitch_rate;
        *roll_rate_ = current_roll_rate;
        *height_calibrated_ = calibrated;

        *motor_estimated_pitch_ =
            axis_x_ > 0.0 ? -((z[0] + z[1]) - (z[2] + z[3])) / (4.0 * axis_x_) : kNaN;
        *motor_estimated_roll_ =
            axis_y_ > 0.0 ? ((z[0] + z[2]) - (z[1] + z[3])) / (4.0 * axis_y_) : kNaN;
    }

private:
    static constexpr size_t kAxisCount = 4;
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static const inline std::array<std::string, kAxisCount> kMotorPrefixes{
        "/dart/chassis/front_left_motor",
        "/dart/chassis/front_back_motor",
        "/dart/chassis/back_left_motor",
        "/dart/chassis/back_right_motor",
    };
    static constexpr std::array<const char*, kAxisCount> kHeightPerMotorRadParameters{
        "front_left_height_per_motor_rad",
        "front_back_height_per_motor_rad",
        "back_left_height_per_motor_rad",
        "back_right_height_per_motor_rad",
    };
    static constexpr std::array<const char*, kAxisCount> kHeightDirectionParameters{
        "front_left_height_direction",
        "front_back_height_direction",
        "back_left_height_direction",
        "back_right_height_direction",
    };

    static double average(const std::array<double, kAxisCount>& values) {
        return std::reduce(values.begin(), values.end(), 0.0) / static_cast<double>(kAxisCount);
    }

    InputInterface<FourZCommand> command_;
    std::array<InputInterface<double>, kAxisCount> motor_angle_;
    std::array<InputInterface<double>, kAxisCount> motor_velocity_;
    std::array<InputInterface<bool>, kAxisCount> bottom_limit_switch_;
    InputInterface<double> imu_pitch_;
    InputInterface<double> imu_roll_;
    InputInterface<double> imu_pitch_rate_;
    InputInterface<double> imu_roll_rate_;

    std::array<OutputInterface<double>, kAxisCount> axis_height_;
    std::array<OutputInterface<double>, kAxisCount> axis_height_velocity_;
    std::array<OutputInterface<double>, kAxisCount> axis_stroke_angle_;
    std::array<OutputInterface<bool>, kAxisCount> bottom_zero_valid_output_;
    OutputInterface<double> pitch_;
    OutputInterface<double> roll_;
    OutputInterface<double> height_;
    OutputInterface<double> height_velocity_;
    OutputInterface<double> pitch_rate_;
    OutputInterface<double> roll_rate_;
    OutputInterface<bool> height_calibrated_;
    OutputInterface<double> motor_estimated_pitch_;
    OutputInterface<double> motor_estimated_roll_;

    double axis_x_ = 0.20;
    double axis_y_ = 0.15;
    std::array<double, kAxisCount> height_per_motor_rad_{0.001, 0.001, 0.001, 0.001};
    std::array<double, kAxisCount> height_direction_{1.0, 1.0, 1.0, 1.0};
    bool clear_bottom_zero_on_calibrate_start_ = false;

    FourZCommand last_command_ = FourZCommand::IDLE;
    std::array<double, kAxisCount> startup_zero_angle_{0.0, 0.0, 0.0, 0.0};
    std::array<bool, kAxisCount> startup_zero_valid_{false, false, false, false};
    std::array<double, kAxisCount> bottom_zero_angle_{0.0, 0.0, 0.0, 0.0};
    std::array<bool, kAxisCount> bottom_zero_valid_{false, false, false, false};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FourZAxisChassisStatus,
                       rmcs_executor::Component)
