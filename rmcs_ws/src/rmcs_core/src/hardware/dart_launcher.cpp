#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <span>
#include <sstream>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/force_sensor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/agent/rmcs_board_pro.hpp"

namespace rmcs_core::hardware {

class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::RmcsBoardPro {
public:
    CatapultDart()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::RmcsBoardPro{get_parameter("serial_filter").as_string()}
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , logger_{get_logger()}
        , pitch_angle_filter_(20.0, 1000.0)
        , pitch_velocity_filter_(20.0, 1000.0)
        , yaw_velocity_filter_(20.0, 1000.0)
        , yaw_filter_(5.0, 1000.0)
        , pitch_filter_(10.0, 1000.0)
        , roll_filter_(10.0, 1000.0)
        , pitch_motor_{*this, *dart_command_, "/dart/pitch_motor"}
        , yaw_motor_{*this, *dart_command_, "/dart/yaw_motor"}
        , force_screw_motor_{*this, *dart_command_, "/dart/force_screw_motor"}
        , drive_belt_motor_left_{*this, *dart_command_, "/dart/drive_belt/left"}
        , drive_belt_motor_right_{*this, *dart_command_, "/dart/drive_belt/right"}
        , force_sensor_{*this}
        , trigger_servo_{"/dart/trigger_servo", *dart_command_, 20.0, 0.5, 2.5}
        , limiting_servo_{*dart_command_, "/dart/limiting_servo", 0x02}
        , lifting_left_motor_{*this, *dart_command_, "/dart/lifting_left"}
        , lifting_right_motor_{*this, *dart_command_, "/dart/lifting_right"}
        , dr16_{*this}
        , imu_{1000, 0.2, 0.0} {

        pitch_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reduction_ratio(19.)
                .set_reversed());
        yaw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        force_screw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        drive_belt_motor_left_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reduction_ratio(19.)
                .enable_multi_turn_angle());
        drive_belt_motor_right_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reversed()
                .set_reduction_ratio(19.)
                .enable_multi_turn_angle());

        lifting_left_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());
        lifting_right_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());

        setup_imu_coordinate_mapping();

        register_output("/imu/catapult_pitch_angle", catapult_pitch_angle_);
        register_output("/imu/catapult_roll_angle", catapult_roll_angle_);
        register_output("/imu/catapult_yaw_angle", catapult_yaw_angle_);

        limiting_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/limiting/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_servo_calibrate_subscription_callback(limiting_servo_, std::move(msg));
            });

        force_sensor_calibrate_ = create_subscription<std_msgs::msg::Int32>(
            "/force_sensor/calibrate", rclcpp::QoS{10},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                force_sensor_calibrate_subscription_callback(std::move(msg));
            });

        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) -> size_t {
            std::lock_guard lock(referee_mutex_);
            size_t count = 0;
            while (count < size && !referee_ring_buffer_receive_.empty()) {
                buffer[count++] = referee_ring_buffer_receive_.front();
                referee_ring_buffer_receive_.pop_front();
            }
            return count;
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) -> size_t {
            auto board = start_transmit();
            board.uart1_transmit({
                .uart_data = std::span{buffer, size}
            });
            return size;
        };
    }

    CatapultDart(const CatapultDart&) = delete;
    CatapultDart& operator=(const CatapultDart&) = delete;
    CatapultDart(CatapultDart&&) = delete;
    CatapultDart& operator=(CatapultDart&&) = delete;
    ~CatapultDart() override = default;

    void update() override {
        dr16_.update_status();
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        drive_belt_motor_left_.update_status();
        drive_belt_motor_right_.update_status();
        force_screw_motor_.update_status();
        force_sensor_.update_status();
        lifting_left_motor_.update_status();
        lifting_right_motor_.update_status();
        imu_.update_status();
        processImuData();

        // 调试：打印多圈角度（每200次打印一次）
        // if (++angle_debug_counter_ >= 200) {
        //     angle_debug_counter_ = 0;
        //     RCLCPP_INFO(
        //         logger_,
        //         "[Multi-turn Angle] left_belt=%.4f rad, right_belt=%.4f rad, "
        //         "lifting_left=%.4f rad, lifting_right=%.4f rad",
        //         drive_belt_motor_left_.angle(), drive_belt_motor_right_.angle(),
        //         lifting_left_motor_.angle(), lifting_right_motor_.angle());
        // }
    }

    void command_update() {
        auto board = start_transmit();

        // Trigger servo: PWM via GPIO
        board.gpio_analog_write(
            librmcs::spec::rmcs_board_pro::kGpioDescriptors[1],
            librmcs::data::GpioAnalogDataView{.value = trigger_servo_.generate_duty_cycle()});

        // Force sensor: polling command on CAN1 (every 100 cycles)
        if (pub_time_count_++ > 100) {
            board.can1_transmit({
                .can_id = 0x301,
                .can_data = device::CanPacket8{0}.as_bytes(),
            });
            pub_time_count_ = 0;
        }

        // DJI motors on CAN2 (pitch/yaw/force_screw: 0x200, belts: 0x1FF)
        board.can2_transmit({
            .can_id = 0x200,
            .can_data =
                device::CanPacket8{
                                   pitch_motor_.generate_command(),
                                   yaw_motor_.generate_command(),
                                   force_screw_motor_.generate_command(),
                                   device::CanPacket8::PaddingQuarter{},
                                   }
                    .as_bytes(),
        });

        board.can2_transmit({
            .can_id = 0x1FF,
            .can_data =
                device::CanPacket8{
                                   drive_belt_motor_left_.generate_command(),
                                   drive_belt_motor_right_.generate_command(),
                                   device::CanPacket8::PaddingQuarter{},
                                   device::CanPacket8::PaddingQuarter{},
                                   }
                    .as_bytes(),
        });

        // LK4005 lifting motors on CAN3 (unicast per motor)
        board.can3_transmit({
            .can_id = 0x141,
            .can_data = lifting_left_motor_.generate_velocity_command().as_bytes(),
        });
        board.can3_transmit({
            .can_id = 0x145,
            .can_data = lifting_right_motor_.generate_velocity_command().as_bytes(),
        });

        // Limiting servo on UART2 (only send when target changes)
        if (!limiting_servo_.calibrate_mode()) {
            uint16_t current_target = limiting_servo_.get_target_angle();
            if (current_target != last_limiting_angle_) {
                size_t uart_data_length;
                auto command_buffer = limiting_servo_.generate_runtime_command(uart_data_length);
                board.uart2_transmit({
                    .uart_data = std::span{command_buffer.get(), uart_data_length}
                });
                last_limiting_angle_ = current_target;
            }
        }
    }

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        const auto can_id = data.can_id;
        if (can_id == 0x302) {
            force_sensor_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        const auto can_id = data.can_id;
        if (can_id == 0x201) {
            pitch_motor_.store_status(data.can_data);
        } else if (can_id == 0x202) {
            yaw_motor_.store_status(data.can_data);
        } else if (can_id == 0x203) {
            force_screw_motor_.store_status(data.can_data);
        } else if (can_id == 0x205) {
            drive_belt_motor_left_.store_status(data.can_data);
        } else if (can_id == 0x206) {
            drive_belt_motor_right_.store_status(data.can_data);
        }
    }

    void can3_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        const auto can_id = data.can_id;
        if (can_id == 0x141) {
            lifting_left_motor_.store_status(data.can_data);
        } else if (can_id == 0x145) {
            lifting_right_motor_.store_status(data.can_data);
        }
    }

    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        std::lock_guard lock(referee_mutex_);
        for (auto byte : data.uart_data) {
            referee_ring_buffer_receive_.push_back(byte);
        }
    }

    void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
        const auto* raw_data = data.uart_data.data();
        const size_t length = data.uart_data.size();

        RCLCPP_DEBUG(
            logger_, "UART2 received: len=%zu [%s]", length,
            bytes_to_hex_string(raw_data, length).c_str());

        if (length < 3) {
            RCLCPP_WARN(logger_, "UART2 data too short: %zu bytes", length);
            return;
        }

        const uint8_t servo_id = static_cast<uint8_t>(raw_data[2]);

        // Only limiting servo remains on UART2
        if (servo_id == 0x03) {
            auto result = limiting_servo_.calibrate_current_angle(logger_, raw_data, length);
            if (!result.first)
                RCLCPP_INFO(logger_, "calibrate: uart2 limiting data store failed");
        } else {
            RCLCPP_DEBUG(logger_, "UART2: unknown servo id 0x%02X", servo_id);
        }
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(const librmcs::data::AccelerometerDataView& data) override {
        imu_.store_accelerometer_status(data.x, data.y, data.z);
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        imu_.store_gyroscope_status(data.x, data.y, data.z);
    }

private:
    void force_sensor_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        auto board = start_transmit();
        board.can1_transmit({
            .can_id = 0x201,
            .can_data = device::CanPacket8{0x0F}.as_bytes(),
        });
        RCLCPP_INFO(logger_, "calibrate");
    }

    void trigger_servo_calibrate_subscription_callback(
        device::TriggerServo& servo, std_msgs::msg::Int32::UniquePtr msg) {
        servo.set_calibrate_mode(msg->data);

        std::unique_ptr<std::byte[]> command_buffer;
        size_t command_length = 0;

        if (msg->data == 0) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
        } else if (msg->data == 1) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
        } else if (msg->data == 2) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
        } else if (msg->data == 3) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
        } else if (msg->data == 4) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
        } else if (msg->data == 5) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
        } else if (msg->data == 6) {
            command_buffer = servo.generate_calibrate_command(
                device::TriggerServo::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
        }

        if (command_buffer && command_length > 0) {
            auto board = start_transmit();
            board.uart2_transmit({
                .uart_data = std::span{command_buffer.get(), command_length}
            });
            RCLCPP_INFO(
                logger_, "UART2 Pub: (length=%zu)[ %s ]", command_length,
                bytes_to_hex_string(command_buffer.get(), command_length).c_str());
        }
    }

    static std::string bytes_to_hex_string(const std::byte* data, size_t size) {
        if (!data || size == 0)
            return "[]";
        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');
        for (size_t i = 0; i < size; ++i)
            ss << std::setw(2) << static_cast<int>(data[i]) << " ";
        std::string result = ss.str();
        if (!result.empty() && result.back() == ' ')
            result.pop_back();
        return result;
    }

    void setup_imu_coordinate_mapping() {
        imu_.set_coordinate_mapping(
            [](double x, double y, double z) -> std::tuple<double, double, double> {
                return {x, -y, -z};
            });
    }

    void processImuData() {
        Eigen::Quaterniond imu_quat(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3());
        Eigen::Matrix3d rot = imu_quat.toRotationMatrix();

        // On the Dart installation, the mechanically observed front/back pitch maps to the
        // standard roll term, while left/right tilt maps to the standard pitch term.
        double sensor_roll = std::atan2(rot(2, 1), rot(2, 2));
        double sensor_pitch = std::asin(std::clamp(-rot(2, 0), -1.0, 1.0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));

        double mechanical_pitch = -roll_filter_.update(sensor_roll);
        double mechanical_roll = pitch_filter_.update(sensor_pitch);
        double t_yaw = -yaw_filter_.update(yaw);

        *catapult_pitch_angle_ = (mechanical_pitch / M_PI) * 180.0;
        *catapult_roll_angle_ = (mechanical_roll / M_PI) * 180.0;
        *catapult_yaw_angle_ = (t_yaw / M_PI) * 180.0;
    }

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDart& dart)
            : dart_(dart) {}
        void update() override { dart_.command_update(); }

    private:
        CatapultDart& dart_;
    };

    std::shared_ptr<DartCommand> dart_command_;
    rclcpp::Logger logger_;

    filter::LowPassFilter<1> pitch_angle_filter_;
    filter::LowPassFilter<1> pitch_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;
    filter::LowPassFilter<1> yaw_filter_;
    filter::LowPassFilter<1> pitch_filter_;
    filter::LowPassFilter<1> roll_filter_;

    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor force_screw_motor_;
    device::DjiMotor drive_belt_motor_left_;
    device::DjiMotor drive_belt_motor_right_;

    device::ForceSensor force_sensor_;

    device::PWMServo trigger_servo_;
    device::TriggerServo limiting_servo_;

    // LK4005 lifting motors (replaces TriggerServo lifting servos)
    device::LkMotor lifting_left_motor_;
    device::LkMotor lifting_right_motor_;

    device::Dr16 dr16_;
    device::Bmi088 imu_;

    OutputInterface<double> catapult_pitch_angle_;
    OutputInterface<double> catapult_roll_angle_;
    OutputInterface<double> catapult_yaw_angle_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr limiting_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr force_sensor_calibrate_;

    std::mutex referee_mutex_;
    std::deque<std::byte> referee_ring_buffer_receive_;
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    int pub_time_count_ = 0;

    uint16_t last_limiting_angle_ = 0xFFFF;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)
