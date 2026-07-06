#include <algorithm>
#include <cmath>
#include <cstddef>
// #include <cstdint>
#include <cstring>
#include <iomanip>
#include <memory>
#include <numbers>
#include <rclcpp/subscription.hpp>
#include <span>
#include <sstream>
#include <string>
#include <tuple>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
// #include "hardware/device/lk_motor.hpp"
// #include "hardware/device/pwm_servo.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/agent/rmcs_board_pro.hpp"
#include "std_msgs/msg/int32.hpp"

namespace rmcs_core::hardware {

class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::RmcsBoardPro {
public:
    CatapultDart()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::RmcsBoardPro{get_parameter("serial_filter").as_string()}
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , logger_{get_logger()}
        , yaw_motor_{*this, *dart_command_, "/dart/yaw_motor"}
        // , trigger_slider_motor{*this, *dart_command_,
        // "/dart/force_screw_motor"} , drive_belt_motors_(
        //       {*this, *dart_command_, "/dart/drive_belt/left"},
        //       {*this, *dart_command_, "/dart/drive_belt/right"})
        , chassis_leveling_motors_(
              {*this, *dart_command_, "/dart/chassis/front_left"}, {*this, *dart_command_, "/dart/chassis/front_right"},
              {*this, *dart_command_, "/dart/chassis/back_right"}, {*this, *dart_command_, "/dart/chassis/back_left"})
        // , filling_lifting_motor_(
        //       {*this, *dart_command_, "/dart/lifting_left"},
        //       {*this, *dart_command_, "/dart/lifting_right"})
        // , trigger_servo_{"/dart/trigger_servo", *dart_command_, 20.0, 0.5, 2.5}
        // , filling_limiting_servo_{*dart_command_, "/dart/limiting_servo", 0x02}
        , dr16_{*this}
        , imu_{1000, 0.2, 0.0} {

        yaw_motor_.configure(device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));

        // trigger_slider_motor.configure(
        //     device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
        //         .set_reduction_ratio(19.)
        //         .enable_multi_turn_angle());

        // drive_belt_motors_[0].configure(
        //     device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
        //         .set_reduction_ratio(19.)
        //         .enable_multi_turn_angle());

        // drive_belt_motors_[1].configure(
        //     device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
        //         .set_reversed()
        //         .set_reduction_ratio(19.)
        //         .enable_multi_turn_angle());

        for (auto& chassis_leveling_motor : chassis_leveling_motors_) {
            chassis_leveling_motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());
        }

        // filling_lifting_motor_[0].configure(
        //     device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());

        // filling_lifting_motor_[1].configure(
        //     device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());

        imu_.set_coordinate_mapping(
            [](double x, double y, double z) -> std::tuple<double, double, double> { return {y, -x, z}; });

        register_output("/dart/chassis/imu/pitch_angle", chassis_pitch_);
        register_output("/dart/chassis/imu/roll_angle", chassis_roll_);
        register_output("/dart/chassis/imu/pitch_rate", chassis_pitch_rate_);
        register_output("/dart/chassis/imu/roll_rate", chassis_roll_rate_);

        // limiting_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
        //     "/limiting/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg)
        //     {
        //         trigger_servo_calibrate_subscription_callback(
        //             filling_limiting_servo_, std::move(msg));
        //     });

        // referee
        register_output("/referee/serial", referee_serial_);
        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            start_transmit().uart0_transmit({.uart_data = std::span<const std::byte>{buffer, size}});
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
        yaw_motor_.update_status();
        // drive_belt_motors_[0].update_status();
        // drive_belt_motors_[1].update_status();
        // trigger_slider_motor.update_status();
        // filling_lifting_motor_[0].update_status();
        // filling_lifting_motor_[1].update_status();

        for (auto& chassis_leveling_motor : chassis_leveling_motors_) {
            chassis_leveling_motor.update_status();
        }

        imu_.update_status();
        imu_data_process(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3(), imu_.gx(), imu_.gy(), imu_.gz());
    }

    void command_update() {
        auto board = start_transmit();

        // Trigger servo: PWM via GPIO
        // board.gpio_analog_write(
        //     librmcs::spec::rmcs_board_pro::kGpioDescriptors[1],
        //     librmcs::data::GpioAnalogDataView{.value = trigger_servo_.generate_duty_cycle()});

        board.can1_transmit({
            .can_id = 0x200,
            .can_data =
                device::CanPacket8{
                    chassis_leveling_motors_[0].generate_command(),
                    chassis_leveling_motors_[1].generate_command(),
                    chassis_leveling_motors_[2].generate_command(),
                    chassis_leveling_motors_[3].generate_command(),
                }
                    .as_bytes(),
        });

        // DJI motors on CAN2 (yaw/force_screw: 0x200, belts: 0x1FF)
        board.can2_transmit({
            .can_id = 0x200,
            .can_data =
                device::CanPacket8{
                    yaw_motor_.generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                    // trigger_slider_motor.generate_command(),
                    // drive_belt_motors_[0].generate_command(),
                    // drive_belt_motors_[1].generate_command(),
                }
                    .as_bytes(),
        });

        // // LK4005 lifting motors on CAN3 (unicast per motor)
        // board.can3_transmit({
        //     .can_id = 0x141,
        //     .can_data = filling_lifting_motor_[0].generate_velocity_command().as_bytes(),
        // });
        // board.can3_transmit({
        //     .can_id = 0x145,
        //     .can_data = filling_lifting_motor_[1].generate_velocity_command().as_bytes(),
        // });

        // // Limiting servo on UART2 (only send when target changes)
        // if (!filling_limiting_servo_.calibrate_mode()) {
        //     uint16_t current_target = filling_limiting_servo_.get_target_angle();
        //     if (current_target != last_limiting_angle_) {
        //         size_t uart_data_length;
        //         auto command_buffer =
        //             filling_limiting_servo_.generate_runtime_command(uart_data_length);
        //         board.uart2_transmit(
        //             {.uart_data = std::span{command_buffer.get(), uart_data_length}});
        //         last_limiting_angle_ = current_target;
        //     }
        // }
    }

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        const auto can_id = data.can_id;
        if (can_id == 0x201) {
            chassis_leveling_motors_[0].store_status(data.can_data);
        } else if (can_id == 0x202) {
            chassis_leveling_motors_[1].store_status(data.can_data);
        } else if (can_id == 0x203) {
            chassis_leveling_motors_[2].store_status(data.can_data);
        } else if (can_id == 0x204) {
            chassis_leveling_motors_[3].store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        const auto can_id = data.can_id;
        if (can_id == 0x201) {
            yaw_motor_.store_status(data.can_data);
        }
        //  else if (can_id == 0x202) {
        //     trigger_slider_motor.store_status(data.can_data);
        // } else if (can_id == 0x203) {
        //     drive_belt_motors_[0].store_status(data.can_data);
        // } else if (can_id == 0x204) {
        //     drive_belt_motors_[1].store_status(data.can_data);
        // }
    }

    // void can3_receive_callback(const librmcs::data::CanDataView& data) override {
    //     if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
    //         return;

    //     const auto can_id = data.can_id;
    //     if (can_id == 0x141) {
    //         filling_lifting_motor_[0].store_status(data.can_data);
    //     } else if (can_id == 0x145) {
    //         filling_lifting_motor_[1].store_status(data.can_data);
    //     }
    // }

    void uart0_receive_callback(const librmcs::data::UartDataView& data) override {
        const auto* uart_data = data.uart_data.data();
        referee_ring_buffer_receive_.emplace_back_n(
            [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; }, data.uart_data.size());
    }

    // void uart2_receive_callback(const librmcs::data::UartDataView& data) override {
    //     const auto* raw_data = data.uart_data.data();
    //     const size_t length = data.uart_data.size();

    //     RCLCPP_DEBUG(
    //         logger_, "UART2 received: len=%zu [%s]", length,
    //         bytes_to_hex_string(raw_data, length).c_str());

    //     if (length < 3) {
    //         RCLCPP_WARN(logger_, "UART2 data too short: %zu bytes", length);
    //         return;
    //     }

    //     const uint8_t servo_id = static_cast<uint8_t>(raw_data[2]);

    //     // Only limiting servo remains on UART2
    //     if (servo_id == 0x03) {
    //         auto result =
    //             filling_limiting_servo_.calibrate_current_angle(logger_, raw_data, length);
    //         if (!result.first)
    //             RCLCPP_INFO(logger_, "calibrate: uart2 limiting data store failed");
    //     } else {
    //         RCLCPP_DEBUG(logger_, "UART2: unknown servo id 0x%02X", servo_id);
    //     }
    // }

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
            board.uart2_transmit({.uart_data = std::span{command_buffer.get(), command_length}});
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

    void imu_data_process(double q0, double q1, double q2, double q3, double wx, double wy, double wz) {
        double sin_pitch = 2.0 * (q0 * q2 - q3 * q1);
        sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);

        double pitch = std::asin(sin_pitch);
        double roll = std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

        *chassis_pitch_ = -pitch;
        *chassis_roll_ = roll;

        *chassis_pitch_rate_ = -std::cos(roll) * wy - std::sin(roll) * wz;
        *chassis_roll_rate_ = wx + std::sin(roll) * std::tan(pitch) * wy + std::cos(roll) * std::tan(pitch) * wz;
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

    device::DjiMotor yaw_motor_;
    // device::DjiMotor trigger_slider_motor;
    // device::DjiMotor drive_belt_motors_[2];       // Left,Right
    device::DjiMotor chassis_leveling_motors_[4]; // FL,FR,BL,BR

    // device::LkMotor filling_lifting_motor_[2];

    // device::PWMServo trigger_servo_;
    // device::TriggerServo filling_limiting_servo_;

    device::Dr16 dr16_;
    device::Bmi088 imu_;

    OutputInterface<double> chassis_pitch_;
    OutputInterface<double> chassis_roll_;
    OutputInterface<double> chassis_pitch_rate_;
    OutputInterface<double> chassis_roll_rate_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr limiting_calibrate_subscription_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    // uint16_t last_limiting_angle_ = 0xFFFF;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)
