#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
<<<<<<< HEAD
#include "hardware/device/bmi088.hpp"
#include "hardware/device/force_sensor.hpp"
#include "librmcs/client/cboard.hpp"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
=======
#include "hardware/device/force_sensor_runtime.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/client/cboard.hpp"
#include <cstddef>
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
<<<<<<< HEAD
#include "filter/low_pass_filter.hpp"
#include <chrono>
=======
#include <std_msgs/msg/int32.hpp>
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e

namespace rmcs_core::hardware {

class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    CatapultDart()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
<<<<<<< HEAD
        , pitch_angle_filter_(20.0, 1000.0)
        , pitch_velocity_filter_(20.0, 1000.0)
        , yaw_velocity_filter_(20.0, 1000.0)
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , imu_(1000, 0.2, 0.0)
        , pitch_motor_(*this, *dart_command_, "/dart/pitch_motor", 
            device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19))
        , yaw_motor_(*this, *dart_command_, "/dart/yaw_motor", 
            device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19))
=======
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , pitch_motor_(
              *this, *dart_command_, "/dart/pitch_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , yaw_motor_(
              *this, *dart_command_, "/dart/yaw_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , force_control_motor_(
              *this, *dart_command_, "/dart/force_control_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
        , drive_belt_motor_(
              {*this, *dart_command_, "/dart/drive_belt/left",
               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.)},
              {*this, *dart_command_, "/dart/drive_belt/right",
               device::DjiMotor::Config{device::DjiMotor::Type::M3508}
                   .set_reduction_ratio(19.)
                   .set_reversed()})
        , force_sensor_(*this)
        , trigger_servo_(*dart_command_, "/dart/trigger_servo")
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
<<<<<<< HEAD
        
        register_output("/dart/pitch/angle", pitch_angle_);
        // register_output("/dart/pitch/velocity", pitch_velocity_output_);
        // register_output("/dart/yaw/velocity", yaw_velocity_output_);
        
        imu_sensitivity_ = this->get_parameter("imu_sensitivity").as_double();
        calibration_start_time_ = this->get_parameter("calibration_start_time").as_double();
        calibration_end_time_ = this->get_parameter("calibration_end_time").as_double();
        
        imu_.set_coordinate_mapping([](double x, double y, double z) {
            return std::make_tuple(x, y, z);
        });

        start_time_ = std::chrono::steady_clock::now();
        calibration_complete_ = false;
=======

        trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_servo_calibrate_subscription_callback(std::move(msg));
            });
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
    }

    ~CatapultDart() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
<<<<<<< HEAD
        imu_.update_status();
        
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
        
        double raw_pitch_angle = extractPitchFromQuaternion(
            imu_.q0() / imu_sensitivity_, 
            imu_.q1() / imu_sensitivity_, 
            imu_.q2() / imu_sensitivity_,
            imu_.q3() / imu_sensitivity_
        );
        
        double raw_pitch_velocity = imu_.gy();
        double raw_yaw_velocity = imu_.gz();
        
        if (!calibration_complete_) {
            if (elapsed_seconds >= calibration_start_time_ && elapsed_seconds <= calibration_end_time_) {
                pitch_angle_sum_ += raw_pitch_angle;
                pitch_velocity_sum_ += raw_pitch_velocity;
                yaw_velocity_sum_ += raw_yaw_velocity;
                sample_count_++;
            } else if (elapsed_seconds > calibration_end_time_) {
                if (sample_count_ > 0) {
                    pitch_angle_bias_ = pitch_angle_sum_ / sample_count_;
                    pitch_velocity_bias_ = pitch_velocity_sum_ / sample_count_;
                    yaw_velocity_bias_ = yaw_velocity_sum_ / sample_count_;
                    
                    RCLCPP_INFO(logger_, "Calibration complete: pitch_angle_bias=%.6f, pitch_vel_bias=%.6f, yaw_vel_bias=%.6f (samples=%zu)",
                              pitch_angle_bias_, pitch_velocity_bias_, yaw_velocity_bias_, sample_count_);
                }
                calibration_complete_ = true;
            }
            
            *pitch_angle_ = 0.0;
            *pitch_velocity_output_ = 0.0;
            *yaw_velocity_output_ = 0.0;
        } 
        else {
            double corrected_pitch_angle = raw_pitch_angle - pitch_angle_bias_;
            double corrected_pitch_velocity = raw_pitch_velocity - pitch_velocity_bias_;
            double corrected_yaw_velocity = raw_yaw_velocity - yaw_velocity_bias_;
            
            *pitch_angle_ = pitch_angle_filter_.update(corrected_pitch_angle);
            *pitch_velocity_output_ = pitch_velocity_filter_.update(corrected_pitch_velocity);
            *yaw_velocity_output_ = yaw_velocity_filter_.update(corrected_yaw_velocity);
        }
        
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        
=======
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        drive_belt_motor_[0].update_status();
        drive_belt_motor_[1].update_status();
        force_control_motor_.update_status();
        force_sensor_.update_status();
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
    }

    void command_update() {
        uint16_t can_commands[4];

        if (pub_time_count_++ > 100) {
            transmit_buffer_.add_can1_transmission(
                0x301, std::bit_cast<uint64_t>(force_sensor_.generate_command()));
            pub_time_count_ = 0;
        }

<<<<<<< HEAD
        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        
        can_commands[0] = pitch_motor_.generate_command();
        can_commands[1] = yaw_motor_.generate_command();
        can_commands[2] = 0;
=======
        can_commands[0] = pitch_motor_.generate_command();
        can_commands[1] = yaw_motor_.generate_command();
        can_commands[2] = force_control_motor_.generate_command();
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = drive_belt_motor_[0].generate_command();
        can_commands[1] = drive_belt_motor_[1].generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

        if (!trigger_servo_.calibrate_mode()) {
            size_t uart_data_length;
            std::unique_ptr<std::byte[]> command_buffer =
                trigger_servo_.generate_runtime_command(uart_data_length);
            const auto trigger_servo_uart_data_ptr = command_buffer.get();
            transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, uart_data_length);

            // std::string hex_string = bytes_to_hex_string(command_buffer.get(), uart_data_length);
            // RCLCPP_INFO(
            //     this->get_logger(), "UART2(length: %zu): [ %s ]", uart_data_length,
            //     hex_string.c_str());
        }

        transmit_buffer_.trigger_transmission();
    }
    int pub_time_count_ = 0;

protected:
<<<<<<< HEAD
    void can2_receive_callback(
=======
    void can1_receive_callback(
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;
<<<<<<< HEAD
        if (can_id == 0x201) {
            pitch_motor_.store_status(can_data);
        } else if (can_id == 0x202) {
            yaw_motor_.store_status(can_data);

=======

        if (can_id == 0x302) {
            force_sensor_.store_status(can_data);
        }
    }

    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
            pitch_motor_.store_status(can_data);
        } else if (can_id == 0x202) {
            yaw_motor_.store_status(can_data);
        } else if (can_id == 0x203) {
            force_control_motor_.store_status(can_data);
        } else if (can_id == 0x205) {
            drive_belt_motor_[0].store_status(can_data);
        } else if (can_id == 0x206) {
            drive_belt_motor_[1].store_status(can_data);
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
        }
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_accelerometer_status(x, y, z);
    }
    
    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_gyroscope_status(x, y, z);
    }

<<<<<<< HEAD
    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
=======
    void uart2_receive_callback(const std::byte* data, uint8_t length) override {
        bool success = trigger_servo_.calibrate_current_angle(logger_, data, length);
        if (!success) {
            RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
        }

        // std::string hex_string = bytes_to_hex_string(data, length);
        // RCLCPP_INFO(this->get_logger(), "UART2(length: %hhu): [ %s ]", length,
        // hex_string.c_str());
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e

    }
private:
<<<<<<< HEAD
    static double extractPitchFromQuaternion(double q0, double q1, double q2, double q3) {
        return std::asin(-2.0 * (q1 * q3 - q0 * q2));
=======
    void trigger_servo_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        /*
        标定命令格式：
        ros2 topic pub --rate 2 --times 5 /trigger/calibrate std_msgs/msg/Int32 "{'data':0}"
        替换data值就行
        */
        trigger_servo_.set_calibrate_mode(msg->data);

        std::unique_ptr<std::byte[]> command_buffer;
        size_t command_length = 0;
        if (msg->data == 0) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
        } else if (msg->data == 1) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
        } else if (msg->data == 2) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
        } else if (msg->data == 3) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
        } else if (msg->data == 4) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
        } else if (msg->data == 5) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
        } else if (msg->data == 6) {
            command_buffer = trigger_servo_.generate_calibrate_command(
                device::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
        }

        const auto trigger_servo_uart_data_ptr = command_buffer.get();
        transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, command_length);

        std::string hex_string = bytes_to_hex_string(command_buffer.get(), command_length);
        RCLCPP_INFO(
            this->get_logger(), "UART2 Pub: (length=%zu)[ %s ]", command_length,
            hex_string.c_str());
    }

    static std::string bytes_to_hex_string(const std::byte* data, size_t size) {
        if (!data || size == 0) {
            return "[]";
        }

        std::stringstream ss;
        ss << std::hex << std::uppercase << std::setfill('0');

        for (size_t i = 0; i < size; ++i) {
            ss << std::setw(2) << static_cast<int>(data[i]) << " ";
        }

        std::string result = ss.str();
        if (!result.empty() && result.back() == ' ') {
            result.pop_back();
        }
        return result;
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
    }

    rclcpp::Logger logger_;

    filter::LowPassFilter<1> pitch_angle_filter_;
    filter::LowPassFilter<1> pitch_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;

    bool calibration_complete_ = false;
    std::chrono::steady_clock::time_point start_time_;
    double calibration_start_time_;
    double calibration_end_time_;    
    
    double pitch_angle_sum_ = 0.0;
    double pitch_velocity_sum_ = 0.0;
    double yaw_velocity_sum_ = 0.0;
    size_t sample_count_;
    
    double pitch_angle_bias_ = 0.0;
    double pitch_velocity_bias_ = 0.0;
    double yaw_velocity_bias_ = 0.0;

    class DartCommand : public rmcs_executor::Component {
    public:
<<<<<<< HEAD
        explicit DartCommand(CatapultDart& dart)
            : dart_(dart) {}
=======
        explicit DartCommand(CatapultDart& robot)
            : dart_(robot) {}
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e

        void update() override { dart_.command_update(); }

        CatapultDart& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    device::Dr16 dr16_;
    device::Bmi088 imu_;
    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor force_control_motor_;
<<<<<<< HEAD
    device::ForceSensor force_sensor_;
=======
    device::DjiMotor drive_belt_motor_[2];

    device::ForceSensorRuntime force_sensor_;
    device::TriggerServo trigger_servo_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;
>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;

    double imu_sensitivity_;
    
    OutputInterface<double> pitch_angle_;
    OutputInterface<double> pitch_velocity_output_;
    OutputInterface<double> yaw_velocity_output_;
};
<<<<<<< HEAD

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
=======
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

>>>>>>> b3b6fa5723ff46ff682219460cabef62d2fd622e
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)