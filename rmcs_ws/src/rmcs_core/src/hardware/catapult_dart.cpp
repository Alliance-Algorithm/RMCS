#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/force_sensor.hpp"
#include "librmcs/client/cboard.hpp"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "filter/low_pass_filter.hpp"
#include <chrono>

namespace rmcs_core::hardware {

class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {
public:
    CatapultDart()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
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
        , drive_belt_motor_(
              {*this, *dart_command_, "/dart/drive_belt/left"}, {*this, *dart_command_, "/dart/drive_belt/right"})
        , force_control_motor_(*this, *dart_command_, "/dart/force_control_motor")
        , force_sensor_(*this)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {
        
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
    }

    ~CatapultDart() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
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
        
    }

    void command_update() {
        uint16_t can_commands[4];

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

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
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

protected:
    void can2_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;
        if (can_id == 0x201) {
            pitch_motor_.store_status(can_data);
        } else if (can_id == 0x202) {
            yaw_motor_.store_status(can_data);

        }
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_accelerometer_status(x, y, z);
    }
    
    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        imu_.store_gyroscope_status(x, y, z);
    }

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);

    }
private:
    static double extractPitchFromQuaternion(double q0, double q1, double q2, double q3) {
        return std::asin(-2.0 * (q1 * q3 - q0 * q2));
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
        explicit DartCommand(CatapultDart& dart)
            : dart_(dart) {}

        void update() override { dart_.command_update(); }

        CatapultDart& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    device::Dr16 dr16_;
    device::Bmi088 imu_;
    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor drive_belt_motor_[2];
    device::DjiMotor force_control_motor_;
    device::ForceSensor force_sensor_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;

    double imu_sensitivity_;
    
    OutputInterface<double> pitch_angle_;
    OutputInterface<double> pitch_velocity_output_;
    OutputInterface<double> yaw_velocity_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)