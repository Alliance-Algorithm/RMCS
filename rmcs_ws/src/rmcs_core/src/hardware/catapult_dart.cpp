#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/force_sensor.hpp"
#include "librmcs/client/cboard.hpp"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include "hardware/device/force_sensor_runtime.hpp"
#include "hardware/device/trigger_servo.hpp"
#include <cstddef>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "filter/low_pass_filter.hpp"
#include <chrono>
#include <std_msgs/msg/int32.hpp>
#include <numeric>
#include <algorithm>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_core::hardware {

class QuaternionBayesFilter {
public:
    QuaternionBayesFilter(double confidence_ratio, double sample_rate)
        : alpha_(confidence_ratio / (confidence_ratio + 1.0))  
        , initialized_(false) {}

    Eigen::Quaterniond update(const Eigen::Quaterniond& measurement) {
        if (!initialized_) {
            belief_ = measurement;
            initialized_ = true;
            return belief_;
        }
        
        belief_ = belief_.slerp(1.0 - alpha_, measurement);
        return belief_;
    }
    
    void reset() { initialized_ = false; }
    bool isInitialized() const { return initialized_; }
    double getConfidence() const { return alpha_; }
    Eigen::Quaterniond getCurrentBelief() const { return belief_; }

private:
    double alpha_;  
    bool initialized_;
    Eigen::Quaterniond belief_;
};

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
        , pitch_angle_filter_(20.0, 1000.0)
        , pitch_velocity_filter_(20.0, 1000.0)
        , yaw_velocity_filter_(20.0, 1000.0)
        , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , imu_(1000, 0.2, 0.0)
        , pitch_motor_(
              *this, *dart_command_, "/dart/pitch_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , yaw_motor_(
              *this, *dart_command_, "/dart/yaw_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
        , force_control_motor_(
              *this, *dart_command_, "/dart/force_control_motor",
              device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
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
        
        register_output("/dart/pitch/angle", pitch_angle_);
        register_output("/tf", tf_);
        register_output("/imu/state/final_roll", final_roll);
        register_output("/imu/state/final_pitch", final_pitch);
        register_output("/imu/state/final_yaw", final_yaw);

        imu_sensitivity_ = this->get_parameter("imu_sensitivity").as_double();
        first_sample_spot_ = this->get_parameter("first_sample_spot").as_double();
        final_sample_spot_ = this->get_parameter("final_sample_spot").as_double();

        imu_sampler_initialize();
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        start_time_ = std::chrono::steady_clock::now();
        calibration_complete_ = false;

        trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_servo_calibrate_subscription_callback(std::move(msg));
            });
    }

    ~CatapultDart() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
 
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        drive_belt_motor_[0].update_status();
        drive_belt_motor_[1].update_status();
        force_control_motor_.update_status();
        force_sensor_.update_status();

        imu_.update_status();
        processImuData();
    }

    void command_update() {
        uint16_t can_commands[4];

        if (pub_time_count_++ > 100) {
            transmit_buffer_.add_can1_transmission(
                0x301, std::bit_cast<uint64_t>(force_sensor_.generate_command()));
            pub_time_count_ = 0;
        }

        can_commands[0] = pitch_motor_.generate_command();
        can_commands[1] = yaw_motor_.generate_command();
        can_commands[2] = force_control_motor_.generate_command();
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
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {

        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

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
        }
    }

    void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        x = static_cast<int16_t>(x / imu_sensitivity_);
        y = static_cast<int16_t>(y / imu_sensitivity_);
        z = static_cast<int16_t>(z / imu_sensitivity_);
        imu_.store_accelerometer_status(x, y, z);
    }
    
    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        x = static_cast<int16_t>(x / imu_sensitivity_);
        y = static_cast<int16_t>(y / imu_sensitivity_);
        z = static_cast<int16_t>(z / imu_sensitivity_);
        imu_.store_gyroscope_status(x, y, z);
    }

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

private:
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
    }

    rclcpp::Logger logger_;

    filter::LowPassFilter<1> pitch_angle_filter_;
    filter::LowPassFilter<1> pitch_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;

    bool calibration_complete_ = false;
    std::chrono::steady_clock::time_point start_time_;    


    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDart& robot)
            : dart_(robot) {}

        void update() override { dart_.command_update(); }

        CatapultDart& dart_;
    };
    std::shared_ptr<DartCommand> dart_command_;

    void imu_sampler_initialize() {
        start_time_ = std::chrono::steady_clock::now();
        calibration_complete_ = false;
        sample_counter_ = 0;
        quaternion_filter_.reset();
        quaternion_smoother_.reset();
        clearSamples();
    }

    void processImuData() {
        Eigen::Quaterniond raw_quaternion{
            imu_.q0() / imu_sensitivity_, 
            imu_.q1() / imu_sensitivity_, 
            imu_.q2() / imu_sensitivity_, 
            imu_.q3() / imu_sensitivity_
        };
        raw_quaternion.normalize();
        
        Eigen::Quaterniond bayes_quaternion = quaternion_filter_.update(raw_quaternion);
        
        Eigen::Quaterniond smoothed_quaternion = quaternion_smoother_.update(bayes_quaternion);
        
        Eigen::Vector3d smoothed_euler = quaternionToEuler(smoothed_quaternion);
        
        auto compensated = applyDriftCompensation(smoothed_euler);
        
        setOutputStates(compensated);
        
        setTfTransforms(compensated);
    }

    struct CompensatedResult { double roll, pitch, yaw; };

    CompensatedResult applyDriftCompensation(const Eigen::Vector3d& angles) {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
        
        if (!calibration_complete_) {
            collectCalibrationSamples(angles, elapsed_seconds);
            return {angles.x(), angles.y(), angles.z()};
        } else {
            double compensated_roll = angles.x() - roll_bias_;
            double compensated_pitch = angles.y() - pitch_bias_;
            double compensated_yaw = angles.z() - (yaw_drift_slope_ * elapsed_seconds + yaw_drift_offset_);

            compensated_yaw = normalizeAngle(compensated_yaw);
            compensated_pitch = normalizeAngle(compensated_pitch);
            compensated_roll = normalizeAngle(compensated_roll);

            return {compensated_roll, compensated_pitch, compensated_yaw};
        }
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    void collectCalibrationSamples(const Eigen::Vector3d& angles, double elapsed_seconds) {
        if (sample_counter_ % 100 == 0) {  // at 1000Hz，sampling 10 times per sec
            if (elapsed_seconds >= first_sample_spot_ && elapsed_seconds <= final_sample_spot_) {
                roll_samples_.push_back(angles.x());
                pitch_samples_.push_back(angles.y());
                yaw_samples_.push_back(angles.z());
                time_samples_.push_back(elapsed_seconds);
            }
        }
        sample_counter_++;

        if (elapsed_seconds >= final_sample_spot_ && !calibration_complete_) {
            calculateCompensationParameters();
            calibration_complete_ = true;
            RCLCPP_INFO(logger_, "IMU calibration complete with %zu samples", roll_samples_.size());
            clearSamples();
        }
    }

    void calculateCompensationParameters() {
        if (roll_samples_.empty()) return;

        roll_bias_ = std::accumulate(roll_samples_.begin(), roll_samples_.end(), 0.0) / roll_samples_.size();
        pitch_bias_ = std::accumulate(pitch_samples_.begin(), pitch_samples_.end(), 0.0) / pitch_samples_.size();
        
        calculateYawDriftCompensation();
        
        RCLCPP_INFO(logger_, "IMU Bias - Roll: %.6f, Pitch: %.6f, Yaw slope: %.6f", 
                   roll_bias_, pitch_bias_, yaw_drift_slope_);
    }

    void calculateYawDriftCompensation() {
        if (yaw_samples_.size() < 2) return;

        double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
        int n = yaw_samples_.size();
        
        for (size_t i = 0; i < n; ++i) {
            sum_x += time_samples_[i];
            sum_y += yaw_samples_[i];
            sum_xy += time_samples_[i] * yaw_samples_[i];
            sum_xx += time_samples_[i] * time_samples_[i];
        }
        
        double denominator = n * sum_xx - sum_x * sum_x;
        if (std::abs(denominator) > 1e-10) {
            yaw_drift_slope_ = (n * sum_xy - sum_x * sum_y) / denominator;
            yaw_drift_offset_ = (sum_y - yaw_drift_slope_ * sum_x) / n;
        }
    }

    void clearSamples() {
        roll_samples_.clear();
        pitch_samples_.clear();
        yaw_samples_.clear();
        time_samples_.clear();
    }

    void setOutputStates(const CompensatedResult& compensated) {
        *final_roll = compensated.roll;
        *final_pitch = compensated.pitch;
        *final_yaw = compensated.yaw;
    }

    void setTfTransforms(const CompensatedResult& compensated) {
        tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(compensated.yaw);
        publishTfTransforms(compensated);
    }

    void publishTfTransforms(const CompensatedResult& compensated) {
        auto now = this->get_clock()->now();
        
        auto create_transform = [&](const std::string& parent, const std::string& child, 
                                  const geometry_msgs::msg::Vector3& trans, 
                                  const geometry_msgs::msg::Quaternion& rot) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = parent;
            tf.child_frame_id = child;
            tf.transform.translation = trans;
            tf.transform.rotation = rot;
            return tf;
        };

        auto create_translation = [](double x, double y, double z) {
            geometry_msgs::msg::Vector3 t; 
            t.x = x; t.y = y; t.z = z; 
            return t;
        };

        auto create_rotation = [](double x, double y, double z, double w) {
            geometry_msgs::msg::Quaternion r; 
            r.x = x; r.y = y; r.z = z; r.w = w; 
            return r;
        };

        geometry_msgs::msg::Vector3 zero_trans = create_translation(0, 0, 0);
        geometry_msgs::msg::Vector3 pitch_trans = create_translation(0, 0, 0.05);
        
        Eigen::Quaterniond yaw_quaternion = 
        Eigen::AngleAxisd(compensated.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(compensated.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(compensated.roll, Eigen::Vector3d::UnitX());
        
        // base_link -> gimbal_center_link
        tf_broadcaster_->sendTransform(create_transform("base_link", "gimbal_center_link", 
            zero_trans, create_rotation(0, 0, 0, 1)));
        
        // gimbal_center_link -> yaw_link
        Eigen::AngleAxisd yaw_axis(compensated.yaw, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond yaw_only_quaternion(yaw_axis);
        tf_broadcaster_->sendTransform(create_transform("gimbal_center_link", "yaw_link", 
            zero_trans, create_rotation(yaw_only_quaternion.x(), yaw_only_quaternion.y(), 
                                                   yaw_only_quaternion.z(), yaw_only_quaternion.w())));
        
        // yaw_link -> pitch_link  
        Eigen::AngleAxisd pitch_axis(compensated.pitch, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond pitch_only_quaternion(pitch_axis);
        tf_broadcaster_->sendTransform(create_transform("yaw_link", "pitch_link", 
            pitch_trans, create_rotation(pitch_only_quaternion.x(), pitch_only_quaternion.y(), 
                                                    pitch_only_quaternion.z(), pitch_only_quaternion.w())));
        
        // pitch_link -> odom_imu
        tf_broadcaster_->sendTransform(create_transform("pitch_link", "odom_imu", 
            zero_trans, create_rotation(yaw_quaternion.x(), yaw_quaternion.y(), 
                                                   yaw_quaternion.z(), yaw_quaternion.w())));
        
        // world -> base_link
        tf_broadcaster_->sendTransform(create_transform("world", "base_link", 
            zero_trans, create_rotation(0, 0, 0, 1)));
    }

    static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
        Eigen::Matrix3d m = q.toRotationMatrix();
        return {std::atan2(m(2,1), m(2,2)), std::asin(-m(2,0)), std::atan2(m(1,0), m(0,0))};
    }

    double imu_sensitivity_;
    double first_sample_spot_;
    double final_sample_spot_;

    QuaternionBayesFilter quaternion_filter_{15.0, 1000.0};   
    QuaternionBayesFilter quaternion_smoother_{10.0, 1000.0}; 
    size_t sample_counter_ = 0;
    std::vector<double> roll_samples_, pitch_samples_, yaw_samples_, time_samples_;
    
    double roll_bias_ = 0.0, pitch_bias_ = 0.0, yaw_drift_slope_ = 0.0, yaw_drift_offset_ = 0.0;

    OutputInterface<rmcs_description::Tf> tf_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    device::Dr16 dr16_;
    device::Bmi088 imu_;
    device::DjiMotor pitch_motor_;
    device::DjiMotor yaw_motor_;
    device::DjiMotor force_control_motor_;
    device::DjiMotor drive_belt_motor_[2];

    device::ForceSensorRuntime force_sensor_;
    device::TriggerServo trigger_servo_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
    
    OutputInterface<double> pitch_angle_;
    OutputInterface<double> final_pitch;
    OutputInterface<double> final_roll;
    OutputInterface<double> final_yaw;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)