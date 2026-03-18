#include <chrono>
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
#include <vector>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "filter/low_pass_filter.hpp"
#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/force_sensor.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/agent/c_board.hpp"

/*
CatapultDartV3Lk — catapult_dart_v3_full 的变体
升降电机替换为瓴控4005 (LkMotor MG4005Ei10)，挂 CAN1 (0x141 左, 0x145 右)。
限位舵机保留 TriggerServo (UART2, ID=0x03)。

升降电机接口 (double, rad):
  输出: /dart/lifting_left/angle, /dart/lifting_left/velocity 等
  输入: /dart/lifting_left/control_velocity (由 DartFilling 写入)
*/

namespace rmcs_core::hardware {

class CatapultDartV3Lk
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    CatapultDartV3Lk()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{get_parameter("serial_filter").as_string()}
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
        , limiting_servo_{*dart_command_, "/dart/limiting_servo", LIMITING_SERVO_ID}
        , lifting_left_motor_{*this, *dart_command_, "/dart/lifting_left"}
        , lifting_right_motor_{*this, *dart_command_, "/dart/lifting_right"}
        , dr16_{*this}
        , imu_{1000, 0.2, 0.0} {

        pitch_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        yaw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        force_screw_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        drive_belt_motor_left_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}.set_reduction_ratio(19.));
        drive_belt_motor_right_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                .set_reversed()
                .set_reduction_ratio(19.));

        lifting_left_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());
        lifting_right_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}.enable_multi_turn_angle());

        first_sample_spot_ = this->get_parameter("first_sample_spot").as_double();
        final_sample_spot_ = this->get_parameter("final_sample_spot").as_double();

        setup_imu_coordinate_mapping();
        start_time_ = std::chrono::steady_clock::now();
        yaw_drift_coefficient_ = 0.0;
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        register_output("/tf", tf_);
        register_output("/imu/catapult_pitch_angle", catapult_pitch_angle_);
        register_output("/imu/catapult_roll_angle", catapult_roll_angle_);
        register_output("/imu/catapult_yaw_angle", catapult_yaw_angle_);

        limiting_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/limiting/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                trigger_servo_calibrate_subscription_callback(limiting_servo_, std::move(msg));
            });

        force_sensor_calibrate_ = create_subscription<std_msgs::msg::Int32>(
            "/force_sensor/calibrate", rclcpp::QoS{0},
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

    CatapultDartV3Lk(const CatapultDartV3Lk&) = delete;
    CatapultDartV3Lk& operator=(const CatapultDartV3Lk&) = delete;
    CatapultDartV3Lk(CatapultDartV3Lk&&) = delete;
    CatapultDartV3Lk& operator=(CatapultDartV3Lk&&) = delete;
    ~CatapultDartV3Lk() override = default;

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
    }

    void command_update() {
        auto board = start_transmit();

        // Trigger servo: PWM via GPIO
        board.gpio_analog_transmit({.channel = 1, .value = trigger_servo_.generate_duty_cycle()});

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

        // LK4005 lifting motors on CAN1 (unicast per motor)
        // board.can1_transmit({
        //     .can_id   = LK_LIFTING_LEFT_ID,
        //     .can_data = lifting_left_motor_.generate_velocity_command().as_bytes(),
        // });
        // board.can1_transmit({
        //     .can_id   = LK_LIFTING_RIGHT_ID,
        //     .can_data = lifting_right_motor_.generate_velocity_command().as_bytes(),
        // });

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
            force_sensor_frame_count_++;
            if (force_sensor_frame_count_ % 200 == 1) {
                RCLCPP_INFO(
                    logger_, "[ForceSensor CAN1:0x302] raw(%zu): %s", data.can_data.size(),
                    bytes_to_hex_string(data.can_data.data(), data.can_data.size()).c_str());
            }
            force_sensor_.store_status(data.can_data);
        } else if (can_id == LK_LIFTING_LEFT_ID) {
            lifting_left_motor_.store_status(data.can_data);
        } else if (can_id == LK_LIFTING_RIGHT_ID) {
            lifting_right_motor_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        const auto can_id = data.can_id;

        if (can_id != 0x201 && can_id != 0x202 && can_id != 0x203 && can_id != 0x205
            && can_id != 0x206) {
            can2_unknown_count_++;
            if (can2_unknown_count_ % 50 == 1) {
                RCLCPP_INFO(
                    logger_, "[CAN2 unknown] id=0x%03X raw(%zu): %s", can_id, data.can_data.size(),
                    bytes_to_hex_string(data.can_data.data(), data.can_data.size()).c_str());
            }
        }

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
        if (servo_id == LIMITING_SERVO_ID) {
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
    static constexpr uint8_t LIMITING_SERVO_ID = 0x03;
    static constexpr uint32_t LK_LIFTING_LEFT_ID = 0x141;
    static constexpr uint32_t LK_LIFTING_RIGHT_ID = 0x145;

    void force_sensor_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
        auto board = start_transmit();
        board.can1_transmit({
            .can_id = 0x201,
            .can_data = device::CanPacket8{0}.as_bytes(),
        });
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
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();

        Eigen::Quaterniond imu_quat(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3());
        Eigen::Matrix3d rot = imu_quat.toRotationMatrix();

        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = std::asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));

        double t_roll = -roll_filter_.update(roll);
        double t_pitch = pitch_filter_.update(pitch);
        double t_yaw = -yaw_filter_.update(yaw);

        if (elapsed_seconds >= first_sample_spot_ && elapsed_seconds <= final_sample_spot_) {
            yaw_samples_.push_back(t_yaw);
            time_samples_.push_back(elapsed_seconds);

            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            double n = static_cast<double>(yaw_samples_.size());
            for (size_t i = 0; i < yaw_samples_.size(); ++i) {
                sum_x += time_samples_[i];
                sum_y += yaw_samples_[i];
                sum_xy += time_samples_[i] * yaw_samples_[i];
                sum_xx += time_samples_[i] * time_samples_[i];
            }
            double denominator = n * sum_xx - sum_x * sum_x;
            if (std::abs(denominator) > 1e-10)
                yaw_drift_coefficient_ = (n * sum_xy - sum_x * sum_y) / denominator;
        }

        t_yaw -= (yaw_drift_coefficient_ + 0.000512) * elapsed_seconds;
        publishTfTransforms(t_roll, t_pitch, t_yaw);

        *catapult_roll_angle_ = (t_roll / M_PI) * 180.0;
        *catapult_pitch_angle_ = (t_pitch / M_PI) * 180.0;
        *catapult_yaw_angle_ = (t_yaw / M_PI) * 180.0;
    }

    void publishTfTransforms(double roll, double pitch, double yaw) {
        auto now = this->get_clock()->now();

        auto make_tf = [&](const std::string& parent, const std::string& child,
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

        auto make_vec = [](double x, double y, double z) {
            geometry_msgs::msg::Vector3 v;
            v.x = x;
            v.y = y;
            v.z = z;
            return v;
        };

        auto make_quat = [](double x, double y, double z, double w) {
            geometry_msgs::msg::Quaternion q;
            q.x = x;
            q.y = y;
            q.z = z;
            q.w = w;
            return q;
        };

        Eigen::Quaterniond roll_q(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond pitch_q(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond yaw_q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond combined = yaw_q * pitch_q * roll_q;

        auto zero = make_vec(0, 0, 0);
        auto p_trans = make_vec(0, 0, 0.05);
        auto id_rot = make_quat(0, 0, 0, 1);

        tf_broadcaster_->sendTransform(make_tf("base_link", "gimbal_center_link", zero, id_rot));
        tf_broadcaster_->sendTransform(make_tf(
            "gimbal_center_link", "yaw_link", zero,
            make_quat(yaw_q.x(), yaw_q.y(), yaw_q.z(), yaw_q.w())));
        tf_broadcaster_->sendTransform(make_tf(
            "yaw_link", "pitch_link", p_trans,
            make_quat(pitch_q.x(), pitch_q.y(), pitch_q.z(), pitch_q.w())));
        tf_broadcaster_->sendTransform(make_tf(
            "pitch_link", "odom_imu", zero,
            make_quat(combined.x(), combined.y(), combined.z(), combined.w())));
        tf_broadcaster_->sendTransform(make_tf("world", "base_link", zero, id_rot));
    }

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDartV3Lk& dart)
            : dart_(dart) {}
        void update() override { dart_.command_update(); }

    private:
        CatapultDartV3Lk& dart_;
    };

    std::shared_ptr<DartCommand> dart_command_;
    rclcpp::Logger logger_;

    filter::LowPassFilter<1> pitch_angle_filter_;
    filter::LowPassFilter<1> pitch_velocity_filter_;
    filter::LowPassFilter<1> yaw_velocity_filter_;
    filter::LowPassFilter<1> yaw_filter_;
    filter::LowPassFilter<1> pitch_filter_;
    filter::LowPassFilter<1> roll_filter_;

    std::chrono::steady_clock::time_point start_time_;

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

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<double> catapult_pitch_angle_;
    OutputInterface<double> catapult_roll_angle_;
    OutputInterface<double> catapult_yaw_angle_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr limiting_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr force_sensor_calibrate_;

    std::mutex referee_mutex_;
    std::deque<std::byte> referee_ring_buffer_receive_;
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    int pub_time_count_ = 0;
    int force_sensor_frame_count_ = 0;
    int can2_unknown_count_ = 0;

    double first_sample_spot_ = 1.0;
    double final_sample_spot_ = 4.0;
    double yaw_drift_coefficient_ = 0.0;
    std::vector<double> yaw_samples_;
    std::vector<double> time_samples_;

    uint16_t last_limiting_angle_ = 0xFFFF;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDartV3Lk, rmcs_executor::Component)
