#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
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
#include "hardware/device/pwm_servo.hpp"
#include "librmcs/agent/c_board.hpp"

namespace rmcs_core::hardware {

class CatapultDartV3Full
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    CatapultDartV3Full()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
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

        force_sensor_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/force_sensor/calibrate", rclcpp::QoS{0},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                if (msg->data == 0) {
                    RCLCPP_INFO(logger_, "Force sensor zero calibration command received.");
                    auto board = start_transmit();
                    board.can1_transmit({
                        .can_id   = 0x201,
                        .can_data = device::CanPacket8{device::ForceSensor::generate_zero_calibration_command()}.as_bytes(),
                    });
                } else {
                    RCLCPP_WARN(logger_, "Unknown force sensor calibration command: %d", msg->data);
                }
            }
        );

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
            board.uart1_transmit({.uart_data = std::span{buffer, size}});
            return size;
        };
    }

    CatapultDartV3Full(const CatapultDartV3Full&) = delete;
    CatapultDartV3Full& operator=(const CatapultDartV3Full&) = delete;
    CatapultDartV3Full(CatapultDartV3Full&&) = delete;
    CatapultDartV3Full& operator=(CatapultDartV3Full&&) = delete;

    ~CatapultDartV3Full() override = default;

    void update() override {
        dr16_.update_status();
        pitch_motor_.update_status();
        yaw_motor_.update_status();
        drive_belt_motor_left_.update_status();
        drive_belt_motor_right_.update_status();
        force_screw_motor_.update_status();
        force_sensor_.update_status();
        imu_.update_status();
        processImuData();
    }

    void command_update() {
        auto board = start_transmit();

        board.gpio_analog_transmit({.channel = 1, .value = trigger_servo_.generate_duty_cycle()});

        if (pub_time_count_++ > 100) {
            board.can1_transmit({
                .can_id = 0x301,
                .can_data = device::CanPacket8::PaddingQuarter{}.as_bytes(),
            });
            pub_time_count_ = 0;
        }

        board.can2_transmit({
            .can_id   = 0x200,
            .can_data =
                device::CanPacket8{
                    pitch_motor_.generate_command(),
                    yaw_motor_.generate_command(),
                    force_screw_motor_.generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                }.as_bytes(),

        });

        board.can2_transmit({
            .can_id   = 0x1FF,
            .can_data =
                device::CanPacket8{
                    drive_belt_motor_left_.generate_command(),
                    drive_belt_motor_right_.generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                    device::CanPacket8::PaddingQuarter{},
                }.as_bytes(),
        });
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
                    logger_, "[ForceSensor CAN2:0x302] raw(%zu): %s",
                    data.can_data.size(),
                    bytes_to_hex_string(data.can_data.data(), data.can_data.size()).c_str());
            }
            force_sensor_.store_status(data.can_data);
        }
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        const auto can_id = data.can_id;

        // Log every unknown CAN2 ID (throttled per ID) to find force sensor response ID
        if (can_id != 0x201 && can_id != 0x202 && can_id != 0x203
            && can_id != 0x205 && can_id != 0x206) {
            can2_unknown_count_++;
            if (can2_unknown_count_ % 50 == 1) {
                RCLCPP_INFO(
                    logger_, "[CAN2 unknown] id=0x%03X raw(%zu): %s",
                    can_id, data.can_data.size(),
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
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(
        const librmcs::data::AccelerometerDataView& data) override {
        imu_.store_accelerometer_status(data.x, data.y, data.z);
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        imu_.store_gyroscope_status(data.x, data.y, data.z);
    }

private:
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
        double elapsed_seconds =
            std::chrono::duration<double>(current_time - start_time_).count();

        Eigen::Quaterniond imu_quat(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3());
        Eigen::Matrix3d rot = imu_quat.toRotationMatrix();

        double roll  = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = std::asin(-rot(2, 0));
        double yaw   = std::atan2(rot(1, 0), rot(0, 0));

        double t_roll  = -roll_filter_.update(roll);
        double t_pitch =  pitch_filter_.update(pitch);
        double t_yaw   = -yaw_filter_.update(yaw);

        if (elapsed_seconds >= first_sample_spot_ && elapsed_seconds <= final_sample_spot_) {
            yaw_samples_.push_back(t_yaw);
            time_samples_.push_back(elapsed_seconds);

            double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
            double n = static_cast<double>(yaw_samples_.size());
            for (size_t i = 0; i < yaw_samples_.size(); ++i) {
                sum_x  += time_samples_[i];
                sum_y  += yaw_samples_[i];
                sum_xy += time_samples_[i] * yaw_samples_[i];
                sum_xx += time_samples_[i] * time_samples_[i];
            }
            double denominator = n * sum_xx - sum_x * sum_x;
            if (std::abs(denominator) > 1e-10)
                yaw_drift_coefficient_ = (n * sum_xy - sum_x * sum_y) / denominator;
        }

        t_yaw -= (yaw_drift_coefficient_ + 0.000512) * elapsed_seconds;
        publishTfTransforms(t_roll, t_pitch, t_yaw);

        *catapult_roll_angle_  = (t_roll  / M_PI) * 180.0;
        *catapult_pitch_angle_ = (t_pitch / M_PI) * 180.0;
        *catapult_yaw_angle_   = (t_yaw   / M_PI) * 180.0;
    }

    void publishTfTransforms(double roll, double pitch, double yaw) {
        auto now = this->get_clock()->now();

        auto make_tf =
            [&](const std::string& parent, const std::string& child,
                const geometry_msgs::msg::Vector3& trans,
                const geometry_msgs::msg::Quaternion& rot) {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp    = now;
                tf.header.frame_id = parent;
                tf.child_frame_id  = child;
                tf.transform.translation = trans;
                tf.transform.rotation    = rot;
                return tf;
            };

        auto make_vec = [](double x, double y, double z) {
            geometry_msgs::msg::Vector3 v;
            v.x = x; v.y = y; v.z = z;
            return v;
        };

        auto make_quat = [](double x, double y, double z, double w) {
            geometry_msgs::msg::Quaternion q;
            q.x = x; q.y = y; q.z = z; q.w = w;
            return q;
        };

        Eigen::Quaterniond roll_q(Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond pitch_q(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond yaw_q(Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond combined = yaw_q * pitch_q * roll_q;

        auto zero    = make_vec(0, 0, 0);
        auto p_trans = make_vec(0, 0, 0.05);
        auto id_rot  = make_quat(0, 0, 0, 1);

        tf_broadcaster_->sendTransform(
            make_tf("base_link", "gimbal_center_link", zero, id_rot));
        tf_broadcaster_->sendTransform(make_tf(
            "gimbal_center_link", "yaw_link", zero,
            make_quat(yaw_q.x(), yaw_q.y(), yaw_q.z(), yaw_q.w())));
        tf_broadcaster_->sendTransform(make_tf(
            "yaw_link", "pitch_link", p_trans,
            make_quat(pitch_q.x(), pitch_q.y(), pitch_q.z(), pitch_q.w())));
        tf_broadcaster_->sendTransform(make_tf(
            "pitch_link", "odom_imu", zero,
            make_quat(combined.x(), combined.y(), combined.z(), combined.w())));
        tf_broadcaster_->sendTransform(
            make_tf("world", "base_link", zero, id_rot));
    }

    class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDartV3Full& dart)
            : dart_(dart) {}
        void update() override { dart_.command_update();}

    private:
        CatapultDartV3Full& dart_;
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

    device::Dr16 dr16_;
    device::Bmi088 imu_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<double> catapult_pitch_angle_;
    OutputInterface<double> catapult_roll_angle_;
    OutputInterface<double> catapult_yaw_angle_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr force_sensor_calibrate_subscription_;

    std::mutex referee_mutex_;
    std::deque<std::byte> referee_ring_buffer_receive_;
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

    int pub_time_count_ = 0;
    int force_sensor_frame_count_ = 0;
    int can2_unknown_count_ = 0;

    double first_sample_spot_     = 1.0;
    double final_sample_spot_     = 4.0;
    double yaw_drift_coefficient_ = 0.0;
    std::vector<double> yaw_samples_;
    std::vector<double> time_samples_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDartV3Full, rmcs_executor::Component)
