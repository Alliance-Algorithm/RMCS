// #include "librmcs/client/cboard.hpp"
// #include "hardware/device/dji_motor.hpp"
// #include "hardware/device/dr16.hpp"
// #include "hardware/device/bmi088.hpp"
// #include "hardware/device/force_sensor_runtime.hpp"
// #include "hardware/device/trigger_servo.hpp"
// #include <cmath>
// #include <cstddef>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include "filter/low_pass_filter.hpp"
// #include <chrono>
// #include <std_msgs/msg/int32.hpp>
// #include <eigen3/Eigen/src/Geometry/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_description/tf_description.hpp>

// namespace rmcs_core::hardware {

// class CatapultDart
//     : public rmcs_executor::Component
//     , public rclcpp::Node
//     , private librmcs::client::CBoard {
// public:
//     CatapultDart()
//         : Node{
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
//         , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
//         , logger_(get_logger())
//         , pitch_angle_filter_(20.0, 1000.0)
//         , pitch_velocity_filter_(20.0, 1000.0)
//         , yaw_velocity_filter_(20.0, 1000.0)
//         , yaw_filter_(5.0, 1000.0)
//         , pitch_filter_(10.0, 1000.0)
//         , roll_filter_(10.0, 1000.0)
//         , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command", *this))
//         , dr16_(*this)
//         , imu_(1000, 0.2, 0.0)
//         , pitch_motor_(
//               *this, *dart_command_, "/dart/pitch_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , yaw_motor_(
//               *this, *dart_command_, "/dart/yaw_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , force_control_motor_(
//               *this, *dart_command_, "/dart/force_control_motor",
//               device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.))
//         , drive_belt_motor_(
//               {*this, *dart_command_, "/dart/drive_belt/left",
//                device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reduction_ratio(19.)},
//               {*this, *dart_command_, "/dart/drive_belt/right",
//                device::DjiMotor::Config{device::DjiMotor::Type::M3508}
//                    .set_reduction_ratio(19.)
//                    .set_reversed()})
//         , force_sensor_(*this)
//         , trigger_servo_(*dart_command_, "/dart/trigger_servo")
//         , transmit_buffer_(*this, 32)
//         , event_thread_([this]() { handle_events(); }) {
        
//         // Register outputs for TF and IMU state
//         register_output("/dart/pitch/angle", pitch_angle_);
//         register_output("/tf", tf_);
//         register_output("/imu/state/final_roll", final_roll);
//         register_output("/imu/state/final_pitch", final_pitch);
//         register_output("/imu/state/final_yaw", final_yaw);

//         // Debug outputs for raw angles from IMU (no transformation)
//         register_output("/imu/debug/raw_roll", debug_raw_roll_);
//         register_output("/imu/debug/raw_pitch", debug_raw_pitch_);
//         register_output("/imu/debug/raw_yaw", debug_raw_yaw_);
        
//         // Debug outputs for transformed angles (unnormalized and normalized)
//         register_output("/imu/debug/transformed_roll_unnorm", debug_transformed_roll_unnorm_);
//         register_output("/imu/debug/transformed_pitch_unnorm", debug_transformed_pitch_unnorm_);
//         register_output("/imu/debug/transformed_yaw_unnorm", debug_transformed_yaw_unnorm_);
        
//         register_output("/imu/debug/transformed_roll_norm", debug_transformed_roll_norm_);
//         register_output("/imu/debug/transformed_pitch_norm", debug_transformed_pitch_norm_);
//         register_output("/imu/debug/transformed_yaw_norm", debug_transformed_yaw_norm_);
        
//         // Additional debug outputs for drift compensation
//         register_output("/imu/debug/drift_coefficient", debug_drift_coefficient_);
//         register_output("/imu/debug/calibration_complete", debug_calibration_complete_);
//         register_output("/imu/debug/compensation_applied", debug_compensation_applied_);

//         // Get IMU parameters
//         imu_sensitivity_ = this->get_parameter("imu_sensitivity").as_double();
//         first_sample_spot_ = this->get_parameter("first_sample_spot").as_double();
//         final_sample_spot_ = this->get_parameter("final_sample_spot").as_double();
        
//         // Get initial offset parameters (default to 0.0)
//         this->declare_parameter("roll_offset", 0.0);
//         this->declare_parameter("pitch_offset", 0.0);
//         this->declare_parameter("yaw_offset", 0.0);
        
//         roll_offset_ = this->get_parameter("roll_offset").as_double();
//         pitch_offset_ = this->get_parameter("pitch_offset").as_double();
//         yaw_offset_ = this->get_parameter("yaw_offset").as_double();

//         setup_imu_coordinate_mapping();
//         imu_sampler_initialize();
//         tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//         start_time_ = std::chrono::steady_clock::now();

//         // Create subscription for trigger calibration
//         trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(std::move(msg));
//             });
//     }

//     ~CatapultDart() override {
//         stop_handling_events();
//         event_thread_.join();
//     }

//     void update() override {
//         dr16_.update_status();
 
//         pitch_motor_.update_status();
//         yaw_motor_.update_status();
//         drive_belt_motor_[0].update_status();
//         drive_belt_motor_[1].update_status();
//         force_control_motor_.update_status();
//         force_sensor_.update_status();

//         imu_.update_status();
//         processImuData();
//     }

//     void command_update() {
//         uint16_t can_commands[4];

//         if (pub_time_count_++ > 100) {
//             transmit_buffer_.add_can1_transmission(
//                 0x301, std::bit_cast<uint64_t>(force_sensor_.generate_command()));
//             pub_time_count_ = 0;
//         }

//         can_commands[0] = pitch_motor_.generate_command();
//         can_commands[1] = yaw_motor_.generate_command();
//         can_commands[2] = force_control_motor_.generate_command();
//         can_commands[3] = 0;
//         transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

//         can_commands[0] = drive_belt_motor_[0].generate_command();
//         can_commands[1] = drive_belt_motor_[1].generate_command();
//         can_commands[2] = 0;
//         can_commands[3] = 0;
//         transmit_buffer_.add_can2_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

//         if (!trigger_servo_.calibrate_mode()) {
//             size_t uart_data_length;
//             std::unique_ptr<std::byte[]> command_buffer =
//                 trigger_servo_.generate_runtime_command(uart_data_length);
//             const auto trigger_servo_uart_data_ptr = command_buffer.get();
//             transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, uart_data_length);
//         }

//         transmit_buffer_.trigger_transmission();
//     }
//     int pub_time_count_ = 0;

// protected:
//     void can1_receive_callback(
//         uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
//         uint8_t can_data_length) override {

//         if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
//             return;

//         if (can_id == 0x302) {
//             force_sensor_.store_status(can_data);
//         }
//     }

//     void can2_receive_callback(
//         uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
//         uint8_t can_data_length) override {
//         if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
//             return;

//         if (can_id == 0x201) {
//             pitch_motor_.store_status(can_data);
//         } else if (can_id == 0x202) {
//             yaw_motor_.store_status(can_data);
//         } else if (can_id == 0x203) {
//             force_control_motor_.store_status(can_data);
//         } else if (can_id == 0x205) {
//             drive_belt_motor_[0].store_status(can_data);
//         } else if (can_id == 0x206) {
//             drive_belt_motor_[1].store_status(can_data);
//         }
//     }

//     void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
//         imu_.store_accelerometer_status(x, y, z);
//     }
    
//     void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
//         imu_.store_gyroscope_status(x, y, z);
//     }

//     void uart2_receive_callback(const std::byte* data, uint8_t length) override {
//         bool success = trigger_servo_.calibrate_current_angle(logger_, data, length);
//         if (!success) {
//             RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//         }
//     }

//     void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
//         dr16_.store_status(uart_data, uart_data_length);
//     }

// private:
//     void trigger_servo_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr msg) {
//         trigger_servo_.set_calibrate_mode(msg->data);

//         std::unique_ptr<std::byte[]> command_buffer;
//         size_t command_length = 0;
//         if (msg->data == 0) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
//         } else if (msg->data == 1) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
//         } else if (msg->data == 2) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
//         } else if (msg->data == 3) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
//         } else if (msg->data == 4) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
//         } else if (msg->data == 5) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
//         } else if (msg->data == 6) {
//             command_buffer = trigger_servo_.generate_calibrate_command(
//                 device::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
//         }

//         const auto trigger_servo_uart_data_ptr = command_buffer.get();
//         transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr, command_length);
//     }

//     static std::string bytes_to_hex_string(const std::byte* data, size_t size) {
//         if (!data || size == 0) {
//             return "[]";
//         }

//         std::stringstream ss;
//         ss << std::hex << std::uppercase << std::setfill('0');

//         for (size_t i = 0; i < size; ++i) {
//             ss << std::setw(2) << static_cast<int>(data[i]) << " ";
//         }

//         std::string result = ss.str();
//         if (!result.empty() && result.back() == ' ') {
//             result.pop_back();
//         }
//         return result;
//     }

//     void setup_imu_coordinate_mapping() {
//         // Setup coordinate mapping for IMU raw data
//         imu_.set_coordinate_mapping(
//             [](double x, double y, double z) -> std::tuple<double, double, double> {
//                 return {x, -y, -z};
//             });
//     }

//     void imu_sampler_initialize() {
//         // Initialize IMU sampler variables
//         start_time_ = std::chrono::steady_clock::now();
//         calibration_started_ = false;
//         calibration_complete_ = false;
//         yaw_drift_coefficient_ = 0.0;
//         yaw_at_first_sample_ = 0.0;
//         yaw_at_final_sample_ = 0.0;
        
//         RCLCPP_INFO(logger_, "IMU sampler initialized");
//     }

//     void processImuData() {
//         auto current_time = std::chrono::steady_clock::now();
//         double elapsed_seconds = std::chrono::duration<double>(current_time - start_time_).count();
        
//         if (!calibration_started_) {
//             calibration_started_ = true;
//             RCLCPP_INFO(logger_, "IMU calibration started at time: %f seconds", elapsed_seconds);
//         }
        
//         // Get quaternion from IMU
//         Eigen::Quaterniond imu_quaternion(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3());
        
//         // Convert quaternion to rotation matrix
//         Eigen::Matrix3d rotation_matrix = imu_quaternion.toRotationMatrix();
        
//         // Extract Euler angles (roll, pitch, yaw) from rotation matrix
//         // Using convention: roll (X), pitch (Y), yaw (Z)
//         double roll = std::atan2(rotation_matrix(2,1), rotation_matrix(2,2));
//         double pitch = std::asin(-rotation_matrix(2,0));
//         double yaw = std::atan2(rotation_matrix(1,0), rotation_matrix(0,0));
        
//         // Output raw angles without any transformation
//         *debug_raw_roll_ = roll;
//         *debug_raw_pitch_ = pitch;
//         *debug_raw_yaw_ = yaw;
        
//         // Apply transformations for inverted IMU mounting:
//         // 1. 给俯仰角（pitch）补偿π
//         // 2. 俯仰和偏航的增量取反（乘以-1）
//         // 3. Roll轴保持不变（根据您的描述，roll可能不需要特殊处理）
//         double transformed_roll = -roll ;
//         double transformed_pitch = pitch ;  // 增量取反并补偿π
//         double transformed_yaw = -yaw;            // 增量取反
        
//         // Apply initial offsets (if any)
//         transformed_roll -= roll_offset_;
//         transformed_pitch -= pitch_offset_;
//         transformed_yaw -= yaw_offset_;
        
//         // Output unnormalized transformed angles
//         *debug_transformed_roll_unnorm_ = (transformed_roll / M_PI) * 180;
//         *debug_transformed_pitch_unnorm_ = (transformed_pitch / M_PI) * 180;
//         *debug_transformed_yaw_unnorm_ = (transformed_yaw /M_PI) * 180;
        
//         // Normalize angles to [-π, π]
//         transformed_roll = normalizeAngle(transformed_roll);
//         transformed_pitch = normalizeAngle(transformed_pitch);
//         transformed_yaw = normalizeAngle(transformed_yaw);
        
//         // Output normalized transformed angles
//         *debug_transformed_roll_norm_ = transformed_roll;
//         *debug_transformed_pitch_norm_ = transformed_pitch;
//         *debug_transformed_yaw_norm_ = transformed_yaw;
        
//         // Apply low-pass filtering to normalized angles
//         double filtered_roll = roll_filter_.update(transformed_roll);
//         double filtered_pitch = pitch_filter_.update(transformed_pitch);
//         double filtered_yaw = yaw_filter_.update(transformed_yaw);
        
//         // Yaw drift calibration using two-point method (use filtered yaw)
//         if (!calibration_complete_) {
//             // First sample point
//             if (elapsed_seconds >= first_sample_spot_ && 
//                 std::abs(yaw_at_first_sample_) < 0.001) {  // Use tolerance for comparison
                
//                 yaw_at_first_sample_ = filtered_yaw;
//                 first_sample_time_ = elapsed_seconds;
//                 RCLCPP_INFO(logger_, "First yaw sample: %f rad at time: %f s", 
//                            yaw_at_first_sample_, first_sample_time_);
//             }
            
//             // Final sample point
//             if (elapsed_seconds >= final_sample_spot_ && 
//                 std::abs(yaw_at_final_sample_) < 0.001) {
                
//                 yaw_at_final_sample_ = filtered_yaw;
//                 final_sample_time_ = elapsed_seconds;
                
//                 // Calculate drift coefficient (radians per second)
//                 if (std::abs(yaw_at_first_sample_) > 0.001) {  // Ensure first sample is recorded
//                     double time_diff = final_sample_time_ - first_sample_time_;
//                     double yaw_diff = yaw_at_final_sample_ - yaw_at_first_sample_;
                    
//                     if (time_diff > 0.1) {  // Ensure sufficient time difference
//                         yaw_drift_coefficient_ = yaw_diff / time_diff;
//                         calibration_complete_ = true;
                        
//                         RCLCPP_INFO(logger_, "Yaw calibration complete");
//                         RCLCPP_INFO(logger_, "  First sample: %f rad at %f s", 
//                                    yaw_at_first_sample_, first_sample_time_);
//                         RCLCPP_INFO(logger_, "  Final sample: %f rad at %f s", 
//                                    yaw_at_final_sample_, final_sample_time_);
//                         RCLCPP_INFO(logger_, "  Time difference: %f s", time_diff);
//                         RCLCPP_INFO(logger_, "  Yaw difference: %f rad", yaw_diff);
//                         RCLCPP_INFO(logger_, "  Drift coefficient: %f rad/s", 
//                                    yaw_drift_coefficient_);
//                     }
//                 }
//             }
//         }
        
//         // Apply yaw drift compensation (use filtered yaw)
//         double compensated_yaw = filtered_yaw;
//         bool compensation_applied = false;
        
//         if (calibration_complete_) {
//             // Calculate time since calibration start
//             double time_since_calibration = elapsed_seconds - first_sample_time_;
            
//             // Apply linear drift compensation to filtered yaw
//             compensated_yaw = filtered_yaw - (yaw_drift_coefficient_ * time_since_calibration);
//             compensation_applied = true;
            
//             // Normalize compensated yaw
//             compensated_yaw = normalizeAngle(compensated_yaw);
//         }
        
//         // Output final normalized angles (use filtered roll/pitch and compensated yaw)
//         *final_roll = filtered_roll;
//         *final_pitch = filtered_pitch;
//         *final_yaw = compensated_yaw;
        
//         // Output debug information
//         *debug_drift_coefficient_ = yaw_drift_coefficient_;
//         *debug_calibration_complete_ = calibration_complete_ ? 1.0 : 0.0;
//         *debug_compensation_applied_ = compensation_applied ? 1.0 : 0.0;
        
//         // Publish TF transforms using filtered roll/pitch and compensated yaw
//         // 注意：需要将转换应用到odom坐标系
//         publishTfTransforms(filtered_roll, filtered_pitch, compensated_yaw);
//     }

//     static double normalizeAngle(double angle) {
//         // Normalize angle to range [-π, π]
//         while (angle > M_PI) angle -= 2.0 * M_PI;
//         while (angle < -M_PI) angle += 2.0 * M_PI;
//         return angle;
//     }

//     void publishTfTransforms(double roll, double pitch, double yaw) {
//         auto now = this->get_clock()->now();
        
//         // Helper function to create transform
//         auto create_transform = [&](const std::string& parent, const std::string& child, 
//                                   const geometry_msgs::msg::Vector3& trans, 
//                                   const geometry_msgs::msg::Quaternion& rot) {
//             geometry_msgs::msg::TransformStamped tf;
//             tf.header.stamp = now;
//             tf.header.frame_id = parent;
//             tf.child_frame_id = child;
//             tf.transform.translation = trans;
//             tf.transform.rotation = rot;
//             return tf;
//         };

//         // Helper function to create translation
//         auto create_translation = [](double x, double y, double z) {
//             geometry_msgs::msg::Vector3 t; 
//             t.x = x; t.y = y; t.z = z; 
//             return t;
//         };

//         // Helper function to create rotation
//         auto create_rotation = [](double x, double y, double z, double w) {
//             geometry_msgs::msg::Quaternion r; 
//             r.x = x; r.y = y; r.z = z; r.w = w; 
//             return r;
//         };

//         // Create zero translation
//         geometry_msgs::msg::Vector3 zero_trans = create_translation(0, 0, 0);
//         geometry_msgs::msg::Vector3 pitch_trans = create_translation(0, 0, 0.05);
        
//         // Create quaternions for each axis rotation
//         // 注意：这里使用的roll, pitch, yaw已经是经过转换、滤波和补偿的最终值
//         Eigen::Quaterniond roll_quat(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
//         Eigen::Quaterniond pitch_quat(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
//         Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        
//         // Combined rotation: yaw -> pitch -> roll
//         // 这个组合顺序需要根据您的TF树结构来确定
//         Eigen::Quaterniond combined_quaternion = yaw_quat * pitch_quat * roll_quat;
        
//         // Publish all TF transforms
//         // 注意：将最终转换应用到odom_imu坐标系
//         tf_broadcaster_->sendTransform(create_transform("base_link", "gimbal_center_link", 
//             zero_trans, create_rotation(0, 0, 0, 1)));
        
//         // Yaw only rotation
//         tf_broadcaster_->sendTransform(create_transform("gimbal_center_link", "yaw_link", 
//             zero_trans, create_rotation(yaw_quat.x(), yaw_quat.y(), 
//                                         yaw_quat.z(), yaw_quat.w())));
        
//         // Pitch only rotation (注意：这里使用了转换后的pitch值)
//         tf_broadcaster_->sendTransform(create_transform("yaw_link", "pitch_link", 
//             pitch_trans, create_rotation(pitch_quat.x(), pitch_quat.y(), 
//                                          pitch_quat.z(), pitch_quat.w())));
        
//         // Combined rotation for odom_imu frame (应用了所有转换)
//         tf_broadcaster_->sendTransform(create_transform("pitch_link", "odom_imu", 
//             zero_trans, create_rotation(combined_quaternion.x(), combined_quaternion.y(), 
//                                         combined_quaternion.z(), combined_quaternion.w())));
        
//         // World to base_link transform
//         tf_broadcaster_->sendTransform(create_transform("world", "base_link", 
//             zero_trans, create_rotation(0, 0, 0, 1)));
//     }

//     rclcpp::Logger logger_;

//     // Low-pass filters for IMU data
//     filter::LowPassFilter<1> pitch_angle_filter_;
//     filter::LowPassFilter<1> pitch_velocity_filter_;
//     filter::LowPassFilter<1> yaw_velocity_filter_;
//     filter::LowPassFilter<1> yaw_filter_;
//     filter::LowPassFilter<1> pitch_filter_;
//     filter::LowPassFilter<1> roll_filter_;

//     std::chrono::steady_clock::time_point start_time_;    

//     // Dart command component
//     class DartCommand : public rmcs_executor::Component {
//     public:
//         explicit DartCommand(CatapultDart& robot)
//             : dart_(robot) {}

//         void update() override { dart_.command_update(); }

//         CatapultDart& dart_;
//     };
//     std::shared_ptr<DartCommand> dart_command_;

//     // IMU parameters
//     double imu_sensitivity_;
//     double first_sample_spot_;
//     double final_sample_spot_;
//     double roll_offset_;
//     double pitch_offset_;
//     double yaw_offset_;

//     // TF related
//     OutputInterface<rmcs_description::Tf> tf_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

//     // Hardware devices
//     device::Dr16 dr16_;
//     device::Bmi088 imu_;
//     device::DjiMotor pitch_motor_;
//     device::DjiMotor yaw_motor_;
//     device::DjiMotor force_control_motor_;
//     device::DjiMotor drive_belt_motor_[2];

//     device::ForceSensorRuntime force_sensor_;
//     device::TriggerServo trigger_servo_;

//     // Subscriptions
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;

//     librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
//     std::thread event_thread_;
    
//     // Output interfaces for IMU data
//     OutputInterface<double> pitch_angle_;
//     OutputInterface<double> final_roll;
//     OutputInterface<double> final_pitch;
//     OutputInterface<double> final_yaw;

//     // Debug output interfaces for raw IMU data
//     OutputInterface<double> debug_raw_roll_;
//     OutputInterface<double> debug_raw_pitch_;
//     OutputInterface<double> debug_raw_yaw_;

//     // Debug output interfaces (only transformed angles)
//     OutputInterface<double> debug_transformed_roll_unnorm_;
//     OutputInterface<double> debug_transformed_pitch_unnorm_;
//     OutputInterface<double> debug_transformed_yaw_unnorm_;
//     OutputInterface<double> debug_transformed_roll_norm_;
//     OutputInterface<double> debug_transformed_pitch_norm_;
//     OutputInterface<double> debug_transformed_yaw_norm_;
    
//     // Debug output for drift compensation
//     OutputInterface<double> debug_drift_coefficient_;
//     OutputInterface<double> debug_calibration_complete_;
//     OutputInterface<double> debug_compensation_applied_;

//     // IMU calibration variables
//     bool calibration_started_ = false;
//     bool calibration_complete_ = false;
//     double yaw_drift_coefficient_ = 0.0;
//     double yaw_at_first_sample_ = 0.0;
//     double yaw_at_final_sample_ = 0.0;
//     double first_sample_time_ = 0.0;
//     double final_sample_time_ = 0.0;
// };
// } // namespace rmcs_core::hardware

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)