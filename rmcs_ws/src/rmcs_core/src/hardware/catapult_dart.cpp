// #include "filter/low_pass_filter.hpp"
// #include "hardware/device/bmi088.hpp"
// #include "hardware/device/dji_motor.hpp"
// #include "hardware/device/dr16.hpp"
// #include "hardware/device/force_sensor_runtime.hpp"
// #include "hardware/device/trigger_servo.hpp"
// #include "librmcs/agent/c_board.hpp"
// #include <chrono>
// #include <cmath>
// #include <cstddef>
// #include <cstdint>
// #include <eigen3/Eigen/src/Geometry/Quaternion.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_description/tf_description.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_msgs/dart_launch_stage.hpp>
// #include <rmcs_msgs/serial_interface.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <tf2_ros/transform_broadcaster.h>

// namespace rmcs_core::hardware {

// class CatapultDart
//     : public rmcs_executor::Component
//     , public rclcpp::Node
//     , public librmcs::agent::CBoard {
// public:
//     CatapultDart()
//         : Node{get_component_name(),
//         rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} ,
//         librmcs::agent::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())} ,
//         logger_(get_logger()) , pitch_angle_filter_(20.0, 1000.0) , pitch_velocity_filter_(20.0,
//         1000.0) , yaw_velocity_filter_(20.0, 1000.0) , yaw_filter_(5.0, 1000.0) ,
//         pitch_filter_(10.0, 1000.0) , roll_filter_(10.0, 1000.0) , dart_command_(
//               create_partner_component<DartCommand>(get_component_name() + "_command", *this))
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
//         , trigger_servo_(*dart_command_, "/dart/trigger_servo", TRIGGER_SERVO_ID)
//         , limiting_servo_(*dart_command_, "/dart/limiting_servo", LIMITING_SERVO_ID)
//         , lifting_left_(*dart_command_, "/dart/lifting_left", LIFTING_LEFT_ID)
//         , lifting_right_(*dart_command_, "/dart/lifting_right", LIFTING_RIGHT_ID)
//         , transmit_buffer_(*this, 32)
//         , event_thread_([this]() { handle_events(); }) {
//         register_output("/tf", tf_);
//         register_output("/imu/catapult_pitch_angle", catapult_pitch_angle_);
//         register_output("/imu/catapult_roll_angle", catapult_roll_angle_);
//         register_output("/imu/catapult_yaw_angle", catapult_yaw_angle_);
//         register_output("/dart/lifting_left/current_angle", lifting_current_angle_left_);
//         register_output("/dart/lifting_right/current_angle", lifting_current_angle_right_);

//         first_sample_spot_ = this->get_parameter("first_sample_spot").as_double();
//         final_sample_spot_ = this->get_parameter("final_sample_spot").as_double();

//         setup_imu_coordinate_mapping();
//         imu_sampler_initialize();
//         tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
//         start_time_ = std::chrono::steady_clock::now();

//         trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(
//                     trigger_servo_, std::move(msg), trigger_servo_uart_data_ptr);
//             });
//         limiting_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/limiting/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg)
//             {
//                 trigger_servo_calibrate_subscription_callback(
//                     limiting_servo_, std::move(msg), limiting_servo_uart_data_ptr);
//             });
//         lifting_left_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/lifting_left/calibrate", rclcpp::QoS{0},
//             [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(
//                     lifting_left_, std::move(msg), lifting_left_uart_data_ptr);
//             });
//         lifting_right_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/lifting_right/calibrate", rclcpp::QoS{0},
//             [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(
//                     lifting_right_, std::move(msg), lifting_right_uart_data_ptr);
//             });

//         register_output("/referee/serial", referee_serial_);
//         referee_serial_->read = [this](std::byte* buffer, size_t size) {
//             return referee_ring_buffer_receive_.pop_front_multi(
//                 [&buffer](std::byte byte) { *buffer++ = byte; }, size);
//         };
//         referee_serial_->write = [this](const std::byte* buffer, size_t size) {
//             transmit_buffer_.add_uart1_transmission(buffer, size);
//             return size;
//         };

//         last_read_left_time_ = this->now();
//         last_read_right_time_ = this->now();
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
//             uint16_t current_target = trigger_servo_.get_target_angle();
//             if (current_target != last_trigger_angle_) {
//                 size_t uart_data_length;
//                 std::unique_ptr<std::byte[]> command_buffer =
//                     trigger_servo_.generate_runtime_command(uart_data_length);
//                 const auto trigger_servo_uart_data_ptr = command_buffer.get();
//                 transmit_buffer_.add_uart2_transmission(
//                     trigger_servo_uart_data_ptr, uart_data_length);
//                 last_trigger_angle_ = current_target;
//             }
//         }

//         if (!limiting_servo_.calibrate_mode()) {
//             uint16_t current_target = limiting_servo_.get_target_angle();
//             if (current_target != last_limiting_angle_) {
//                 size_t uart_data_length;
//                 auto command_buffer = limiting_servo_.generate_runtime_command(uart_data_length);
//                 transmit_buffer_.add_uart2_transmission(command_buffer.get(), uart_data_length);
//                 last_limiting_angle_ = current_target;
//             }
//         }

//         if (!lifting_left_.calibrate_mode() && !lifting_right_.calibrate_mode()) {
//             uint16_t current_target_left = lifting_left_.get_target_angle();
//             uint16_t current_target_right = lifting_right_.get_target_angle();
//             if (current_target_left != last_lifting_left_angle_) {
//                 size_t uart_data_length;
//                 uint16_t runtime_left = 0;
//                 uint16_t runtime_right = 0;
//                 std::unique_ptr<std::byte[]> command_buffer =
//                     device::TriggerServo::generate_sync_run_command(
//                         uart_data_length, LIFTING_LEFT_ID, LIFTING_RIGHT_ID, current_target_left,
//                         current_target_right, runtime_left, runtime_right);
//                 const auto lifting_table_uart_data_ptr = command_buffer.get();
//                 transmit_buffer_.add_uart2_transmission(
//                     lifting_table_uart_data_ptr, uart_data_length);
//                 last_lifting_left_angle_ = current_target_left;
//             }
//         }

//         auto now = this->now();

//         if (!lifting_left_.calibrate_mode() && (now - last_read_left_time_) >= read_interval_) {
//             size_t uart_data_length;
//             auto command_buffer = lifting_left_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::READ_CURRENT_ANGLE, uart_data_length);
//             transmit_buffer_.add_uart2_transmission(command_buffer.get(), uart_data_length);
//             last_read_left_time_ = now;
//         }
//         if (!lifting_right_.calibrate_mode() && (now - last_read_right_time_) >= read_interval_)
//         {
//             if ((now - last_read_left_time_) >= rclcpp::Duration::from_seconds(0.01)) {
//                 size_t uart_data_length;
//                 auto command_buffer = lifting_right_.generate_calibrate_command(
//                     device::TriggerServo::CalibrateOperation::READ_CURRENT_ANGLE,
//                     uart_data_length);
//                 transmit_buffer_.add_uart2_transmission(command_buffer.get(), uart_data_length);
//                 last_read_right_time_ = now;
//             }
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

//     void uart1_receive_callback(const std::byte* data, uint8_t length) override {
//         referee_ring_buffer_receive_.emplace_back_multi(
//             [&data](std::byte* storage) { *storage = *data++; }, length);
//     }

//     void uart2_receive_callback(const std::byte* data, uint8_t length) override {
//         std::string hex_string = bytes_to_hex_string(data, length);
//         RCLCPP_DEBUG(this->get_logger(), "UART2 received: len=%d [%s]", length,
//         hex_string.c_str());

//         if (length < 3) {
//             RCLCPP_WARN(logger_, "UART2 data too short: %d bytes", length);
//             return;
//         }

//         uint8_t servo_id = static_cast<uint8_t>(data[2]);
//         std::pair<bool, uint16_t> result{false, 0};

//         switch (servo_id) {
//         case TRIGGER_SERVO_ID:
//             result = trigger_servo_.calibrate_current_angle(logger_, data, length);
//             if (!result.first) {
//                 RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//             }
//             break;
//         case LIMITING_SERVO_ID:
//             result = limiting_servo_.calibrate_current_angle(logger_, data, length);
//             if (!result.first) {
//                 RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//             }
//             break;
//         case LIFTING_LEFT_ID:
//             result = lifting_left_.calibrate_current_angle(logger_, data, length);
//             if (result.first) {
//                 *lifting_current_angle_left_ = result.second;
//             } else {
//                 RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//             }
//             break;
//         case LIFTING_RIGHT_ID:
//             result = lifting_right_.calibrate_current_angle(logger_, data, length);
//             if (result.first) {
//                 *lifting_current_angle_right_ = result.second;
//             } else {
//                 RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//             }
//             break;
//         default: RCLCPP_INFO(logger_, "calibrate: uart2 data store failed"); break;
//         }
//     }

//     void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
//         dr16_.store_status(uart_data, uart_data_length);
//     }

//     void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
//         imu_.store_accelerometer_status(x, y, z);
//     }

//     void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
//         imu_.store_gyroscope_status(x, y, z);
//     }

// private:
//     static constexpr uint8_t TRIGGER_SERVO_ID = 0x01;
//     static constexpr uint8_t LIMITING_SERVO_ID = 0x02;
//     static constexpr uint8_t LIFTING_LEFT_ID = 0x03;
//     static constexpr uint8_t LIFTING_RIGHT_ID = 0x04;
//     std::byte* trigger_servo_uart_data_ptr;
//     std::byte* limiting_servo_uart_data_ptr;
//     std::byte* lifting_left_uart_data_ptr;
//     std::byte* lifting_right_uart_data_ptr;

//     void trigger_servo_calibrate_subscription_callback(
//         device::TriggerServo& servo_, std_msgs::msg::Int32::UniquePtr msg,
//         std::byte* servo_uart_data_ptr) {
//         /*
//         标定命令格式：
//         ros2 topic pub --rate 2 --times 5 /trigger/calibrate std_msgs/msg/Int32 "{'data':0}"
//         替换data值就行
//         */
//         servo_.set_calibrate_mode(msg->data);

//         std::unique_ptr<std::byte[]> command_buffer;
//         size_t command_length = 0;
//         if (msg->data == 0) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::SWITCH_TO_SERVO_MODE, command_length);
//         } else if (msg->data == 1) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::SWITCH_TO_MOTOR_MODE, command_length);
//         } else if (msg->data == 2) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_FORWARD_MODE, command_length);
//         } else if (msg->data == 3) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_REVERSE_MODE, command_length);
//         } else if (msg->data == 4) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_RUNTIME_CONTROL, command_length);
//         } else if (msg->data == 5) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::MOTOR_DISABLE_CONTROL, command_length);
//         } else if (msg->data == 6) {
//             command_buffer = servo_.generate_calibrate_command(
//                 device::TriggerServo::CalibrateOperation::READ_CURRENT_ANGLE, command_length);
//         }

//         servo_uart_data_ptr = command_buffer.get();
//         transmit_buffer_.add_uart2_transmission(servo_uart_data_ptr, command_length);

//         std::string hex_string = bytes_to_hex_string(command_buffer.get(), command_length);
//         RCLCPP_INFO(
//             this->get_logger(), "UART2 Pub: (length=%zu)[ %s ]", command_length,
//             hex_string.c_str());
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
//     } // DEBUG TOOL

//     void setup_imu_coordinate_mapping() {
//         imu_.set_coordinate_mapping(
//             [](double x, double y, double z) -> std::tuple<double, double, double> {
//                 return {x, -y, -z};
//             });
//     }

//     void imu_sampler_initialize() {
//         start_time_ = std::chrono::steady_clock::now();
//         yaw_drift_coefficient_ = 0.0;
//     }

//     void processImuData() {
//         auto current_time = std::chrono::steady_clock::now();
//         double elapsed_seconds = std::chrono::duration<double>(current_time -
//         start_time_).count();

//         Eigen::Quaterniond imu_quaternion(imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3());
//         Eigen::Matrix3d rotation_matrix = imu_quaternion.toRotationMatrix();

//         double roll = std::atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
//         double pitch = std::asin(-rotation_matrix(2, 0));
//         double yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

//         double transformed_roll = -roll_filter_.update(roll);
//         double transformed_pitch = pitch_filter_.update(pitch);
//         double transformed_yaw = -yaw_filter_.update(yaw);

//         if (elapsed_seconds >= first_sample_spot_ && elapsed_seconds <= final_sample_spot_) {
//             yaw_samples_.push_back(transformed_yaw);
//             time_samples_.push_back(elapsed_seconds);
//             double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
//             double n = static_cast<double>(yaw_samples_.size());

//             for (int i = 0; i < n; ++i) {
//                 sum_x += time_samples_[i];
//                 sum_y += yaw_samples_[i];
//                 sum_xy += time_samples_[i] * yaw_samples_[i];
//                 sum_xx += time_samples_[i] * time_samples_[i];
//             }

//             double denominator = n * sum_xx - sum_x * sum_x;
//             if (std::abs(denominator) > 1e-10) {
//                 yaw_drift_coefficient_ = (n * sum_xy - sum_x * sum_y) / denominator;
//             }
//         }

//         transformed_yaw -= ((yaw_drift_coefficient_ + 0.000512) * elapsed_seconds);
//         publishTfTransforms(transformed_roll, transformed_pitch, transformed_yaw);

//         *catapult_roll_angle_ = (transformed_roll / M_PI) * 180.0;
//         *catapult_pitch_angle_ = (transformed_pitch / M_PI) * 180.0;
//         *catapult_yaw_angle_ = (transformed_yaw / M_PI) * 180.0;
//     }

//     void publishTfTransforms(double roll, double pitch, double yaw) {
//         auto now = this->get_clock()->now();

//         auto create_transform = [&](const std::string& parent, const std::string& child,
//                                     const geometry_msgs::msg::Vector3& trans,
//                                     const geometry_msgs::msg::Quaternion& rot) {
//             geometry_msgs::msg::TransformStamped tf;
//             tf.header.stamp = now;
//             tf.header.frame_id = parent;
//             tf.child_frame_id = child;
//             tf.transform.translation = trans;
//             tf.transform.rotation = rot;
//             return tf;
//         };

//         auto create_translation = [](double x, double y, double z) {
//             geometry_msgs::msg::Vector3 t;
//             t.x = x;
//             t.y = y;
//             t.z = z;
//             return t;
//         };

//         auto create_rotation = [](double x, double y, double z, double w) {
//             geometry_msgs::msg::Quaternion r;
//             r.x = x;
//             r.y = y;
//             r.z = z;
//             r.w = w;
//             return r;
//         };

//         geometry_msgs::msg::Vector3 zero_trans = create_translation(0, 0, 0);
//         geometry_msgs::msg::Vector3 pitch_trans = create_translation(0, 0, 0.05);

//         Eigen::Quaterniond roll_quat(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
//         Eigen::Quaterniond pitch_quat(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
//         Eigen::Quaterniond yaw_quat(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
//         Eigen::Quaterniond combined_quaternion = yaw_quat * pitch_quat * roll_quat;

//         tf_broadcaster_->sendTransform(create_transform(
//             "base_link", "gimbal_center_link", zero_trans, create_rotation(0, 0, 0, 1)));
//         tf_broadcaster_->sendTransform(create_transform(
//             "gimbal_center_link", "yaw_link", zero_trans,
//             create_rotation(yaw_quat.x(), yaw_quat.y(), yaw_quat.z(), yaw_quat.w())));
//         tf_broadcaster_->sendTransform(create_transform(
//             "yaw_link", "pitch_link", pitch_trans,
//             create_rotation(pitch_quat.x(), pitch_quat.y(), pitch_quat.z(), pitch_quat.w())));
//         tf_broadcaster_->sendTransform(create_transform(
//             "pitch_link", "odom_imu", zero_trans,
//             create_rotation(
//                 combined_quaternion.x(), combined_quaternion.y(), combined_quaternion.z(),
//                 combined_quaternion.w())));
//         tf_broadcaster_->sendTransform(
//             create_transform("world", "base_link", zero_trans, create_rotation(0, 0, 0, 1)));
//     }

//     rclcpp::Logger logger_;
//     filter::LowPassFilter<1> pitch_angle_filter_;
//     filter::LowPassFilter<1> pitch_velocity_filter_;
//     filter::LowPassFilter<1> yaw_velocity_filter_;
//     filter::LowPassFilter<1> yaw_filter_;
//     filter::LowPassFilter<1> pitch_filter_;
//     filter::LowPassFilter<1> roll_filter_;
//     std::chrono::steady_clock::time_point start_time_;
//     class DartCommand : public rmcs_executor::Component {
//     public:
//         explicit DartCommand(CatapultDart& robot)
//             : dart_(robot) {}
//         void update() override { dart_.command_update(); }
//         CatapultDart& dart_;
//     };

//     std::shared_ptr<DartCommand> dart_command_;
//     double first_sample_spot_;
//     double final_sample_spot_;
//     OutputInterface<rmcs_description::Tf> tf_;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//     device::Dr16 dr16_;
//     device::Bmi088 imu_;
//     device::DjiMotor pitch_motor_;
//     device::DjiMotor yaw_motor_;
//     device::DjiMotor force_control_motor_;
//     device::DjiMotor drive_belt_motor_[2];
//     device::ForceSensorRuntime force_sensor_;
//     device::TriggerServo trigger_servo_;
//     device::TriggerServo limiting_servo_;
//     device::TriggerServo lifting_left_;
//     device::TriggerServo lifting_right_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr trigger_calibrate_subscription_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr limiting_calibrate_subscription_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lifting_left_calibrate_subscription_;
//     rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lifting_right_calibrate_subscription_;
//     librmcs::utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
//     OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
//     librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
//     std::thread event_thread_;
//     OutputInterface<double> catapult_pitch_angle_;
//     OutputInterface<double> catapult_roll_angle_;
//     OutputInterface<double> catapult_yaw_angle_;
//     OutputInterface<uint16_t> lifting_current_angle_left_;
//     OutputInterface<uint16_t> lifting_current_angle_right_;
//     double yaw_drift_coefficient_ = 0.0;
//     std::vector<double> yaw_samples_, time_samples_;
//     uint16_t last_trigger_angle_ = 0xFFFF;
//     uint16_t last_limiting_angle_ = 0xFFFF;
//     uint16_t last_lifting_left_angle_ = 0xFFFF;
//     rclcpp::Time last_read_left_time_;
//     rclcpp::Time last_read_right_time_;
//     const rclcpp::Duration read_interval_ = rclcpp::Duration::from_seconds(0.5);
// };
// } // namespace rmcs_core::hardware

// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)