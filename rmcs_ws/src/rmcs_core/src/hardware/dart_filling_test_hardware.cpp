// #include "librmcs/client/cboard.hpp"
// #include "hardware/device/dr16.hpp"
// #include "hardware/device/trigger_servo.hpp"
// #include <cmath>
// #include <cstddef>
// #include <cstdint>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/node.hpp>
// #include <rmcs_msgs/serial_interface.hpp>
// #include <std_msgs/msg/int32.hpp>
// #include <eigen3/Eigen/src/Geometry/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <rmcs_executor/component.hpp>
// #include <rmcs_description/tf_description.hpp>
// #include <rmcs_msgs/dart_launch_stage.hpp>

// namespace rmcs_core::hardware {

// class DartFillingTestHardware
//     : public rmcs_executor::Component
//     , public rclcpp::Node
//     , private librmcs::client::CBoard {
// public:
//     DartFillingTestHardware()
//         : Node{
//               get_component_name(),
//               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
//         , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
//         , logger_(get_logger())
//         , dart_command_(create_partner_component<DartCommand>(get_component_name() + "_command",
//         *this)) , dr16_(*this) , trigger_servo_(*dart_command_, "/dart/trigger_servo",
//         TRIGGER_SERVO_ID) , limiting_servo_(*dart_command_, "/dart/limiting_servo",
//         LIMITING_SERVO_ID) , lifting_left_(*dart_command_, "/dart/lifting_left", LIFTING_LEFT_ID)
//         , lifting_right_(*dart_command_, "/dart/lifting_right", LIFTING_RIGHT_ID)
//         , transmit_buffer_(*this, 32)
//         , event_thread_([this]() { handle_events(); }) {
//         register_output("/dart/lifting_left/current_angle", lifting_current_angle_left_);
//         register_output("/dart/lifting_right/current_angle", lifting_current_angle_right_);

//         trigger_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/trigger/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
//                 trigger_servo_calibrate_subscription_callback(trigger_servo_, std::move(msg),
//                 trigger_servo_uart_data_ptr);
//             });
//         limiting_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/limiting/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg)
//             {
//                 trigger_servo_calibrate_subscription_callback(limiting_servo_, std::move(msg),
//                 limiting_servo_uart_data_ptr);
//             });
//         lifting_left_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/lifting_left/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&&
//             msg) {
//                 trigger_servo_calibrate_subscription_callback(lifting_left_, std::move(msg),
//                 lifting_left_uart_data_ptr);
//             });
//         lifting_right_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
//             "/lifting_right/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&&
//             msg) {
//                 trigger_servo_calibrate_subscription_callback(lifting_right_, std::move(msg),
//                 lifting_right_uart_data_ptr);
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

//     ~DartFillingTestHardware() override {
//         stop_handling_events();
//         event_thread_.join();
//     }

//     void update() override {
//         dr16_.update_status();
//     }

//     void command_update() {

//         if (!trigger_servo_.calibrate_mode()) {
//             uint16_t current_target = trigger_servo_.get_target_angle();
//             if (current_target != last_trigger_angle_) {
//                 size_t uart_data_length;
//                 std::unique_ptr<std::byte[]> command_buffer =
//                     trigger_servo_.generate_runtime_command(uart_data_length);
//                 const auto trigger_servo_uart_data_ptr = command_buffer.get();
//                 transmit_buffer_.add_uart2_transmission(trigger_servo_uart_data_ptr,
//                 uart_data_length); last_trigger_angle_ = current_target;
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
//                     device::TriggerServo::generate_sync_run_command(uart_data_length,
//                                                                     LIFTING_LEFT_ID,
//                                                                     LIFTING_RIGHT_ID,
//                                                                     current_target_left,
//                                                                     current_target_right,
//                                                                     runtime_left, runtime_right);
//                 const auto lifting_table_uart_data_ptr = command_buffer.get();
//                 transmit_buffer_.add_uart2_transmission(lifting_table_uart_data_ptr,
//                 uart_data_length); last_lifting_left_angle_ = current_target_left;
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
//             case TRIGGER_SERVO_ID:
//                 result = trigger_servo_.calibrate_current_angle(logger_, data, length);
//                 if (!result.first) {
//                     RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//                 }
//                 break;
//             case LIMITING_SERVO_ID:
//                 result = limiting_servo_.calibrate_current_angle(logger_, data, length);
//                 if (!result.first) {
//                     RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//                 }
//                 break;
//             case LIFTING_LEFT_ID:
//                 result = lifting_left_.calibrate_current_angle(logger_, data, length);
//                 if (result.first) {
//                     *lifting_current_angle_left_ = result.second;
//                 } else {
//                     RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//                 }
//                 break;
//             case LIFTING_RIGHT_ID:
//                 result = lifting_right_.calibrate_current_angle(logger_, data, length);
//                 if (result.first) {
//                     *lifting_current_angle_right_ = result.second;
//                 } else {
//                     RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//                 }
//                 break;
//             default: RCLCPP_INFO(logger_, "calibrate: uart2 data store failed");
//                 break;
//         }

//     }

//     void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
//         dr16_.store_status(uart_data, uart_data_length);
//     }

// private:
//     static constexpr uint8_t TRIGGER_SERVO_ID  = 0x01;
//     static constexpr uint8_t LIMITING_SERVO_ID = 0x02;
//     static constexpr uint8_t LIFTING_LEFT_ID   = 0x03;
//     static constexpr uint8_t LIFTING_RIGHT_ID  = 0x04;
//     std::byte * trigger_servo_uart_data_ptr;
//     std::byte * limiting_servo_uart_data_ptr;
//     std::byte * lifting_left_uart_data_ptr;
//     std::byte * lifting_right_uart_data_ptr;

//     void trigger_servo_calibrate_subscription_callback(device::TriggerServo& servo_
//                                                      , std_msgs::msg::Int32::UniquePtr msg
//                                                      , std::byte* servo_uart_data_ptr) {
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

//     rclcpp::Logger logger_;
//     class DartCommand : public rmcs_executor::Component {
//     public:
//         explicit DartCommand(DartFillingTestHardware& robot) : dart_(robot) {}
//         void update() override { dart_.command_update(); }
//         DartFillingTestHardware& dart_;
//     };

//     std::shared_ptr<DartCommand> dart_command_;
//     device::Dr16 dr16_;
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
//     OutputInterface<uint16_t> lifting_current_angle_left_;
//     OutputInterface<uint16_t> lifting_current_angle_right_;
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
// PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DartFillingTestHardware, rmcs_executor::Component)