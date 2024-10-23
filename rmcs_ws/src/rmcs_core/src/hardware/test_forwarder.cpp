#include <memory>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/imu.hpp"
#include "hardware/forwarder/cboard.hpp"

namespace rmcs_core::hardware {

class TestForwarder : public rmcs_executor::Component,
                      public rclcpp::Node,
                      private forwarder::CBoard {
public:
  TestForwarder()
      : Node{get_component_name(),
             rclcpp::NodeOptions{}
                 .automatically_declare_parameters_from_overrides(true)},
        forwarder::CBoard{
            static_cast<uint16_t>(get_parameter("usb_pid").as_int()),
            get_logger()},
        logger_(get_logger()),
        infantry_command_(create_partner_component<TestForwarderCommand>(
            get_component_name() + "_command", *this)),
        transmit_buffer_(*this, 16) {
    using namespace device;

    test_motor.configure(DjiMotorConfig{DjiMotorType::M3508}
                             .reverse()
                             .set_reduction_ratio(19.)
                             .enable_multi_turn_angle());

    register_output("/motor/test/control_torque_unrestricted",
                    control_torque_unrestricted_, nan);
  }

  void update() override {
    update_motors();
    update_imu();
    dr16_.update();
  }

  void command_update() {
    uint16_t can_commands[4];

    can_commands[0] = std::bit_cast<uint16_t>(be_uint16_t(300));
    can_commands[1] = 0;
    can_commands[2] = 0;
    can_commands[3] = 0;
    transmit_buffer_.add_can1_transmission(
        0x200, std::bit_cast<uint64_t>(can_commands));

    transmit_buffer_.trigger_transmission();
  }

private:
  void update_motors() {
    using namespace rmcs_description;
    test_motor.update();
  }

  void update_imu() {}

protected:
  void can1_receive_callback(uint32_t can_id, uint64_t can_data,
                             bool is_extended_can_id,
                             bool is_remote_transmission,
                             uint8_t can_data_length) override {
    if (is_extended_can_id || is_remote_transmission || can_data_length < 8)
        [[unlikely]]
      return;

    if (can_id == 0x201) {
      auto &motor = test_motor;
      motor.store_status(can_data);
    }
  }

  void can2_receive_callback(uint32_t, uint64_t, bool, bool, uint8_t) override {
  }

  void uart1_receive_callback(const std::byte *uart_data,
                              uint8_t uart_data_length) override {
    referee_ring_buffer_receive_.emplace_back_multi(
        [&uart_data](std::byte *storage) { *storage = *uart_data++; },
        uart_data_length);
  }

  void dbus_receive_callback(const std::byte *, uint8_t) override {}

  void accelerometer_receive_callback(int16_t, int16_t, int16_t) override {}

  void gyroscope_receive_callback(int16_t, int16_t, int16_t) override {}

private:
  rclcpp::Logger logger_;

  class TestForwarderCommand : public rmcs_executor::Component {
  public:
    explicit TestForwarderCommand(TestForwarder &infantry)
        : infantry_(infantry) {}

    void update() override { infantry_.command_update(); }

    TestForwarder &infantry_;
  };
  std::shared_ptr<TestForwarderCommand> infantry_command_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      gimbal_calibrate_subscription_;

  device::DjiMotor test_motor{*this, *infantry_command_, "/motor/test"};

  device::Dr16 dr16_{*this};

  struct alignas(8) ImuData {
    int16_t x, y, z;
  };
  std::atomic<ImuData> accelerometer_data_, gyroscope_data_;
  static_assert(std::atomic<ImuData>::is_always_lock_free);
  device::Imu imu_;

  OutputInterface<double> gimbal_yaw_velocity_imu_;
  OutputInterface<double> gimbal_pitch_velocity_imu_;

  OutputInterface<rmcs_description::Tf> tf_;
  OutputInterface<Eigen::Vector3d> imu_gyro_;

  RingBuffer<std::byte> referee_ring_buffer_receive_{256};
  OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
  OutputInterface<double> control_torque_unrestricted_;

  forwarder::CBoard::TransmitBuffer transmit_buffer_;

  static constexpr double inf = std::numeric_limits<double>::infinity();
  static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TestForwarder,
                       rmcs_executor::Component)