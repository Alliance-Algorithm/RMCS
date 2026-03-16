#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

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

#include "hardware/device/can_packet.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/trigger_servo.hpp"
#include "librmcs/agent/c_board.hpp"

namespace rmcs_core::hardware {

class CatapultDartFillingTest
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    CatapultDartFillingTest()
        : Node{get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::agent::CBoard{get_parameter("serial_filter").as_string()}
        , dart_command_(
              create_partner_component<DartCommand>(get_component_name() + "_command", *this))
        , logger_{get_logger()}
        , lifting_left_motor_{*this, *dart_command_, "/dart/lifting_left"}
        , lifting_right_motor_{*this, *dart_command_, "/dart/lifting_right"}
        , limiting_servo_{*dart_command_, "/dart/limiting_servo", 0x04}

        , dr16_{*this} {
        lifting_left_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                .enable_multi_turn_angle());
        lifting_right_motor_.configure(
            device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                .enable_multi_turn_angle());
    }

    CatapultDartFillingTest(const CatapultDartFillingTest&) = delete;
    CatapultDartFillingTest& operator=(const CatapultDartFillingTest&) = delete;
    CatapultDartFillingTest(CatapultDartFillingTest&&) = delete;
    CatapultDartFillingTest& operator=(CatapultDartFillingTest&&) = delete;

    ~CatapultDartFillingTest() override = default;

    void update() override {
        dr16_.update_status();
        lifting_left_motor_.update_status();
        lifting_right_motor_.update_status();
    }

    void command_update() {
        auto board = start_transmit();

        board.can1_transmit({
            .can_id   = 0x141,
            .can_data = lifting_left_motor_.generate_velocity_command().as_bytes(),
        });

        board.can1_transmit({
            .can_id   = 0x145,
            .can_data = lifting_right_motor_.generate_velocity_command().as_bytes(),
        });

        if (!limiting_servo_.calibrate_mode()) {
            uint16_t current_target = limiting_servo_.get_target_angle();
            if (current_target != last_limiting_angle_) {
                size_t uart_data_length;
                auto command_buffer = limiting_servo_.generate_runtime_command(uart_data_length);
                board.uart2_transmit(
                    {.uart_data = std::span{command_buffer.get(), uart_data_length}});
                last_limiting_angle_ = current_target;
                RCLCPP_INFO(logger_, "limiting_servo angle: 0x%04X", current_target);
            }
        }
    }

protected:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
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

       class DartCommand : public rmcs_executor::Component {
    public:
        explicit DartCommand(CatapultDartFillingTest& dart)
            : dart_(dart) {}
        void update() override { dart_.command_update();}

    private:
        CatapultDartFillingTest& dart_;
    };

    std::shared_ptr<DartCommand> dart_command_;
    rclcpp::Logger logger_;

    uint16_t last_limiting_angle_{0xFFFF};

    device::LkMotor lifting_left_motor_;
    device::LkMotor lifting_right_motor_;

    device::TriggerServo limiting_servo_;

    device::Dr16 dr16_;

    std::mutex referee_mutex_;
    std::deque<std::byte> referee_ring_buffer_receive_;
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDartFillingTest, rmcs_executor::Component)
