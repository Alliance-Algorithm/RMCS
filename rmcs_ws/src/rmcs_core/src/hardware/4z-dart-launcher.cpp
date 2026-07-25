#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/pwm_servo.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/uart_servo.hpp"
#include "librmcs/board/rmcs_board_pro.hpp"
#include "librmcs/data/datas.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>

namespace rmcs_core::hardware {

class CatapultDart
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    CatapultDart()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<CatapultDartCommand>(
                  get_component_name() + "_command", *this)) {

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        dart_board_ = std::make_unique<DartBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string());
    }

    CatapultDart(const CatapultDart&) = delete;
    CatapultDart& operator=(const CatapultDart&) = delete;
    CatapultDart(CatapultDart&&) = delete;
    CatapultDart& operator=(CatapultDart&&) = delete;

    ~CatapultDart() override = default;

    void update() override {
        dart_board_->update();
        remote_control_->update();
    }

    void command_update() { dart_board_->command_update(); }

private:
    class CatapultDartCommand : public rmcs_executor::Component {
    public:
        explicit CatapultDartCommand(CatapultDart& hero)
            : hero_(hero) {}

        void update() override { hero_.command_update(); }

        CatapultDart& hero_;
    };
    std::shared_ptr<CatapultDartCommand> command_component_;

    struct DartBoard final : librmcs::board::RmcsBoardPro::Callback {
        explicit DartBoard(
            CatapultDart& catapult_dart, CatapultDartCommand& catapult_dart_command,
            std::string_view board_serial = {})
            : logger_(catapult_dart.get_logger())
            , imu_(1000, 0.2, 0.0)
            , chassis_motors_(
                  {catapult_dart, catapult_dart_command, "/dart/chassis/front_left_motor"},
                  {catapult_dart, catapult_dart_command, "/dart/chassis/front_back_motor"},
                  {catapult_dart, catapult_dart_command, "/dart/chassis/back_left_motor"},
                  {catapult_dart, catapult_dart_command, "/dart/chassis/back_right_motor"})
            , drive_belt_motors_(
                  {catapult_dart, catapult_dart_command, "/dart/belt/left_motor"},
                  {catapult_dart, catapult_dart_command, "/dart/belt/right_motor"})
            , yaw_motor_(catapult_dart, catapult_dart_command, "/dart/yaw/motor")
            , trigger_motor_(catapult_dart, catapult_dart_command, "/dart/trigger/position_motor")
            , filling_lift_motor_(
                  {catapult_dart, catapult_dart_command, "/dart/filling_lift/left_motor"},
                  {catapult_dart, catapult_dart_command, "/dart/filling_lift/right_motor"})
            , filling_limit_servo_{catapult_dart_command, "/dart/limiting_servo", 0x02}
            , trigger_servo_{catapult_dart_command, "/dart/trigger_servo", 20.0, 0.5, 2.5} {

            chassis_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());
            chassis_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());
            chassis_motors_[2].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());
            chassis_motors_[3].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 4}
                    .set_reduction_ratio(1.)
                    .enable_multi_turn_angle());

            drive_belt_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            drive_belt_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 4}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            yaw_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            trigger_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}
                    .set_reduction_ratio(19.)
                    .enable_multi_turn_angle());

            filling_lift_motor_[0].configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                    .enable_multi_turn_angle());
            filling_lift_motor_[1].configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4005Ei10}
                    .enable_multi_turn_angle());

            imu_.set_coordinate_mapping(
                [](double x, double y, double z) -> std::tuple<double, double, double> {
                    return {x, -y, -z};
                });

            catapult_dart.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(

                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };

            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            catapult_dart.remote_control_->register_dr16(&dr16_);

            board_ = std::make_unique<librmcs::board::RmcsBoardPro>(*this, board_serial);
        }

        void update() {
            imu_.update_status();

            dr16_.update_status();

            for (auto& i : chassis_motors_) {
                i.update_status();
            }

            for (auto& i : drive_belt_motors_) {
                i.update_status();
            }

            yaw_motor_.update_status();
            trigger_motor_.update_status();

            for (auto& i : filling_lift_motor_) {
                i.update_status();
            }
        }

        void command_update() {
            auto builder = board_->start_transmit();

            builder.gpio_analog_write(
                librmcs::spec::rmcs_board_pro::kGpioDescriptors[2],
                librmcs::data::GpioAnalogDataView{.value = trigger_servo_.generate_duty_cycle()});

            builder.can_transmit(
                Spec::kCans.kCan1, //
                {
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            chassis_motors_[0].generate_command(),
                            chassis_motors_[1].generate_command(),
                            chassis_motors_[2].generate_command(),
                            chassis_motors_[3].generate_command(),
                        }
                            .as_bytes(),
                });

            builder.can_transmit(
                Spec::kCans.kCan2, //
                {
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            yaw_motor_.generate_command(),
                            trigger_motor_.generate_command(),
                            drive_belt_motors_[0].generate_command(),
                            drive_belt_motors_[1].generate_command(),
                        }
                            .as_bytes(),
                });

            builder.can_transmit(
                Spec::kCans.kCan3,
                {
                    .can_id = 0x141,
                    .can_data = filling_lift_motor_[0].generate_velocity_command().as_bytes(),
                });

            builder.can_transmit(
                Spec::kCans.kCan3,
                {
                    .can_id = 0x145,
                    .can_data = filling_lift_motor_[1].generate_velocity_command().as_bytes(),
                });

            if (!filling_limit_servo_.calibrate_mode()) {
                uint16_t current_target = filling_limit_servo_.get_target_angle();
                if (current_target != last_limiting_angle_) {
                    size_t uart_data_length;
                    auto command_buffer =
                        filling_limit_servo_.generate_runtime_command(uart_data_length);
                    builder.uart_transmit(
                        Spec::kUarts.kUart2,
                        {.uart_data = std::span{command_buffer.get(), uart_data_length}});
                    last_limiting_angle_ = current_target;
                }
            }
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;

            if (can == Spec::kCans.kCan1) {
                if (can_id == 0x201) {
                    chassis_motors_[0].store_status(data.can_data);
                } else if (can_id == 0x202) {
                    chassis_motors_[1].store_status(data.can_data);
                } else if (can_id == 0x203) {
                    chassis_motors_[2].store_status(data.can_data);
                } else if (can_id == 0x204) {
                    chassis_motors_[3].store_status(data.can_data);
                }
            }

            if (can == Spec::kCans.kCan2) {
                if (can_id == 0x201) {
                    yaw_motor_.store_status(data.can_data);
                } else if (can_id == 0x202) {
                    trigger_motor_.store_status(data.can_data);
                } else if (can_id == 0x203) {
                    drive_belt_motors_[0].store_status(data.can_data);
                } else if (can_id == 0x204) {
                    drive_belt_motors_[1].store_status(data.can_data);
                }
            }

            if (can == Spec::kCans.kCan3) {
                if (can_id == 0x141) {
                    filling_lift_motor_[0].store_status(data.can_data);
                } else if (can_id == 0x145) {
                    filling_lift_motor_[1].store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
            } else if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        [[nodiscard]] device::Dr16& dr16() noexcept { return dr16_; }
        [[nodiscard]] const device::Dr16& dr16() const noexcept { return dr16_; }

        uint16_t last_limiting_angle_ = 0xFFFF;

        rclcpp::Logger logger_;

        device::Bmi088 imu_;
        device::Dr16 dr16_;

        device::DjiMotor chassis_motors_[4];
        device::DjiMotor drive_belt_motors_[2];
        device::DjiMotor yaw_motor_;
        device::DjiMotor trigger_motor_;

        device::LkMotor filling_lift_motor_[2];

        device::UartServo filling_limit_servo_;
        device::PWMServo trigger_servo_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        std::unique_ptr<librmcs::board::RmcsBoardPro> board_;
    };

    std::shared_ptr<DartBoard> dart_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::CatapultDart, rmcs_executor::Component)
