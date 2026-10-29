#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include <cmath>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <string>

namespace rmcs_core::hardware {

class ShootingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingTest()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , shooting_command_(create_partner_component<ShootingCommand>("shooting_command", *this))
        , muzzle_speed_monitor_(create_partner_component<MuzzleSpeedMonitor>(
              "shooting_muzzle_speed", *this,
              get_parameter("muzzle_speed_min_change").as_double()))
        , lite_board_(
              *this, *shooting_command_, get_parameter("board_serial_lite_board").as_string()) {
        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/keyboard", keyboard_, rmcs_msgs::Keyboard::zero());
        register_output("/remote/mouse", mouse_, rmcs_msgs::Mouse::zero());
        register_output(
            "/remote/rotary_knob_switch", rotary_knob_switch_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/rotary_knob", rotary_knob_, 0.0);
    }

    ~ShootingTest() override = default;
    void update() override {
        lite_board_.update();
        update_remote();
    }
    void command() { lite_board_.command(); }

private:
    // 遥控失联时开关全部置 UNKNOWN，下游控制器据此清零输出
    void update_remote() {
        const auto& dr16 = lite_board_.dr16_;
        const bool valid = dr16.valid();

        *switch_left_ = valid ? dr16.switch_left() : rmcs_msgs::Switch::UNKNOWN;
        *switch_right_ = valid ? dr16.switch_right() : rmcs_msgs::Switch::UNKNOWN;
        *joystick_left_ = valid ? dr16.joystick_left() : Eigen::Vector2d::Zero();
        *joystick_right_ = valid ? dr16.joystick_right() : Eigen::Vector2d::Zero();
        *keyboard_ = valid ? dr16.keyboard() : rmcs_msgs::Keyboard::zero();
        *mouse_ = valid ? dr16.mouse() : rmcs_msgs::Mouse::zero();
        *rotary_knob_switch_ = valid ? dr16.rotary_knob_switch() : rmcs_msgs::Switch::UNKNOWN;
        // 与 device::RemoteControl 保持一致：转发原始值，负值 = 上拨
        *rotary_knob_ = valid ? dr16.rotary_knob() : 0.0;
    }

    class ShootingCommand : public rmcs_executor::Component {
    public:
        explicit ShootingCommand(ShootingTest& shooting)
            : shooting_(shooting) {}
        void update() override { shooting_.command(); }

        ShootingTest& shooting_;
    };
    std::shared_ptr<ShootingCommand> shooting_command_;

    // 弹速来自 referee::Status，而本体输出的 /referee/serial 正是它的输入，
    // 本体直接读弹速会成环，所以放在伙伴组件里（排在 referee::Status 之后执行）。
    // 裁判的枪口速度是 float，ValueBroadcaster 只支持 double，这里顺便转一手。
    class MuzzleSpeedMonitor : public rmcs_executor::Component {
    public:
        MuzzleSpeedMonitor(ShootingTest& shooting, double min_change)
            : shooting_(shooting)
            , min_change_(min_change) {
            register_input("/referee/shooter/initial_speed", initial_speed_);
            register_output("/shooting/muzzle_speed", muzzle_speed_, 0.0);
        }

        void update() override {
            const auto speed = static_cast<double>(*initial_speed_);
            *muzzle_speed_ = speed;

            // 裁判只在开火后更新这个值，速度不变说明没有新的一发
            if (std::abs(speed - last_logged_) < min_change_)
                return;
            last_logged_ = speed;

            RCLCPP_INFO(shooting_.get_logger(), "muzzle speed: %.2f m/s", speed);
        }

    private:
        ShootingTest& shooting_;
        const double min_change_;

        double last_logged_ = 0.0;

        InputInterface<float> initial_speed_;
        OutputInterface<double> muzzle_speed_;
    };
    std::shared_ptr<MuzzleSpeedMonitor> muzzle_speed_monitor_;

    class LiteBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        friend class ShootingTest;
        explicit LiteBoard(
            ShootingTest& shooting, ShootingCommand& shooting_command,
            const std::string& serial_filter)
            : friction_motors_(
                  {shooting, shooting_command, "/shooting/friction_motor/upper"},
                  {shooting, shooting_command, "/shooting/friction_motor/left"},
                  {shooting, shooting_command, "/shooting/friction_motor/right"})
            , gripper_motor_(shooting, shooting_command, "/shooting/gripper_motor")
            , bullet_feeder_motor_(shooting, shooting_command, "/gimbal/bullet_feeder") {
            friction_motors_[0].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.set_reduction_ratio(1.0));
            friction_motors_[1].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    1.0));
            friction_motors_[2].configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}.reverse().set_reduction_ratio(
                    1.0));
            gripper_motor_.configure(
                device::DjiMotorConfig{device::DjiMotorType::M3508}
                    .enable_multi_turn_angle()
                    .set_reduction_ratio(1.0));
            bullet_feeder_motor_.configure(
                device::LKMotorConfig{device::LKMotorType::MG5010E_i10V3}
                    .reverse()
                    .enable_multi_turn_angle());

            rmcs_msgs::SerialInterface referee_serial;
            referee_serial.read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial.write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = {buffer, size}});
                return size;
            };
            shooting.register_output("/referee/serial", referee_serial_, std::move(referee_serial));

            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, static_cast<std::string>(serial_filter), librmcs::board::AdvancedOptions{});
            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 115200});
        }
        ~LiteBoard() final {
            // 连发多帧零力矩：总线忙时可能丢帧，只发一次有概率留下最后一帧非零
            auto tx = board_->start_transmit();
            for (int i = 0; i < 10; i++) {
                tx.can_transmit(
                    Spec::kCans.kCan1, {
                                           .can_id = 0x200,
                                           .can_data =
                                               device::CanPacket8{
                                                   device::CanPacket8::PaddingQuarter{},
                                                   device::CanPacket8::PaddingQuarter{},
                                                   device::CanPacket8::PaddingQuarter{},
                                                   device::CanPacket8::PaddingQuarter{},
                                               }
                                                   .as_bytes(),
                                       });
                tx.can_transmit(
                    Spec::kCans.kCan3,
                    {
                        .can_id = 0x142,
                        .can_data = bullet_feeder_motor_.lk_zero_torque_command().as_bytes(),
                    });
            }
        }

        void update() {
            for (auto& motor : friction_motors_)
                motor.update();
            gripper_motor_.update();
            bullet_feeder_motor_.update();
            dr16_.update_status();
        }

        void command() {
            auto tx = board_->start_transmit();
            tx.can_transmit(
                Spec::kCans.kCan1, {
                                       .can_id = 0x200,
                                       .can_data =
                                           device::CanPacket8{
                                               friction_motors_[0].generate_command(),
                                               friction_motors_[1].generate_command(),
                                               friction_motors_[2].generate_command(),
                                               gripper_motor_.generate_command(),
                                           }
                                               .as_bytes(),
                                   });
            tx.can_transmit(
                Spec::kCans.kCan3,
                {
                    .can_id = 0x142,
                    .can_data = bullet_feeder_motor_.generate_torque_command().as_bytes(),
                });
        }

    protected:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission)
                [[unlikely]]
                return;
            if (can == Spec::kCans.kCan1) {
                if (data.can_id == 0x201) {
                    friction_motors_[0].store_status(data.can_data);
                } else if (data.can_id == 0x202) {
                    friction_motors_[1].store_status(data.can_data);
                } else if (data.can_id == 0x203) {
                    friction_motors_[2].store_status(data.can_data);
                } else if (data.can_id == 0x204) {
                    gripper_motor_.store_status(data.can_data);
                }
            } else if (can == Spec::kCans.kCan3) {
                if (data.can_id == 0x142) {
                    bullet_feeder_motor_.store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data);
            } else if (uart == Spec::kUarts.kUart0) {
                auto* uart_data = data.uart_data.data();
                referee_ring_buffer_.push_back_n(
                    [&uart_data]() noexcept { return *uart_data++; }, data.uart_data.size());
            }
        }

    private:
        device::DjiMotor friction_motors_[3];
        device::DjiMotor gripper_motor_;
        device::LKMotor bullet_feeder_motor_;
        device::Dr16 dr16_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_{4096};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        // 放在最后：析构时最先销毁，保证回调线程停下后其他成员才析构
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    } lite_board_;

    OutputInterface<rmcs_msgs::Switch> switch_left_;
    OutputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<Eigen::Vector2d> joystick_left_;
    OutputInterface<Eigen::Vector2d> joystick_right_;
    OutputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<rmcs_msgs::Mouse> mouse_;
    OutputInterface<rmcs_msgs::Switch> rotary_knob_switch_;
    OutputInterface<double> rotary_knob_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ShootingTest, rmcs_executor::Component)
