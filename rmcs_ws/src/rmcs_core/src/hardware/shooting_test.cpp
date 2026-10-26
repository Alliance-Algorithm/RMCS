#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <librmcs/board/rmcs_board_lite.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmcs_core::hardware {

class ShootingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ShootingTest()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_(create_partner_component<ShootingCommand>("shooting_command", *this))
        , board_(*this, *command_) {

        register_output("/remote/switch/left", switch_left_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", switch_right_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());
        register_output("/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        register_output("/remote/keyboard", keyboard_, rmcs_msgs::Keyboard::zero());
        register_output("/remote/mouse", mouse_, rmcs_msgs::Mouse::zero());
        register_output("/remote/rotary_knob_switch", knob_switch_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/rotary_knob", knob_value_, 0.0);

        register_output("/shooting/hardware_ready", hardware_ready_, false);
    }

    ~ShootingTest() override = default;

    void update() override {
        board_.update();

        const auto& dr16 = board_.dr16();
        const bool valid = dr16.valid();

        *switch_left_ = valid ? dr16.switch_left() : rmcs_msgs::Switch::UNKNOWN;
        *switch_right_ = valid ? dr16.switch_right() : rmcs_msgs::Switch::UNKNOWN;
        *knob_switch_ = valid ? dr16.rotary_knob_switch() : rmcs_msgs::Switch::UNKNOWN;
        // 与 device::RemoteControl 保持一致：转发原始值，负值 = 上拨
        *knob_value_ = valid ? dr16.rotary_knob() : 0.0;
        *joystick_left_ = valid ? dr16.joystick_left() : Eigen::Vector2d::Zero();
        *joystick_right_ = valid ? dr16.joystick_right() : Eigen::Vector2d::Zero();
        *keyboard_ = valid ? dr16.keyboard() : rmcs_msgs::Keyboard::zero();
        *mouse_ = valid ? dr16.mouse() : rmcs_msgs::Mouse::zero();

        const bool ready = board_.ready();
        if (ready != *hardware_ready_)
            RCLCPP_INFO(get_logger(), "hardware %s", ready ? "ready" : "not ready");
        *hardware_ready_ = ready;
    }

    void command() { board_.command(); }

private:
    class ShootingCommand : public rmcs_executor::Component {
    public:
        explicit ShootingCommand(ShootingTest& shooting)
            : shooting_(shooting) {}

        void update() override { shooting_.command(); }

    private:
        ShootingTest& shooting_;
    };

    class Board final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        Board(ShootingTest& shooting, ShootingCommand& command)
            : shooting_(shooting)
            , command_(command)
            , friction_motors_{
                  {shooting, command, "/shooting/friction_motor/upper"},
                  {shooting, command, "/shooting/friction_motor/left"},
                  {shooting, command, "/shooting/friction_motor/right"}}
            , gripper_motor_{shooting, command, "/shooting/gripper_motor"}
            , bullet_feeder_motor_{shooting, command, "/gimbal/bullet_feeder"} {

            const auto dji_channel = shooting.get_parameter("dji_can_channel").as_int();
            const auto feeder_channel = shooting.get_parameter("feeder_can_channel").as_int();
            const auto channel_count = static_cast<int64_t>(Spec::kCans.size());
            if (dji_channel < 0 || dji_channel >= channel_count || feeder_channel < 0
                || feeder_channel >= channel_count || dji_channel == feeder_channel)
                throw std::invalid_argument(
                    "dji_can_channel and feeder_can_channel must be distinct and in range");
            dji_channel_ = static_cast<size_t>(dji_channel);
            feeder_channel_ = static_cast<size_t>(feeder_channel);

            const auto ids = shooting.get_parameter("dji_motor_ids").as_integer_array();
            const auto ratios = shooting.get_parameter("dji_reduction_ratios").as_double_array();
            const auto reversed = shooting.get_parameter("dji_reversed").as_bool_array();
            if (ids.size() != 4 || ratios.size() != 4 || reversed.size() != 4)
                throw std::invalid_argument(
                    "dji_motor_ids/dji_reduction_ratios/dji_reversed must hold 4 entries");

            for (size_t i = 0; i < 4; ++i) {
                if (ids[i] < 1 || ids[i] > 8 || !std::isfinite(ratios[i]) || ratios[i] <= 0)
                    throw std::invalid_argument("invalid dji motor id or reduction ratio");
                for (size_t j = 0; j < i; ++j)
                    if (ids[i] == ids[j])
                        throw std::invalid_argument("dji motor ids must be unique per can bus");
                motor_ids_[i] = static_cast<unsigned>(ids[i]);

                auto config = device::DjiMotorConfig{device::DjiMotorType::M3508};
                config.set_reduction_ratio(ratios[i]);
                if (reversed[i])
                    config.reverse();
                if (i == 3)
                    config.enable_multi_turn_angle();
                dji_motor(i).configure(config);
            }

            const auto feeder_id = shooting.get_parameter("feeder_can_id").as_int();
            const auto feeder_ratio = shooting.get_parameter("feeder_gear_ratio").as_double();
            if (feeder_id < 0x141 || feeder_id > 0x160 || !std::isfinite(feeder_ratio)
                || feeder_ratio <= 0)
                throw std::invalid_argument("invalid feeder can id or gear ratio");
            feeder_can_id_ = static_cast<uint32_t>(feeder_id);

            const auto feeder_type = shooting.get_parameter("feeder_motor_type").as_string();
            auto feeder_config = device::LKMotorConfig{
                feeder_type == "MG4010E_i10V3" ? device::LKMotorType::MG4010E_i10V3
                                               : device::LKMotorType::MG5010E_i10V3};
            feeder_config.set_gear_ratio(feeder_ratio).enable_multi_turn_angle();
            if (shooting.get_parameter("feeder_reversed").as_bool())
                feeder_config.reverse();
            bullet_feeder_motor_.configure(feeder_config);

            feedback_timeout_ = shooting.get_parameter("feedback_timeout").as_double();
            if (!std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0)
                throw std::invalid_argument("feedback_timeout must be positive");

            const auto referee_uart = shooting.get_parameter("referee_uart").as_int();
            if (referee_uart < 0 || referee_uart + 1 >= static_cast<int64_t>(Spec::kUarts.size()))
                throw std::invalid_argument("referee_uart out of range");
            referee_uart_ = static_cast<size_t>(referee_uart) + 1;

            rmcs_msgs::SerialInterface referee_serial;
            referee_serial.read = [this](std::byte* destination, size_t count) {
                return referee_rx_.pop_front_n(
                    [&destination](std::byte value) noexcept { *destination++ = value; }, count);
            };
            referee_serial.write = [this](const std::byte* source, size_t count) {
                if (board_ == nullptr)
                    return size_t{0};
                board_->start_transmit().uart_transmit(
                    Spec::kUarts[referee_uart_], {.uart_data = {source, count}});
                return count;
            };
            shooting.register_output("/referee/serial", referee_serial_, std::move(referee_serial));

            // 板子创建后回调可能立刻触发，所以必须放在所有成员都构造完之后
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(
                *this, shooting.get_parameter("board_serial_lite_board").as_string());
            board_->start_transmit().uart_config(Spec::kUarts[referee_uart_], {.baudrate = 115200});
        }

        ~Board() override {
            try {
                // 连发多帧零力矩：总线忙时可能丢帧，只发一次有概率留下最后一帧非零
                for (int i = 0; i < 10; ++i)
                    transmit(false);
            } catch (...) {
            }
            board_.reset();
        }

        void update() {
            const auto now = Clock::now();
            ready_ = true;

            for (size_t i = 0; i < 5; ++i) {
                const auto received = last_receive_[i].load(std::memory_order_acquire);
                ready_ &= received != Clock::time_point{}
                       && std::chrono::duration<double>(now - received).count()
                              <= feedback_timeout_;
                if (received == Clock::time_point{})
                    continue;
                if (i < 4)
                    dji_motor(i).update();
                else
                    bullet_feeder_motor_.update();
            }

            dr16_.update_status();
        }

        void command() { transmit(ready_ && dr16_.valid()); }

        const device::Dr16& dr16() const { return dr16_; }
        bool ready() const { return ready_; }

    protected:
        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
                || data.can_data.size() != 8)
                return;

            if (can == Spec::kCans[dji_channel_]) {
                for (size_t i = 0; i < 4; ++i) {
                    if (data.can_id == 0x200 + motor_ids_[i]) {
                        dji_motor(i).store_status(data.can_data);
                        last_receive_[i].store(Clock::now(), std::memory_order_release);
                        return;
                    }
                }
            } else if (can == Spec::kCans[feeder_channel_] && data.can_id == feeder_can_id_) {
                const auto status = std::to_integer<uint8_t>(data.can_data.front());
                if (status == 0x9C || status == 0xA4 || status == 0xA1 || status == 0xAD) {
                    bullet_feeder_motor_.store_status(data.can_data);
                    last_receive_[4].store(Clock::now(), std::memory_order_release);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data);
            } else if (uart == Spec::kUarts[referee_uart_]) {
                auto* cursor = data.uart_data.data();
                referee_rx_.push_back_n(
                    [&cursor]() noexcept { return *cursor++; }, data.uart_data.size());
            }
        }

    private:
        using Clock = std::chrono::steady_clock;

        device::DjiMotor& dji_motor(size_t index) {
            return index < 3 ? friction_motors_[index] : gripper_motor_;
        }

        void transmit(bool enabled) {
            std::array<device::CanPacket8, 2> frames{
                device::CanPacket8{uint64_t{0}}, device::CanPacket8{uint64_t{0}}};
            std::array<bool, 2> used{};

            for (size_t i = 0; i < 4; ++i) {
                // 大疆一帧装 4 个电机：0x200 对应 ID1~4、0x1FF 对应 ID5~8，ID 决定槽位
                const auto slot = motor_ids_[i] - 1;
                used[slot / 4] = true;
                if (enabled)
                    frames[slot / 4].data[slot % 4] = dji_motor(i).generate_command().data;
            }

            auto tx = board_->start_transmit();

            for (size_t group = 0; group < 2; ++group)
                if (used[group])
                    tx.can_transmit(
                        Spec::kCans[dji_channel_],
                        {.can_id = group == 0 ? 0x200u : 0x1FFu,
                         .can_data = frames[group].as_bytes()});

            tx.can_transmit(
                Spec::kCans[feeder_channel_],
                {.can_id = feeder_can_id_,
                 .can_data = (enabled ? bullet_feeder_motor_.generate_torque_command()
                                      : device::LKMotor::lk_zero_torque_command())
                                 .as_bytes()});
        }

        ShootingTest& shooting_;
        ShootingCommand& command_;

        device::DjiMotor friction_motors_[3];
        device::DjiMotor gripper_motor_;
        device::LKMotor bullet_feeder_motor_;
        device::Dr16 dr16_;

        size_t dji_channel_{}, feeder_channel_{1}, referee_uart_{1};
        uint32_t feeder_can_id_{0x141};
        std::array<unsigned, 4> motor_ids_{};
        double feedback_timeout_{0.1};
        bool ready_{};

        std::array<std::atomic<Clock::time_point>, 5> last_receive_{};
        rmcs_utility::RingBuffer<std::byte> referee_rx_{4096};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    std::shared_ptr<ShootingCommand> command_;
    Board board_;

    OutputInterface<bool> hardware_ready_;
    OutputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    OutputInterface<rmcs_msgs::Switch> knob_switch_;
    OutputInterface<double> knob_value_;
    OutputInterface<Eigen::Vector2d> joystick_left_, joystick_right_;
    OutputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<rmcs_msgs::Mouse> mouse_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ShootingTest, rmcs_executor::Component)
