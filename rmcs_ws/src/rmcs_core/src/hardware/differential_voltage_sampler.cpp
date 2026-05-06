#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/k10_voltage_sampler.hpp"
#include "librmcs/agent/c_board.hpp"
#include "librmcs/agent/common.hpp"

namespace rmcs_core::hardware {

class DifferentialVoltageSampler
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    DifferentialVoltageSampler()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::CBoard{get_parameter("serial_filter").as_string()}
        , sampler_command_(
              create_partner_component<SamplerCommand>(get_component_name() + "_command", *this))
        , logger_(get_logger())
        , modbus_addr_(static_cast<uint8_t>(get_parameter("modbus_addr").as_int()))
        , poll_interval_cycles_(get_parameter("poll_interval_cycles").as_int())
        , voltage_sampler_(*this, modbus_addr_) {}

    DifferentialVoltageSampler(const DifferentialVoltageSampler&) = delete;
    DifferentialVoltageSampler& operator=(const DifferentialVoltageSampler&) = delete;
    DifferentialVoltageSampler(DifferentialVoltageSampler&&) = delete;
    DifferentialVoltageSampler& operator=(DifferentialVoltageSampler&&) = delete;
    ~DifferentialVoltageSampler() override = default;

    void update() override {
        voltage_sampler_.update_status();

        // 定期打印调试信息
        if (++debug_counter_ >= 1000) { // 每秒打印一次 (1000Hz)
            debug_counter_ = 0;
            RCLCPP_INFO(
                logger_,
                "Voltage sampler stats - Parse success: %lu, Buffer size: %zu",
                voltage_sampler_.get_parse_success_count(),
                voltage_sampler_.get_buffer_size());
        }
    }

    void command_update() {
        // 定期发送 ModBus 读取命令到 UART1
        if (++poll_counter_ >= poll_interval_cycles_) {
            poll_counter_ = 0;

            auto board = start_transmit();
            auto cmd = voltage_sampler_.generate_read_command();

            RCLCPP_INFO_THROTTLE(
                logger_,
                *get_clock(),
                1000,
                "Sending ModBus command: %zu bytes [%02X %02X %02X %02X %02X %02X %02X %02X]",
                cmd.size(),
                static_cast<uint8_t>(cmd[0]), static_cast<uint8_t>(cmd[1]),
                static_cast<uint8_t>(cmd[2]), static_cast<uint8_t>(cmd[3]),
                static_cast<uint8_t>(cmd[4]), static_cast<uint8_t>(cmd[5]),
                static_cast<uint8_t>(cmd[6]), static_cast<uint8_t>(cmd[7]));

            board.uart1_transmit({
                .uart_data = std::span{cmd.data(), cmd.size()}
            });
        }
    }

protected:
    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        RCLCPP_INFO_THROTTLE(
            logger_,
            *get_clock(),
            1000,
            "UART1 received: %zu bytes",
            data.uart_data.size());

        // 将接收到的数据传递给电压采样器解析
        voltage_sampler_.store_response(data.uart_data);
    }

private:
    class SamplerCommand : public rmcs_executor::Component {
    public:
        explicit SamplerCommand(DifferentialVoltageSampler& sampler)
            : sampler_(sampler) {}
        void update() override { sampler_.command_update(); }

    private:
        DifferentialVoltageSampler& sampler_;
    };

    std::shared_ptr<SamplerCommand> sampler_command_;
    rclcpp::Logger logger_;
    uint8_t modbus_addr_;
    int poll_interval_cycles_;
    int poll_counter_ = 0;
    int debug_counter_ = 0;

    device::K10VoltageSampler voltage_sampler_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DifferentialVoltageSampler, rmcs_executor::Component)
