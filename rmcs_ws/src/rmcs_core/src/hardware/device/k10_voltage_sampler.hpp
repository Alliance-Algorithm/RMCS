#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <rmcs_executor/component.hpp>
#include <span>
#include <vector>

namespace rmcs_core::hardware::device {

// K10-3U4D16D 差分电压采样器 ModBus RTU 协议封装
class K10VoltageSampler {
public:
    // ModBus 寄存器地址
    static constexpr uint16_t REG_AI1_ENGINEERING = 0x0010; // AI1 工程量 (mV)
    static constexpr uint16_t REG_AI2_ENGINEERING = 0x0011; // AI2 工程量 (mV)
    static constexpr uint16_t REG_AI3_ENGINEERING = 0x0012; // AI3 工程量 (mV)
    static constexpr uint16_t REG_AI4_ENGINEERING = 0x0013; // AI4 工程量 (mV)

    // ModBus 功能码
    static constexpr uint8_t FUNC_READ_HOLDING_REG = 0x03; // 读保持寄存器

    explicit K10VoltageSampler(
        rmcs_executor::Component& status_component, uint8_t modbus_addr = 0x01)
        : modbus_addr_(modbus_addr) {

        status_component.register_output(
            "/differential_sampler/channel_1/voltage", voltage_ch1_, 0.0);
        status_component.register_output(
            "/differential_sampler/channel_2/voltage", voltage_ch2_, 0.0);
        status_component.register_output(
            "/differential_sampler/channel_3/voltage", voltage_ch3_, 0.0);
        status_component.register_output(
            "/differential_sampler/channel_4/voltage", voltage_ch4_, 0.0);
    }

    // 存储从 UART1 接收到的数据
    void store_response(std::span<const std::byte> uart_data) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);

        // 将数据追加到接收缓冲区
        for (auto byte : uart_data) {
            receive_buffer_.push_back(static_cast<uint8_t>(byte));

            // 限制缓冲区大小，防止溢出
            if (receive_buffer_.size() > 256) {
                receive_buffer_.erase(receive_buffer_.begin());
            }
        }

        // 尝试解析完整的响应
        bool parsed = parse_response();

        if (parsed) {
            parse_success_count_++;
        }
    }

    // 更新输出接口
    void update_status() {
        auto voltages = voltages_.load(std::memory_order_relaxed);
        *voltage_ch1_ = voltages[0] / 1000.0; // mV -> V
        *voltage_ch2_ = voltages[1] / 1000.0;
        *voltage_ch3_ = voltages[2] / 1000.0;
        *voltage_ch4_ = voltages[3] / 1000.0;
    }

    uint64_t get_parse_success_count() const { return parse_success_count_; }
    size_t get_buffer_size() const {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        return receive_buffer_.size();
    }

    // 生成读取工程量命令 (返回字节数组用于 UART1 发送)
    std::vector<std::byte> generate_read_command() const {
        std::vector<uint8_t> cmd = {
            modbus_addr_,          // ModBus 地址
            FUNC_READ_HOLDING_REG, // 功能码 0x03
            0x00,
            0x10,                  // 起始地址 0x0010
            0x00,
            0x04                   // 读取 4 个寄存器
        };
        append_crc16(cmd);

        // 转换为 std::byte
        std::vector<std::byte> result;
        result.reserve(cmd.size());
        for (auto byte : cmd) {
            result.push_back(static_cast<std::byte>(byte));
        }
        return result;
    }

private:
    bool parse_response() {
        // 最小响应长度: 地址(1) + 功能码(1) + 字节数(1) + 数据(8) + CRC(2) = 13
        if (receive_buffer_.size() < 13) {
            return false;
        }

        // 查找有效的响应帧
        for (size_t i = 0; i + 13 <= receive_buffer_.size(); ++i) {
            if (receive_buffer_[i] == modbus_addr_
                && receive_buffer_[i + 1] == FUNC_READ_HOLDING_REG
                && receive_buffer_[i + 2] == 0x08) { // 8 字节数据

                // 提取完整帧
                std::vector<uint8_t> frame(
                    receive_buffer_.begin() + i, receive_buffer_.begin() + i + 13);

                // 验证 CRC
                if (verify_crc16(frame)) {
                    // 解析 4 个通道的电压值 (大端序)
                    std::array<uint16_t, 4> voltages;
                    for (int ch = 0; ch < 4; ch++) {
                        voltages[ch] = (static_cast<uint16_t>(frame[3 + ch * 2]) << 8)
                                     | static_cast<uint16_t>(frame[4 + ch * 2]);
                    }

                    // 原子更新
                    voltages_.store(voltages, std::memory_order_relaxed);

                    // 清除已处理的数据
                    receive_buffer_.erase(
                        receive_buffer_.begin(), receive_buffer_.begin() + i + 13);
                    return true;
                }
            }
        }

        // 如果缓冲区太大但没有找到有效帧，清除旧数据
        if (receive_buffer_.size() > 64) {
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + 32);
        }

        return false;
    }

    // ModBus CRC16 计算
    static uint16_t calculate_crc16(const std::vector<uint8_t>& data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // 添加 CRC16 校验码
    static void append_crc16(std::vector<uint8_t>& cmd) {
        uint16_t crc = calculate_crc16(cmd, cmd.size());
        cmd.push_back(crc & 0xFF);        // CRC 低字节
        cmd.push_back((crc >> 8) & 0xFF); // CRC 高字节
    }

    // 验证 CRC16 校验码
    static bool verify_crc16(const std::vector<uint8_t>& data) {
        if (data.size() < 3)
            return false;
        uint16_t received_crc = data[data.size() - 2] | (data[data.size() - 1] << 8);
        uint16_t calculated_crc = calculate_crc16(data, data.size() - 2);
        return received_crc == calculated_crc;
    }

    uint8_t modbus_addr_;

    mutable std::mutex buffer_mutex_;
    std::vector<uint8_t> receive_buffer_;

    std::atomic<std::array<uint16_t, 4>> voltages_{
        std::array<uint16_t, 4>{0, 0, 0, 0}
    };
    static_assert(decltype(voltages_)::is_always_lock_free);

    std::atomic<uint64_t> parse_success_count_{0};

    rmcs_executor::Component::OutputInterface<double> voltage_ch1_;
    rmcs_executor::Component::OutputInterface<double> voltage_ch2_;
    rmcs_executor::Component::OutputInterface<double> voltage_ch3_;
    rmcs_executor::Component::OutputInterface<double> voltage_ch4_;
};

} // namespace rmcs_core::hardware::device
