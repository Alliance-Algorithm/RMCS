#include "hardware/device/tof.hpp"
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
namespace rmcs_core::hardware {

class Test
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Test()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(
              create_partner_component<TestCommand>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_logger(),
            static_cast<int>(get_parameter("usb_pid_bottom_board").as_int()));
    }

    ~Test() override = default;

    void command_update() { bottom_board_->command_update(); }

    void before_updating() override { bottom_board_->before_updating(); }
    void update() override { bottom_board_->update(); }

private:
    class TestCommand : public rmcs_executor::Component {
    public:
        explicit TestCommand(Test& test)
            : test_(test) {}

        void update() override { test_.command_update(); }

        Test& test_;
    };

    std::shared_ptr<TestCommand> command_component_;

    class BottomBoard final : private librmcs::client::CBoard {
    public:
        friend class Test;
        explicit BottomBoard(
            Test& test, TestCommand& test_command, rclcpp::Logger logger, int usb_pid = -1)
            : librmcs::client::CBoard(usb_pid)
            , logger_(std::move(logger))
            , tof_(test, "/tof")
            // , gantry_double_motors_(
            //       {test, test_command, "/gantry/left_motor",
            //        device::DjiMotor::Config{device::DjiMotor::Type::M2006}
            //            .enable_multi_turn_angle()
            //            .set_reversed()},
            //       {test, test_command, "/gantry/right_motor",
            //        device::DjiMotor::Config{device::DjiMotor::Type::M2006}
            //            .enable_multi_turn_angle()
            //            .set_reversed()})

            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {}

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void before_updating() {

            // tof_.store_status();
            // bool valid= tof_.get_valid();
            // RCLCPP_INFO(logger_, "tof valid: %s", valid ? "true" : "false");
        }
        void update() {
            //    RCLCPP_INFO(logger_, "Received UART1 data");

            tof_.update_status();
        }
        void command_update() {
            // uint16_t can_commands[4];

            // can_commands[0] = 0;
            // can_commands[1] = 0;
            // for (int i = 0; i < 2; i++)
            //     can_commands[i+2] = gantry_double_motors_[i].generate_command();
            // transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            // can_commands[0] = 0;
            // can_commands[1] = 0;
            // can_commands[2] = 0;
            // can_commands[3] = 0;
            // transmit_buffer_.add_can1_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            // can_commands[0] = 0;
            // can_commands[1] = 0;
            // can_commands[2] = 0;
            // can_commands[3] = 0;
            // transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

            // can_commands[0] = 0;
            // can_commands[1] = 0;
            // can_commands[2] = 0;
            // can_commands[3] = 0;
            // transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

            // transmit_buffer_.trigger_transmission();
        }

    private:
        // void can1_receive_callback(
        //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        //     bool is_remote_transmission, uint8_t can_data_length) override {
        //     if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
        //         return;

        //     // if (can_id == 0x201) {
        //     //     gantry_double_motors_[0].store_status(can_data);
        //     // } else if (can_id == 0x202) {
        //     //     gantry_double_motors_[1].store_status(can_data);
        //     // }
        //     if (can_id == 0x203) {
        //         gantry_double_motors_[0].store_status(can_data);
        //     } else if (can_id == 0x204) {
        //         gantry_double_motors_[1].store_status(can_data);
        //     }
        // }

        //         void uart2_receive_callback(const std::byte* data, uint8_t length) override {
        // //            RCLCPP_INFO(logger_, "%x  %d ",*(data),length);

        //             ring_buff.emplace_back_multi([&data](std::byte* storage) { *storage =
        //             *data++; }, length);

        //             auto* front = ring_buff.front();

        //             if (front){
        //                 if (ring_buff.readable() >= 16) {
        //                     RCLCPP_INFO(logger_, "Received buff data");
        //                     std::byte rx_data[16];
        //                     std::byte* rx_data_ptr = &rx_data[0];
        //                     ring_buff.pop_front_multi(
        //                         [&rx_data_ptr](std::byte storage) { *rx_data_ptr++ = storage; },
        //                         16);

        //                     RCLCPP_INFO(logger_, "%02x  %02x %02x %02x %02x
        //                     ",static_cast<unsigned int>(rx_data[0]),static_cast<unsigned
        //                     int>(rx_data[1]),static_cast<unsigned
        //                     int>(rx_data[2]),static_cast<unsigned
        //                     int>(rx_data[3]),static_cast<unsigned int>(rx_data[4]));
        //                     tof_.store_status(rx_data, 16);
        //                 }
        //             }
        //         }

        void uart2_receive_callback(const std::byte* data, uint8_t length) override {
            // 把收到的 data 写入环形缓冲（用值捕获，mutable 允许在 lambda 中修改 data 指针）
            ring_buff.emplace_back_multi(
                [data](std::byte* storage) mutable { *storage = *data++; }, length);

            // 如果缓冲有数据并且至少能组成一帧（16 字节），就尝试解析（可能连续解析多帧）
            while (ring_buff.front() && ring_buff.readable() >= 16) {
                // 先读取并弹出第一个字节，作为可能的帧头
                std::byte rx_data[16];
                std::byte* rx_ptr = rx_data;

                // 先弹出一个字节到 rx_data[0]
                ring_buff.pop_front_multi([&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 1);

                // 如果第一个字节不是帧头 0x57，则丢弃它并继续（寻找下一个可能的帧头）
                if (rx_data[0] != std::byte{0x57}) {
                    // 不是帧头：继续循环，下一次会检查新的第一个字节
                    continue;
                }

                // 已确认帧头，弹出剩余 15 字节组成完整 16 字节帧
                ring_buff.pop_front_multi(
                    [&rx_ptr](std::byte storage) { *rx_ptr++ = storage; }, 15);

                // 打印收到的前 5 个字节（以两位十六进制显示），用于调试
                RCLCPP_INFO(
                    logger_, "%02x %02x %02x %02x %02x", std::to_integer<unsigned int>(rx_data[0]),
                    std::to_integer<unsigned int>(rx_data[8]),
                    std::to_integer<unsigned int>(rx_data[9]),
                    std::to_integer<unsigned int>(rx_data[10]),
                    std::to_integer<unsigned int>(rx_data[11]));

                // 把整帧数据传给解析函数（传入 rx_data 的起始地址）
                tof_.store_status(rx_data, 16);
            }
        }

        rclcpp::Logger logger_;

        rmcs_core::hardware::device::Tof tof_;
        librmcs::utility::RingBuffer<std::byte> ring_buff{16};
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
    };

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)