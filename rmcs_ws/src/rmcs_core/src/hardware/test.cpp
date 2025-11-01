#include "hardware/device/tof.hpp"
#include <librmcs/client/cboard.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/int32.hpp>
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
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
            // , tof_(test, "/tof")
           
            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) 
            ,dr16_(test){}

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }

        void update() {
            //    RCLCPP_INFO(logger_, "Received UART1 data");

            // tof_.update_status();
        }
        void command_update() {
              uint64_t command_;
                 command_ = std::bit_cast<uint64_t>(uint64_t{0x9C});
                  transmit_buffer_.add_can1_transmission(
                    0x142, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                     transmit_buffer_.add_can1_transmission(
                    0x141, std::bit_cast<uint64_t>(std::bit_cast<uint64_t>(uint64_t{command_})));
                    transmit_buffer_.trigger_transmission();
        }

    protected:
    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            dr16_.store_status(uart_data, uart_data_length);
            RCLCPP_INFO(logger_, "Received DBUS data");
        }
        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;
  RCLCPP_INFO(logger_, "%x",can_id);
               
        }

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

        // void uart2_receive_callback(const std::byte* data, uint8_t length) override {
        //     // 把收到的 data 写入环形缓冲（用值捕获，mutable 允许在 lambda 中修改 data 指针）
        //     ring_buff.emplace_back_multi(
        //         [data](std::byte* storage) mutable { *storage = *data++; }, length);

        //     // 如果缓冲有数据并且至少能组成一帧（16 字节），就尝试解析（可能连续解析多帧）
        //     if (ring_buff.front() && ring_buff.readable() >= 16) {
        //         // 先读取并弹出第一个字节，作为可能的帧头
        //         std::byte rx_data[16];
        //         auto prt= &rx_data[0];

        //         // 先弹出一个字节到 rx_data[0]
        //         ring_buff.pop_front_multi([&prt](std::byte storage) { *prt++ = storage; }, 16);
        //         // 打印收到的前 5 个字节（以两位十六进制显示），用于调试
                

        //         // 把整帧数据传给解析函数（传入 rx_data 的起始地址）
        //         tof_.store_status(rx_data, 16);
        //     }
        // }

        rclcpp::Logger logger_;

        // rmcs_core::hardware::device::Tof tof_;
        librmcs::utility::RingBuffer<std::byte> ring_buff{255};
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        std::thread event_thread_;
        device::Dr16 dr16_;
    };

    std::unique_ptr<BottomBoard> bottom_board_;
};

} // namespace rmcs_core::hardware
#include <pluginlib/class_list_macros.hpp>
#include <utility>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Test, rmcs_executor::Component)