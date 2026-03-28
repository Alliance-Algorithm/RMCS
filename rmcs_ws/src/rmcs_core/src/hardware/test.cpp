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


            , transmit_buffer_(*this, 32)
            , event_thread_([this]() { handle_events(); }) {}

        ~BottomBoard() final {
            stop_handling_events();
            event_thread_.join();
        }
        void before_updating() {

        }
        void update() {
   

            tof_.update();
        }
        void command_update() {

        }

    private:
 
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
                // RCLCPP_INFO(
                //     logger_, "%02x %02x %02x %02x %02x", std::to_integer<unsigned int>(rx_data[0]),
                //     std::to_integer<unsigned int>(rx_data[8]),
                //     std::to_integer<unsigned int>(rx_data[9]),
                //     std::to_integer<unsigned int>(rx_data[10]),
                //     std::to_integer<unsigned int>(rx_data[11]));

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