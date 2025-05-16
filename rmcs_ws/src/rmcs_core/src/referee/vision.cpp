#include <eigen3/Eigen/Eigen>
#include <keyboard.hpp>
#include <mouse.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>
#include <serial_interface.hpp>

#include "referee/frame.hpp"

#include <serial/serial.h>

namespace rmcs_core::referee {

class Vision
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Vision()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {

        std::string path;
        if (get_parameter("path", path))
            RCLCPP_INFO(get_logger(), "Path: %s", path.c_str());
        else
            throw std::runtime_error{"Unable to get parameter 'path'"};

        const std::string stty_command = "stty -F " + path + " raw";
        if (std::system(stty_command.c_str()) != 0)
            throw std::runtime_error{"Unable to call '" + stty_command + "'"};
        // register_input("/referee/vision/serial", serial_);
        register_output("/referee/vision/serial", serial_, path, 921600, serial::Timeout::simpleTimeout(0));

        register_output("/referee/vision/custom", custom_data_ );
        register_output("/referee/vision/controller_keyboard", controller_keyboard_, rmcs_msgs::Keyboard::zero());
        register_output("/referee/vision/controller_mouse", controller_mouse_, rmcs_msgs::Mouse::zero());

        std::fill((*custom_data_).begin(), (*custom_data_).end(), 0);
        
        custom_watchdog_.reset(5'000);
        controller_watchdog_.reset(5'000);
    }

    void update() override {

        if (!serial_.active() || !serial_->available())
            return;

        if (cache_size_ >= sizeof(frame_.header)) {
            
            auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                            + frame_.header.data_length + sizeof(uint16_t);
            cache_size_ += serial_->read(
                reinterpret_cast<uint8_t*>(&frame_) + cache_size_, frame_size - cache_size_);

            if (cache_size_ == frame_size) {
                cache_size_ = 0;
                if (rmcs_utility::dji_crc::verify_crc16(&frame_, frame_size)) {
                    process_frame();
                } else {
                    RCLCPP_WARN(logger_, "Body crc16 invalid");
                }
            }
        } else {
            auto result = rmcs_utility::receive_package<uint8_t>(
                *serial_, frame_.header, cache_size_,
                static_cast<uint8_t>(0xa5), [](const FrameHeader& header) {
                    return rmcs_utility::dji_crc::verify_crc8(header);
                });
            if (result == rmcs_utility::ReceiveResult::HEADER_INVALID) {
                // RCLCPP_WARN(logger_, "Header start invalid");
            } else if (result == rmcs_utility::ReceiveResult::VERIFY_INVALID) {
                // RCLCPP_WARN(logger_, "Header crc8 invalid");
            } else if (result == rmcs_utility::ReceiveResult::TIMEOUT) {
                // RCLCPP_WARN(logger_, "Referee vision package receive timeout.");
            } else {
                // RCLCPP_WARN(logger_, "Referee vision package receive success.");
            }
        }

        if (custom_watchdog_.tick()) {
            RCLCPP_ERROR(logger_, "Vision custom data receiving timeout. Set data to zero.");
            std::fill((*custom_data_).begin(), (*custom_data_).end(), 0);
        }
        if (controller_watchdog_.tick()) {
            RCLCPP_ERROR(logger_, "Vision controller data receiving timeout. Set data to zero.");
            *controller_keyboard_ = rmcs_msgs::Keyboard::zero();
            *controller_mouse_ = rmcs_msgs::Mouse::zero();
        }
    }

private:
    void process_frame() {
        auto command_id = frame_.body.command_id;

        if (command_id == 0x0302)
        // RCLCPP_INFO(logger_, "Receiving cmannd_id %04x.", command_id);
            update_custom_data();
        if (command_id == 0x0304)
            update_controller_data();
    }

    void update_custom_data() {
        std::copy(
            reinterpret_cast<const uint8_t*>(frame_.body.data),
            reinterpret_cast<const uint8_t*>(frame_.body.data) + frame_.header.data_length,
            (*custom_data_).begin()
        );
        // RCLCPP_INFO(this->get_logger(),"%d",frame_.header.data_length);
        // auto custom_data = *custom_data_;
        // std::ostringstream oss;
        // oss << std::hex << std::setfill('0'); // 设置十六进制和补零格式
        // for (size_t i = 0; i < custom_data.size(); ++i) {
        //     // 将每个字节转为两位十六进制，并添加空格（最后一个不加）
        //     oss << std::setw(2) << static_cast<int>(custom_data[i]);
        //     if (i != custom_data.size() - 1) {
        //         oss << " ";
        //     }
        // }
        // std::string hex_str = oss.str();
        // RCLCPP_INFO(get_logger(), "%s", hex_str.c_str());
        custom_watchdog_.reset(500);
    }

    void update_controller_data() {
        controller_watchdog_.reset(500);

        // TODO
        // auto data = frame_.body.data;
    }
    rclcpp::Logger logger_;

    OutputInterface<serial::Serial> serial_;
    // InputInterface<rmcs_msgs::SerialInterface> serial_;
    Frame frame_;
    size_t cache_size_ = 0;

    rmcs_utility::TickTimer custom_watchdog_;
    OutputInterface<std::array<uint8_t, 30>> custom_data_;

    rmcs_utility::TickTimer controller_watchdog_;
    OutputInterface<rmcs_msgs::Keyboard> controller_keyboard_;
    OutputInterface<rmcs_msgs::Mouse> controller_mouse_;
};

} // namespace rmcs_core::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::Vision, rmcs_executor::Component)