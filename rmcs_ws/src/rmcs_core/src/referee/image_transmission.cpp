#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "referee/frame.hpp"

#include <serial/serial.h>

namespace rmcs_core::referee {

class ImageTransmissionLink
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ImageTransmissionLink()
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

        register_output(
            "/referee/image_transmission/serial", serial_, path, 921600,
            serial::Timeout::simpleTimeout(0));

        register_output("/referee/image_transmission/custom", custom_data_);
        std::fill((*custom_data_).begin(), (*custom_data_).end(), 0);
        custom_watchdog_.reset(5'000);

        register_output("/referee/image_transmission/vt13_frame", vt13_frame_data_);
        std::fill((*vt13_frame_data_).begin(), (*vt13_frame_data_).end(), 0);
    }

    void update() override {
        if (!serial_.active())
            return;

        size_t available = serial_->available();
        if (available > 0) {
            size_t old_size = buffer_.size();
            buffer_.resize(old_size + available);
            serial_->read(reinterpret_cast<uint8_t*>(buffer_.data() + old_size), available);
        }

        while (buffer_read_pos_ < buffer_.size()) {
            auto front = buffer_[buffer_read_pos_];

            if (front == std::byte{0xa9}) {
                if (!try_parse_remote_control())
                    break;
            } else if (front == std::byte{0xa5}) {
                if (!try_parse_referee_frame())
                    break;
            } else {
                buffer_read_pos_++;
            }
        }

        compact_buffer();

        if (custom_watchdog_.tick()) {
            RCLCPP_WARN(logger_, "Vision custom data receiving timeout. Set data to zero.");
            std::fill((*custom_data_).begin(), (*custom_data_).end(), 0);
        }

        if (vt13_watchdog_.tick()) {
            RCLCPP_WARN(logger_, "VT13 remote control receiving timeout.");
            std::fill((*vt13_frame_data_).begin(), (*vt13_frame_data_).end(), 0);
        }
    }

private:
    static constexpr std::size_t kRemoteControlFrameSize = 21;

    bool try_parse_remote_control() {
        if (buffer_.size() - buffer_read_pos_ < kRemoteControlFrameSize)
            return false;

        const auto* frame_ptr = buffer_.data() + buffer_read_pos_;

        // Check header magic: 0xa9 0x53
        if (frame_ptr[0] != std::byte{0xa9} || frame_ptr[1] != std::byte{0x53}) {
            buffer_read_pos_++;
            return true;
        }
        if (!rmcs_utility::dji_crc::verify_crc16(
                reinterpret_cast<const void*>(frame_ptr), kRemoteControlFrameSize)) {
            buffer_read_pos_++;
            return true;
        }

        // Output raw verified frame
        std::copy(
            reinterpret_cast<const uint8_t*>(frame_ptr),
            reinterpret_cast<const uint8_t*>(frame_ptr) + kRemoteControlFrameSize,
            (*vt13_frame_data_).begin());
        vt13_watchdog_.reset(500);

        buffer_read_pos_ += kRemoteControlFrameSize;
        return true;
    }

    // ---- Referee frame (0xA5) ----

    static constexpr std::size_t kRefereeFrameMaxSize = 256;

    bool try_parse_referee_frame() {
        if (buffer_.size() - buffer_read_pos_ < sizeof(frame_.header))
            return false;

        std::memcpy(&frame_.header, buffer_.data() + buffer_read_pos_, sizeof(frame_.header));

        if (!rmcs_utility::dji_crc::verify_crc8(frame_.header)) {
            buffer_read_pos_++;
            return true;
        }

        std::size_t total_frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id)
                                     + frame_.header.data_length + sizeof(uint16_t);

        if (total_frame_size > kRefereeFrameMaxSize) {
            buffer_read_pos_++;
            return true;
        }

        if (buffer_.size() - buffer_read_pos_ < total_frame_size)
            return false;

        const auto* frame_ptr = buffer_.data() + buffer_read_pos_;
        if (!rmcs_utility::dji_crc::verify_crc16(
                reinterpret_cast<const void*>(frame_ptr), total_frame_size)) {
            buffer_read_pos_++;
            return true;
        }

        uint16_t command_id;
        std::memcpy(&command_id, frame_ptr + sizeof(frame_.header), sizeof(uint16_t));

        if (command_id == 0x0302) {
            std::copy(
                reinterpret_cast<const uint8_t*>(frame_.body.data),
                reinterpret_cast<const uint8_t*>(frame_.body.data) + frame_.header.data_length,
                (*custom_data_).begin());
        }

        buffer_read_pos_ += total_frame_size;
        return true;
    }

    void compact_buffer() {
        if (buffer_read_pos_ == 0)
            return;
        if (buffer_read_pos_ >= buffer_.size()) {
            buffer_.clear();
            buffer_read_pos_ = 0;
            return;
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + buffer_read_pos_);
        buffer_read_pos_ = 0;
    }

    rclcpp::Logger logger_;

    OutputInterface<serial::Serial> serial_;
    std::vector<std::byte> buffer_;
    std::size_t buffer_read_pos_ = 0;

    OutputInterface<std::array<uint8_t, 30>> custom_data_;
    rmcs_utility::TickTimer custom_watchdog_;

    OutputInterface<std::array<uint8_t, 21>> vt13_frame_data_;
    rmcs_utility::TickTimer vt13_watchdog_;

    Frame frame_;
};

} // namespace rmcs_core::referee

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::referee::ImageTransmissionLink, rmcs_executor::Component)
