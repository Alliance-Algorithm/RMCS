#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <memory>
#include <numbers>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

#include "endian_promise.hpp"
#include "test_controller/usb_cdc_forwarder/package_receiver.hpp"
#include "test_controller/usb_cdc_forwarder/package_sender.hpp"
#include "test_controller/usb_cdc_forwarder/wheel.hpp"

namespace usb_cdc_forwarder {

class ForwarderNode : public rclcpp::Node {
public:
    explicit ForwarderNode()
        : Node("usb_cdc_forwarder", rclcpp::NodeOptions().use_intra_process_comms(true))
        , serial_("/dev/ttyACM0", 9600, serial::Timeout::simpleTimeout(0))
        , package_sender_(serial_)
        , package_receiver_(serial_) {

        package_receiver_.subscribe(
            0x11, std::bind(&ForwarderNode::can1_receive_callback, this, std::placeholders::_1));

        package_send_receive_thread_ =
            std::thread{&ForwarderNode::package_send_receive_thread_main, this};
    }

    virtual ~ForwarderNode() { package_send_receive_thread_.join(); }

private:
    void can1_receive_callback(std::unique_ptr<Package> package) {
        auto& static_part = package->static_part();

        if (package->dymatic_part_size() < sizeof(can_id_t)) {
            RCLCPP_ERROR(
                this->get_logger(), "Package does not contain can id: [0x%02X 0x%02X] (size = %d)",
                static_part.type, static_part.index, static_part.data_size);
            return;
        }

        auto can_id = package->dymatic_part<can_id_t>();
        if (can_id == 0x201) {
            chassis_wheels[0].publish_status(std::move(package));
        } else if (can_id == 0x202) {
            chassis_wheels[1].publish_status(std::move(package));
        } else if (can_id == 0x203) {
            chassis_wheels[2].publish_status(std::move(package));
        } else if (can_id == 0x204) {
            chassis_wheels[3].publish_status(std::move(package));
        }
    }

    void package_send_receive_thread_main() {
        using namespace std::chrono_literals;

        constexpr auto send_rate = 1000;
        constexpr auto period    = std::chrono::nanoseconds(1'000'000'000 / send_rate);
        auto next_send_time      = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            if (std::chrono::steady_clock::now() >= next_send_time) {
                chassis_wheels.write_control_package(package_sender_.package);
                // std::for_each(
                //     reinterpret_cast<uint8_t*>(&package_sender_.package),
                //     reinterpret_cast<uint8_t*>(&package_sender_.package) + sizeof(package_sender_.package),
                //     [](uint8_t byte) { printf("%02X ", byte); });
                // putchar('\n');
                package_sender_.Send();
                next_send_time += period;
            }
            package_receiver_.update();
            std::this_thread::sleep_for(10us);
        }
    }

    WheelCollection chassis_wheels{
        this,
        {"/chassis_wheel/right_front", "/chassis_wheel/left_front", "/chassis_wheel/left_back",
          "/chassis_wheel/right_back"}
    };

    serial::Serial serial_;
    PackageSender package_sender_;
    PackageReceiver package_receiver_;
    std::thread package_send_receive_thread_;
};

} // namespace usb_cdc_forwarder