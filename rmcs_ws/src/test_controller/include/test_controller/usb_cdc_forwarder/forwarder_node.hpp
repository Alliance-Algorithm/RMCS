#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <numbers>
#include <string>
#include <thread>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <tf2_ros/transform_broadcaster.h>

#include "endian_promise.hpp"
#include "test_controller/usb_cdc_forwarder/fps_counter.hpp"
#include "test_controller/usb_cdc_forwarder/gm6020.hpp"
#include "test_controller/usb_cdc_forwarder/imu.hpp"
#include "test_controller/usb_cdc_forwarder/package.hpp"
#include "test_controller/usb_cdc_forwarder/package_receiver.hpp"
#include "test_controller/usb_cdc_forwarder/package_sender.hpp"
#include "test_controller/usb_cdc_forwarder/remote_control.hpp"
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
        package_receiver_.subscribe(
            0x23, std::bind(&ForwarderNode::dbus_receive_callback, this, std::placeholders::_1));
        package_receiver_.subscribe(
            0x31, std::bind(&ForwarderNode::imu_receive_callback, this, std::placeholders::_1));
        package_receiver_.subscribe(0x26, [](std::unique_ptr<Package>) {});

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
            chassis_wheels_[0].publish_status(std::move(package));
        } else if (can_id == 0x202) {
            chassis_wheels_[1].publish_status(std::move(package));
        } else if (can_id == 0x203) {
            chassis_wheels_[2].publish_status(std::move(package));
        } else if (can_id == 0x204) {
            chassis_wheels_[3].publish_status(std::move(package));
        } else if (can_id == 0x205) {
            gimbal_yaw_motor_.publish_status(std::move(package));
        } else if (can_id == 0x206) {
            gimbal_pitch_motor_.publish_status(std::move(package));
        }
    }

    void dbus_receive_callback(std::unique_ptr<Package> package) {
        remote_control_.publish_status(std::move(package));
    }

    struct __attribute__((packed)) ImuData {
        int16_t gyro_x, gyro_y, gyro_z;
        int16_t acc_x, acc_y, acc_z;
    };

    void imu_receive_callback(std::unique_ptr<Package> package) {
        static FPSCounter counter;
        auto& data      = package->dymatic_part<ImuData>();
        auto solve_acc  = [](int16_t value) { return value / 32767.0 * 6.0; };
        auto solve_gyro = [](int16_t value) {
            return value / 32767.0 * 2000.0 / 180.0 * std::numbers::pi;
        };
        double gx = solve_gyro(data.gyro_x), gy = solve_gyro(data.gyro_y),
               gz = solve_gyro(data.gyro_z);
        double ax = solve_acc(data.acc_x), ay = solve_acc(data.acc_y), az = solve_acc(data.acc_z);

        imu_.update(gx, gy, gz, ax, ay, az);
    }

    // void receive_package_timer_callback() {
    //     package_receiver_.update();
    // }

    // void wheel_transmit_package_timer_callback() {

    // }

    void package_send_receive_thread_main() {
        using namespace std::chrono_literals;

        constexpr auto send_rate = 1000;
        constexpr auto period    = std::chrono::nanoseconds(1'000'000'000 / send_rate);
        auto next_send_time      = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {
            if (std::chrono::steady_clock::now() >= next_send_time) {
                auto& static_part  = package_sender_.package.static_part();
                auto& dymatic_part = package_sender_.package.dymatic_part<PackageC620ControlPart>();
                static_part.type   = 0x11;

                static_part.index     = 0;
                static_part.data_size = sizeof(PackageC620ControlPart);
                dymatic_part.can_id   = 0x1FF;
                gimbal_yaw_motor_.write_control_current_to_package(dymatic_part, 0);
                gimbal_pitch_motor_.write_control_current_to_package(dymatic_part, 1);
                dymatic_part.current[2] = dymatic_part.current[3] = 0;
                package_sender_.Send();
                std::this_thread::sleep_for(200us);

                chassis_wheels_.write_control_package(package_sender_.package);
                package_sender_.Send();

                next_send_time += period;
            }
            package_receiver_.update();
            std::this_thread::sleep_for(10us);
        }
    }

    Imu imu_{this};

    WheelCollection<true> chassis_wheels_{
        this,
        {"/chassis_wheel/left_front", "/chassis_wheel/right_front", "/chassis_wheel/right_back",
          "/chassis_wheel/left_back"}
    };
    GM6020<false> gimbal_yaw_motor_{this, "/gimbal/yaw"},
        gimbal_pitch_motor_{this, "/gimbal/pitch"};

    RemoteControl remote_control_{this};

    serial::Serial serial_;
    PackageSender package_sender_;
    PackageReceiver package_receiver_;
    std::thread package_send_receive_thread_;
};

} // namespace usb_cdc_forwarder