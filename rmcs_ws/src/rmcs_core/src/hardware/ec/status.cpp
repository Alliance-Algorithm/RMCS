#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_core/msgs.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>
#include <serial_util/package_receive.hpp>

#include "hardware/fps_counter.hpp"

namespace rmcs_core::hardware::ec {

class Status
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Status()
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

        register_output("/serial", serial_, path, 9600, serial::Timeout::simpleTimeout(0));
        register_output("/tf", tf_);
        register_output("/robot_id", robot_id_);
        register_output("/auto_rune", auto_rune_);
        register_output("/robot_color", color_);
    }
    ~Status() = default;

    void update() override {
        while (true) {
            auto result = serial_util::receive_package(
                *serial_, package_, cache_size_, static_cast<uint8_t>(header_),
                [](const PackageReceive& package) {
                    if (package.type != type_)
                        return false;
                    if (package.data_size != size_)
                        return false;
                    auto* data = reinterpret_cast<const uint8_t*>(&package);
                    return package.check_sum
                        == std::accumulate(
                               data, data + sizeof(package) - 1, static_cast<uint8_t>(0));
                });

            if (result == serial_util::ReceiveResult::TIMEOUT)
                return;
            if (result == serial_util::ReceiveResult::SUCCESS) {
                cache_size_ = 0;
                update_data();
                if (!successfully_received_) {
                    successfully_received_ = true;
                    RCLCPP_INFO(logger_, "Successfully received the first package");
                }
                if (fps_counter_.count()) {
                    RCLCPP_INFO(logger_, "Quaternion fps: %d", fps_counter_.get_fps());
                }
                continue;
            }
            // Limit the number of failure logs to prevent log files from becoming too large
            if (failures_count_ >= 100)
                continue;
            failures_count_++;
            if (result == serial_util::ReceiveResult::HEADER_INVALID)
                RCLCPP_WARN(logger_, "Receive failed: Header invalid");
            else if (result == serial_util::ReceiveResult::VERIFY_INVALID)
                RCLCPP_WARN(logger_, "Receive failed: Verify invalid");
        }
    }

private:
    void update_data() {
        auto gimbal_imu_pose = Eigen::Quaterniond{package_.w, package_.x, package_.y, package_.z};
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());

        *auto_rune_ = package_.auto_rune == 1;
        *robot_id_  = package_.robot_id > 100 ? package_.robot_id - 100 : package_.robot_id;
        *color_     = package_.robot_id > 100 ? rmcs_core::msgs::RoboticColor::Blue
                                              : rmcs_core::msgs::RoboticColor::Red;
    }

    FpsCounter fps_counter_;

    rclcpp::Logger logger_;

    OutputInterface<uint8_t> robot_id_;
    OutputInterface<rmcs_core::msgs::RoboticColor> color_;
    OutputInterface<bool> auto_rune_;
    OutputInterface<serial::Serial> serial_;

    struct __attribute__((packed)) PackageReceive {
        uint8_t head;
        uint8_t type;
        uint8_t index;
        uint8_t data_size;
        float w, x, y, z;
        uint8_t robot_id;
        uint8_t auto_rune;
        uint8_t check_sum;
    } package_;
    size_t cache_size_ = 0;

    bool successfully_received_ = false;
    int failures_count_         = 0;

    // Package static data
    static const inline uint8_t header_ = 0xaf;
    static const inline uint8_t type_   = 0x32;
    static const inline uint8_t size_   = 0x12;

    OutputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::hardware::ec

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ec::Status, rmcs_executor::Component)