#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <serial/serial.h>
#include <serial_util/package_receive.hpp>

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
    }
    ~Status() = default;

    void update() override {
        auto result = serial_util::receive_package(
            *serial_, package_, cache_size_, static_cast<uint8_t>(0xaf),
            [](const PackageReceive& package) {
                if (package.type != 0x32)
                    return false;
                if (package.data_size != 16)
                    return false;
                auto* data = reinterpret_cast<const uint8_t*>(&package);
                return package.check_sum
                    == std::accumulate(data, data + sizeof(package), static_cast<uint8_t>(0));
            });

        if (result == serial_util::ReceiveResult::TIMEOUT)
            return;
        if (result == serial_util::ReceiveResult::SUCCESS) {
            cache_size_ = 0;
            update_quaternion();
            if (!successfully_received_) {
                successfully_received_ = true;
                RCLCPP_INFO(logger_, "Successfully received the first package");
            }
            return;
        }
        // Limit the number of failure logs to prevent log files from becoming too large
        if (failures_count_ >= 100)
            return;
        failures_count_++;
        if (result == serial_util::ReceiveResult::HEADER_INVAILD)
            RCLCPP_WARN(logger_, "Receive failed: Header Invaild");
        else if (result == serial_util::ReceiveResult::VERIFY_INVAILD)
            RCLCPP_WARN(logger_, "Receive failed: Verify Invaild");
    }

private:
    void update_quaternion() {
        Eigen::AngleAxisd angle_axis{
            Eigen::Quaterniond{package_.w, package_.x, package_.y, package_.z}
            .normalized()
        };
        // Some hacks for wiping the butt.
        angle_axis.axis().x() = -angle_axis.axis().x();

        auto gimbal_imu_pose = Eigen::Quaterniond{angle_axis};
        tf_->set_transform<rmcs_description::ImuLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate());
    }

    rclcpp::Logger logger_;

    OutputInterface<serial::Serial> serial_;
    struct PackageReceive {
        uint8_t head;
        uint8_t type;
        uint8_t index;
        uint8_t data_size;
        float w, x, y, z;
        uint8_t check_sum;
    } package_;
    size_t cache_size_ = 0;

    bool successfully_received_ = false;
    int failures_count_         = 0;

    OutputInterface<rmcs_description::Tf> tf_;
};

} // namespace rmcs_core::hardware::ec

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::ec::Status, rmcs_executor::Component)