#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "hardware/tof_sensors.hpp"

namespace rmcs_core::hardware {

class TofSensor final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin
    , public librmcs::board::RmcsBoardLite::Callback {
public:
    TofSensor()
        : Node{get_component_name(), node::options()} {
        const auto serial = get_parameter_or<std::string>("board_serial", "");
        const auto port = get_parameter_or<std::string>("port", "can3");
        const auto prefix = get_parameter_or<std::string>("output_prefix", "/tof");
        const auto timeout_seconds = get_parameter_or("feedback_timeout", 0.2);
        const auto configured_ids =
            get_parameter_or<std::vector<std::int64_t>>("sensor_ids", {0, 1});

        if (port == "can0")
            port_ = &Spec::kCans.kCan0;
        else if (port == "can1")
            port_ = &Spec::kCans.kCan1;
        else if (port == "can2")
            port_ = &Spec::kCans.kCan2;
        else if (port == "can3")
            port_ = &Spec::kCans.kCan3;
        else
            throw std::invalid_argument("TOF port must be can0, can1, can2 or can3");

        tof_ = std::make_unique<TofSensors>(*this, prefix, configured_ids, timeout_seconds);
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial);
        node::info("TOFSense receiving {} sensors on {}", configured_ids.size(), port);
    }

    void update() override { tof_->update_status(); }

private:
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != *port_ || data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;
        tof_->store_status(data.can_id, data.can_data);
    }

    const Spec::Can* port_ = nullptr;
    std::unique_ptr<TofSensors> tof_;
    // Stop receive callbacks before destroying the device they access.
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TofSensor, rmcs_executor::Component)
