#include <algorithm>
#include <chrono>
#include <cmath>
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

#include "hardware/device/tof_sense.hpp"

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

        const auto timeout = std::chrono::duration<double>{timeout_seconds};
        if (!std::isfinite(timeout_seconds) || timeout < Clock::duration{1}
            || timeout >= Clock::duration::max())
            throw std::invalid_argument("TOF feedback_timeout must be positive and finite");

        if (configured_ids.empty())
            throw std::invalid_argument("TOF sensor_ids must not be empty");
        std::vector<std::uint8_t> ids;
        ids.reserve(configured_ids.size());
        for (const auto id : configured_ids) {
            if (id < 0 || id > 255)
                throw std::invalid_argument("TOF sensor IDs must be between 0 and 255");
            const auto sensor_id = static_cast<std::uint8_t>(id);
            if (std::ranges::find(ids, sensor_id) != ids.end())
                throw std::invalid_argument("TOF sensor IDs must be unique");
            ids.push_back(sensor_id);
        }

        tof_ = std::make_unique<device::TofSense>(
            *this, prefix, ids, std::chrono::duration_cast<Clock::duration>(timeout));
        board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial);
        node::info("TOFSense receiving {} sensors on {}", ids.size(), port);
    }

    void update() override { tof_->update_status(); }

private:
    using Clock = std::chrono::steady_clock;

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != *port_ || data.is_fdcan || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;
        tof_->store_can_status(data.can_id, data.can_data);
    }

    const Spec::Can* port_ = nullptr;
    std::unique_ptr<device::TofSense> tof_;
    // Stop receive callbacks before destroying the device they access.
    std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::TofSensor, rmcs_executor::Component)
