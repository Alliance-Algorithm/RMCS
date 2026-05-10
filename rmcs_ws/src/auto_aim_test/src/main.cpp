#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include "hikcamera.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

class AutoAimTestApp;

class AutoAimTestApp final {
public:
    AutoAimTestApp()
        : node_{std::make_shared<rclcpp::Node>("auto_aim_test")} {

        auto camera_config = Hikcamera::CameraConfig::cs016_default();
        camera_config.exposure_us =
            static_cast<float>(node_->declare_parameter<double>("exposure_us", 2000.0));
        auto camera_name = node_->declare_parameter<std::string>("camera_name", "");
        camera_ = std::make_unique<Hikcamera>(
            camera_config, [this](Hikcamera::Frame&& frame) { on_frame(std::move(frame)); },
            camera_name);

        watchdog_ = node_->create_wall_timer(std::chrono::milliseconds{100}, [this] {
            try {
                camera_->rethrow_pending_exception();
            } catch (const std::exception& exception) {
                RCLCPP_FATAL(node_->get_logger(), "Camera callback failed: %s", exception.what());
                rclcpp::shutdown();
            }
        });

        RCLCPP_INFO(
            node_->get_logger(), "auto_aim_test started with callback Bayer capture on GPIO %s",
            board_gpio_name_.c_str());
    }

    auto spin() -> void { rclcpp::spin(node_); }

    auto on_trigger_edge(const std::uint32_t raw_timestamp_quarter_us) -> void {
        const auto trigger_count = trigger_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        const auto last_frame_id = last_frame_id_.load(std::memory_order_relaxed);

        RCLCPP_INFO(
            node_->get_logger(), "trigger #%llu: board_ts_quarter_us=%u latest_frame_id=%u",
            static_cast<unsigned long long>(trigger_count), raw_timestamp_quarter_us,
            last_frame_id);
    }

private:
    auto on_frame(Hikcamera::Frame&& frame) -> void {
        const auto frame_count = frame_count_.fetch_add(1, std::memory_order_relaxed) + 1;
        last_frame_id_.store(frame.frame_id, std::memory_order_relaxed);

        if (frame_count == 1 || frame_count % 60 == 0) {
            RCLCPP_INFO(
                node_->get_logger(),
                "frame #%llu: id=%u %ux%u bytes=%zu pixel_type=0x%x device_ts=%llu host_ts=%lld",
                static_cast<unsigned long long>(frame_count), frame.frame_id, frame.width,
                frame.height, frame.size, static_cast<unsigned int>(frame.pixel_type),
                static_cast<unsigned long long>(frame.device_timestamp),
                static_cast<long long>(frame.host_timestamp));
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string camera_name_;
    std::string board_serial_;
    std::string board_gpio_name_;
    std::unique_ptr<Hikcamera> camera_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::atomic<std::uint64_t> frame_count_ = 0;
    std::atomic<std::uint64_t> trigger_count_ = 0;
    std::atomic<std::uint32_t> last_frame_id_ = 0;

    friend class TriggerBoard;
};

} // namespace

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    try {
        AutoAimTestApp app;
        app.spin();
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(
            rclcpp::get_logger("auto_aim_test"), "auto_aim_test failed: %s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
}
