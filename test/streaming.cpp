#include "modules/capturer/hikcamera.hpp"
#include "modules/debug/framerate.hpp"
#include "modules/debug/visualization/stream_session.hpp"
#include "utility/image.details.hpp"
#include "utility/node.hpp"

#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>

int main(int argc, char** argv) {
    using namespace rmcs;

    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](auto) { rclcpp::shutdown(); });

    auto node = util::Node { "streaming_test" };

    constexpr auto host = "127.0.0.1";
    constexpr auto port = "5000";
    constexpr auto hz   = int { 80 };
    constexpr auto w    = int { 1440 };
    constexpr auto h    = int { 1080 };

    // NOTE: Stream Session
    using namespace rmcs::debug;

    auto check = StreamContext::check_support();
    if (!check) node.rclcpp_error("{}", check.error());

    auto config   = StreamSession::Config {};
    config.target = StreamTarget { host, port };
    config.type   = StreamType::RTP_JEPG;
    config.format = VideoFormat { w, h, hz };

    auto stream_session = StreamSession {};
    stream_session.set_notifier([&](auto msg) { //
        node.rclcpp_info("[StreamSession] {}", msg);
    });
    if (auto result = stream_session.open(config); !result) {
        node.rclcpp_error("{}", result.error());
        rclcpp::shutdown();
    }
    if (auto sdp = stream_session.session_description_protocol()) {
        node.rclcpp_info("\n\n\n{}\n\n", sdp.value());
    } else {
        node.rclcpp_error("{}", sdp.error());
    }
    // NOTE: End

    auto hikcamera = std::make_unique<cap::hik::Hikcamera>();
    hikcamera->configure({});

    if (auto ret = hikcamera->connect(); !ret) {
        node.rclcpp_error("Failed to init camera: {}", ret.error());
    } else {
        node.rclcpp_info("Successfully initialize camera");
    }

    auto once_flag = std::once_flag {};
    auto framerate = FramerateCounter {};

    while (rclcpp::ok()) {

        auto image = hikcamera->wait_image();
        if (!image.has_value()) {
            node.rclcpp_error("Failed to read image: {}", image.error());

            if (auto ret = hikcamera->connect(); !ret) {
                node.rclcpp_error("Failed to init camera: {}", ret.error());
                std::this_thread::sleep_for(std::chrono::seconds { 2 });
            } else {
                node.rclcpp_info("Successfully initialize camera");
            }

            continue;
        }

        auto& current_frame = image.value();
        std::call_once(once_flag, [&] {
            const auto frame_w = current_frame->details().get_cols();
            const auto frame_h = current_frame->details().get_rows();

            if (frame_w != w || frame_h != h) {
                node.rclcpp_error("Given size {}x{} != target[{}x{}]", frame_w, frame_h, w, h);
                rclcpp::shutdown();
            } else {
                node.rclcpp_info("First frame was received");
            }
        });
        if (framerate.tick()) {
            node.rclcpp_info("Hz: {}", framerate.fps());
        }

        // NOTE: Stream Session
        if (!stream_session.push_frame(current_frame->details().mat)) {
            node.rclcpp_warn("Frame was pushed failed");
        }
        // NOTE: End
    }

    return rclcpp::shutdown();
}
