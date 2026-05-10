#include <atomic>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_utility/memory_pool.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hikcamera.hpp"

namespace {

class AutoAimTestApp final : public rclcpp::Node {
public:
    AutoAimTestApp()
        : Node("auto_aim_test") {
        auto camera_config = Hikcamera::CameraConfig::cs016_default();
        camera_config.exposure_us =
            static_cast<float>(declare_parameter<double>("exposure_us", 2000.0));
        auto camera_name = declare_parameter<std::string>("camera_name", "");

        camera_ = std::make_unique<Hikcamera>(
            camera_config,
            [this](const Hikcamera::Frame& frame) noexcept { frame_callback(frame); }, camera_name);

        thread_ = std::thread(&AutoAimTestApp::thread_main, this);

        RCLCPP_INFO(get_logger(), "auto_aim_test started");
    }

    ~AutoAimTestApp() {
        stopped_.test_and_set(std::memory_order::relaxed);
        notify_event();
        thread_.join();
    }

private:
    void thread_main() {
        while (!stopped_.test(std::memory_order::relaxed)) {
            auto old = event_count_.load(std::memory_order::relaxed);
            if (!unmatched_image_buffer_.pop_front([this](UnmatchedImage&& image) noexcept {
                    if (image.frame_id % 100 == 0) {
                    RCLCPP_INFO(
                        get_logger(), "frame #%u: device_ts=%llu", image.frame_id,
                        static_cast<unsigned long long>(
                            image.timestamp.time_since_epoch().count()));
                    }

                    // Process Image ...

                    std::destroy_at(std::launder(image.frame));
                    {
                        auto guard = std::scoped_lock{frame_pool_mutex_};
                        frame_pool_.free(image.frame);
                    }
                })) {
                event_count_.wait(old, std::memory_order::acquire);
            }
        }
    }

    void frame_callback(const Hikcamera::Frame& frame) {
        void* pub_frame_ptr;
        {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            pub_frame_ptr = frame_pool_.allocate();
        }
        if (!pub_frame_ptr)
            return;

        auto pub_frame = new (pub_frame_ptr) PublishFrame{};
        std::memcpy(pub_frame->data, frame.data.data(), frame.data.size());

        unmatched_image_buffer_.emplace_back(pub_frame, frame.frame_id, frame.timestamp);

        notify_event();
    }

    void notify_event() {
        event_count_.fetch_add(1, std::memory_order::release);
        event_count_.notify_one();
    }

    struct BoardClock {
        // 250ns / tick
        using duration = std::chrono::duration<std::int64_t, std::ratio<1, 4'000'000>>;
        using rep = duration::rep;
        using period = duration::period;
        using time_point = std::chrono::time_point<BoardClock>;

        [[maybe_unused]] static constexpr bool is_steady = true;
    };

    struct PublishFrame {
        static constexpr std::uint32_t kWidth = Hikcamera::Cs016FrameWidth;
        static constexpr std::uint32_t kHeight = Hikcamera::Cs016FrameHeight;
        static constexpr std::size_t kFrameSize = 1 * kWidth * kHeight;

        alignas(std::uintptr_t) std::byte data[kFrameSize];
        MvGvspPixelType pixel_type;

        Eigen::Quaterniond imu_snapshot;
        BoardClock::time_point timestamp;
    };
    rmcs_utility::MemoryPool<sizeof(PublishFrame), alignof(PublishFrame)> frame_pool_{100};
    std::mutex frame_pool_mutex_;

    struct UnmatchedImage {
        PublishFrame* frame;
        std::uint32_t frame_id;
        Hikcamera::HikDeviceClock::time_point timestamp;
    };
    rmcs_utility::RingBuffer<UnmatchedImage> unmatched_image_buffer_{16};

    std::unique_ptr<Hikcamera> camera_;

    std::atomic_flag stopped_;
    std::atomic<std::size_t> event_count_ = 0;
    std::thread thread_;
};

} // namespace

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    try {
        auto app = std::make_shared<AutoAimTestApp>();
        rclcpp::spin(app);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(
            rclcpp::get_logger("auto_aim_test"), "auto_aim_test failed: %s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
}
