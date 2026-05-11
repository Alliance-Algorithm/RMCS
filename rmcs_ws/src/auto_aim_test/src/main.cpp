#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <experimental/scope>
#include <format>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rmcs_utility/atomic_wait_timeout.hpp>
#include <rmcs_utility/memory_pool.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hikcamera.hpp"
#include "linear_sync_model.hpp"
#include "trigger_board.hpp"

namespace {

class AutoAimTestApp final : public rclcpp::Node {
public:
    AutoAimTestApp()
        : Node("auto_aim_test") {
        auto camera_config = Hikcamera::CameraConfig::cs016_default();
        camera_config.exposure_us =
            static_cast<float>(declare_parameter<double>("exposure_us", 2000.0));
        auto camera_name = declare_parameter<std::string>("camera_name", "");

        board_ = std::make_unique<TriggerBoard>(
            [this](TriggerBoard::Clock::time_point timestamp) { signal_callback(timestamp); });

        camera_ = std::make_unique<Hikcamera>(
            camera_config,
            [this](const Hikcamera::Frame& frame) noexcept { frame_callback(frame); }, camera_name);

        worker_ = std::thread(&AutoAimTestApp::worker_main, this);

        RCLCPP_INFO(get_logger(), "auto_aim_test started");
    }

    ~AutoAimTestApp() {
        stopped_.test_and_set(std::memory_order::relaxed);
        notify_event();
        worker_.join();

        camera_.reset();
        {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            unmatched_image_buffer_.pop_front_n([this](UnmatchedImage&& image) noexcept {
                std::destroy_at(std::launder(image.frame));
                frame_pool_.free(image.frame);
            });
        }

        board_.reset();
    }

private:
    enum class WorkerState { kReseting, kMatching, kConfirming, kLocked };
    using TimestampPair = LinearSyncModel::TimestampPair;

    static constexpr double kResidualThresholdSec = LinearSyncModel::kDefaultResidualThresholdSec;
    static constexpr double kRlsLambda = 0.9995;
    static constexpr std::uint32_t kAllowedFrameIdGap = 2;

    void frame_callback(const Hikcamera::Frame& frame) {
        void* pub_frame_ptr;
        {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            pub_frame_ptr = frame_pool_.allocate();
        }
        if (!pub_frame_ptr)
            return;

        if (!unmatched_image_buffer_.emplace_back_n(
                [&](std::byte* storage) noexcept {
                    auto pub_frame = new (pub_frame_ptr) PublishFrame{};
                    std::memcpy(pub_frame->data, frame.data.data(), frame.data.size());
                    new (storage) UnmatchedImage{pub_frame, frame.frame_id, frame.timestamp};
                },
                1)) {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            frame_pool_.free(pub_frame_ptr);
            return;
        }

        notify_event();
    }

    void signal_callback(TriggerBoard::Clock::time_point timestamp) {
        if (!unmatched_signal_buffer_.emplace_back(timestamp))
            return;

        notify_event();
    }

    void worker_main() {
        using namespace std::chrono_literals;

        auto state = WorkerState::kReseting;
        auto idle_duration = std::chrono::steady_clock::duration::zero();

        auto matching_start = std::chrono::steady_clock::time_point::max();
        std::vector<TimestampPair> matching_array{1000};

        const auto update_state = [&](WorkerState new_state, const char* message) {
            state = new_state;
            switch (new_state) {
            case WorkerState::kReseting: {
                sync_model_.reset();
                last_locked_frame_id_.reset();
                camera_->set_soft_trigger(true);
                RCLCPP_INFO(get_logger(), "[RESETING] %s", message);
            } break;
            case WorkerState::kMatching: {
                sync_model_.reset();
                last_locked_frame_id_.reset();
                camera_->set_soft_trigger(false);
                matching_start = std::chrono::steady_clock::now();
                matching_array.clear();
                RCLCPP_INFO(get_logger(), "[MATCHING] %s", message);
            } break;
            case WorkerState::kConfirming: {
                camera_->set_soft_trigger(true);
                RCLCPP_INFO(get_logger(), "[CONFIRMING] %s", message);
            } break;
            case WorkerState::kLocked: {
                last_locked_frame_id_.reset();
                camera_->set_soft_trigger(false);
                RCLCPP_INFO(get_logger(), "[LOCKED] %s", message);
            } break;
            }
        };

        while (!stopped_.test(std::memory_order::relaxed)) {
            if (idle_duration >= 100ms) {
                if (state == WorkerState::kReseting)
                    update_state(WorkerState::kMatching, "Trying to match signal and image...");
                else if (state == WorkerState::kConfirming) {
                    if (!unmatched_signal_buffer_.readable()
                        && !unmatched_image_buffer_.readable()) {
                        auto ls_result = sync_model_.initialize_from_least_squares(matching_array);
                        if (!ls_result) {
                            update_state(
                                WorkerState::kReseting,
                                "Least-squares fit failed to initialize lock model");
                        } else if (!(0.8 < ls_result->a && ls_result->a < 1.2)) {
                            update_state(
                                WorkerState::kReseting,
                                std::format(
                                    "LS slope out of range: a={:.9f} is outside (0.8, 1.2)",
                                    ls_result->a)
                                    .c_str());
                        } else if (!(ls_result->max_abs_residual_sec < kResidualThresholdSec)) {
                            update_state(
                                WorkerState::kReseting,
                                std::format(
                                    "LS residual too large: max_abs_residual(ms)={:.3f} >= "
                                    "threshold(ms)={:.3f}",
                                    ls_result->max_abs_residual_sec * 1000.0,
                                    kResidualThresholdSec * 1000.0)
                                    .c_str());
                        } else {
                            update_state(
                                WorkerState::kLocked,
                                std::format(
                                    "Hard-sync locked successfully: a={:.9f}, b={:.9f}, "
                                    "var_a={:.3e}, var_b={:.3e}, max_residual(ms)={:.3f}",
                                    ls_result->a, ls_result->b, ls_result->covariance(0, 0),
                                    ls_result->covariance(1, 1),
                                    ls_result->max_abs_residual_sec * 1000.0)
                                    .c_str());
                        }
                    } else {
                        update_state(
                            WorkerState::kReseting,
                            "Failed to match: Mismatched signals and images count");
                    }
                }
            }

            if (state == WorkerState::kMatching
                && std::chrono::steady_clock::now() - matching_start >= 2s) {
                if (matching_array.size() < 50) {
                    update_state(
                        WorkerState::kReseting,
                        std::format(
                            "Too few frames matched: collected ({}) < minimum (50)",
                            matching_array.size())
                            .c_str());
                } else {
                    update_state(
                        WorkerState::kConfirming,
                        "Frames collected. Waiting bus idle to ensure no data loss...");
                }
            }

            if (state == WorkerState::kReseting) {
                unmatched_signal_buffer_.clear();
                {
                    auto guard = std::scoped_lock{frame_pool_mutex_};
                    unmatched_image_buffer_.pop_front_n([this](UnmatchedImage&& image) noexcept {
                        std::destroy_at(image.frame);
                        frame_pool_.free(image.frame);
                    });
                }
            }

            const auto old = event_count_.load(std::memory_order::relaxed);
            if (!unmatched_signal_buffer_.readable() || !unmatched_image_buffer_.readable()) {
                if (!rmcs_utility::atomic_wait_timeout(
                        event_count_, old, 50ms, std::memory_order::acquire)) {
                    idle_duration += 50ms;
                    continue;
                } else {
                    idle_duration = decltype(idle_duration)::zero();
                    if (!unmatched_signal_buffer_.readable() || !unmatched_image_buffer_.readable())
                        continue;
                }
            }

            if (state == WorkerState::kReseting)
                continue;

            if (state == WorkerState::kMatching || state == WorkerState::kConfirming) {
                matching_array.push_back(consume_frame());
                continue;
            }

            if (state == WorkerState::kLocked) {
                if (!sync_model_.initialized()) {
                    update_state(
                        WorkerState::kReseting, "RLS model was unexpectedly uninitialized");
                    continue;
                }

                auto* next_image = unmatched_image_buffer_.peek_front();
                if (next_image == nullptr)
                    continue;

                if (last_locked_frame_id_) {
                    const auto expected_frame_id = *last_locked_frame_id_ + 1U;
                    const auto next_frame_id = next_image->frame_id;

                    if (next_frame_id < expected_frame_id) {
                        update_state(
                            WorkerState::kReseting,
                            std::format(
                                "Frame ID moved backward: expected >= {}, got {}",
                                expected_frame_id, next_frame_id)
                                .c_str());
                        continue;
                    }

                    const auto frame_gap = next_frame_id - expected_frame_id;
                    if (frame_gap > 0U) {
                        if (frame_gap > kAllowedFrameIdGap) {
                            update_state(
                                WorkerState::kReseting,
                                std::format(
                                    "Frame ID gap too large: expected {}, got {}, gap {} > {}",
                                    expected_frame_id, next_frame_id, frame_gap, kAllowedFrameIdGap)
                                    .c_str());
                            continue;
                        }

                        if (unmatched_signal_buffer_.readable()
                            < static_cast<std::size_t>(frame_gap) + 1U) {
                            continue;
                        }

                        drop_signals(frame_gap);
                        RCLCPP_WARN(
                            get_logger(),
                            "Tolerating camera frame gap: expected %u, got %u, dropped %u queued "
                            "signals",
                            expected_frame_id, next_frame_id, frame_gap);
                    }
                }

                const auto timestamp_pair = consume_frame();
                last_locked_frame_id_ = next_image->frame_id;
                const auto residual = sync_model_.residual_for(
                    timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
                if (!(std::abs(residual) < kResidualThresholdSec)) {
                    update_state(
                        WorkerState::kReseting,
                        std::format(
                            "RLS residual too large: residual(ms)={:.3f}, threshold(ms)={:.3f}",
                            residual * 1000.0, kResidualThresholdSec * 1000.0)
                            .c_str());
                    continue;
                }

                const auto updated_residual = sync_model_.update(
                    timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
                if (!std::isfinite(updated_residual) || !sync_model_.initialized()) {
                    update_state(WorkerState::kReseting, "RLS update failed");
                }
            }
        }
    }

    void drop_signals(const std::uint32_t count) {
        const auto dropped = unmatched_signal_buffer_.pop_front_n(
            [](TriggerBoard::Clock::time_point&&) noexcept {}, count);
        if (dropped != count) {
            std::fprintf(stderr, "drop_signals expected %u but dropped %zu\n", count, dropped);
            std::terminate();
        }
    }

    TimestampPair consume_frame() {
        TimestampPair timestamp_pair;

        if (!unmatched_signal_buffer_.pop_front(
                [&](TriggerBoard::Clock::time_point&& timestamp) noexcept {
                    timestamp_pair.board_timestamp_sec =
                        std::chrono::duration_cast<std::chrono::duration<double>>(
                            timestamp.time_since_epoch())
                            .count();
                })) {
            std::fprintf(stderr, "holy\n");
            std::terminate();
        }

        if (!unmatched_image_buffer_.pop_front([&](UnmatchedImage&& image) noexcept {
                timestamp_pair.camera_timestamp_sec =
                    std::chrono::duration_cast<std::chrono::duration<double>>(
                        image.timestamp.time_since_epoch())
                        .count();

                if (image.frame_id % 100 == 0) {
                    RCLCPP_INFO(
                        get_logger(), "frame #%u: camera_ts(ms)=%.3f, signal_ts(ms)=%.3f",
                        image.frame_id, timestamp_pair.camera_timestamp_sec * 1000.0,
                        timestamp_pair.board_timestamp_sec * 1000.0);
                }

                // Process Image ...

                std::destroy_at(image.frame);
                {
                    auto guard = std::scoped_lock{frame_pool_mutex_};
                    frame_pool_.free(image.frame);
                }
            }))
            std::terminate();

        return timestamp_pair;
    }

    void notify_event() {
        event_count_.fetch_add(1, std::memory_order::release);
        event_count_.notify_one();
    }

    struct PublishFrame {
        static constexpr std::uint32_t kWidth = Hikcamera::Cs016FrameWidth;
        static constexpr std::uint32_t kHeight = Hikcamera::Cs016FrameHeight;
        static constexpr std::size_t kFrameSize = 1 * kWidth * kHeight;

        alignas(std::uintptr_t) std::byte data[kFrameSize];
        MvGvspPixelType pixel_type;

        Eigen::Quaterniond imu_snapshot;
        TriggerBoard::Clock::time_point timestamp;
    };
    rmcs_utility::MemoryPool<sizeof(PublishFrame), alignof(PublishFrame)> frame_pool_{100};
    std::mutex frame_pool_mutex_;

    rmcs_utility::RingBuffer<TriggerBoard::Clock::time_point> unmatched_signal_buffer_{32};
    std::unique_ptr<TriggerBoard> board_;

    struct UnmatchedImage {
        PublishFrame* frame;
        std::uint32_t frame_id;
        Hikcamera::HikDeviceClock::time_point timestamp;
    };
    rmcs_utility::RingBuffer<UnmatchedImage> unmatched_image_buffer_{16};
    std::unique_ptr<Hikcamera> camera_;
    LinearSyncModel sync_model_{kRlsLambda, kResidualThresholdSec};
    std::optional<std::uint32_t> last_locked_frame_id_;

    std::atomic_flag stopped_;
    std::atomic<std::uint32_t> event_count_ = 0;
    std::thread worker_;
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
