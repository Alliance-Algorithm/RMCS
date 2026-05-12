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
        camera_config_ = Hikcamera::CameraConfig::cs016_default();
        camera_config_.exposure_us =
            static_cast<float>(declare_parameter<double>("exposure_us", 2000.0));
        camera_name_ = declare_parameter<std::string>("camera_name", "");

        board_ = std::make_unique<TriggerBoard>(
            [this](TriggerBoard::Clock::time_point timestamp) { signal_callback(timestamp); });
        last_frame_time_.store(
            std::chrono::steady_clock::time_point::min(), std::memory_order::relaxed);

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
    enum class CameraTransportState { kDown, kUp, kReconnecting };
    using TimestampPair = LinearSyncModel::TimestampPair;

    static constexpr double kResidualThresholdSec = LinearSyncModel::kDefaultResidualThresholdSec;
    static constexpr double kRlsLambda = 0.9995;
    static constexpr std::uint32_t kAllowedFrameIdGap = 2;
    static constexpr std::size_t kExpectedFrameSize =
        static_cast<std::size_t>(Hikcamera::Cs016FrameWidth)
        * static_cast<std::size_t>(Hikcamera::Cs016FrameHeight);
    static constexpr auto kFrameWatchdogTimeout = std::chrono::milliseconds{250};
    static constexpr auto kMaxReconnectBackoff = std::chrono::milliseconds{5000};

    static auto transport_state_name(const CameraTransportState state) noexcept -> const char* {
        switch (state) {
        case CameraTransportState::kDown: return "DOWN";
        case CameraTransportState::kUp: return "UP";
        case CameraTransportState::kReconnecting: return "RECONNECTING";
        }

        return "UNKNOWN";
    }

    static auto transport_fault_message_to_string(const unsigned int message_type) noexcept
        -> const char* {
        switch (message_type) {
        case MV_EXCEPTION_DEV_DISCONNECT: return "device disconnected";
        default: return "unknown transport exception";
        }
    }

    static auto sdk_error_to_string(const int code) noexcept -> const char* {
        switch (static_cast<std::uint32_t>(code)) {
        case MV_OK: return "success";
        case MV_E_HANDLE: return "invalid handle";
        case MV_E_BUSY: return "device busy or network disconnected";
        case MV_E_PRECONDITION: return "precondition/environment changed";
        case MV_E_USB_DEVICE: return "USB device exception";
        case MV_E_NODATA: return "no data";
        default: return "unknown Hik SDK error";
        }
    }

    void clear_sync_state(
        WorkerState& state, std::vector<TimestampPair>& matching_array, const char* reason) {
        sync_model_.reset();
        last_locked_frame_id_.reset();
        matching_array.clear();
        clear_pending_data();
        state = WorkerState::kReseting;
        RCLCPP_INFO(get_logger(), "[RESETING] %s", reason);
    }

    void clear_pending_data() {
        unmatched_signal_buffer_.clear();
        {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            unmatched_image_buffer_.pop_front_n([this](UnmatchedImage&& image) noexcept {
                std::destroy_at(image.frame);
                frame_pool_.free(image.frame);
            });
        }
    }

    void set_transport_state(
        CameraTransportState& transport_state, const CameraTransportState new_state,
        const char* reason) {
        if (transport_state == new_state)
            return;

        RCLCPP_INFO(
            get_logger(), "[TRANSPORT] %s -> %s: %s", transport_state_name(transport_state),
            transport_state_name(new_state), reason);
        transport_state = new_state;
    }

    void handle_transport_fault(
        CameraTransportState& transport_state, WorkerState& state,
        std::vector<TimestampPair>& matching_array, const std::string& reason) {
        if (transport_state == CameraTransportState::kReconnecting)
            return;

        RCLCPP_ERROR(get_logger(), "[TRANSPORT] Fault detected: %s", reason.c_str());
        camera_.reset();
        set_transport_state(transport_state, CameraTransportState::kReconnecting, reason.c_str());
        clear_sync_state(state, matching_array, "Camera transport fault - waiting to reconnect");
        reconnect_attempts_ = 0;
        next_reconnect_time_ = std::chrono::steady_clock::now();
    }

    auto try_connect_camera(
        CameraTransportState& transport_state, WorkerState& state,
        std::vector<TimestampPair>& matching_array) -> bool {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_reconnect_time_)
            return false;

        const bool is_reconnect = reconnect_attempts_ > 0 || transport_state == CameraTransportState::kReconnecting;
        ++reconnect_attempts_;
        RCLCPP_INFO(
            get_logger(), "[TRANSPORT] %s attempt #%zu", is_reconnect ? "Reconnect" : "Connect",
            reconnect_attempts_);

        try {
            camera_ = std::make_unique<Hikcamera>(
                camera_config_,
                [this](const Hikcamera::Frame& frame) noexcept { frame_callback(frame); },
                camera_name_);
            last_frame_time_.store(std::chrono::steady_clock::now(), std::memory_order::release);
            set_transport_state(transport_state, CameraTransportState::kUp, "Camera connected");
            clear_sync_state(
                state, matching_array,
                is_reconnect ? "Camera reconnected - restarting sync"
                             : "Camera connected - starting sync");
            reconnect_attempts_ = 0;
            return true;
        } catch (const std::exception& exception) {
            const auto exponential =
                std::chrono::milliseconds{200 * (1ULL << std::min<std::size_t>(reconnect_attempts_ - 1, 4))};
            const auto backoff = std::min(exponential, kMaxReconnectBackoff);
            next_reconnect_time_ = now + backoff;
            camera_.reset();
            set_transport_state(
                transport_state, CameraTransportState::kReconnecting,
                is_reconnect ? "Reconnect attempt failed" : "Connect attempt failed");
            RCLCPP_ERROR(
                get_logger(), "[TRANSPORT] %s attempt #%zu failed: %s. Retry in %lld ms",
                is_reconnect ? "Reconnect" : "Connect", reconnect_attempts_, exception.what(),
                static_cast<long long>(backoff.count()));
            clear_sync_state(
                state, matching_array,
                is_reconnect ? "Reconnect attempt failed" : "Connect attempt failed");
            return false;
        }
    }

    auto set_trigger_mode_checked(
        CameraTransportState& transport_state, WorkerState& state,
        std::vector<TimestampPair>& matching_array, const bool enabled, const char* action)
        -> bool {
        if (camera_ == nullptr) {
            handle_transport_fault(
                transport_state, state, matching_array,
                std::format("{} failed because camera is not connected", action));
            return false;
        }

        const auto code = camera_->set_soft_trigger(enabled);
        if (code != MV_OK) {
            handle_transport_fault(
                transport_state, state, matching_array,
                std::format(
                    "{} failed with Hik SDK error {} ({})", action, code, sdk_error_to_string(code)));
            return false;
        }

        return true;
    }

    auto update_state(
        WorkerState& state, const WorkerState new_state, const char* message,
        CameraTransportState& transport_state, std::vector<TimestampPair>& matching_array,
        std::chrono::steady_clock::time_point& matching_start) -> bool {
        if (transport_state != CameraTransportState::kUp)
            return false;

        switch (new_state) {
        case WorkerState::kReseting:
            sync_model_.reset();
            last_locked_frame_id_.reset();
            if (!set_trigger_mode_checked(
                    transport_state, state, matching_array, true, "Enable trigger mode"))
                return false;
            state = new_state;
            RCLCPP_INFO(get_logger(), "[RESETING] %s", message);
            return true;
        case WorkerState::kMatching:
            sync_model_.reset();
            last_locked_frame_id_.reset();
            if (!set_trigger_mode_checked(
                    transport_state, state, matching_array, false, "Disable trigger mode"))
                return false;
            matching_start = std::chrono::steady_clock::now();
            matching_array.clear();
            state = new_state;
            RCLCPP_INFO(get_logger(), "[MATCHING] %s", message);
            return true;
        case WorkerState::kConfirming:
            if (!set_trigger_mode_checked(
                    transport_state, state, matching_array, true, "Enable trigger mode"))
                return false;
            state = new_state;
            RCLCPP_INFO(get_logger(), "[CONFIRMING] %s", message);
            return true;
        case WorkerState::kLocked:
            last_locked_frame_id_.reset();
            if (!set_trigger_mode_checked(
                    transport_state, state, matching_array, false, "Disable trigger mode"))
                return false;
            state = new_state;
            RCLCPP_INFO(get_logger(), "[LOCKED] %s", message);
            return true;
        }

        return false;
    }

    void frame_callback(const Hikcamera::Frame& frame) {
        last_frame_time_.store(std::chrono::steady_clock::now(), std::memory_order::release);

        if (frame.width != Hikcamera::Cs016FrameWidth || frame.height != Hikcamera::Cs016FrameHeight
            || frame.data.size() != kExpectedFrameSize) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping malformed frame #%u: width=%u (expected %u), height=%u (expected %u), "
                "size=%zu (expected %zu)",
                frame.frame_id, frame.width, Hikcamera::Cs016FrameWidth, frame.height,
                Hikcamera::Cs016FrameHeight, frame.data.size(), kExpectedFrameSize);
            return;
        }

        void* pub_frame_ptr;
        {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            pub_frame_ptr = frame_pool_.allocate();
        }
        if (!pub_frame_ptr) {
            RCLCPP_WARN(
                get_logger(), "Dropping frame #%u: frame pool exhausted (capacity=%zu)",
                frame.frame_id, frame_pool_.max_size());
            return;
        }

        if (!unmatched_image_buffer_.emplace_back_n(
                [&](std::byte* storage) noexcept {
                    auto pub_frame = new (pub_frame_ptr) PublishFrame{};
                    std::memcpy(pub_frame->data, frame.data.data(), kExpectedFrameSize);
                    new (storage) UnmatchedImage{pub_frame, frame.frame_id, frame.timestamp};
                },
                1)) {
            auto guard = std::scoped_lock{frame_pool_mutex_};
            frame_pool_.free(pub_frame_ptr);
            RCLCPP_WARN(
                get_logger(),
                "Dropping frame #%u: unmatched image buffer full (capacity=%zu)",
                frame.frame_id, unmatched_image_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void signal_callback(TriggerBoard::Clock::time_point timestamp) {
        if (!unmatched_signal_buffer_.emplace_back(timestamp)) {
            RCLCPP_WARN(
                get_logger(), "Dropping trigger signal: unmatched signal buffer full (capacity=%zu)",
                unmatched_signal_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void worker_main() {
        using namespace std::chrono_literals;

        auto state = WorkerState::kReseting;
        auto transport_state = CameraTransportState::kDown;
        auto idle_duration = std::chrono::steady_clock::duration::zero();

        auto matching_start = std::chrono::steady_clock::time_point::max();
        std::vector<TimestampPair> matching_array{1000};

        while (!stopped_.test(std::memory_order::relaxed)) {
            if (transport_state != CameraTransportState::kUp) {
                try_connect_camera(transport_state, state, matching_array);
            }

            if (camera_ != nullptr) {
                if (const auto fault_message = camera_->take_transport_fault_message()) {
                    handle_transport_fault(
                        transport_state, state, matching_array,
                        std::format(
                            "MVS exception 0x{:08x} ({})", *fault_message,
                            transport_fault_message_to_string(*fault_message)));
                } else {
                    const auto last_frame_time = last_frame_time_.load(std::memory_order::acquire);
                    if (last_frame_time != std::chrono::steady_clock::time_point::min()) {
                        const auto frame_age = std::chrono::steady_clock::now() - last_frame_time;
                        if (frame_age > kFrameWatchdogTimeout) {
                            handle_transport_fault(
                                transport_state, state, matching_array,
                                std::format(
                                    "Frame watchdog timeout: no frame for {} ms",
                                    std::chrono::duration_cast<std::chrono::milliseconds>(frame_age)
                                        .count()));
                        }
                    }
                }
            }

            if (transport_state != CameraTransportState::kUp) {
                const auto old = event_count_.load(std::memory_order::relaxed);
                std::ignore = rmcs_utility::atomic_wait_timeout(
                    event_count_, old, 50ms, std::memory_order::acquire);
                continue;
            }

            if (idle_duration >= 100ms) {
                if (state == WorkerState::kReseting)
                    std::ignore = update_state(
                        state, WorkerState::kMatching, "Trying to match signal and image...",
                        transport_state, matching_array, matching_start);
                else if (state == WorkerState::kConfirming) {
                    if (!unmatched_signal_buffer_.readable()
                        && !unmatched_image_buffer_.readable()) {
                        auto ls_result = sync_model_.initialize_from_least_squares(matching_array);
                        if (!ls_result) {
                            std::ignore = update_state(
                                state, WorkerState::kReseting,
                                "Least-squares fit failed to initialize lock model",
                                transport_state, matching_array, matching_start);
                        } else if (!(0.8 < ls_result->a && ls_result->a < 1.2)) {
                            std::ignore = update_state(
                                state, WorkerState::kReseting,
                                std::format(
                                    "LS slope out of range: a={:.9f} is outside (0.8, 1.2)",
                                    ls_result->a)
                                    .c_str(),
                                transport_state, matching_array, matching_start);
                        } else if (!(ls_result->max_abs_residual_sec < kResidualThresholdSec)) {
                            std::ignore = update_state(
                                state, WorkerState::kReseting,
                                std::format(
                                    "LS residual too large: max_abs_residual(ms)={:.3f} >= "
                                    "threshold(ms)={:.3f}",
                                    ls_result->max_abs_residual_sec * 1000.0,
                                    kResidualThresholdSec * 1000.0)
                                    .c_str(),
                                transport_state, matching_array, matching_start);
                        } else {
                            std::ignore = update_state(
                                state, WorkerState::kLocked,
                                std::format(
                                    "Hard-sync locked successfully: a={:.9f}, b={:.9f}, "
                                    "var_a={:.3e}, var_b={:.3e}, max_residual(ms)={:.3f}",
                                    ls_result->a, ls_result->b, ls_result->covariance(0, 0),
                                    ls_result->covariance(1, 1),
                                    ls_result->max_abs_residual_sec * 1000.0)
                                    .c_str(),
                                transport_state, matching_array, matching_start);
                        }
                    } else {
                        std::ignore = update_state(
                            state, WorkerState::kReseting,
                            "Failed to match: Mismatched signals and images count",
                            transport_state, matching_array, matching_start);
                    }
                }
            }

            if (state == WorkerState::kMatching
                && std::chrono::steady_clock::now() - matching_start >= 2s) {
                if (matching_array.size() < 50) {
                    std::ignore = update_state(
                        state, WorkerState::kReseting,
                        std::format(
                            "Too few frames matched: collected ({}) < minimum (50)",
                            matching_array.size())
                            .c_str(),
                        transport_state, matching_array, matching_start);
                } else {
                    std::ignore = update_state(
                        state, WorkerState::kConfirming,
                        "Frames collected. Waiting bus idle to ensure no data loss...",
                        transport_state, matching_array, matching_start);
                }
            }

            if (state == WorkerState::kReseting) {
                clear_pending_data();
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
                    std::ignore = update_state(
                        state, WorkerState::kReseting,
                        "RLS model was unexpectedly uninitialized", transport_state,
                        matching_array, matching_start);
                    continue;
                }

                auto* next_image = unmatched_image_buffer_.peek_front();
                if (next_image == nullptr)
                    continue;
                const auto next_frame_id = next_image->frame_id;

                if (last_locked_frame_id_) {
                    const auto expected_frame_id = *last_locked_frame_id_ + 1U;

                    if (next_frame_id < expected_frame_id) {
                        std::ignore = update_state(
                            state, WorkerState::kReseting,
                            std::format(
                                "Frame ID moved backward: expected >= {}, got {}",
                                expected_frame_id, next_frame_id)
                                .c_str(),
                            transport_state, matching_array, matching_start);
                        continue;
                    }

                    const auto frame_gap = next_frame_id - expected_frame_id;
                    if (frame_gap > 0U) {
                        if (frame_gap > kAllowedFrameIdGap) {
                            std::ignore = update_state(
                                state, WorkerState::kReseting,
                                std::format(
                                    "Frame ID gap too large: expected {}, got {}, gap {} > {}",
                                    expected_frame_id, next_frame_id, frame_gap, kAllowedFrameIdGap)
                                    .c_str(),
                                transport_state, matching_array, matching_start);
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
                last_locked_frame_id_ = next_frame_id;
                const auto residual = sync_model_.residual_for(
                    timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
                if (!(std::abs(residual) < kResidualThresholdSec)) {
                    std::ignore = update_state(
                        state, WorkerState::kReseting,
                        std::format(
                            "RLS residual too large: residual(ms)={:.3f}, threshold(ms)={:.3f}",
                            residual * 1000.0, kResidualThresholdSec * 1000.0)
                            .c_str(),
                        transport_state, matching_array, matching_start);
                    continue;
                }

                const auto updated_residual = sync_model_.update(
                    timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
                if (!std::isfinite(updated_residual) || !sync_model_.initialized()) {
                    std::ignore = update_state(
                        state, WorkerState::kReseting, "RLS update failed", transport_state,
                        matching_array, matching_start);
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
        [[maybe_unused]] static constexpr std::uint32_t kWidth = Hikcamera::Cs016FrameWidth;
        [[maybe_unused]] static constexpr std::uint32_t kHeight = Hikcamera::Cs016FrameHeight;
        [[maybe_unused]] static constexpr std::size_t kFrameSize = kExpectedFrameSize;

        alignas(std::uintptr_t) std::byte data[kExpectedFrameSize];
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
    Hikcamera::CameraConfig camera_config_{};
    std::string camera_name_;
    LinearSyncModel sync_model_{kRlsLambda, kResidualThresholdSec};
    std::optional<std::uint32_t> last_locked_frame_id_;
    std::atomic<std::chrono::steady_clock::time_point> last_frame_time_;
    std::chrono::steady_clock::time_point next_reconnect_time_ = std::chrono::steady_clock::time_point::min();
    std::size_t reconnect_attempts_ = 0;

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
