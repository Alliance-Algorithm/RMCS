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
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_utility/atomic_wait_timeout.hpp>
#include <rmcs_utility/memory_pool.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hikcamera.hpp"
#include "linear_sync_model.hpp"

namespace auto_aim_test {

class AutoAimTestComponent final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimTestComponent()
        : Node(
          get_component_name(),
          rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , camera_signal_([
          this](const rmcs_msgs::BoardClock::time_point& timestamp) {
              signal_callback(timestamp);
          }) {
        camera_config_ = Hikcamera::CameraConfig::cs016_default();
        camera_config_.exposure_us =
            static_cast<float>(declare_parameter<double>("exposure_us", 2000.0));
        camera_name_ = declare_parameter<std::string>("camera_name", "");
        camera_config_.gain =
            static_cast<float>(declare_parameter<double>("gain", Hikcamera::Cs016MaxGain));
        camera_config_.framerate = static_cast<float>(
            declare_parameter<double>("framerate", Hikcamera::Cs016MaxFramerate));
        camera_config_.invert_image = declare_parameter<bool>("invert_image", false);
        rls_tau_sec_ = declare_parameter<double>("rls_tau_sec", 10.0);
        residual_threshold_sec_ = 1.0 / static_cast<double>(camera_config_.framerate) / 2.0;
        sync_model_ = LinearSyncModel{rls_tau_sec_, residual_threshold_sec_};

        register_input("/gimbal/auto_aim/camera_signal", camera_signal_);
        last_frame_time_.store(
            std::chrono::steady_clock::time_point::min(), std::memory_order::relaxed);

        RCLCPP_INFO(get_logger(), "auto_aim_test component constructed");
    }

    ~AutoAimTestComponent() override {
        stopped_.test_and_set(std::memory_order::relaxed);
        notify_event();
        if (worker_.joinable())
            worker_.join();

        camera_.reset();
        unmatched_image_buffer_.pop_front_n(
            [this](UnmatchedImage&& image) noexcept { release_frame(std::launder(image.frame)); });
    }

    void before_updating() override {
        if (!worker_.joinable())
            worker_ = std::thread(&AutoAimTestComponent::worker_main, this);
    }

    void update() override {}

private:
    enum class WorkerState { kResetting, kMatching, kConfirming, kLocked };
    enum class CameraTransportState { kDown, kUp, kReconnecting };
    using TimestampPair = LinearSyncModel::TimestampPair;

    static constexpr std::uint32_t kAllowedFrameIdGap = 2;
    static constexpr std::size_t kExpectedFrameSize =
        static_cast<std::size_t>(Hikcamera::Cs016FrameWidth)
        * static_cast<std::size_t>(Hikcamera::Cs016FrameHeight);
    static constexpr auto kFrameWatchdogTimeout = std::chrono::milliseconds{250};
    static constexpr auto kReconnectRetryInterval = std::chrono::milliseconds{2500};

    static auto transport_state_name(const CameraTransportState state) noexcept -> const char* {
        switch (state) {
        case CameraTransportState::kDown: return "DOWN";
        case CameraTransportState::kUp: return "UP";
        case CameraTransportState::kReconnecting: return "RECONNECTING";
        }

        return "UNKNOWN";
    }

    static auto transport_fault_message_to_string(const unsigned int message_type) noexcept -> const
        char* {
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

    void reset_sync_state() {
        sync_model_.reset();
        last_locked_frame_id_.reset();
        matching_array_.clear();
        clear_pending_data();
        worker_state_ = WorkerState::kResetting;
    }

    void clear_pending_data() {
        unmatched_signal_buffer_.clear();
        unmatched_image_buffer_.pop_front_n(
            [this](UnmatchedImage&& image) noexcept { release_frame(image.frame); });
    }

    void set_transport_state(const CameraTransportState new_state, const char* reason) {
        if (transport_state_ == new_state)
            return;

        RCLCPP_INFO(
            get_logger(), "[TRANSPORT] %s -> %s: %s", transport_state_name(transport_state_),
            transport_state_name(new_state), reason);
        transport_state_ = new_state;
    }

    void handle_transport_fault(const std::string& reason) {
        if (transport_state_ == CameraTransportState::kReconnecting)
            return;

        RCLCPP_ERROR(
            get_logger(), "[TRANSPORT] %s -> RECONNECTING: %s",
            transport_state_name(transport_state_), reason.c_str());
        camera_.reset();
        transport_state_ = CameraTransportState::kReconnecting;
        reset_sync_state();
        reconnect_attempts_ = 0;
        next_reconnect_time_ = std::chrono::steady_clock::now();
    }

    auto try_connect_camera() -> bool {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_reconnect_time_)
            return false;

        const bool is_reconnect =
            reconnect_attempts_ > 0 || transport_state_ == CameraTransportState::kReconnecting;
        ++reconnect_attempts_;

        try {
            camera_ = std::make_unique<Hikcamera>(
                camera_config_,
                [this](const Hikcamera::Frame& frame) noexcept { frame_callback(frame); },
                camera_name_);
            last_frame_time_.store(std::chrono::steady_clock::now(), std::memory_order::release);
            RCLCPP_INFO(
                get_logger(), "[TRANSPORT] %s -> UP: Camera %s (attempt #%zu)",
                transport_state_name(transport_state_), is_reconnect ? "reconnected" : "connected",
                reconnect_attempts_);
            transport_state_ = CameraTransportState::kUp;
            reset_sync_state();
            reconnect_attempts_ = 0;
            return true;
        } catch (const std::exception& exception) {
            const auto backoff = kReconnectRetryInterval;
            next_reconnect_time_ = now + backoff;
            camera_.reset();
            set_transport_state(
                CameraTransportState::kReconnecting,
                is_reconnect ? "Reconnect attempt failed" : "Connect attempt failed");
            RCLCPP_ERROR(
                get_logger(), "[TRANSPORT] %s #%zu failed: %s (retry in %lld ms)",
                is_reconnect ? "Reconnect" : "Connect", reconnect_attempts_, exception.what(),
                static_cast<long long>(backoff.count()));
            reset_sync_state();
            return false;
        }
    }

    auto set_trigger_mode_checked(const bool enabled, const char* action) -> bool {
        if (camera_ == nullptr) {
            handle_transport_fault(
                std::format("{} failed because camera is not connected", action));
            return false;
        }

        const auto code = camera_->set_soft_trigger(enabled);
        if (code != MV_OK) {
            handle_transport_fault(
                std::format(
                    "{} failed with Hik SDK error {} ({})", action, code,
                    sdk_error_to_string(code)));
            return false;
        }

        return true;
    }

    static auto state_name(const WorkerState state) noexcept -> const char* {
        switch (state) {
        case WorkerState::kResetting: return "RESETTING";
        case WorkerState::kMatching: return "MATCHING";
        case WorkerState::kConfirming: return "CONFIRMING";
        case WorkerState::kLocked: return "LOCKED";
        }
        return "UNKNOWN";
    }

    bool transition_to(WorkerState new_state, const char* message) {
        if (transport_state_ != CameraTransportState::kUp)
            return false;

        bool trigger_on =
            (new_state == WorkerState::kResetting || new_state == WorkerState::kConfirming);
        if (!set_trigger_mode_checked(
                trigger_on, trigger_on ? "Enable trigger mode" : "Disable trigger mode"))
            return false;

        switch (new_state) {
        case WorkerState::kResetting:
            sync_model_.reset();
            last_locked_frame_id_.reset();
            matching_array_.clear();
            clear_pending_data();
            break;
        case WorkerState::kMatching:
            sync_model_.reset();
            last_locked_frame_id_.reset();
            matching_start_ = std::chrono::steady_clock::now();
            matching_array_.clear();
            break;
        case WorkerState::kConfirming: break;
        case WorkerState::kLocked:
            last_locked_frame_id_.reset();
            locked_frame_count_ = 0;
            last_locked_summary_count_ = 0;
            max_residual_sec_ = 0.0;
            last_locked_summary_time_ = std::chrono::steady_clock::now();
            break;
        }
        worker_state_ = new_state;
        RCLCPP_INFO(get_logger(), "[%s] %s", state_name(new_state), message);
        return true;
    }

    void frame_callback(const Hikcamera::Frame& frame) {
        last_frame_time_.store(std::chrono::steady_clock::now(), std::memory_order::release);

        if (frame.width != Hikcamera::Cs016FrameWidth || frame.height != Hikcamera::Cs016FrameHeight
            || frame.data.size() != kExpectedFrameSize) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
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
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Dropping frame #%u: frame pool exhausted (capacity=%zu)", frame.frame_id,
                frame_pool_.max_size());
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
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Dropping frame #%u: unmatched image buffer full (capacity=%zu)", frame.frame_id,
                unmatched_image_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void signal_callback(rmcs_msgs::BoardClock::time_point timestamp) {
        if (!unmatched_signal_buffer_.emplace_back(timestamp)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Dropping trigger signal: unmatched signal buffer full (capacity=%zu)",
                unmatched_signal_buffer_.max_size());
            return;
        }

        notify_event();
    }

    void worker_main() {
        using namespace std::chrono_literals;
        matching_array_.reserve(1000);

        while (!stopped_.test(std::memory_order::relaxed)) {
            if (!ensure_transport_up())
                continue;
            tick_idle_transitions();
            if (worker_state_ == WorkerState::kMatching
                && std::chrono::steady_clock::now() - matching_start_ >= 2s) {
                if (matching_array_.size() < 50) {
                    std::ignore = transition_to(
                        WorkerState::kResetting,
                        std::format(
                            "Too few frames matched: collected ({}) < minimum (50)",
                            matching_array_.size())
                            .c_str());
                } else {
                    std::ignore = transition_to(
                        WorkerState::kConfirming, "Draining buffers before fitting...");
                }
            }
            if (worker_state_ == WorkerState::kResetting)
                clear_pending_data();
            if (!wait_for_data())
                continue;
            if (worker_state_ == WorkerState::kResetting)
                continue;
            consume_and_process();
        }
    }

    bool wait_event(std::chrono::milliseconds timeout) {
        const auto old = event_count_.load(std::memory_order::relaxed);
        return rmcs_utility::atomic_wait_timeout(
            event_count_, old, timeout, std::memory_order::acquire);
    }

    bool ensure_transport_up() {
        if (transport_state_ != CameraTransportState::kUp)
            try_connect_camera();

        if (camera_ != nullptr) {
            if (const auto fault_message = camera_->take_transport_fault_message()) {
                handle_transport_fault(
                    std::format(
                        "MVS exception 0x{:08x} ({})", *fault_message,
                        transport_fault_message_to_string(*fault_message)));
            } else {
                const auto last_frame_time = last_frame_time_.load(std::memory_order::acquire);
                if (last_frame_time != std::chrono::steady_clock::time_point::min()) {
                    const auto frame_age = std::chrono::steady_clock::now() - last_frame_time;
                    if (frame_age > kFrameWatchdogTimeout) {
                        handle_transport_fault(
                            std::format(
                                "Frame watchdog timeout: no frame for {} ms",
                                std::chrono::duration_cast<std::chrono::milliseconds>(frame_age)
                                    .count()));
                    }
                }
            }
        }

        if (transport_state_ != CameraTransportState::kUp) {
            wait_event(std::chrono::milliseconds{50});
            return false;
        }
        return true;
    }

    void tick_idle_transitions() {
        using namespace std::chrono_literals;
        if (idle_duration_ < 100ms)
            return;

        if (worker_state_ == WorkerState::kResetting) {
            std::ignore = transition_to(WorkerState::kMatching, "Matching signals and images...");
        } else if (worker_state_ == WorkerState::kConfirming) {
            if (!unmatched_signal_buffer_.readable() && !unmatched_image_buffer_.readable()) {
                auto ls_result = sync_model_.initialize_from_least_squares(matching_array_);
                if (!ls_result) {
                    std::ignore = transition_to(
                        WorkerState::kResetting,
                        "Least-squares fit failed to initialize lock model");
                } else if (!(0.8 < ls_result->a && ls_result->a < 1.2)) {
                    std::ignore = transition_to(
                        WorkerState::kResetting,
                        std::format(
                            "LS slope out of range: a={:.3f} (expected 0.8..1.2)", ls_result->a)
                            .c_str());
                } else if (!(ls_result->max_abs_residual_sec < residual_threshold_sec_)) {
                    std::ignore = transition_to(
                        WorkerState::kResetting,
                        std::format(
                            "LS max residual={:.3f} ms >= threshold={:.3f} ms",
                            ls_result->max_abs_residual_sec * 1000.0,
                            residual_threshold_sec_ * 1000.0)
                            .c_str());
                } else {
                    std::ignore = transition_to(
                        WorkerState::kLocked,
                        std::format(
                            "Locked from {} frames: a={:.3f}, b={:.3f}, var_a={:.3e}, "
                            "var_b={:.3e}, max_residual={:.3f} ms",
                            matching_array_.size(), ls_result->a, ls_result->b,
                            ls_result->covariance(0, 0), ls_result->covariance(1, 1),
                            ls_result->max_abs_residual_sec * 1000.0)
                            .c_str());
                }
            } else {
                std::ignore =
                    transition_to(WorkerState::kResetting, "Buffer count mismatch after drain");
            }
        }
    }

    bool wait_for_data() {
        using namespace std::chrono_literals;
        const auto old = event_count_.load(std::memory_order::relaxed);
        if (unmatched_signal_buffer_.readable() && unmatched_image_buffer_.readable())
            return true;
        if (!rmcs_utility::atomic_wait_timeout(
                event_count_, old, 50ms, std::memory_order::acquire)) {
            idle_duration_ += 50ms;
            return false;
        }
        idle_duration_ = std::chrono::steady_clock::duration::zero();
        return unmatched_signal_buffer_.readable() && unmatched_image_buffer_.readable();
    }

    void process_locked_frame() {
        if (!sync_model_.initialized()) {
            std::ignore =
                transition_to(WorkerState::kResetting, "RLS model was unexpectedly uninitialized");
            return;
        }

        auto* next_image = unmatched_image_buffer_.peek_front();
        if (next_image == nullptr)
            return;
        const auto next_frame_id = next_image->frame_id;

        if (last_locked_frame_id_) {
            const auto expected_frame_id = *last_locked_frame_id_ + 1U;

            if (next_frame_id < expected_frame_id) {
                std::ignore = transition_to(
                    WorkerState::kResetting, std::format(
                                                 "Frame ID moved backward: expected >= {}, got {}",
                                                 expected_frame_id, next_frame_id)
                                                 .c_str());
                return;
            }

            const auto frame_gap = next_frame_id - expected_frame_id;
            if (frame_gap > 0U) {
                if (frame_gap > kAllowedFrameIdGap) {
                    std::ignore = transition_to(
                        WorkerState::kResetting,
                        std::format(
                            "Frame ID gap too large: expected {}, got {}, gap {} > {}",
                            expected_frame_id, next_frame_id, frame_gap, kAllowedFrameIdGap)
                            .c_str());
                    return;
                }

                if (unmatched_signal_buffer_.readable() < static_cast<std::size_t>(frame_gap) + 1U)
                    return;

                drop_signals(frame_gap);
                RCLCPP_WARN(
                    get_logger(),
                    "Tolerating camera frame gap: expected %u, got %u, dropped %u queued signals",
                    expected_frame_id, next_frame_id, frame_gap);
            }
        }

        const auto timestamp_pair = consume_frame();
        last_locked_frame_id_ = next_frame_id;
        locked_frame_count_++;
        if (const auto now = std::chrono::steady_clock::now();
            now - last_locked_summary_time_ >= std::chrono::milliseconds{5000}) {
            const auto window_frames = locked_frame_count_ - last_locked_summary_count_;
            const auto fps = static_cast<double>(window_frames)
                           / std::chrono::duration<double>(now - last_locked_summary_time_).count();
            RCLCPP_INFO(
                get_logger(),
                "[LOCKED] %zu frames total, avg %.1f FPS, a=%.3f, b=%.3f, max_residual=%.3f ms",
                locked_frame_count_, fps, sync_model_.a(), sync_model_.b(),
                max_residual_sec_ * 1000.0);
            last_locked_summary_count_ = locked_frame_count_;
            max_residual_sec_ = 0.0;
            last_locked_summary_time_ = now;
        }
        const auto residual = sync_model_.residual_for(
            timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
        max_residual_sec_ = std::max(max_residual_sec_, std::abs(residual));
        if (!(std::abs(residual) < residual_threshold_sec_)) {
            std::ignore = transition_to(
                WorkerState::kResetting,
                std::format(
                    "RLS residual too large: residual={:.3f} ms, threshold={:.3f} ms, "
                    "camera_ts={:.3f} ms, signal_ts={:.3f} ms",
                    residual * 1000.0, residual_threshold_sec_ * 1000.0,
                    timestamp_pair.camera_timestamp_sec * 1000.0,
                    timestamp_pair.board_timestamp_sec * 1000.0)
                    .c_str());
            return;
        }

        const auto updated_residual = sync_model_.update(
            timestamp_pair.camera_timestamp_sec, timestamp_pair.board_timestamp_sec);
        if (!std::isfinite(updated_residual) || !sync_model_.initialized()) {
            std::ignore = transition_to(WorkerState::kResetting, "RLS update failed");
        }
    }

    void consume_and_process() {
        if (worker_state_ == WorkerState::kMatching || worker_state_ == WorkerState::kConfirming) {
            matching_array_.push_back(consume_frame());
        } else if (worker_state_ == WorkerState::kLocked) {
            process_locked_frame();
        }
    }

    void drop_signals(const std::uint32_t count) {
        const auto dropped = unmatched_signal_buffer_.pop_front_n(
            [](rmcs_msgs::BoardClock::time_point&&) noexcept {}, count);
        if (dropped != count) {
            std::fprintf(stderr, "drop_signals expected %u but dropped %zu\n", count, dropped);
            std::terminate();
        }
    }

    TimestampPair consume_frame() {
        TimestampPair timestamp_pair;

        if (!unmatched_signal_buffer_.pop_front(
                [&](rmcs_msgs::BoardClock::time_point&& timestamp) noexcept {
                    timestamp_pair.board_timestamp_sec =
                        std::chrono::duration_cast<std::chrono::duration<double>>(
                            timestamp.time_since_epoch())
                            .count();
                })) {
            std::fprintf(stderr, "consume_frame: signal buffer unexpectedly empty\n");
            std::terminate();
        }

        if (!unmatched_image_buffer_.pop_front([&](UnmatchedImage&& image) noexcept {
                timestamp_pair.camera_timestamp_sec =
                    std::chrono::duration_cast<std::chrono::duration<double>>(
                        image.timestamp.time_since_epoch())
                        .count();

                // Process Image ...

                release_frame(image.frame);
            }))
            std::terminate();

        return timestamp_pair;
    }

    void notify_event() {
        event_count_.fetch_add(1, std::memory_order::release);
        event_count_.notify_one();
    }

    struct PublishFrame;

    void release_frame(PublishFrame* frame) {
        std::destroy_at(frame);
        auto guard = std::scoped_lock{frame_pool_mutex_};
        frame_pool_.free(frame);
    }

    struct PublishFrame {
        [[maybe_unused]] static constexpr std::uint32_t kWidth = Hikcamera::Cs016FrameWidth;
        [[maybe_unused]] static constexpr std::uint32_t kHeight = Hikcamera::Cs016FrameHeight;
        [[maybe_unused]] static constexpr std::size_t kFrameSize = kExpectedFrameSize;

        alignas(std::uintptr_t) std::byte data[kExpectedFrameSize];
        int opencv_cvt_color_code;

        Eigen::Quaterniond imu_snapshot;
        rmcs_msgs::BoardClock::time_point timestamp;
    };
    rmcs_utility::MemoryPool<sizeof(PublishFrame), alignof(PublishFrame)> frame_pool_{100};
    std::mutex frame_pool_mutex_;

    rmcs_utility::RingBuffer<rmcs_msgs::BoardClock::time_point> unmatched_signal_buffer_{32};
    EventInputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_;

    struct UnmatchedImage {
        PublishFrame* frame;
        std::uint32_t frame_id;
        Hikcamera::HikDeviceClock::time_point timestamp;
    };
    rmcs_utility::RingBuffer<UnmatchedImage> unmatched_image_buffer_{16};
    std::unique_ptr<Hikcamera> camera_;
    Hikcamera::CameraConfig camera_config_{};
    std::string camera_name_;
    WorkerState worker_state_ = WorkerState::kResetting;
    CameraTransportState transport_state_ = CameraTransportState::kDown;
    std::chrono::steady_clock::duration idle_duration_{};
    std::chrono::steady_clock::time_point matching_start_{};
    std::vector<TimestampPair> matching_array_{};
    double rls_tau_sec_ = 8.0;
    double residual_threshold_sec_ = 1.0 / 249.1 / 2.0;
    LinearSyncModel sync_model_{rls_tau_sec_, residual_threshold_sec_};
    std::optional<std::uint32_t> last_locked_frame_id_;
    std::size_t locked_frame_count_ = 0;
    std::size_t last_locked_summary_count_ = 0;
    double max_residual_sec_ = 0.0;
    std::chrono::steady_clock::time_point last_locked_summary_time_{};
    std::atomic<std::chrono::steady_clock::time_point> last_frame_time_;
    std::chrono::steady_clock::time_point next_reconnect_time_ =
        std::chrono::steady_clock::time_point::min();
    std::size_t reconnect_attempts_ = 0;

    std::atomic_flag stopped_ = ATOMIC_FLAG_INIT;
    std::atomic<std::uint32_t> event_count_ = 0;
    std::thread worker_;
};

} // namespace auto_aim_test

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(auto_aim_test::AutoAimTestComponent, rmcs_executor::Component)
