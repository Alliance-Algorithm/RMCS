#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <librmcs/agent/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/spec/rmcs_board_lite/gpio.hpp>

#include "hikcamera/capturer.hpp"

namespace {

using namespace std::chrono_literals;

constexpr auto kCameraBufferCapacity = std::size_t{512};
constexpr auto kTriggerBufferCapacity = std::size_t{512};

struct CameraFrame final {
    std::uint32_t frame_id = 0;
    std::uint64_t camera_timestamp = 0;
    std::int64_t sdk_host_timestamp = 0;
    hikcamera::Camera::Image::Stamp host_timestamp{};
};

struct TriggerEvent final {
    std::uint64_t seq = 0;
    std::uint64_t timestamp_quarter_us = 0;
};

template <typename T>
auto trim_deque(std::deque<T>& queue, const std::size_t capacity) -> void {
    while (queue.size() > capacity)
        queue.pop_front();
}

class ForgettingRls final {
public:
    explicit ForgettingRls(double lambda = 0.9995)
        : lambda_(lambda) {}

    auto reset(std::uint64_t mcu_ref_quarter_us, std::uint64_t camera_ref_timestamp) -> void {
        mcu_ref_quarter_us_ = mcu_ref_quarter_us;
        camera_ref_timestamp_ = camera_ref_timestamp;
        theta_ = {1.0, 0.0};
        p00_ = 1e6;
        p01_ = 0.0;
        p10_ = 0.0;
        p11_ = 1e6;
        sample_count_ = 0;
        initialized_ = true;
    }

    [[nodiscard]] auto initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto sample_count() const noexcept -> std::size_t { return sample_count_; }
    [[nodiscard]] auto a() const noexcept -> double { return theta_[0]; }
    [[nodiscard]] auto b() const noexcept -> double { return theta_[1]; }

    [[nodiscard]] auto predict(std::uint64_t mcu_timestamp_quarter_us) const noexcept -> double {
        const auto x = make_regressor(mcu_timestamp_quarter_us);
        return theta_[0] * x[0] + theta_[1];
    }

    [[nodiscard]] auto residual_for(
        std::uint64_t mcu_timestamp_quarter_us, std::uint64_t camera_timestamp) const noexcept
        -> double {
        return make_observation(camera_timestamp) - predict(mcu_timestamp_quarter_us);
    }

    auto update(std::uint64_t mcu_timestamp_quarter_us, std::uint64_t camera_timestamp) noexcept
        -> double {
        const auto x = make_regressor(mcu_timestamp_quarter_us);
        const auto y = make_observation(camera_timestamp);
        const auto residual = y - (theta_[0] * x[0] + theta_[1]);

        const auto px0 = p00_ * x[0] + p01_ * x[1];
        const auto px1 = p10_ * x[0] + p11_ * x[1];
        const auto denom = lambda_ + x[0] * px0 + x[1] * px1;

        const auto k0 = px0 / denom;
        const auto k1 = px1 / denom;

        theta_[0] += k0 * residual;
        theta_[1] += k1 * residual;

        const auto xp0 = x[0] * p00_ + x[1] * p10_;
        const auto xp1 = x[0] * p01_ + x[1] * p11_;

        const auto next_p00 = (p00_ - k0 * xp0) / lambda_;
        const auto next_p01 = (p01_ - k0 * xp1) / lambda_;
        const auto next_p10 = (p10_ - k1 * xp0) / lambda_;
        const auto next_p11 = (p11_ - k1 * xp1) / lambda_;

        p00_ = next_p00;
        p01_ = next_p01;
        p10_ = next_p10;
        p11_ = next_p11;
        ++sample_count_;

        return residual;
    }

private:
    [[nodiscard]] auto make_regressor(std::uint64_t mcu_timestamp_quarter_us) const noexcept
        -> std::array<double, 2> {
        const auto delta_quarter_us = mcu_timestamp_quarter_us - mcu_ref_quarter_us_;
        return {static_cast<double>(delta_quarter_us) * 0.25, 1.0};
    }

    [[nodiscard]] auto make_observation(std::uint64_t camera_timestamp) const noexcept -> double {
        return static_cast<double>(camera_timestamp - camera_ref_timestamp_);
    }

    double lambda_ = 0.9995;
    std::uint64_t mcu_ref_quarter_us_ = 0;
    std::uint64_t camera_ref_timestamp_ = 0;
    std::array<double, 2> theta_{1.0, 0.0};
    double p00_ = 1e6;
    double p01_ = 0.0;
    double p10_ = 0.0;
    double p11_ = 1e6;
    std::size_t sample_count_ = 0;
    bool initialized_ = false;
};

enum class SyncState {
    kUnlocked,
    kSyncing,
    kLocked,
    kLostLock,
};

[[nodiscard]] auto to_string(const SyncState state) noexcept -> const char* {
    switch (state) {
    case SyncState::kUnlocked: return "UNLOCKED";
    case SyncState::kSyncing: return "SYNCING";
    case SyncState::kLocked: return "LOCKED";
    case SyncState::kLostLock: return "LOST_LOCK";
    }
    return "UNKNOWN";
}

[[nodiscard]] auto select_gpio_descriptor(std::string_view name)
    -> const librmcs::spec::rmcs_board_lite::GpioDescriptor& {
    using librmcs::spec::rmcs_board_lite::kGpioDescriptors;

    if (name == "uart0_rx")
        return kGpioDescriptors.kUart0Rx;
    if (name == "uart0_tx")
        return kGpioDescriptors.kUart0Tx;
    if (name == "uart1_rx")
        return kGpioDescriptors.kUart1Rx;
    if (name == "uart1_tx")
        return kGpioDescriptors.kUart1Tx;

    throw std::invalid_argument{"Unsupported GPIO name: " + std::string{name}};
}

class AutoAimTestNode;

class TriggerBoard final : private librmcs::agent::RmcsBoardLite {
public:
    TriggerBoard(
        AutoAimTestNode& node, std::string_view serial_filter,
        const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio);

private:
    void gpio_digital_read_result_callback(
        const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio,
        const librmcs::data::GpioDigitalDataView& data) override;

    AutoAimTestNode& node_;
    const librmcs::spec::rmcs_board_lite::GpioDescriptor* gpio_ = nullptr;
};

class AutoAimTestNode : public rclcpp::Node {
public:
    AutoAimTestNode()
        : rclcpp::Node{"auto_aim_test"} {
        const auto timeout_ms = std::max(
            std::int64_t{1}, declare_parameter<std::int64_t>("timeout_ms", 2'000));
        const auto exposure_us = declare_parameter<double>("exposure_us", 2'000.0);
        const auto framerate = declare_parameter<double>("framerate", 249.1);
        const auto gain = declare_parameter<double>("gain", hikcamera::kMaxGain);

        camera_config_.timeout_ms = static_cast<unsigned int>(timeout_ms);
        camera_config_.exposure_us = static_cast<float>(exposure_us);
        camera_config_.framerate = static_cast<float>(framerate);
        camera_config_.gain = static_cast<float>(gain);
        camera_config_.invert_image = declare_parameter<bool>("invert_image", false);
        camera_config_.software_sync = declare_parameter<bool>("software_sync", false);
        camera_config_.trigger_mode = declare_parameter<bool>("trigger_mode", false);
        camera_config_.fixed_framerate = declare_parameter<bool>("fixed_framerate", true);

        if (camera_config_.software_sync || camera_config_.trigger_mode) {
            throw std::invalid_argument{
                "auto_aim_test demo expects free-running steady state (software_sync=false, "
                "trigger_mode=false). It switches to software trigger only during relock."};
        }

        wait_key_ms_ = static_cast<int>(
            std::max(std::int64_t{1}, declare_parameter<std::int64_t>("wait_key_ms", 1)));
        window_name_ = declare_parameter<std::string>("window_name", "auto_aim_test");
        board_serial_ = declare_parameter<std::string>("board_serial", "");
        board_gpio_name_ = declare_parameter<std::string>("board_gpio", "uart1_tx");
        sync_pause_ms_ = static_cast<int>(
            std::max(std::int64_t{1}, declare_parameter<std::int64_t>("sync_pause_ms", 100)));
        sync_timeout_ms_ = static_cast<int>(std::max(
            std::int64_t{10}, declare_parameter<std::int64_t>("sync_timeout_ms", 1'000)));
        rls_lambda_ = declare_parameter<double>("rls_lambda", 0.9995);
        warmup_samples_ = static_cast<std::size_t>(std::max(
            std::int64_t{1}, declare_parameter<std::int64_t>("warmup_samples", 16)));
        max_consecutive_bad_residuals_ = static_cast<std::size_t>(std::max(
            std::int64_t{1}, declare_parameter<std::int64_t>("bad_residual_limit", 3)));
        residual_soft_ratio_ = declare_parameter<double>("residual_soft_ratio", 0.45);
        residual_hard_ratio_ = declare_parameter<double>("residual_hard_ratio", 0.90);

        validate_parameters(exposure_us, framerate, gain);

        rls_ = ForgettingRls{rls_lambda_};
        camera_.configure(camera_config_);
        board_ = std::make_unique<TriggerBoard>(
            *this, board_serial_, select_gpio_descriptor(board_gpio_name_));
    }

    auto run() -> int {
        if (const auto result = camera_.connect(); !result) {
            RCLCPP_ERROR(get_logger(), "Failed to connect hikcamera: %s", result.error().c_str());
            return 1;
        }

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        last_report_time_ = std::chrono::steady_clock::now();

        if (!perform_resync("initial startup")) {
            shutdown_camera();
            cv::destroyAllWindows();
            return 1;
        }

        while (rclcpp::ok()) {
            if (pending_resync_) {
                if (!perform_resync(resync_reason_)) {
                    std::this_thread::sleep_for(100ms);
                    continue;
                }
            }

            auto image_result = camera_.read_image_with_timestamp();
            if (!image_result) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 1'000, "Failed to capture frame: %s",
                    image_result.error().c_str());
                continue;
            }

            ++frames_since_report_;
            handle_frame(*image_result);
            // cv::imshow(window_name_, image_result->mat);

            if (const auto key = cv::waitKey(wait_key_ms_); key == 27 || key == 'q' || key == 'Q') {
                RCLCPP_INFO(get_logger(), "Exit requested from viewer window.");
                break;
            } else if (key == 's' || key == 'r' || key == 'R') {
                request_resync("manual resync requested from keyboard");
            } else if (key == 'x' || key == 'X') {
                inject_fault();
            }

            if (cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1.0) {
                RCLCPP_INFO(get_logger(), "Viewer window closed.");
                break;
            }

            maybe_report_status();
        }

        cv::destroyAllWindows();
        shutdown_camera();
        return 0;
    }

    auto on_trigger_edge(const std::uint32_t raw_timestamp_quarter_us) -> void {
        std::lock_guard lock{trigger_mutex_};

        if (have_last_raw_trigger_timestamp_ && raw_timestamp_quarter_us < last_raw_trigger_timestamp_
            && (last_raw_trigger_timestamp_ - raw_timestamp_quarter_us) > (1ULL << 31U)) {
            ++trigger_timestamp_wrap_count_;
        }

        have_last_raw_trigger_timestamp_ = true;
        last_raw_trigger_timestamp_ = raw_timestamp_quarter_us;

        ++next_trigger_seq_;
        trigger_events_.push_back(TriggerEvent{
            .seq = next_trigger_seq_,
            .timestamp_quarter_us =
                (trigger_timestamp_wrap_count_ << 32U) | raw_timestamp_quarter_us,
        });
        trim_deque(trigger_events_, kTriggerBufferCapacity);
        trigger_cv_.notify_all();
    }

private:
    auto validate_parameters(const double exposure_us, const double framerate, const double gain)
        const -> void {
        if (!(exposure_us > 0.0)) {
            throw std::invalid_argument{"exposure_us must be positive"};
        }
        if (!(framerate > 0.0 && framerate <= 1000.0)) {
            throw std::invalid_argument{"framerate must be in (0, 1000]"};
        }
        if (!(gain >= 0.0 && gain <= hikcamera::kMaxGain)) {
            throw std::invalid_argument{"gain must be within [0, hikcamera::kMaxGain]"};
        }
        if (!(sync_pause_ms_ >= 1 && sync_pause_ms_ <= 5000)) {
            throw std::invalid_argument{"sync_pause_ms must be in [1, 5000]"};
        }
        if (!(sync_timeout_ms_ >= 10 && sync_timeout_ms_ <= 10000)) {
            throw std::invalid_argument{"sync_timeout_ms must be in [10, 10000]"};
        }
        if (!(rls_lambda_ > 0.0 && rls_lambda_ <= 1.0)) {
            throw std::invalid_argument{"rls_lambda must be in (0, 1]"};
        }
        if (!(warmup_samples_ >= 1 && warmup_samples_ <= kCameraBufferCapacity)) {
            throw std::invalid_argument{"warmup_samples must be within camera buffer capacity"};
        }
        if (!(max_consecutive_bad_residuals_ >= 1 && max_consecutive_bad_residuals_ <= 64)) {
            throw std::invalid_argument{"bad_residual_limit must be in [1, 64]"};
        }
        if (!(residual_soft_ratio_ > 0.0 && residual_soft_ratio_ < 1.0)) {
            throw std::invalid_argument{"residual_soft_ratio must be in (0, 1)"};
        }
        if (!(residual_hard_ratio_ > residual_soft_ratio_ && residual_hard_ratio_ <= 2.0)) {
            throw std::invalid_argument{
                "residual_hard_ratio must be greater than residual_soft_ratio and <= 2.0"};
        }
    }

    auto shutdown_camera() -> void {
        if (!camera_.connected())
            return;

        if (const auto result = camera_.disconnect(); !result) {
            RCLCPP_WARN(get_logger(), "Failed to disconnect hikcamera: %s", result.error().c_str());
        }
    }

    auto request_resync(std::string reason) -> void {
        if (!pending_resync_) {
            RCLCPP_WARN(get_logger(), "Relock requested: %s", reason.c_str());
        }
        pending_resync_ = true;
        resync_reason_ = std::move(reason);
        state_ = SyncState::kLostLock;
        camera_frames_.clear();
        last_captured_frame_id_.reset();
        last_matched_trigger_seq_.reset();
        consecutive_bad_residuals_ = 0;
    }

    auto inject_fault() -> void {
        if (state_ != SyncState::kLocked || !delta_) {
            RCLCPP_WARN(get_logger(), "Ignoring fault injection because the demo is not locked.");
            return;
        }

        ++(*delta_);
        RCLCPP_WARN(
            get_logger(),
            "Injected demo fault: delta perturbed to %lld. Next matched frame should trigger lost-lock detection.",
            static_cast<long long>(*delta_));
    }

    auto perform_resync(const std::string& reason) -> bool {
        pending_resync_ = false;
        state_ = SyncState::kSyncing;
        camera_frames_.clear();
        clear_trigger_queue();

        RCLCPP_WARN(get_logger(), "Starting relock: %s", reason.c_str());

        if (const auto result = camera_.set_trigger_mode(true); !result) {
            RCLCPP_ERROR(get_logger(), "Failed to enable software trigger mode: %s", result.error().c_str());
            pending_resync_ = true;
            return false;
        }
        if (const auto result = camera_.set_trigger_source_software(); !result) {
            RCLCPP_ERROR(get_logger(), "Failed to select software trigger source: %s", result.error().c_str());
            restore_runtime_capture_mode();
            pending_resync_ = true;
            return false;
        }
        if (const auto result = camera_.clear_image_buffer(); !result) {
            RCLCPP_WARN(get_logger(), "Failed to clear camera buffer before relock: %s", result.error().c_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds{sync_pause_ms_});
        const auto last_seq_before_anchor = latest_trigger_seq();

        if (const auto result = camera_.execute_software_trigger(); !result) {
            RCLCPP_ERROR(get_logger(), "Failed to execute software trigger: %s", result.error().c_str());
            restore_runtime_capture_mode();
            pending_resync_ = true;
            return false;
        }

        auto anchor_frame = camera_.read_image_with_timestamp();
        auto anchor_trigger = wait_for_trigger_after(
            last_seq_before_anchor, std::chrono::milliseconds{sync_timeout_ms_});

        restore_runtime_capture_mode();

        if (!anchor_frame) {
            RCLCPP_ERROR(get_logger(), "Failed to read anchor frame: %s", anchor_frame.error().c_str());
            pending_resync_ = true;
            return false;
        }
        if (!anchor_trigger) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for anchor trigger edge after seq=%llu", static_cast<unsigned long long>(last_seq_before_anchor));
            pending_resync_ = true;
            return false;
        }

        delta_ = static_cast<std::int64_t>(anchor_frame->frame_id)
            - static_cast<std::int64_t>(anchor_trigger->seq);
        rls_.reset(anchor_trigger->timestamp_quarter_us, anchor_frame->device_timestamp);
        state_ = SyncState::kLocked;
        resync_reason_.clear();
        camera_frames_.clear();
        discard_triggers_up_to(anchor_trigger->seq);
        last_captured_frame_id_.reset();
        last_matched_trigger_seq_.reset();
        last_camera_timestamp_.reset();
        camera_period_estimate_.reset();
        residual_abs_ema_.reset();
        residual_rms_ema_.reset();
        consecutive_bad_residuals_ = 0;
        matched_pairs_ = 0;

        RCLCPP_WARN(
            get_logger(),
            "Relock success: anchor frame_id=%u trigger_seq=%llu delta=%lld t_cam=%llu t_mcu_q=%llu",
            anchor_frame->frame_id, static_cast<unsigned long long>(anchor_trigger->seq),
            static_cast<long long>(*delta_),
            static_cast<unsigned long long>(anchor_frame->device_timestamp),
            static_cast<unsigned long long>(anchor_trigger->timestamp_quarter_us));
        return true;
    }

    auto restore_runtime_capture_mode() -> void {
        if (const auto result = camera_.set_trigger_mode(false); !result) {
            RCLCPP_WARN(get_logger(), "Failed to restore free-running mode: %s", result.error().c_str());
        }
        if (const auto result = camera_.clear_image_buffer(); !result) {
            RCLCPP_WARN(get_logger(), "Failed to clear camera buffer after relock: %s", result.error().c_str());
        }
    }

    auto latest_trigger_seq() const -> std::uint64_t {
        std::lock_guard lock{trigger_mutex_};
        return next_trigger_seq_;
    }

    auto clear_trigger_queue() -> void {
        std::lock_guard lock{trigger_mutex_};
        trigger_events_.clear();
    }

    auto discard_triggers_up_to(const std::uint64_t seq_inclusive) -> void {
        std::lock_guard lock{trigger_mutex_};
        while (!trigger_events_.empty() && trigger_events_.front().seq <= seq_inclusive)
            trigger_events_.pop_front();
    }

    auto wait_for_trigger_after(
        const std::uint64_t last_seq, const std::chrono::milliseconds timeout)
        -> std::optional<TriggerEvent> {
        std::unique_lock lock{trigger_mutex_};
        const auto has_new_event = [&] {
            return std::ranges::any_of(trigger_events_, [&](const TriggerEvent& event) {
                return event.seq > last_seq;
            });
        };

        if (!trigger_cv_.wait_for(lock, timeout, has_new_event))
            return std::nullopt;

        const auto it = std::ranges::find_if(trigger_events_, [&](const TriggerEvent& event) {
            return event.seq > last_seq;
        });
        if (it == trigger_events_.end())
            return std::nullopt;
        return *it;
    }

    auto handle_frame(const hikcamera::Camera::Image& image) -> void {
        if (state_ != SyncState::kLocked || !delta_)
            return;

        if (last_captured_frame_id_ && image.frame_id != (*last_captured_frame_id_ + 1U)) {
            request_resync(
                "camera frame id discontinuity: expected "
                + std::to_string(*last_captured_frame_id_ + 1U) + ", got "
                + std::to_string(image.frame_id));
            return;
        }

        if (last_camera_timestamp_ && image.device_timestamp > *last_camera_timestamp_) {
            const auto current_period =
                static_cast<double>(image.device_timestamp - *last_camera_timestamp_);
            if (camera_period_estimate_) {
                *camera_period_estimate_ = 0.9 * *camera_period_estimate_ + 0.1 * current_period;
            } else {
                camera_period_estimate_ = current_period;
            }
        }
        last_camera_timestamp_ = image.device_timestamp;
        last_captured_frame_id_ = image.frame_id;

        camera_frames_.push_back(CameraFrame{
            .frame_id = image.frame_id,
            .camera_timestamp = image.device_timestamp,
            .sdk_host_timestamp = image.sdk_host_timestamp,
            .host_timestamp = image.timestamp,
        });
        trim_deque(camera_frames_, kCameraBufferCapacity);

        process_matches();
    }

    auto process_matches() -> void {
        while (state_ == SyncState::kLocked && delta_ && !camera_frames_.empty()) {
            const auto& frame = camera_frames_.front();
            const auto expected_seq_signed =
                static_cast<std::int64_t>(frame.frame_id) - *delta_;
            if (expected_seq_signed <= 0) {
                request_resync(
                    "expected trigger sequence became non-positive for frame "
                    + std::to_string(frame.frame_id));
                return;
            }

            const auto expected_seq = static_cast<std::uint64_t>(expected_seq_signed);
            std::optional<TriggerEvent> trigger_event;
            {
                std::lock_guard lock{trigger_mutex_};
                while (!trigger_events_.empty() && trigger_events_.front().seq < expected_seq)
                    trigger_events_.pop_front();

                if (trigger_events_.empty())
                    return;

                if (trigger_events_.front().seq > expected_seq) {
                    request_resync(
                        "trigger sequence gap: expected " + std::to_string(expected_seq)
                        + ", next available " + std::to_string(trigger_events_.front().seq));
                    return;
                }

                trigger_event = trigger_events_.front();
                trigger_events_.pop_front();
            }

            camera_frames_.pop_front();
            consume_match(frame, *trigger_event);
        }
    }

    auto consume_match(const CameraFrame& frame, const TriggerEvent& trigger) -> void {
        if (last_matched_trigger_seq_ && trigger.seq != (*last_matched_trigger_seq_ + 1U)) {
            request_resync(
                "matched trigger sequence discontinuity: expected "
                + std::to_string(*last_matched_trigger_seq_ + 1U) + ", got "
                + std::to_string(trigger.seq));
            return;
        }

        if (!rls_.initialized()) {
            request_resync("RLS was unexpectedly uninitialized");
            return;
        }

        const auto residual = rls_.residual_for(trigger.timestamp_quarter_us, frame.camera_timestamp);
        const auto absolute_residual = std::abs(residual);
        const auto warmed_up = rls_.sample_count() >= warmup_samples_ && camera_period_estimate_.has_value();

        if (warmed_up) {
            const auto soft_threshold = residual_soft_ratio_ * *camera_period_estimate_;
            const auto hard_threshold = residual_hard_ratio_ * *camera_period_estimate_;

            if (absolute_residual >= hard_threshold) {
                request_resync(
                    "hard residual threshold exceeded: residual=" + std::to_string(absolute_residual)
                    + ", hard_threshold=" + std::to_string(hard_threshold));
                return;
            }

            if (absolute_residual >= soft_threshold) {
                ++consecutive_bad_residuals_;
                if (consecutive_bad_residuals_ >= max_consecutive_bad_residuals_) {
                    request_resync(
                        "soft residual threshold exceeded repeatedly: residual="
                        + std::to_string(absolute_residual) + ", soft_threshold="
                        + std::to_string(soft_threshold));
                    return;
                }
            } else {
                consecutive_bad_residuals_ = 0;
            }
        }

        const auto updated_residual = rls_.update(trigger.timestamp_quarter_us, frame.camera_timestamp);
        (void)updated_residual;
        last_matched_trigger_seq_ = trigger.seq;
        ++matched_pairs_;

        if (residual_abs_ema_) {
            *residual_abs_ema_ = 0.95 * *residual_abs_ema_ + 0.05 * absolute_residual;
            *residual_rms_ema_ = 0.95 * *residual_rms_ema_ + 0.05 * (residual * residual);
        } else {
            residual_abs_ema_ = absolute_residual;
            residual_rms_ema_ = residual * residual;
        }
    }

    auto maybe_report_status() -> void {
        using Clock = std::chrono::steady_clock;

        const auto now = Clock::now();
        const auto elapsed = std::chrono::duration<double>{now - last_report_time_};
        if (elapsed < 1s)
            return;

        const auto fps = static_cast<double>(frames_since_report_) / elapsed.count();
        const auto residual_rms = residual_rms_ema_ ? std::sqrt(*residual_rms_ema_) : 0.0;
        const auto residual_abs = residual_abs_ema_.value_or(0.0);
        const auto delta = delta_.value_or(0);

        std::size_t trigger_queue_size = 0;
        {
            std::lock_guard lock{trigger_mutex_};
            trigger_queue_size = trigger_events_.size();
        }

        RCLCPP_INFO(
            get_logger(),
            "state=%s fps=%.2f delta=%lld a=%.9f b=%.3f residual_abs=%.3f residual_rms=%.3f matched=%zu camera_q=%zu trigger_q=%zu",
            to_string(state_), fps, static_cast<long long>(delta), rls_.a(), rls_.b(), residual_abs,
            residual_rms, matched_pairs_, camera_frames_.size(), trigger_queue_size);

        frames_since_report_ = 0;
        last_report_time_ = now;
    }

    hikcamera::Config camera_config_{};
    hikcamera::Camera camera_{};
    std::unique_ptr<TriggerBoard> board_;

    std::string window_name_;
    std::string board_serial_;
    std::string board_gpio_name_;
    int wait_key_ms_ = 1;
    int sync_pause_ms_ = 100;
    int sync_timeout_ms_ = 1000;

    double rls_lambda_ = 0.9995;
    std::size_t warmup_samples_ = 16;
    std::size_t max_consecutive_bad_residuals_ = 3;
    double residual_soft_ratio_ = 0.45;
    double residual_hard_ratio_ = 0.90;

    std::deque<CameraFrame> camera_frames_;
    mutable std::mutex trigger_mutex_;
    std::condition_variable trigger_cv_;
    std::deque<TriggerEvent> trigger_events_;

    SyncState state_ = SyncState::kUnlocked;
    bool pending_resync_ = false;
    std::string resync_reason_;

    bool have_last_raw_trigger_timestamp_ = false;
    std::uint32_t last_raw_trigger_timestamp_ = 0;
    std::uint64_t trigger_timestamp_wrap_count_ = 0;
    std::uint64_t next_trigger_seq_ = 0;

    std::optional<std::int64_t> delta_;
    std::optional<std::uint32_t> last_captured_frame_id_;
    std::optional<std::uint64_t> last_camera_timestamp_;
    std::optional<std::uint64_t> last_matched_trigger_seq_;
    std::optional<double> camera_period_estimate_;
    std::optional<double> residual_abs_ema_;
    std::optional<double> residual_rms_ema_;

    std::size_t consecutive_bad_residuals_ = 0;
    std::size_t matched_pairs_ = 0;
    std::size_t frames_since_report_ = 0;
    std::chrono::steady_clock::time_point last_report_time_{};
    ForgettingRls rls_{};
};

TriggerBoard::TriggerBoard(
    AutoAimTestNode& node, std::string_view serial_filter,
    const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio)
    : librmcs::agent::RmcsBoardLite{serial_filter}
    , node_{node}
    , gpio_{&gpio} {
    start_transmit().gpio_digital_read(
        *gpio_,
        {
            .period_ms = 0,
            .asap = false,
            .rising_edge = false,
            .falling_edge = true,
            .capture_timestamp = true,
            .pull = librmcs::data::GpioPull::kUp,
        });
}

void TriggerBoard::gpio_digital_read_result_callback(
    const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio,
    const librmcs::data::GpioDigitalDataView& data) {
    if (!(gpio == *gpio_) || !data.timestamp_quarter_us)
        return;

    node_.on_trigger_edge(*data.timestamp_quarter_us);
}

} // namespace

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<AutoAimTestNode>();
        const auto exit_code = node->run();
        rclcpp::shutdown();
        return exit_code;
    } catch (const cv::Exception& exception) {
        RCLCPP_ERROR(
            rclcpp::get_logger("auto_aim_test"), "OpenCV viewer failed: %s", exception.what());
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return 1;
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(
            rclcpp::get_logger("auto_aim_test"), "auto_aim_test failed: %s", exception.what());
        cv::destroyAllWindows();
        rclcpp::shutdown();
        return 1;
    }
}
