#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <MvCameraControl.h>
#include <librmcs/agent/rmcs_board_lite.hpp>
#include <librmcs/data/datas.hpp>
#include <librmcs/spec/rmcs_board_lite/gpio.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr auto kMaxGain = 16.9807F;

struct CameraConfig final {
    unsigned int timeout_ms = 2000;
    float exposure_us = 2000.F;
    float framerate = 80.F;
    float gain = kMaxGain;
    bool invert_image = false;
    bool software_sync = false;
    bool trigger_mode = false;
    bool fixed_framerate = true;
};

[[nodiscard]] auto make_error_message(const std::string_view action, const int code)
    -> std::string {
    return std::string{action} + " failed with MVSDK error code " + std::to_string(code);
}

auto check_mv(const std::string_view action, const int code) -> void {
    if (code != MV_OK)
        throw std::runtime_error{make_error_message(action, code)};
}

[[nodiscard]] auto device_name_matches(const MV_CC_DEVICE_INFO& info, const std::string_view name)
    -> bool {
    const unsigned char* raw_name = nullptr;

    switch (info.nTLayerType) {
    case MV_GIGE_DEVICE: raw_name = info.SpecialInfo.stGigEInfo.chUserDefinedName; break;
    case MV_USB_DEVICE: raw_name = info.SpecialInfo.stUsb3VInfo.chUserDefinedName; break;
    default: return false;
    }

    const auto* device_name = reinterpret_cast<const char*>(raw_name);
    return device_name != nullptr && name == device_name;
}

[[nodiscard]] auto select_device(const std::string_view name) -> MV_CC_DEVICE_INFO* {
    MV_CC_DEVICE_INFO_LIST devices{};
    std::memset(&devices, 0, sizeof(devices));

    check_mv(
        "enumerate camera devices", MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices));

    if (devices.nDeviceNum == 0)
        throw std::runtime_error{"No Hik camera device was found"};

    if (name.empty()) {
        if (devices.nDeviceNum != 1 || devices.pDeviceInfo[0] == nullptr) {
            throw std::runtime_error{
                "Expected exactly one Hik camera device, found "
                + std::to_string(devices.nDeviceNum)};
        }
        return devices.pDeviceInfo[0];
    }

    for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
        auto* device = devices.pDeviceInfo[index];
        if (device != nullptr && device_name_matches(*device, name))
            return device;
    }

    throw std::runtime_error{"No Hik camera matched user-defined name '" + std::string{name} + "'"};
}

[[nodiscard]] auto select_gpio_descriptor(const std::string_view name)
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

class Hikcamera final {
public:
    struct Frame final {
        std::unique_ptr<std::uint8_t[]> data;
        std::size_t size = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        MvGvspPixelType pixel_type{};
        std::uint32_t frame_id = 0;
        std::uint64_t device_timestamp = 0;
        std::int64_t host_timestamp = 0;
        std::chrono::steady_clock::time_point timestamp{};
    };

    using FrameCallback = std::function<void(Frame)>;

    Hikcamera(
        const CameraConfig& config, const std::string_view device_name, FrameCallback callback)
        : callback_(std::move(callback)) {
        if (!callback_)
            throw std::invalid_argument{"Frame callback must not be empty"};

        auto* device = select_device(device_name);
        check_mv("create camera handle", MV_CC_CreateHandleWithoutLog(&handle_, device));

        try {
            open_and_configure(*device, config);
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~Hikcamera() noexcept { cleanup(); }

    Hikcamera(const Hikcamera&) = delete;
    Hikcamera& operator=(const Hikcamera&) = delete;

    auto rethrow_pending_exception() -> void {
        auto pending = std::exception_ptr{};
        {
            std::lock_guard lock{exception_mutex_};
            pending = std::exchange(pending_exception_, nullptr);
        }

        if (pending)
            std::rethrow_exception(pending);
    }

private:
    static void __stdcall
        image_callback(unsigned char* data, MV_FRAME_OUT_INFO_EX* frame_info, void* user) {
        auto* self = static_cast<Hikcamera*>(user);
        if (self == nullptr || data == nullptr || frame_info == nullptr)
            return;

        try {
            self->on_frame(data, *frame_info);
        } catch (...) {
            self->store_exception(std::current_exception());
        }
    }

    auto open_and_configure(const MV_CC_DEVICE_INFO& device, const CameraConfig& config) -> void {
        check_mv("open camera device", MV_CC_OpenDevice(handle_));

        if (device.nTLayerType == MV_GIGE_DEVICE) {
            const auto packet_size = MV_CC_GetOptimalPacketSize(handle_);
            if (packet_size <= 0) {
                throw std::runtime_error{
                    "GetOptimalPacketSize failed with MVSDK error code "
                    + std::to_string(packet_size)};
            }
            check_mv(
                "set optimal packet size",
                MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size));
        }

        check_mv(
            "disable auto exposure",
            MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
        check_mv("set reverse x", MV_CC_SetBoolValue(handle_, "ReverseX", config.invert_image));
        check_mv("set reverse y", MV_CC_SetBoolValue(handle_, "ReverseY", config.invert_image));
        check_mv(
            "set exposure time", MV_CC_SetFloatValue(handle_, "ExposureTime", config.exposure_us));
        check_mv("set gain", MV_CC_SetFloatValue(handle_, "Gain", config.gain));

        const auto trigger_enabled = config.trigger_mode || config.software_sync;
        check_mv(
            "set trigger mode", MV_CC_SetEnumValue(
                                    handle_, "TriggerMode",
                                    trigger_enabled ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF));

        if (config.software_sync) {
            check_mv(
                "set trigger source",
                MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE));
        }

        check_mv(
            "set frame rate enable",
            MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", config.fixed_framerate));

        if (config.fixed_framerate) {
            check_mv(
                "set frame rate",
                MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", config.framerate));
        }

        check_mv(
            "register image callback",
            MV_CC_RegisterImageCallBackEx(handle_, &Hikcamera::image_callback, this));
        check_mv("start grabbing", MV_CC_StartGrabbing(handle_));
    }

    auto on_frame(const unsigned char* data, const MV_FRAME_OUT_INFO_EX& frame_info) -> void {
        const auto size = static_cast<std::size_t>(frame_info.nFrameLen);
        auto buffer = std::make_unique<std::uint8_t[]>(size);
        std::memcpy(buffer.get(), data, size);

        const auto device_timestamp =
            (static_cast<std::uint64_t>(frame_info.nDevTimeStampHigh) << 32U)
            | static_cast<std::uint64_t>(frame_info.nDevTimeStampLow);

        callback_(
            Frame{
                .data = std::move(buffer),
                .size = size,
                .width = frame_info.nWidth,
                .height = frame_info.nHeight,
                .pixel_type = frame_info.enPixelType,
                .frame_id = frame_info.nFrameNum,
                .device_timestamp = device_timestamp,
                .host_timestamp = frame_info.nHostTimeStamp,
                .timestamp = std::chrono::steady_clock::now(),
            });
    }

    auto store_exception(std::exception_ptr pending) -> void {
        if (!pending)
            return;

        std::lock_guard lock{exception_mutex_};
        if (!pending_exception_)
            pending_exception_ = std::move(pending);
    }

    auto cleanup() noexcept -> void {
        if (handle_ == nullptr)
            return;

        std::ignore = MV_CC_StopGrabbing(handle_);
        std::ignore = MV_CC_CloseDevice(handle_);
        std::ignore = MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    void* handle_ = nullptr;
    FrameCallback callback_;
    std::mutex exception_mutex_;
    std::exception_ptr pending_exception_;
};

class AutoAimTestApp;

class TriggerBoard final : private librmcs::agent::RmcsBoardLite {
public:
    TriggerBoard(
        AutoAimTestApp& app, const std::string_view serial_filter,
        const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio);

private:
    void gpio_digital_read_result_callback(
        const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio,
        const librmcs::data::GpioDigitalDataView& data) override;

    AutoAimTestApp& app_;
    const librmcs::spec::rmcs_board_lite::GpioDescriptor* gpio_ = nullptr;
};

class AutoAimTestApp final {
public:
    AutoAimTestApp()
        : node_{std::make_shared<rclcpp::Node>("auto_aim_test")} {
        const auto timeout_ms =
            std::max(std::int64_t{1}, node_->declare_parameter<std::int64_t>("timeout_ms", 2000));
        camera_config_.timeout_ms = static_cast<unsigned int>(timeout_ms);
        camera_config_.exposure_us =
            static_cast<float>(node_->declare_parameter<double>("exposure_us", 2000.0));
        camera_config_.framerate =
            static_cast<float>(node_->declare_parameter<double>("framerate", 249.1));
        camera_config_.gain =
            static_cast<float>(node_->declare_parameter<double>("gain", kMaxGain));
        camera_config_.invert_image = node_->declare_parameter<bool>("invert_image", false);
        camera_config_.software_sync = node_->declare_parameter<bool>("software_sync", false);
        camera_config_.trigger_mode = node_->declare_parameter<bool>("trigger_mode", false);
        camera_config_.fixed_framerate = node_->declare_parameter<bool>("fixed_framerate", true);

        camera_name_ = node_->declare_parameter<std::string>("camera_name", "");
        board_serial_ = node_->declare_parameter<std::string>("board_serial", "");
        board_gpio_name_ = node_->declare_parameter<std::string>("board_gpio", "uart1_tx");

        camera_ = std::make_unique<Hikcamera>(
            camera_config_, camera_name_,
            [this](Hikcamera::Frame frame) { on_frame(std::move(frame)); });
        board_ = std::make_unique<TriggerBoard>(
            *this, board_serial_, select_gpio_descriptor(board_gpio_name_));

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
    auto on_frame(Hikcamera::Frame frame) -> void {
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
    CameraConfig camera_config_;
    std::string camera_name_;
    std::string board_serial_;
    std::string board_gpio_name_;
    std::unique_ptr<Hikcamera> camera_;
    std::unique_ptr<TriggerBoard> board_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    std::atomic<std::uint64_t> frame_count_ = 0;
    std::atomic<std::uint64_t> trigger_count_ = 0;
    std::atomic<std::uint32_t> last_frame_id_ = 0;

    friend class TriggerBoard;
};

TriggerBoard::TriggerBoard(
    AutoAimTestApp& app, const std::string_view serial_filter,
    const librmcs::spec::rmcs_board_lite::GpioDescriptor& gpio)
    : librmcs::agent::RmcsBoardLite{serial_filter}
    , app_{app}
    , gpio_{&gpio} {
    start_transmit().gpio_digital_read(
        *gpio_, {
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

    app_.on_trigger_edge(*data.timestamp_quarter_us);
}

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
