#pragma once

#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <tuple>
#include <utility>

#include <MvCameraControl.h>

class Hikcamera final {
public:
    static constexpr float Cs016MaxGain = 16.9807F;
    static constexpr float Cs016MaxFramerate = 249.1F;
    static constexpr std::uint32_t Cs016FrameWidth = 1440;
    static constexpr std::uint32_t Cs016FrameHeight = 1080;

    struct CameraConfig final {
        float exposure_us = 2000.F;
        float gain = 0.0F;
        float framerate = 10.0F;

        bool invert_image = false;

        static constexpr CameraConfig cs016_default() {
            return {
                .gain = Cs016MaxGain,
                .framerate = Cs016MaxFramerate,
            };
        }
    };

    struct HikDeviceClock {
        // 10ns / tick
        using duration = std::chrono::duration<std::int64_t, std::ratio<1, 100'000'000>>;
        using rep = duration::rep;
        using period = duration::period;
        using time_point = std::chrono::time_point<HikDeviceClock>;

        [[maybe_unused]] static constexpr bool is_steady = true;
    };

    struct Frame {
        std::span<const std::byte> data;
        std::uint32_t width;
        std::uint32_t height;
        MvGvspPixelType pixel_type;
        std::uint32_t frame_id;
        HikDeviceClock::time_point timestamp;
    };

    template <typename CallbackT>
    Hikcamera(
        const CameraConfig& config, CallbackT callback, const std::string_view device_name = {})
        requires(
            std::is_nothrow_invocable_v<CallbackT, const Frame&>
            && std::is_trivially_copyable_v<CallbackT>
            && std::is_trivially_destructible_v<CallbackT>
            && sizeof(CallbackT) <= sizeof(std::uintptr_t)) {

        auto* device = select_device(device_name);
        check_hik("create camera handle", MV_CC_CreateHandleWithoutLog(&handle_, device));

        auto sdk_callback = [](unsigned char* data, MV_FRAME_OUT_INFO_EX* frame_info,
                               void* user_data) noexcept {
            if (data == nullptr || frame_info == nullptr)
                return;

            const auto device_timestamp =
                (static_cast<std::uint64_t>(frame_info->nDevTimeStampHigh) << 32U)
                | static_cast<std::uint64_t>(frame_info->nDevTimeStampLow);

            auto frame = Frame{
                .data = {reinterpret_cast<const std::byte*>(data), frame_info->nFrameLen},
                .width = frame_info->nWidth,
                .height = frame_info->nHeight,
                .pixel_type = frame_info->enPixelType,
                .frame_id = frame_info->nFrameNum,
                .timestamp = HikDeviceClock::time_point{HikDeviceClock::duration{
                    static_cast<int64_t>(device_timestamp)}},
            };
            auto callback = std::bit_cast<CallbackT>(user_data);
            callback(static_cast<const Frame&>(frame));
        };

        try {
            open_and_configure(*device, config, sdk_callback, std::bit_cast<void*>(callback));
        } catch (...) {
            cleanup();
            throw;
        }
    }

    [[nodiscard]] auto set_soft_trigger(bool enabled) noexcept -> int {
        return MV_CC_SetEnumValue(
            handle_, "TriggerMode", enabled ? MV_TRIGGER_MODE_ON : MV_TRIGGER_MODE_OFF);
    }

    [[nodiscard]] auto trigger_soft() noexcept -> int {
        return MV_CC_SetCommandValue(handle_, "TriggerSoftware");
    }

    [[nodiscard]] auto take_transport_fault_message() noexcept -> std::optional<unsigned int> {
        if (!transport_fault_.exchange(false, std::memory_order::acq_rel))
            return std::nullopt;

        return last_transport_fault_message_.load(std::memory_order::acquire);
    }

    ~Hikcamera() noexcept { cleanup(); }

    Hikcamera(const Hikcamera&) = delete;
    Hikcamera& operator=(const Hikcamera&) = delete;

private:
    static void __stdcall transport_exception_callback(
        const unsigned int message_type, void* user_data) noexcept {
        if (user_data == nullptr)
            return;

        auto* self = static_cast<Hikcamera*>(user_data);
        self->last_transport_fault_message_.store(message_type, std::memory_order::release);
        self->transport_fault_.store(true, std::memory_order::release);
    }

    [[nodiscard]] static MV_CC_DEVICE_INFO* select_device(const std::string_view name) {
        MV_CC_DEVICE_INFO_LIST devices{};
        std::memset(&devices, 0, sizeof(devices));

        check_hik(
            "enumerate camera devices",
            MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devices));

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

        throw std::runtime_error{
            "No Hik camera matched user-defined name '" + std::string{name} + "'"};
    }

    [[nodiscard]] static bool
        device_name_matches(const MV_CC_DEVICE_INFO& info, const std::string_view name) {
        const unsigned char* raw_name = nullptr;

        switch (info.nTLayerType) {
        case MV_GIGE_DEVICE: raw_name = info.SpecialInfo.stGigEInfo.chUserDefinedName; break;
        case MV_USB_DEVICE: raw_name = info.SpecialInfo.stUsb3VInfo.chUserDefinedName; break;
        default: return false;
        }

        const auto* device_name = reinterpret_cast<const char*>(raw_name);
        return device_name != nullptr && name == device_name;
    }

    void open_and_configure(
        const MV_CC_DEVICE_INFO& device, const CameraConfig& config, MvImageCallbackEx callback,
        void* user_data) {
        check_hik("open camera device", MV_CC_OpenDevice(handle_));

        if (device.nTLayerType == MV_GIGE_DEVICE) {
            const auto packet_size = MV_CC_GetOptimalPacketSize(handle_);
            if (packet_size <= 0) {
                throw std::runtime_error{
                    "GetOptimalPacketSize failed with Hik SDK error code "
                    + std::to_string(packet_size)};
            }
            check_hik(
                "set optimal packet size",
                MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", packet_size));
        }

        check_hik(
            "disable auto exposure",
            MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF));
        check_hik(
            "set exposure time", MV_CC_SetFloatValue(handle_, "ExposureTime", config.exposure_us));
        check_hik("set gain", MV_CC_SetFloatValue(handle_, "Gain", config.gain));
        check_hik("set adc bit depth", MV_CC_SetEnumValue(handle_, "ADCBitDepth", 0)); // 8-Bit

        check_hik(
            "set frame rate enable",
            MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true));
        check_hik(
            "set frame rate",
            MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", config.framerate));

        check_hik("set reverse x", MV_CC_SetBoolValue(handle_, "ReverseX", config.invert_image));
        check_hik("set reverse y", MV_CC_SetBoolValue(handle_, "ReverseY", config.invert_image));

        check_hik(
            "set trigger mode", MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_ON));
        check_hik(
            "set trigger source",
            MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE));

        check_hik("set offset x", MV_CC_SetIntValueEx(handle_, "OffsetX", 0));
        check_hik("set offset y", MV_CC_SetIntValueEx(handle_, "OffsetY", 0));
        MVCC_INTVALUE_EX value;
        check_hik("get frame width max", MV_CC_GetIntValueEx(handle_, "WidthMax", &value));
        check_hik("set frame width", MV_CC_SetIntValueEx(handle_, "Width", value.nCurValue));
        check_hik("get frame height max", MV_CC_GetIntValueEx(handle_, "HeightMax", &value));
        check_hik("set frame height", MV_CC_SetIntValueEx(handle_, "Height", value.nCurValue));

        check_hik(
            "set strobe line",
            MV_CC_SetEnumValueByString(handle_, "LineSelector", "Line1"));
        check_hik(
            "set strobe source",
            MV_CC_SetEnumValueByString(handle_, "LineSource", "ExposureStartActive"));
        check_hik("set strobe enable", MV_CC_SetBoolValue(handle_, "StrobeEnable", true));

        check_hik(
            "register exception callback",
            MV_CC_RegisterExceptionCallBack(handle_, transport_exception_callback, this));
        check_hik(
            "register image callback", MV_CC_RegisterImageCallBackEx(handle_, callback, user_data));
        check_hik("start grabbing", MV_CC_StartGrabbing(handle_));
    }

    static void check_hik(const std::string_view action, const int code) {
        if (code != MV_OK)
            throw std::runtime_error{
                std::format("{} failed with Hik SDK error code {}", action, code)};
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
    std::atomic<bool> transport_fault_ = false;
    std::atomic<unsigned int> last_transport_fault_message_ = 0;
};
