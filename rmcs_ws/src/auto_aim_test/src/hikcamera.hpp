#pragma once

#include <any>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <MvCameraControl.h>
#include <rmcs_utility/memory_pool.hpp>

class Hikcamera final {
public:
    static constexpr float Cs016MaxGain = 16.9807F;
    static constexpr float Cs016MaxFramerate = 249.1F;
    static constexpr std::uint32_t Cs016FrameWidth = 1440;
    static constexpr std::uint32_t Cs016FrameHeight = 1080;

    struct CameraConfig final {
        unsigned int timeout_ms = 2000;

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

    template <std::size_t frame_size>
    struct Frame final {
        static constexpr std::size_t kFrameSize = frame_size;

        alignas(std::uintptr_t) std::byte data[frame_size];
        std::uint32_t width;
        std::uint32_t height;
        MvGvspPixelType pixel_type;
        std::uint32_t frame_id;
        std::uint64_t device_timestamp;
        std::int64_t host_timestamp;
        std::chrono::steady_clock::time_point timestamp;
    };

    template <std::size_t frame_size>
    using MemoryPool = rmcs_utility::MemoryPool<Frame<frame_size>>;

    template <std::size_t frame_size>
    using FrameUniquePtr = MemoryPool<frame_size>::UniquePtr;

    template <std::size_t frame_size, typename CallbackT>
    requires requires(CallbackT&& f, FrameUniquePtr<frame_size>&& frame) {
        { f(std::move(frame)) } noexcept;
    }
    Hikcamera(
        const CameraConfig& config, MemoryPool<frame_size>& frame_memory_pool, CallbackT&& callback,
        const std::string_view device_name = {}) {

        auto* device = select_device(device_name);
        check_hik("create camera handle", MV_CC_CreateHandleWithoutLog(&handle_, device));

        struct UserContext {
            MemoryPool<frame_size>& frame_memory_pool;
            CallbackT callback;
        };
        user_context_ = UserContext{
            .frame_memory_pool = frame_memory_pool, .callback = std::forward<CallbackT>(callback)};

        auto image_callback = [](unsigned char* data, MV_FRAME_OUT_INFO_EX* frame_info,
                                 void* user) noexcept {
            auto* self = static_cast<Hikcamera*>(user);
            if (self == nullptr || data == nullptr || frame_info == nullptr)
                return;
            if (frame_info->nFrameLen != frame_size)
                return;

            UserContext& context = std::any_cast<UserContext&>(self->user_context_);
            auto frame = context.frame_memory_pool.allocate_unique();
            if (!frame)
                return;
            std::memcpy(frame->data, data, frame_size);

            const auto device_timestamp =
                (static_cast<std::uint64_t>(frame_info->nDevTimeStampHigh) << 32U)
                | static_cast<std::uint64_t>(frame_info->nDevTimeStampLow);

            frame->width = frame_info->nWidth;
            frame->height = frame_info->nHeight, frame->pixel_type = frame_info->enPixelType;
            frame->frame_id = frame_info->nFrameNum;
            frame->device_timestamp = device_timestamp;
            frame->host_timestamp = frame_info->nHostTimeStamp;
            frame->timestamp = std::chrono::steady_clock::now();

            context.callback(std::move(frame));
        };

        try {
            open_and_configure(*device, config, image_callback);
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~Hikcamera() noexcept { cleanup(); }

    Hikcamera(const Hikcamera&) = delete;
    Hikcamera& operator=(const Hikcamera&) = delete;

private:
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
        const MV_CC_DEVICE_INFO& device, const CameraConfig& config, MvImageCallbackEx callback) {
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
            "register image callback", MV_CC_RegisterImageCallBackEx(handle_, callback, this));
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

    std::any user_context_;
};