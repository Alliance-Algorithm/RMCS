#pragma once
#include <hikcamera/capturer.hpp>

#include "utility/image.hpp"
#include "utility/serializable.hpp"

namespace rmcs::cap::hik {

using NormalResult = std::expected<void, std::string>;
using ImageResult  = std::expected<std::unique_ptr<Image>, std::string>;

struct Hikcamera : public hikcamera::Camera {
    using Camera::Camera;

    struct Config : hikcamera::Config, util::Serializable {
        static constexpr std::tuple metas {
            &Config::timeout_ms,
            "timeout_ms",
            &Config::exposure_us,
            "exposure_us",
            &Config::framerate,
            "framerate",
            &Config::gain,
            "gain",
            &Config::invert_image,
            "invert_image",
            &Config::software_sync,
            "software_sync",
            &Config::trigger_mode,
            "trigger_mode",
            &Config::fixed_framerate,
            "fixed_framerate",
        };
    };

    auto wait_image() noexcept -> ImageResult;

    static constexpr auto get_prefix() noexcept { return "hikcamera"; }
};

}
