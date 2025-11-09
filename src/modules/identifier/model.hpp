#pragma once
#include "utility/image.hpp"
#include "utility/model/armor_detection.hpp"
#include "utility/pimpl.hpp"

#include <coroutine>
#include <expected>
#include <functional>

#include <opencv2/core/types.hpp>
#include <yaml-cpp/yaml.h>

namespace rmcs::identifier {

struct OpenVinoNet final {
    RMCS_PIMPL_DEFINITION(OpenVinoNet)

public:
    using Result = std::expected<std::vector<ArmorDetection<>>, std::string>;

    using Callback = std::function<void(Result)>;
    auto async_infer(const Image&, Callback) noexcept -> void;

    struct AsyncResult final {
        using handle_type = std::coroutine_handle<>;

        OpenVinoNet& network;
        const Image& readonly;

        Result result { std::unexpected { "Nothing here" } };

        auto await_resume() noexcept { return std::move(result); }

        auto await_suspend(handle_type coroutine) noexcept {
            network.async_infer(readonly, //
                [=, this](auto result) {
                    this->result = std::move(result);
                    coroutine.resume();
                });
        }

        static constexpr auto await_ready() noexcept { return false; }
    };
    auto await_infer(const Image& image) noexcept -> AsyncResult {
        return AsyncResult {
            .network  = *this,
            .readonly = image,
        };
    }

    auto sync_infer(const Image&) const noexcept -> Result;

    auto configure(const YAML::Node&) noexcept -> std::expected<void, std::string>;
};

}
