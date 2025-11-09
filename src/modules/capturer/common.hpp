#pragma once
#include "utility/image.hpp"

#include <expected>
#include <yaml-cpp/yaml.h>

namespace rmcs::cap {

using NormalResult = std::expected<void, std::string>;
using ImageResult  = std::expected<std::unique_ptr<Image>, std::string>;

using Yaml = YAML::Node;

struct Interface {
    virtual auto wait_image() noexcept -> ImageResult {
        return std::unexpected { "Unimplement interface: 'wait'" };
    }
    virtual auto connect() noexcept -> NormalResult {
        return std::unexpected { "Unimplement interface: 'connect'" };
    }
    virtual auto connected() const noexcept -> bool { return false; }

    virtual ~Interface() noexcept = default;
};

template <class Impl>
struct Adapter : public Impl, Interface {
public:
    using Impl::Impl;
    using Config = Impl::Config;

    auto configure_yaml(const Yaml& yaml) noexcept -> NormalResult {
        auto config = Config {};
        auto result = config.serialize(yaml);

        if (result.has_value()) {
            Impl::configure(config);
            return {};
        } else {
            return std::unexpected { result.error() };
        }
    }

    auto wait_image() noexcept -> ImageResult override { return Impl::wait_image(); }

    auto connect() noexcept -> NormalResult override { return Impl::connect(); }

    auto connected() const noexcept -> bool override { return Impl::connected(); }
};

}
