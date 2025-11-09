#include "identifier.hpp"
#include "utility/robot/armor.hpp"

using namespace rmcs::kernel;

struct Identifier::Impl {
    using ImageUnique = std::unique_ptr<Image>;

    struct Detector {
        virtual auto sync_detect(const Image&) const noexcept -> Armors {
            //
            return Armors {};
        }
        virtual ~Detector() = default;
    };
    std::unique_ptr<Detector> detector;

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string> {
        std::ignore = detector;
        return std::unexpected { "Not implemented" };
    }

    auto identify(const Image&) noexcept {
        // ...
    }
};

auto Identifier::initialize(const YAML::Node& yaml) noexcept -> std::expected<void, std::string> {
    return pimpl->initialize(yaml);
}

auto Identifier::sync_identify(const Image&) noexcept -> void { }

Identifier::Identifier() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Identifier::~Identifier() noexcept = default;
