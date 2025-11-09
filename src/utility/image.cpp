#include "image.details.hpp"

using namespace rmcs;

struct Image::Impl {
    Stamp timestamp;
    Details details;
};

auto Image::details() noexcept -> Details& { return pimpl->details; }
auto Image::details() const noexcept -> const Details& { return pimpl->details; }

auto Image::get_timestamp() const noexcept -> Stamp //
{
    return pimpl->timestamp;
}
auto Image::set_timestamp(Stamp timestamp) noexcept -> void //
{
    pimpl->timestamp = timestamp;
}

Image::Image() noexcept
    : pimpl { std::make_unique<Impl>() } { }

Image::~Image() noexcept = default;
