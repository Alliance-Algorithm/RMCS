#pragma once
#include <cstddef>

namespace rmcs::util {

struct TimesLimit {

    std::size_t count = 0;
    std::size_t limit;

    bool stop = false;

    explicit TimesLimit(std::size_t limit) noexcept
        : limit { limit } { }

    auto tick() noexcept -> bool {
        if (stop) return false;

        if (count++ < limit) {
            return true;
        } else {
            return false;
        }
    }

    auto enabled() const noexcept { return !stop; }

    auto enable() noexcept { stop = false; }

    auto disable() noexcept { stop = true; }

    auto reset() noexcept { count = 0; }
};

}
