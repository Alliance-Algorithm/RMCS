#pragma once

#include <utility>

namespace librmcs::host::utility {

template <typename Functor>
requires requires(Functor& action) {
    { action() } noexcept;
} struct FinalAction {
    constexpr explicit FinalAction(Functor action)
        : action_{std::move(action)} {}

    constexpr FinalAction(const FinalAction&) = delete;
    constexpr FinalAction& operator=(const FinalAction&) = delete;
    constexpr FinalAction(FinalAction&&) = delete;
    constexpr FinalAction& operator=(FinalAction&&) = delete;

    ~FinalAction() noexcept {
        if (enabled_) {
            action_();
        }
    }

    void disable() { enabled_ = false; }

private:
    bool enabled_{true};
    Functor action_;
};

} // namespace librmcs::host::utility
