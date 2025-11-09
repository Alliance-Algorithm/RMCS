#pragma once
#include <concepts>
#include <utility>

namespace rmcs::pipe {

template <class T>
concept pipeline_trait = requires {
    typename T::I;
    typename T::O;
    { std::declval<T>()(std::declval<typename T::I>()) } -> std::same_as<typename T::O>;
};

/// @test
/// You can find the result on godbolt:
/// https://godbolt.org/z/T6cPenaaK
///
template <pipeline_trait Impl>
struct Pipe : public Impl {
    using I = Impl::I;
    using O = Impl::O;

    template <typename... Args>
    constexpr explicit Pipe(Args&&... args) noexcept
        : Impl { std::forward<Args>(args)... } { }

    template <class Next>
        requires std::same_as<O, typename Next::I>
    constexpr auto operator|(const Pipe<Next>& next) const noexcept {
        struct Return {
            using I = Impl::I;
            using O = Next::O;

            Pipe<Impl> last;
            Pipe<Next> next;

            constexpr auto operator()(I i) const -> O { return next(last(i)); }
        };
        return Pipe<Return> { *this, next };
    }

    constexpr auto operator()(I i) const -> O { return Impl::operator()(i); }
};

}
