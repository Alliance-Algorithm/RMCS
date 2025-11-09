#pragma once
#include <concepts>
#include <tuple>

namespace rmcs {

template <class T>
concept is_duck_checker = requires {
    { T::template result<void>::v };
};

template <typename... Ts>
class duck_array {
public:
    using type_list = std::tuple<Ts...>;

    template <std::size_t index>
    using element_type = std::tuple_element_t<index, std::tuple<Ts...>>;

    template <is_duck_checker checker>
    explicit duck_array(checker, Ts&&... elements) noexcept
        requires(checker::template result<Ts>::v && ...)
        : elements_{std::forward<Ts>(elements)...} {}

    [[nodiscard]] static constexpr auto size() noexcept { return sizeof...(Ts); }

    template <typename T>
    [[nodiscard]] static constexpr auto contains() noexcept -> bool {
        return (std::same_as<T, Ts> || ...);
    }

    template <std::size_t index>
    [[nodiscard]] auto at(this auto&& self) {
        return std::get<index>(self.elements_);
    }

    [[deprecated("Prefer at<index>() for compile-time access")]]
    auto at(this auto&& self, std::size_t index, auto&& f) -> void
        requires(std::invocable<decltype(f), Ts&> && ...) {
        self.impl_at(std::forward<decltype(f)>(f), index, std::index_sequence_for<Ts...>{});
    }

    auto foreach (this auto&& self, auto&& f) -> void
        requires(std::invocable<decltype(f), Ts&> && ...) {
        std::apply([=](auto&... args) { (f(args), ...); }, self.elements_);
    }

private:
    std::tuple<Ts...> elements_;

    template <std::size_t... Is>
    void impl_at(this auto&& self, auto&& f, std::size_t index, std::index_sequence<Is...>) {
        ((index == Is ? (void)f(std::get<Is>(self.elements_)) : void()), ...);
    }
};

} // namespace rmcs
