#pragma once
#include "static_string.hpp"
#include <algorithm>
#include <tuple>

namespace rmcs_utility::tf::details {

struct MonoState {};

template <typename T>
concept node_trait = requires {
    T::name;
    T::child_amount;
    T::total_amount;
};

struct node_checker_type {
    template <class T>
    struct result {
        constexpr static auto v = node_trait<T>;
    };
};
constexpr node_checker_type node_checker;

template <typename Token, StaticString name, typename T>
struct JointTransfroms {
    static inline T state = T{};
};

} // namespace rmcs_utility::tf::details
namespace rmcs_utility {

template <StaticString Name, class T = tf::details::MonoState>
struct Link {};

template <StaticString name_, class State_ = tf::details::MonoState, class... Ts_>
struct Joint {
    using State = State_;

    constexpr explicit Joint(Link<name_, State_>, Ts_...) noexcept {}

    static constexpr std::size_t child_amount = sizeof...(Ts_);
    static constexpr std::size_t total_amount = 1 + (0 + ... + Ts_::total_amount);

    static constexpr auto name = name_.view();
    static constexpr auto static_name = name_;

    template <typename F>
    static constexpr auto foreach_df(F&& f) noexcept -> void {
        f.template operator()<Joint>();
        [[maybe_unused]] const auto recursion = [&]<class T>() { //
            T::foreach_df(std::forward<F>(f));
        };
        (recursion.template operator()<Ts_>(), ...);
    }

    template <typename F>
    static constexpr auto foreach_df_with_parent(F&& f) noexcept -> void {
        static_assert(
            requires { f.template operator()<Joint>(std::string_view{}); },
            "\n错误的函数类型，它应该形如："
            "\n  []<class Joint>(std::string_view father){ ... }\n");
        f.template operator()<Joint>(std::string_view{});
        [[maybe_unused]] const auto recursion = [&]<class T>() {
            T::template foreach_df_with_parent_impl<static_name>(std::forward<F>(f));
        };
        (recursion.template operator()<Ts_>(), ...);
    }

    template <StaticString query_name>
    static constexpr auto find(auto&& callback) noexcept -> bool {
        if constexpr (query_name == name) {
            static_assert(
                requires { callback.template operator()<Joint>(); },
                "\n错误的函数类型，它应该形如："
                "\n  []<class Joint>(){ ... }\n");
            callback.template operator()<Joint>();
            return true;
        } else {
            [[maybe_unused]] const auto recursion_find = [&]<class T>() {
                return T::template find<query_name>(callback);
            };
            return (false || ... || recursion_find.template operator()<Ts_>());
        }
    }

    template <StaticString query_name>
    static constexpr auto contains() noexcept {
        if constexpr (query_name == name)
            return true;
        return (false || ... || Ts_::template contains<query_name>());
    }

    template <StaticString child>
    static constexpr auto child_distance() noexcept -> std::size_t {
        static_assert(Joint::contains<child>(), "子节点未找到");
        return impl_traversal_child<child>();
    }
    template <StaticString a, StaticString b>
    static constexpr auto distance() noexcept {
        static_assert(a != b, "它们是同一个节点");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch = std::ranges::mismatch(to_a, to_b);
        auto common_len = std::ranges::distance(to_a.begin(), mismatch.in1);

        return to_a.size() + to_b.size() - 2 * common_len;
    }

    template <StaticString parent, StaticString child>
    static constexpr auto child_distance() noexcept -> std::size_t {
        static_assert(Joint::contains<parent>(), "父节点未找到");
        static_assert(Joint::contains<child>(), "子节点未找到");

        auto result = std::size_t{0};
        Joint::find<parent>([&]<class T>() {
            if (result == 0)
                result = T::template child_distance<child>();
        });
        return result;
    }

    template <StaticString child, std::size_t N = child_distance<child>()>
    static constexpr auto child_path(bool traversal_down = false) noexcept {
        static_assert(Joint::contains<child>(), "子节点未找到");

        auto result = std::array<std::string_view, N>{};
        if (N == 0)
            return result;

        auto index = traversal_down ? N - 1 : 0;
        auto step = traversal_down ? -1 : +1;
        impl_traversal_child<child>([&]<class T>() {
            result[index] = T::name;
            index += step;
        });

        result[traversal_down ? N - 1 : 0] = child.view();
        return result;
    }

    template <StaticString parent, StaticString child>
    static constexpr auto child_path(bool traversal_down = false) noexcept {
        static_assert(parent != child, "它们是同一个节点");
        static_assert(Joint::contains<parent>(), "父节点未找到");
        static_assert(Joint::contains<child>(), "子节点未找到");

        constexpr auto n{child_distance<parent, child>()};
        auto result = std::array<std::string_view, n>{};

        find<parent>([&]<class T>() {
            // Specify the result length to let lsp take a rest
            result = T::template child_path<child, n>(traversal_down);
        });
        return result;
    }

    template <StaticString a, StaticString b>
    static constexpr auto find_lca() noexcept {
        static_assert(a != b, "它们是同一个节点");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch = std::ranges::mismatch(to_a, to_b);
        return std::make_tuple(*std::prev(mismatch.in1));
    }

    template <StaticString a, StaticString b>
    static constexpr auto distance_to_lca() noexcept {
        static_assert(a != b, "它们是同一个节点");

        constexpr auto to_a = child_path<a>(true);
        constexpr auto to_b = child_path<b>(true);

        auto mismatch = std::ranges::mismatch(to_a, to_b);
        auto common_len = std::ranges::distance(to_a.begin(), mismatch.in1);

        return std::make_tuple(to_a.size() - common_len, to_b.size() - common_len);
    }

    template <StaticString begin, StaticString final, std::size_t N = distance<begin, final>()>
    static constexpr auto path() noexcept {
        constexpr auto to_begin = child_path<begin>(true);
        constexpr auto to_final = child_path<final>(true);

        auto mismatch = std::ranges::mismatch(to_begin, to_final);
        auto common_len = std::ranges::distance(to_begin.begin(), mismatch.in1);

        auto begin_to_lca = to_begin.size() - common_len;

        auto result = std::array<std::string_view, N>{};
        auto index = 0;
        for (std::size_t i = 0; i < begin_to_lca; ++i) {
            auto side = to_begin.size() - 1;
            result[index++] = to_begin[side - i];
        }
        for (std::size_t i = common_len; i < to_final.size(); ++i) {
            result[index++] = to_final[i];
        }
        return result;
    }

    template <StaticString name, typename T>
    using Transforms = tf::details::JointTransfroms<Joint, name, T>;

    /// Index using type
    template <class T = Joint>
    constexpr static auto get_type_state(auto&& callback) noexcept {
        static_assert(
            requires { callback(Transforms<T::static_name, typename T::State>::state); },
            "错误的回调类型，它应该类似这样: callback(state&) 或者 callback(const state&)");
        callback(Transforms<T::static_name, typename T::State>::state);
    }
    template <class T = Joint, typename return_type>
    constexpr static auto get_type_state() noexcept {
        auto result = return_type{};
        get_type_state<T>([&]<typename S>(const S& state) {
            static_assert(
                std::constructible_from<return_type, S>, "返回值的类型无法以 state 为构造参数构造");
            result = return_type{state};
        });
        return result;
    }
    template <class T = Joint>
    constexpr static auto set_type_state(const auto& _state) noexcept {
        get_type_state<T>([&](auto& state) { state = _state; });
    }

    /// Index using name
    template <StaticString name>
    constexpr static auto get_state(auto&& callback) noexcept {
        static_assert(Joint::contains<name>(), "变换的名称不存在");
        std::ignore = find<name>([&]<class T> { get_type_state<T>(callback); });
    }
    template <StaticString name, typename return_type>
    constexpr static auto get_state() noexcept {
        auto result = return_type{};
        get_state<name>([&]<typename S>(const S& state) {
            static_assert(
                std::constructible_from<return_type, S>, "返回值的类型无法以 state 为构造参数构造");
            result = return_type{state};
        });
        return result;
    }
    template <StaticString name>
    constexpr static auto set_state(const auto& state) noexcept {
        get_state<name>([&](auto& state_) {
            static_assert(
                requires { state_ = state; }, "State 无法被输入值赋值，请检查类型是否正确");
            state_ = state;
        });
    }

    // Parent 无任何作用，只是为了写明关系而存在的接口
    template <StaticString unused_parent, StaticString child>
    constexpr static auto set_state(const auto& state) noexcept {
        static_assert(contains<unused_parent>(), "父变换不存在");
        set_state<child>(state);
    }

    template <StaticString begin, StaticString final, class SE3>
    constexpr static auto look_up() noexcept {
        static_assert(requires { SE3::Identity(); }, "SE3 不是一个标准的变换类型");
        auto lca_to_begin = SE3::Identity();
        auto lca_to_final = SE3::Identity();
        Joint::impl_look_up<begin, final, SE3>([&]([[maybe_unused]] auto, auto se3, bool is_begin) {
            if constexpr (requires { se3 * std::declval<SE3>(); }) {
                if (is_begin == true) {
                    lca_to_begin = lca_to_begin * se3;
                }
                if (is_begin == false) {
                    lca_to_final = lca_to_final * se3;
                }
            }
        });
        return SE3{lca_to_begin.inverse() * lca_to_final};
    }

public:
    template <StaticString parent_name, typename F>
    static constexpr auto foreach_df_with_parent_impl(F&& f) noexcept -> void {
        f.template operator()<Joint>(parent_name.view());
        [[maybe_unused]] const auto recursion = [&]<class T>() {
            T::template foreach_df_with_parent_impl<static_name>(std::forward<F>(f));
        };
        (recursion.template operator()<Ts_>(), ...);
    }

    template <StaticString child>
    static constexpr auto impl_traversal_child(auto&& on_recursion) noexcept -> std::size_t {
        auto result = std::size_t{0};
        auto recursion = [&]<class T>() {
            if (result)
                return;
            /*  */ if (child == T::name) {
                result = 1;       // End Point
                on_recursion.template operator()<T>();
            } else if (auto sub = T::template impl_traversal_child<child>(on_recursion)) {
                result = 1 + sub; // Recursion Back
                on_recursion.template operator()<T>();
            }
        };
        (recursion.template operator()<Ts_>(), ...);
        return result;
    }
    template <StaticString child>
    static constexpr auto impl_traversal_child() noexcept {
        return impl_traversal_child<child>([]<typename T>() {});
    }

    template <StaticString begin, StaticString final, class SE3>
    constexpr static auto impl_look_up(auto&& callback) noexcept
        requires requires { callback(std::string_view{}, SE3{}, bool{}); } {
        auto [begin_len, final_len] = distance_to_lca<begin, final>();
        // calculate tf from begin to lca
        impl_traversal_child<begin>([&]<class T>() {
            using State = typename T::State;
            if (begin_len-- > 0) {
                callback(T::name, Transforms<T::static_name, State>::state, true);
            }
        });
        // calculate tf from final to lca>
        impl_traversal_child<final>([&]<class T>() {
            using State = typename T::State;
            if (final_len-- > 0) {
                callback(T::name, Transforms<T::static_name, State>::state, false);
            }
        });
    }
};

} // namespace rmcs_utility

template <typename T>
constexpr auto operator*(const rmcs_utility::tf::details::MonoState&, const T& other) noexcept
    -> decltype(auto) {
    return other;
}

template <typename T>
constexpr auto operator*(const T& other, const rmcs_utility::tf::details::MonoState&) noexcept
    -> decltype(auto) {
    return other;
}
