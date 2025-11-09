#pragma once
#include <coroutine>
#include <expected>
#include <string>

namespace rmcs::co {

/// @note:
///  - No raii for coroutine manager, destroy manually
///  - Launch immediately
///  - Destroy lazy
template <typename result_type>
struct task {

    auto done() const noexcept { return coroutine_handle.done(); }

    auto destroy() noexcept {
        if (coroutine_handle) {
            coroutine_handle.destroy();
        }
    }

    auto result() const noexcept -> const result_type& {
        //
        return coroutine_handle.promise().result;
    }

    struct promise_type {
        result_type result;

        auto get_return_object() noexcept {
            return task { std::coroutine_handle<promise_type>::from_promise(*this) };
        }
        auto return_value(result_type _result) noexcept {
            //
            result = std::move(_result);
        }

        auto unhandled_exception() noexcept -> void {
            using error_type = std::unexpected<std::string>;
            if constexpr (std::convertible_to<error_type, result_type>) {
                try {
                    throw;
                } catch (const std::exception& e) {
                    result = std::unexpected { e.what() };
                } catch (...) {
                    result = std::unexpected { "Unknown exception" };
                }
            }
        }

        static constexpr auto initial_suspend() noexcept {
            // Launch immediately
            return std::suspend_never {};
        }
        static constexpr auto final_suspend() noexcept {
            // Lazy destory
            return std::suspend_always {};
        }
    };
    std::coroutine_handle<promise_type> coroutine_handle;
};

}
