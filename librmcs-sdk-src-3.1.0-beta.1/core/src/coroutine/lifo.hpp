#pragma once

#include <coroutine>
#include <cstddef>
#include <new>
#include <span>
#include <type_traits>
#include <utility>

#include "core/src/utility/assert.hpp"
#include "core/src/utility/stack_allocator.hpp"

namespace librmcs::core::coroutine {

class LifoContext {
public:
    constexpr explicit LifoContext(std::span<std::byte> buffer) noexcept
        : stack_allocator_(buffer) {}

    constexpr void* allocate(std::size_t n) noexcept { return stack_allocator_.allocate(n); }

    constexpr void deallocate(void* p, std::size_t n) noexcept {
        stack_allocator_.deallocate(p, n);
    }

    constexpr bool resume() noexcept {
        if (!waiting_coroutine_ || waiting_coroutine_.done())
            return false;

        auto h = waiting_coroutine_;
        waiting_coroutine_ = nullptr;
        h.resume();

        return true;
    }

    struct Awaiter {
        constexpr explicit Awaiter(LifoContext& ctx) noexcept
            : context(ctx) {}

        static constexpr bool await_ready() noexcept { return false; }

        constexpr void await_suspend(std::coroutine_handle<> h) noexcept {
            context.waiting_coroutine_ = h;
        }

        static constexpr void await_resume() noexcept {}

        LifoContext& context;
    };
    constexpr Awaiter suspend() noexcept { return Awaiter{*this}; }

private:
    utility::StackAllocator stack_allocator_;

    std::coroutine_handle<> waiting_coroutine_{nullptr};
};

template <size_t stack_size>
class InlineLifoContext : public LifoContext {
public:
    constexpr InlineLifoContext() noexcept
        : LifoContext(stack_buffer_) {}

    constexpr LifoContext& context() { return static_cast<LifoContext&>(*this); }

private:
    alignas(std::max_align_t) std::byte stack_buffer_[stack_size]{};
};

template <LifoContext* static_context = nullptr>
struct LifoStackedPromise {
    template <typename... Args>
    requires(static_context == nullptr)
    static void* operator new(std::size_t n, LifoContext& context, const Args&...) noexcept {
        // Allocate coroutine frame from the LIFO context and append a back-pointer trailer.
        constexpr std::size_t align = alignof(LifoContext*);
        n = (n + align - 1) & ~(align - 1);

        auto* p = static_cast<std::byte*>(context.allocate(n + sizeof(LifoContext*)));
        if (!p)
            return nullptr;
        new (p + n) LifoContext*(&context);

        return static_cast<void*>(p);
    }

    template <typename... Args>
    requires(static_context != nullptr)
    static void* operator new(std::size_t n, const Args&...) noexcept {
        return static_context->allocate(n);
    }

    // NOLINTNEXTLINE(misc-new-delete-overloads)
    static void operator delete(void* p, std::size_t n) noexcept {
        if constexpr (static_context == nullptr) {
            constexpr std::size_t align = alignof(LifoContext*);
            n = (n + align - 1) & ~(align - 1);

            // Recover the original LifoContext from the trailer and deallocate the whole block.
            auto* context =
                *std::launder(reinterpret_cast<LifoContext**>(static_cast<std::byte*>(p) + n));
            context->deallocate(p, n + sizeof(LifoContext*));
        } else {
            static_context->deallocate(p, n);
        }
    }
};

template <typename T = void, LifoContext* static_context = nullptr>
requires(
    std::is_same_v<T, void> || (std::is_move_assignable_v<T> && std::is_default_constructible_v<T>))
class LifoTask {
public:
    struct promise_type : LifoStackedPromise<static_context> {
        friend class LifoTask;

        [[noreturn]] static LifoTask get_return_object_on_allocation_failure() noexcept {
            utility::assert_failed_debug();
        }

        constexpr LifoTask get_return_object() noexcept {
            return LifoTask{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        static constexpr std::suspend_never initial_suspend() noexcept { return {}; }

        struct FinalAwaiter {
            static constexpr bool await_ready() noexcept { return false; }

            static constexpr std::coroutine_handle<>
                await_suspend(std::coroutine_handle<promise_type> h) noexcept {
                auto& promise = h.promise();

                if (promise.continuation_)
                    return promise.continuation_;

                return std::noop_coroutine();
            }

            constexpr void await_resume() noexcept {}
        };
        static constexpr FinalAwaiter final_suspend() noexcept { return {}; }

        void return_value(T value) noexcept(std::is_nothrow_move_assignable_v<T>) {
            result_ = std::move(value);
        }

        [[noreturn]] static void unhandled_exception() noexcept { utility::assert_failed_debug(); }

    private:
        T result_;
        std::coroutine_handle<> continuation_{nullptr};
    };

    LifoTask(const LifoTask&) = delete;
    LifoTask& operator=(const LifoTask&) = delete;
    LifoTask(LifoTask&&) = delete;
    LifoTask& operator=(LifoTask&&) = delete;

    ~LifoTask() noexcept { handle_.destroy(); }

    struct TaskAwaiter {
        const std::coroutine_handle<promise_type> handle;

        constexpr bool await_ready() const noexcept { return handle.done(); }

        constexpr void await_suspend(std::coroutine_handle<> awaiting_coroutine) const noexcept {
            handle.promise().continuation_ = awaiting_coroutine;
        }

        T await_resume() noexcept(std::is_nothrow_move_constructible_v<T>) {
            utility::assert_debug(handle.done());
            return std::move(handle.promise().result_);
        }
    };
    constexpr TaskAwaiter operator co_await() && noexcept { return TaskAwaiter{handle_}; }

    [[nodiscard]] constexpr void* identifier() const noexcept { return handle_.address(); }

    [[nodiscard]] constexpr bool done() const noexcept { return handle_.done(); }

    T& result() noexcept {
        utility::assert_debug(handle_.done());
        return handle_.promise().result_;
    }

    const T& result() const noexcept {
        utility::assert_debug(handle_.done());
        return handle_.promise().result_;
    }

private:
    explicit constexpr LifoTask(std::coroutine_handle<promise_type> h) noexcept
        : handle_(h) {}

    const std::coroutine_handle<promise_type> handle_;
};

template <LifoContext* static_context>
class LifoTask<void, static_context> {
public:
    struct promise_type : LifoStackedPromise<static_context> {
        friend class LifoTask;

        [[noreturn]] static LifoTask get_return_object_on_allocation_failure() noexcept {
            utility::assert_failed_debug();
        }

        constexpr LifoTask get_return_object() noexcept {
            return LifoTask{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        static constexpr std::suspend_never initial_suspend() noexcept { return {}; }

        struct FinalAwaiter {
            static constexpr bool await_ready() noexcept { return false; }

            static constexpr std::coroutine_handle<>
                await_suspend(std::coroutine_handle<promise_type> h) noexcept {
                auto& promise = h.promise();

                if (promise.continuation_)
                    return promise.continuation_;

                return std::noop_coroutine();
            }

            constexpr void await_resume() noexcept {}
        };
        static constexpr FinalAwaiter final_suspend() noexcept { return {}; }

        constexpr void return_void() noexcept {}

        [[noreturn]] static void unhandled_exception() noexcept { utility::assert_failed_debug(); }

    private:
        std::coroutine_handle<> continuation_{nullptr};
    };

    LifoTask(const LifoTask&) = delete;
    LifoTask& operator=(const LifoTask&) = delete;
    LifoTask(LifoTask&&) = delete;
    LifoTask& operator=(LifoTask&&) = delete;

    ~LifoTask() noexcept { handle_.destroy(); }

    struct TaskAwaiter {
        const std::coroutine_handle<promise_type> handle;

        constexpr bool await_ready() const noexcept { return handle.done(); }

        constexpr void await_suspend(std::coroutine_handle<> awaiting_coroutine) const noexcept {
            handle.promise().continuation_ = awaiting_coroutine;
        }

        constexpr void await_resume() noexcept { utility::assert_debug(handle.done()); }
    };
    constexpr TaskAwaiter operator co_await() && noexcept { return TaskAwaiter{handle_}; }

    [[nodiscard]] constexpr bool done() const noexcept { return handle_.done(); }

    constexpr void result() const noexcept { utility::assert_debug(handle_.done()); }

private:
    explicit constexpr LifoTask(std::coroutine_handle<promise_type> h) noexcept
        : handle_(h) {}

    const std::coroutine_handle<promise_type> handle_;
};

} // namespace librmcs::core::coroutine
