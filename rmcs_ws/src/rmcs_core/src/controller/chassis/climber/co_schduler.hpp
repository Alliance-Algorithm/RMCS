#pragma once

#include <atomic>
#include <chrono>
#include <coroutine>
#include <exception>
#include <functional>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

namespace rmcs_core {

// 协程调度器：对齐 rmcs-navigation Lua 调度器语义
// - 任务携带 resume_request 谓词，每拍先问谓词，true 才 resume
// - 等待原语挂起时替换谓词，且至少让出一拍（控制循环时序确定性）
// - append 返回弱持有 Handle，cancel 为共享标志，任务消亡后 no-op
struct CoSchduler {
    struct Task {
        struct promise_type {
            std::function<bool()> resume_request{[] { return true; }};
            std::exception_ptr error{};

            static constexpr auto initial_suspend() noexcept { return std::suspend_always{}; }
            static constexpr auto final_suspend() noexcept { return std::suspend_always{}; }

            auto get_return_object();

            static constexpr auto return_void() noexcept {}

            auto unhandled_exception() noexcept { error = std::current_exception(); }

            // co_yield {} 等价于 Tick：下拍唤醒并重置谓词
            auto yield_value(std::monostate) noexcept {
                resume_request = [] { return true; };
                return std::suspend_always{};
            }
        };

        explicit Task(std::coroutine_handle<promise_type> handle) noexcept
            : handle_{handle} {}

        Task(const Task&) = delete;
        Task& operator=(const Task&) = delete;

        Task(Task&& other) noexcept
            : handle_{std::exchange(other.handle_, {})} {}

        auto operator=(Task&& other) noexcept -> Task& {
            if (this != &other) {
                if (handle_)
                    handle_.destroy();
                handle_ = std::exchange(other.handle_, {});
            }
            return *this;
        }

        ~Task() {
            if (handle_)
                handle_.destroy();
        }

        auto release() noexcept { return std::exchange(handle_, {}); }

    private:
        std::coroutine_handle<promise_type> handle_{};
    };

    // co_await Tick{}：下一拍唤醒
    struct Tick {
        static constexpr auto await_ready() noexcept { return false; }

        template <typename Promise>
        static auto await_suspend(std::coroutine_handle<Promise> handle) {
            handle.promise().resume_request = [] { return true; };
        }

        static constexpr auto await_resume() noexcept {}
    };

    // co_await Sleep{ duration }：到点唤醒（steady_clock，至少一拍）
    struct Sleep {
        std::chrono::steady_clock::duration duration;

        static constexpr auto await_ready() noexcept { return false; }

        template <typename Promise>
        auto await_suspend(std::coroutine_handle<Promise> handle) {
            const auto deadline = std::chrono::steady_clock::now() + duration;
            handle.promise().resume_request = [deadline] {
                return std::chrono::steady_clock::now() >= deadline;
            };
        }

        static constexpr auto await_resume() noexcept {}
    };

    // co_await WaitUntil{ .monitor = ..., .timeout = ... }：条件满足或超时唤醒
    // await_resume 返回 is_timeout
    struct WaitUntil {
        std::function<bool()> monitor;
        std::chrono::steady_clock::duration timeout = std::chrono::steady_clock::duration::max();

        struct State {
            std::function<bool()> monitor;
            std::chrono::steady_clock::time_point deadline;
            bool timed_out = false;
        };

        std::shared_ptr<State> state{};

        static constexpr auto await_ready() noexcept { return false; }

        template <typename Promise>
        auto await_suspend(std::coroutine_handle<Promise> handle) {
            state = std::make_shared<State>(State{
                .monitor = std::move(monitor),
                .deadline = std::chrono::steady_clock::now() + timeout,
            });

            handle.promise().resume_request = [state = state] {
                if (state->monitor())
                    return true;

                state->timed_out = std::chrono::steady_clock::now() >= state->deadline;
                return state->timed_out;
            };
        }

        auto await_resume() const noexcept { return state->timed_out; }
    };

    struct Slot {
        explicit Slot(std::coroutine_handle<Task::promise_type> handle) noexcept
            : handle{handle} {}

        Slot(const Slot&) = delete;
        Slot& operator=(const Slot&) = delete;

        ~Slot() {
            if (handle)
                handle.destroy();
        }

        std::coroutine_handle<Task::promise_type> handle;
        std::atomic_bool cancelled{false};
    };

    // 取消句柄：弱持有任务，任务消亡后操作均为 no-op
    struct Handle {
        Handle() = default;

        auto cancel() const {
            if (const auto locked = slot.lock())
                locked->cancelled.store(true, std::memory_order::relaxed);
        }

        auto done() const {
            const auto locked = slot.lock();
            return !locked || locked->cancelled.load(std::memory_order::relaxed)
                || locked->handle.done();
        }

    private:
        friend struct CoSchduler;

        explicit Handle(std::weak_ptr<Slot> slot) noexcept
            : slot{std::move(slot)} {}

        std::weak_ptr<Slot> slot;
    };

    // 接管任务所有权，返回取消句柄；fire-and-forget 合法
    auto append(Task task) {
        auto slot = std::make_shared<Slot>(task.release());
        pending_.push_back(slot);
        return Handle{slot};
    }

    auto spin_once() {
        slots_.insert(slots_.end(), pending_.begin(), pending_.end());
        pending_.clear();

        auto error = std::exception_ptr{};

        std::erase_if(slots_, [&](const std::shared_ptr<Slot>& slot) {
            if (slot->cancelled.load(std::memory_order::relaxed))
                return true;

            auto& promise = slot->handle.promise();

            if (slot->handle.done()) {
                if (promise.error && !error)
                    error = promise.error;
                return true;
            }

            if (promise.resume_request()) {
                slot->handle.resume();

                if (slot->handle.done()) {
                    if (promise.error && !error)
                        error = promise.error;
                    return true;
                }
            }

            return false;
        });

        if (error)
            std::rethrow_exception(error);
    }

    // 急停清场：销毁全部任务帧，协程局部变量正常析构
    auto stop_all() {
        slots_.clear();
        pending_.clear();
    }

private:
    std::vector<std::shared_ptr<Slot>> slots_{};
    std::vector<std::shared_ptr<Slot>> pending_{};
};

inline auto CoSchduler::Task::promise_type::get_return_object() {
    return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
}

} // namespace rmcs_core
