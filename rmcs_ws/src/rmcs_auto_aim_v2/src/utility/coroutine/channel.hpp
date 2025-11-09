#pragma once
#include <coroutine>
#include <optional>
#include <queue>

namespace rmcs::co {

template <typename T>
struct Channel {

    std::queue<std::coroutine_handle<>> waiters;
    std::queue<T> items;
    std::optional<T> item;

    struct AwaitableItem {
        Channel& channel;
        std::optional<T> item;

        auto await_ready() noexcept -> bool {
            if (channel.items.empty()) {
                return false;
            }
            item = std::move(channel.items.front());
            channel.items.pop();
            return true;
        }
        auto await_suspend(std::coroutine_handle<> h) noexcept -> void {
            //
            channel.waiters.push(h);
        }
        auto await_resume() noexcept -> T {
            if (item.has_value()) {
                return std::move(item.value());
            } else {
                return std::move(channel.item.value());
            }
        }
    };

    auto send(T t) noexcept -> void {
        if (waiters.empty()) {
            items.push(std::move(t));
        } else {
            auto waiter = waiters.front();
            waiters.pop();

            item = std::move(t);
            waiter.resume();
        }
    }

    auto recv() noexcept { return AwaitableItem { *this }; }
};

}
