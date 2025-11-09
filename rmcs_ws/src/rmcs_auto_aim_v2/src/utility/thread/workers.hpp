#pragma once
#include "utility/pimpl.hpp"
#include <future>

namespace rmcs {

class WorkersContext {
    RMCS_PIMPL_DEFINITION(WorkersContext)

    struct InternalTask {
        virtual auto run() noexcept -> void { }
        virtual ~InternalTask() noexcept = default;
    };

public:
    template <typename F, typename... Args>
        requires std::is_nothrow_invocable_v<F, Args...>
    auto enqueue(F&& f, Args&&... args) noexcept {
        using return_type = std::invoke_result_t<F, Args...>;

        struct Task : public InternalTask {
            std::packaged_task<return_type(Args...)> task;
            std::tuple<Args...> args;

            explicit Task(F&& f, Args&&... args) noexcept
                : task { std::forward<F>(f) }
                , args { std::forward<Args>(args)... } { }

            ~Task() noexcept override = default;

            auto run() noexcept -> void override { std::apply(task, args); }

            auto future() { return task.get_future(); }
        };
        auto enqueue_task  = std::make_unique<Task>( //
            std::forward<F>(f), std::forward<Args>(args)...);
        auto return_future = enqueue_task->future();

        internal_enqueue(std::move(enqueue_task));
        return return_future;
    }

private:
    auto internal_enqueue(std::unique_ptr<InternalTask> task) noexcept -> void;
};

}
