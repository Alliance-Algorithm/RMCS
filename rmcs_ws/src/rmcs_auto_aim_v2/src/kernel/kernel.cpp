#include "kernel/kernel.hpp"
#include "kernel/common.hpp"
#include "modules/debug/framerate.hpp"
#include "utility/shared/context.hpp"
#include "utility/shared/interprocess.hpp"
#include "utility/thread/spsc_queue.hpp"

#include "kernel/capturer.hpp"
#include "kernel/identifier.hpp"
#include "kernel/visualization.hpp"

using namespace rmcs::kernel;

struct AutoAim::Impl {
    using Context = shared::Context;
    using Client  = shm::Client<Context>::Send;

private:
    // ROS2
    util::Node& node;
    std::shared_ptr<rclcpp::TimerBase> timer;

    // Context
    FramerateCounter event_framerate;

    util::spsc_queue<handle_type, 20> coroutines;
    std::vector<co::task<result_type>> tasks;

    // Capturer
    kernel::Capturer capturer {};

    // Identifier
    kernel::Identifier identifier {};

    // Calculator

    // Debug
    kernel::Visualization visualization;

    Client client {};

    auto switch_to_kernel() noexcept {
        struct awaitable {
            decltype(coroutines)& coroutines_ref;
            bool pushed = false;

            auto await_suspend(handle_type co) noexcept {
                // Resume immediately if push failed
                return pushed = coroutines_ref.push(co);
            }
            auto await_resume() const noexcept { return pushed; }

            static constexpr auto await_ready() noexcept { return false; }
        };
        return awaitable { coroutines };
    }

public:
    explicit Impl(util::Node& _node) noexcept
        : node { _node } {

        initialize();

        using namespace std::chrono_literals;
        timer = node.create_wall_timer(1ms, [this] { kernel_event_loop(); });
    }

    auto initialize() noexcept -> void {
        node.info("AutoAim Kernel is initializing...");

        // if (auto ret = config.serialize("", node); !ret) {
        //     node.error("Failed to read kernel config.");
        //     node.error("  e: {}", ret.error());
        //     rclcpp::shutdown();
        // }

        // initialize_kernel(capturer, node);

        // initialize_kernel(kernel1, node);
        // initialize_kernel(kernel2, node);

        // if (config.use_visualization) {
        //     // initialize_kernel(visualization, node);
        // }

        if (!client.open(shared::id)) {
            node.error("Failed to create shared memory");
        } else {
            node.info("Shared memory was opened");
        }

        node.info("AutoAim Kernel is initialized");
    }

    /// @NOTE: The main loop for auto aiming
    ///  - Event 1: query image from capturer and publish task
    ///  - Event 2: resume avaliable task and clear them
    ///  - Finally registerd in ros2 executor
    auto kernel_event_loop() noexcept -> void {
        // Publish task here
        if (auto image = capturer.fetch_image()) {
            auto task = make_consumption_task(std::move(image));
            tasks.emplace_back(task);
        }
        // Resume avaliable coroutine task
        std::coroutine_handle<> to_resume;
        while (coroutines.pop(to_resume)) {
            to_resume.resume();
        }
        // Check and remove finished task
        using task_type  = co::task<result_type>;
        const auto check = [this](task_type& task) {
            if (!task.done()) return false;
            if (auto& ret = task.result(); !ret) {
                node.error("Failed to exec task");
                node.error("    e: {}", ret.error());
            }
            task.destroy();
            return true;
        };
        std::erase_if(tasks, check);

        event_framerate.tick();
    }

    /// @NOTE:
    ///  - Use "switch_to_kernel" to come back from other thread
    ///  - No blocking function on kernel context after switching
    auto make_consumption_task(std::unique_ptr<Image> _image) noexcept //
        -> co::task<result_type> {

        auto image = std::shared_ptr<Image> { _image.release() };

        identifier.sync_identify(*image);

        co_return {};
    }
};

AutoAim::AutoAim() noexcept
    : Node { "AutoAim", util::options }
    , pimpl { std::make_unique<Impl>(*this) } { }

AutoAim::~AutoAim() noexcept = default;
