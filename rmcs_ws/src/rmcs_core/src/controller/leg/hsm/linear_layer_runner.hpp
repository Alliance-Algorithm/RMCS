#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rmcs_core::controller::leg::hsm::linear {

enum class RunnerStatus {
    Idle,
    Running,
    Paused,
    Completed,
    Fault,
};

enum class PauseReason {
    None,
    Breakpoint,
};

template <typename LayerId>
struct StartRequest {
    std::vector<LayerId> layers{};
};

struct LayerTickResult {
    enum class Code {
        Running,
        Completed,
        Fault,
    };

    Code code{Code::Running};

    static LayerTickResult running() {
        return {};
    }

    static LayerTickResult completed() {
        return {Code::Completed};
    }

    static LayerTickResult fault() {
        return {Code::Fault};
    }
};

template <typename Ctx>
class ILayer {
public:
    virtual ~ILayer() = default;

    virtual bool onEnter(Ctx& ctx) = 0;
    virtual LayerTickResult onTick(Ctx& ctx) = 0;
    virtual void onExit(Ctx& ctx) = 0;
};

template <typename Ctx, typename LayerId>
class LinearLayerRunner {
public:
    using LayerContract = ILayer<Ctx>;
    using Resolver = std::function<LayerContract*(LayerId)>;

    explicit LinearLayerRunner(Resolver resolver)
        : resolver_(std::move(resolver)) {}

    bool start(const StartRequest<LayerId>& request, Ctx& ctx) {
        if (!resolver_ || request.layers.empty()) {
            setFault();
            return false;
        }

        plan_ = request.layers;
        current_index_ = 0;
        pending_boundary_index_.reset();
        setRunning();
        return enterCurrentLayer(ctx);
    }

    void stop() {
        plan_.clear();
        breakpoints_.clear();
        current_index_ = 0;
        pending_boundary_index_.reset();
        status_ = RunnerStatus::Idle;
        pause_reason_ = PauseReason::None;
    }

    void tick(Ctx& ctx) {
        if (status_ != RunnerStatus::Running) {
            return;
        }
        if (current_index_ >= plan_.size()) {
            status_ = RunnerStatus::Completed;
            pause_reason_ = PauseReason::None;
            return;
        }

        LayerContract* layer = resolveCurrentLayer();
        if (!layer) {
            setFault();
            return;
        }

        const LayerTickResult result = layer->onTick(ctx);
        switch (result.code) {
        case LayerTickResult::Code::Running:
            return;
        case LayerTickResult::Code::Fault:
            setFault();
            return;
        case LayerTickResult::Code::Completed:
            layer->onExit(ctx);
            transitionToBoundary(current_index_ + 1, ctx);
            return;
        }
    }

    bool resume(Ctx& ctx) {
        if (status_ != RunnerStatus::Paused || pause_reason_ != PauseReason::Breakpoint
            || !pending_boundary_index_) {
            return false;
        }

        setRunning();
        current_index_ = *pending_boundary_index_;
        pending_boundary_index_.reset();
        return enterCurrentLayer(ctx);
    }

    bool addBreakpoint(std::size_t boundary_index) {
        if (boundary_index == 0 || boundary_index >= plan_.size()) {
            return false;
        }
        return breakpoints_.insert(boundary_index).second;
    }

    bool removeBreakpoint(std::size_t boundary_index) {
        return breakpoints_.erase(boundary_index) > 0;
    }

    void clearBreakpoints() {
        breakpoints_.clear();
    }

private:
    void setRunning() {
        status_ = RunnerStatus::Running;
        pause_reason_ = PauseReason::None;
    }

    void setFault() {
        status_ = RunnerStatus::Fault;
        pause_reason_ = PauseReason::None;
    }

    bool enterCurrentLayer(Ctx& ctx) {
        if (current_index_ >= plan_.size()) {
            status_ = RunnerStatus::Completed;
            pause_reason_ = PauseReason::None;
            return false;
        }

        LayerContract* layer = resolveCurrentLayer();
        if (!layer || !layer->onEnter(ctx)) {
            setFault();
            return false;
        }
        return true;
    }

    LayerContract* resolveCurrentLayer() const {
        return resolver_ ? resolver_(plan_[current_index_]) : nullptr;
    }

    void transitionToBoundary(std::size_t next_index, Ctx& ctx) {
        if (next_index >= plan_.size()) {
            status_ = RunnerStatus::Completed;
            pause_reason_ = PauseReason::None;
            return;
        }

        if (breakpoints_.contains(next_index)) {
            pending_boundary_index_ = next_index;
            status_ = RunnerStatus::Paused;
            pause_reason_ = PauseReason::Breakpoint;
            return;
        }

        current_index_ = next_index;
        (void)enterCurrentLayer(ctx);
    }

    Resolver resolver_;
    std::vector<LayerId> plan_{};
    std::unordered_set<std::size_t> breakpoints_{};
    std::optional<std::size_t> pending_boundary_index_{};
    std::size_t current_index_{0};
    RunnerStatus status_{RunnerStatus::Idle};
    PauseReason pause_reason_{PauseReason::None};
};

} // namespace rmcs_core::controller::leg::hsm::linear
