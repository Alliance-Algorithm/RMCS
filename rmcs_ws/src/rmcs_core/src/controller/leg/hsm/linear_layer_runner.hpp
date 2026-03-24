#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace rmcs_core::controller::leg::hsm::linear {

enum class RunnerStatus {
    Running,
    Paused,
    Completed,
};

template <typename LayerId>
struct StartRequest {
    std::vector<LayerId> layers{};
};

enum class LayerTickResult {
    Running,
    Completed,
};

template <typename Ctx>
class ILayer {
public:
    virtual ~ILayer() = default;

    virtual bool onEnter(Ctx& ctx)           = 0;
    virtual LayerTickResult onTick(Ctx& ctx) = 0;
    virtual void onExit(Ctx& ctx)            = 0;
};

template <typename LayerId>
class IConnection {
public:
    virtual ~IConnection() = default;

    virtual LayerId getConnectionId() const = 0;
    virtual bool allowTransition() { return true; }
};

template <typename LayerId>
class LambdaConnection : public IConnection<LayerId> {
public:
    using Func = std::function<bool()>;

    LambdaConnection(LayerId id, Func func)
        : id_(id)
        , func_(std::move(func)) {}

    LayerId getConnectionId() const override { return id_; }

    bool allowTransition() override { return func_ ? func_() : true; }

private:
    LayerId id_;
    Func func_;
};

template <typename Ctx, typename LayerId>
class LinearLayerRunner {
public:
    using Resolver = std::function<ILayer<Ctx>*(LayerId)>;

    explicit LinearLayerRunner(Resolver resolver)
        : resolver_(std::move(resolver)) {}

    bool add_connection(LayerId id, std::function<bool()> func) {
        return setConnection(std::make_unique<LambdaConnection<LayerId>>(id, std::move(func)));
    }

    void clearConnections() { connections_.clear(); }

    bool start(const StartRequest<LayerId>& request, Ctx& ctx) {
        if (!resolver_ || request.layers.empty()) {
            return false;
        }

        plan_          = request.layers;
        current_index_ = 0;
        pending_boundary_index_.reset();
        status_ = RunnerStatus::Running;

        if (!enterCurrentLayer(ctx)) {
            stop();
            return false;
        }
        return true;
    }

    std::optional<LayerId> current_layer_id() const {
        if (status_ == RunnerStatus::Completed) {
            if (plan_.empty()) {
                return std::nullopt;
            }
            return plan_.back();
        }
        if (status_ == RunnerStatus::Paused) {
            if (pending_boundary_index_ && *pending_boundary_index_ > 0
                && *pending_boundary_index_ <= plan_.size()) {
                return plan_[*pending_boundary_index_ - 1];
            }
            if (current_index_ < plan_.size()) {
                return plan_[current_index_];
            }
        }
        if (status_ == RunnerStatus::Running) {
            if (current_index_ < plan_.size()) {
                return plan_[current_index_];
            }
        }
        return std::nullopt;
    }

    void stop() {
        plan_.clear();
        current_index_ = 0;
        pending_boundary_index_.reset();
        status_ = RunnerStatus::Paused;
    }

    void tick(Ctx& ctx) {
        if (status_ == RunnerStatus::Paused && !pending_boundary_index_)
            return;
        if (pending_boundary_index_) {
            transitionToBoundary(*pending_boundary_index_, ctx);
            return;
        }

        if (current_index_ >= plan_.size()) {
            status_ = RunnerStatus::Completed;
            return;
        }

        ILayer<Ctx>* layer = resolveCurrentLayer();
        status_            = RunnerStatus::Running;
        if (!layer) {
            status_ = RunnerStatus::Paused;
            return;
        }

        const LayerTickResult result = layer->onTick(ctx);
        switch (result) {
        case LayerTickResult::Running: return;
        case LayerTickResult::Completed:
            layer->onExit(ctx);
            pending_boundary_index_ = current_index_ + 1;
            transitionToBoundary(*pending_boundary_index_, ctx);
            return;
        }
    }

private:
    bool setConnection(std::unique_ptr<IConnection<LayerId>> connection) {
        if (!connection) {
            return false;
        }

        const LayerId id = connection->getConnectionId();
        for (auto& item : connections_) {
            if (item && item->getConnectionId() == id) {
                item = std::move(connection);
                return true;
            }
        }

        connections_.push_back(std::move(connection));
        return true;
    }
    bool enterCurrentLayer(Ctx& ctx) {
        if (current_index_ >= plan_.size()) {
            status_ = RunnerStatus::Completed;
            return false;
        }

        ILayer<Ctx>* layer = resolveCurrentLayer();
        if (!layer) {
            return false;
        }
        return layer->onEnter(ctx);
    }

    ILayer<Ctx>* resolveCurrentLayer() const {
        return resolver_ ? resolver_(plan_[current_index_]) : nullptr;
    }

    IConnection<LayerId>* resolveConnectionForLayer(LayerId id) const {
        for (const auto& item : connections_) {
            if (item && item->getConnectionId() == id) {
                return item.get();
            }
        }
        return nullptr;
    }
    bool allowTransitionFromCurrentLayer() const {
        if (current_index_ >= plan_.size()) {
            return true;
        }

        IConnection<LayerId>* connection = resolveConnectionForLayer(plan_[current_index_]);
        if (!connection) {
            return true; // 默认是allow
        }
        return connection->allowTransition();
    }

    void transitionToBoundary(std::size_t next_index, Ctx& ctx) {
        if (next_index >= plan_.size()) {
            pending_boundary_index_.reset();
            status_ = RunnerStatus::Completed;
            return;
        }

        if (!allowTransitionFromCurrentLayer()) {
            pending_boundary_index_ = next_index;
            status_                 = RunnerStatus::Paused;
            return;
        }

        status_ = RunnerStatus::Running;
        current_index_ = next_index;
        pending_boundary_index_.reset();
        if (!enterCurrentLayer(ctx)) {
            stop();
        }
    }
    Resolver resolver_;
    std::vector<LayerId> plan_{};
    std::vector<std::unique_ptr<IConnection<LayerId>>> connections_{};
    std::optional<std::size_t> pending_boundary_index_{};
    std::size_t current_index_{0};
    RunnerStatus status_{RunnerStatus::Paused};
};

} // namespace rmcs_core::controller::leg::hsm::linear
