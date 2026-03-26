#pragma once

#include "controller/arm/trajectory.hpp"
#include "controller/leg/hsm/linear_layer_runner.hpp"

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmcs_core::controller::leg::hsm::up_stairs {

inline constexpr std::size_t LegJointCount = 4;
inline constexpr std::size_t FullLayerParamCount = LegJointCount + 2;
inline constexpr std::size_t HalfLayerParamCount = (LegJointCount / 2) + 2;

using LayerId         = std::string;
using LayerParameters = std::unordered_map<LayerId, std::vector<double>>;

class UpStairsPlanner {
public:
    UpStairsPlanner()
        : trajectory_(LegJointCount) {}

    void load(const LayerParameters& layer_parameters) {
        layer_parameters_ = layer_parameters;
        loaded_           = true;
        reset();
    }

    bool loaded() const { return loaded_; }

    void unload() {
        loaded_ = false;
        active_layer_.reset();
        reset();
    }

    void reset() {
        active_layer_.reset();
        trajectory_.reset();
    }

    bool beginLayer(
        const LayerId& layer, const std::array<double, LegJointCount>& start_joints,
        double speed_x) {
        if (!loaded_) {
            return false;
        }
        const auto* parameter = parameterForLayer(layer);
        if (!parameter) {
            return false;
        }

        std::array<double, LegJointCount> end_point{};
        double k = 0.0;
        double b = 0.0;
        if (!decodeLayerParameter(*parameter, end_point, k, b)) {
            return false;
        }

        active_layer_ = layer;
        trajectory_.reset();
        trajectory_.set_start_point(toVector(start_joints));
        trajectory_.set_end_point(toVector(end_point));
        trajectory_.set_total_step(calculateSteps(k, b, speed_x));
        return true;
    }

    bool tickLayer(const LayerId& layer, std::array<double, LegJointCount>& out_joints) {
        if (loaded_ && active_layer_ && *active_layer_ == layer) {
            if (trajectory_.get_complete()) {
                return true;
            }

            const auto joints = trajectory_.trajectory();
            std::copy(joints.begin(), joints.end(), out_joints.begin());
            return false;
        } else
            return false;
    }

private:
    static constexpr int MinStep       = 200;
    static constexpr int MaxStep       = 4000;
    static constexpr double VReference = 1.5;

    static bool decodeLayerParameter(
        const std::vector<double>& parameter, std::array<double, LegJointCount>& end_point,
        double& k, double& b) {
        if (parameter.size() == FullLayerParamCount) {
            std::copy_n(parameter.begin(), LegJointCount, end_point.begin());
            k = parameter[LegJointCount];
            b = parameter[LegJointCount + 1];
            return true;
        }
        if (parameter.size() == HalfLayerParamCount) {
            end_point[0] = parameter[0];
            end_point[1] = parameter[1];
            end_point[2] = parameter[1];
            end_point[3] = parameter[0];
            k            = parameter[2];
            b            = parameter[3];
            return true;
        }
        return false;
    }

    static std::vector<double> toVector(const std::array<double, LegJointCount>& values) {
        return std::vector<double>{values[0], values[1], values[2], values[3]};
    }

    int calculateSteps(double k, double b, double speed_x) const {
        const double raw    = k * (speed_x - VReference) + b;
        const int unclamped = static_cast<int>(raw);
        return std::clamp(unclamped, MinStep, MaxStep);
    }

    const std::vector<double>* parameterForLayer(const LayerId& layer) const {
        const auto it = layer_parameters_.find(layer);
        if (it == layer_parameters_.end()) {
            return nullptr;
        }
        const auto& parameter = it->second;
        if (parameter.size() != FullLayerParamCount && parameter.size() != HalfLayerParamCount) {
            return nullptr;
        }
        return &parameter;
    }

    LayerParameters layer_parameters_{};
    bool loaded_{false};
    std::optional<LayerId> active_layer_{};
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        trajectory_;
};

struct UpStairsRunnerContext {
    UpStairsPlanner* planner{nullptr};
    std::function<std::array<double, LegJointCount>()> read_joints{};
    std::function<double()> read_speed_x{};
    std::array<double, LegJointCount>* result{nullptr};
};

class UpStairsLayer : public linear::ILayer<UpStairsRunnerContext> {
public:
    explicit UpStairsLayer(LayerId layer_id)
        : layer_id_(std::move(layer_id)) {}

    const LayerId& getLayerId() const { return layer_id_; }

    bool onEnter(UpStairsRunnerContext& ctx) override {
        if (!ctx.planner || !ctx.read_joints || !ctx.read_speed_x) {
            return false;
        }
        const std::array<double, LegJointCount> start_state = ctx.read_joints();
        if (ctx.result) {
            *ctx.result = start_state;
        }
        return ctx.planner->beginLayer(layer_id_, start_state, ctx.read_speed_x());
    }

    linear::LayerTickResult onTick(UpStairsRunnerContext& ctx) override {
        std::array<double, LegJointCount> output_state =
            ctx.result ? *ctx.result : std::array<double, LegJointCount>{};
        const bool layer_complete = ctx.planner->tickLayer(layer_id_, output_state);
        if (ctx.result) {
            *ctx.result = output_state;
        }
        if (!layer_complete) {
            return linear::LayerTickResult::Running;
        }
        return linear::LayerTickResult::Completed;
    }

    void onExit(UpStairsRunnerContext& /*ctx*/) override {}

private:
    LayerId layer_id_;
};

inline std::array<double, LegJointCount> makeNaNResult() {
    return {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
}

class Auto_Leg_Up_Stairs {
public:
    using LayerRunner = linear::LinearLayerRunner<UpStairsRunnerContext, LayerId>;

    Auto_Leg_Up_Stairs(rmcs_executor::Component& component, const rclcpp::Logger& logger)
        : logger_(logger)
        , context_{
              &planner_, [this]() { return readCurrentLegJoints(); },
              [this]() { return readCurrentSpeedX(); }, &result_}
        , runner_([this](LayerId id) { return resolveLayer(id); }) {
        resetResult();
        component.register_input("/leg/encoder/lf/angle", theta_lf_);
        component.register_input("/leg/encoder/lb/angle", theta_lb_);
        component.register_input("/leg/encoder/rb/angle", theta_rb_);
        component.register_input("/leg/encoder/rf/angle", theta_rf_);
        component.register_input("/chassis/control_velocity", speed_);
    }

    bool start() {
        linear::StartRequest<LayerId> linear_request;
        linear_request.layers = defaultPlan();

        if (!runner_.start(linear_request, context_)) {
            RCLCPP_WARN(logger_, "up one stairs start failed: runner rejected request");
            return false;
        }
        return true;
    }

    void set_layers(std::initializer_list<LayerId> ids) {
        layers_.clear();
        layers_.reserve(ids.size());
        for (const auto& id : ids) {
            layers_.emplace_back(id);
        }
    }

    void set_layer_connections(const LayerId& id, std::function<bool()> func) {
        runner_.add_connection(id, std::move(func));
    }

    std::vector<LayerId> defaultPlan() const {
        std::vector<LayerId> ids;
        ids.reserve(layers_.size());
        for (const auto& layer : layers_) {
            ids.push_back(layer.getLayerId());
        }
        return ids;
    }

    void tick() { runner_.tick(context_); }

    bool addLayerConnection(const LayerId& id, std::function<bool()> connection) {

        return runner_.add_connection(id, std::move(connection));
    }

    void clearLayerConnections() { runner_.clearConnections(); }

    std::optional<LayerId> get_current_layer_id() const { return runner_.current_layer_id(); }

    void stop() {
        runner_.stop();
        resetResult();
    }

    const std::array<double, LegJointCount>& get_result() const { return result_; }

    Auto_Leg_Up_Stairs& load(const LayerParameters& layer_parameters) {
        if (!validateLayerParameters(layer_parameters)) {
            planner_.unload();
            resetResult();
            return *this;
        }

        planner_.load(layer_parameters);
        resetResult();
        return *this;
    }

private:
    void resetResult() { result_ = makeNaNResult(); }
    std::array<double, LegJointCount> readCurrentLegJoints() const {
        return {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_};
    }

    double readCurrentSpeedX() const { return (*speed_)->x(); }

    bool validateLayerParameters(const LayerParameters& layer_parameters) {
        const auto plan = defaultPlan();
        if (plan.empty()) {
            RCLCPP_ERROR(logger_, "up stairs load failed: layer sequence is empty");
            return false;
        }
        for (const auto& id : plan) {
            const auto iter = layer_parameters.find(id);
            if (iter == layer_parameters.end()) {
                RCLCPP_ERROR(
                    logger_, "up stairs load failed: missing parameter for layer '%s'", id.c_str());
                return false;
            }
            if (iter->second.size() != FullLayerParamCount
                && iter->second.size() != HalfLayerParamCount) {
                RCLCPP_ERROR(
                    logger_,
                    "up stairs load failed: layer '%s' size=%zu expected=%zu(one-side) or "
                    "%zu(full)",
                    id.c_str(), iter->second.size(), HalfLayerParamCount, FullLayerParamCount);
                return false;
            }
        }
        return true;
    }

    linear::ILayer<UpStairsRunnerContext>* resolveLayer(const LayerId& id) {
        auto it = std::find_if(layers_.begin(), layers_.end(), [id](const UpStairsLayer& layer) {
            return layer.getLayerId() == id;
        });

        return (it == layers_.end()) ? nullptr : &(*it);
    }

    rclcpp::Logger logger_;
    UpStairsPlanner planner_;
    std::array<double, LegJointCount> result_{};
    UpStairsRunnerContext context_{};
    std::vector<UpStairsLayer> layers_{};

    rmcs_executor::Component::InputInterface<double> theta_lf_;
    rmcs_executor::Component::InputInterface<double> theta_lb_;
    rmcs_executor::Component::InputInterface<double> theta_rb_;
    rmcs_executor::Component::InputInterface<double> theta_rf_;
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector> speed_;

    LayerRunner runner_;
};

} // namespace rmcs_core::controller::leg::hsm::up_stairs
