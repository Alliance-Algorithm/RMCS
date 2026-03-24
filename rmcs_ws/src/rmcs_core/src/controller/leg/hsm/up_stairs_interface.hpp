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
#include <utility>
#include <vector>

namespace rmcs_core::controller::leg::hsm::up_stairs {

using InputDouble = rmcs_executor::Component::InputInterface<double>;
using InputVelocity =
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>;
inline constexpr std::size_t LegJointCount = 4;

enum class UpLayerId : std::size_t {
    Initial = 0,
    Press,
    Lift,
    LiftAndInitial,
    InitialAgain,
    PressAgain,
    LiftAgain,
    Count,
};

enum class UpStairsType {
    OneStairs,
    TwoStairsLift,
    TwoStairsInitial,
};

constexpr std::size_t toIndex(UpLayerId layer) { return static_cast<std::size_t>(layer); }

inline constexpr std::size_t UpLayerCount = toIndex(UpLayerId::Count);

class UpStairsPlanner {
public:
    UpStairsPlanner()
        : trajectory_(LegJointCount) {}

    void load(const std::vector<std::vector<double>>& layer_parameters) {
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
        UpLayerId layer, const std::array<double, LegJointCount>& start_joints, double speed_x) {
        if (!loaded_) {
            return false;
        }
        const auto* parameter = parameterForLayer(layer);
        if (!parameter) {
            return false;
        }

        std::array<double, LegJointCount> end_point{};
        std::copy_n(parameter->begin(), LegJointCount, end_point.begin());
        const double k = (*parameter)[LegJointCount];
        const double b = (*parameter)[LegJointCount + 1];

        active_layer_ = layer;
        trajectory_.reset();
        trajectory_.set_start_point(toVector(start_joints));
        trajectory_.set_end_point(toVector(end_point));
        trajectory_.set_total_step(calculateSteps(k, b, speed_x));
        return true;
    }

    bool tickLayer(UpLayerId layer, std::array<double, LegJointCount>& out_joints) {
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
    using LegJointTrajectory =
        rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>;
    static constexpr int MinStep       = 200;
    static constexpr int MaxStep       = 4000;
    static constexpr double VReference = 1.5;

    static std::vector<double> toVector(const std::array<double, LegJointCount>& values) {
        return std::vector<double>{values[0], values[1], values[2], values[3]};
    }

    int calculateSteps(double k, double b, double speed_x) const {
        const double raw    = k * (speed_x - VReference) + b;
        const int unclamped = static_cast<int>(raw);
        return std::clamp(unclamped, MinStep, MaxStep);
    }

    const std::vector<double>* parameterForLayer(UpLayerId layer) const {
        const std::size_t idx = toIndex(layer);
        if (idx >= layer_parameters_.size()) {
            return nullptr;
        }
        const auto& parameter = layer_parameters_[idx];
        if (parameter.size() != LegJointCount + 2) {
            return nullptr;
        }
        return &parameter;
    }

    std::vector<std::vector<double>> layer_parameters_{};
    bool loaded_{false};
    std::optional<UpLayerId> active_layer_{};
    LegJointTrajectory trajectory_;
};

struct UpStairsRunnerContext {
    UpStairsPlanner* planner{nullptr};
    std::function<std::array<double, LegJointCount>()> read_joints{};
    std::function<double()> read_speed_x{};
    std::array<double, LegJointCount>* result{nullptr};
};

class UpStairsLayer : public linear::ILayer<UpStairsRunnerContext> {
public:
    explicit UpStairsLayer(UpLayerId layer_id)
        : layer_id_(layer_id) {}

    UpLayerId getLayerId() const { return layer_id_; }

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
    UpLayerId layer_id_{UpLayerId::Initial};
};

inline std::array<double, LegJointCount> makeNaNResult() {
    return {
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
}

class Auto_Leg_Up_Stairs {
public:
    using LayerRunner = linear::LinearLayerRunner<UpStairsRunnerContext, UpLayerId>;

    Auto_Leg_Up_Stairs(rmcs_executor::Component& component, const rclcpp::Logger& logger)
        : logger_(logger)
        , context_{
              &planner_, [this]() { return readCurrentLegJoints(); },
              [this]() { return readCurrentSpeedX(); }, &result_}
        , runner_([this](UpLayerId id) { return resolveLayer(id); }) {
        resetResult();
        component.register_input("/leg/encoder/lf/angle", theta_lf_);
        component.register_input("/leg/encoder/lb/angle", theta_lb_);
        component.register_input("/leg/encoder/rb/angle", theta_rb_);
        component.register_input("/leg/encoder/rf/angle", theta_rf_);
        component.register_input("/chassis/control_velocity", speed_);
    }

    Auto_Leg_Up_Stairs& configure(const UpStairsType& type) {

        clearLayerConnections();

        switch (type) {
        case UpStairsType::OneStairs: {
            set_layers({UpLayerId::Initial, UpLayerId::Press, UpLayerId::Lift});
            set_layer_connections(UpLayerId::Initial, []() { return true; });
            break;
        }
        case UpStairsType::TwoStairsLift: {
            set_layers(
                {UpLayerId::Initial, UpLayerId::Press, UpLayerId::Lift, UpLayerId::LiftAndInitial,
                 UpLayerId::PressAgain, UpLayerId::LiftAgain});
            set_layer_connections(UpLayerId::Initial, []() { return true; });
            set_layer_connections(UpLayerId::LiftAndInitial, []() { return true; });
            break;
        }
        case UpStairsType::TwoStairsInitial: {
            set_layers(
                {UpLayerId::Initial, UpLayerId::Press, UpLayerId::Lift, UpLayerId::InitialAgain,
                 UpLayerId::PressAgain, UpLayerId::LiftAgain});
            set_layer_connections(UpLayerId::Initial, []() { return true; });
            set_layer_connections(UpLayerId::InitialAgain, []() { return true; });
            break;
        }
        default: {
            RCLCPP_WARN(logger_, "no up stairs type input");
        }
        }
        return *this;
    }

    bool start() {
        linear::StartRequest<UpLayerId> linear_request;
        linear_request.layers = defaultPlan();

        if (!runner_.start(linear_request, context_)) {
            RCLCPP_WARN(logger_, "up one stairs start failed: runner rejected request");
            return false;
        }
        return true;
    }

    void set_layers(std::initializer_list<UpLayerId> ids) {
        layers_.clear();
        layers_.reserve(ids.size());
        for (UpLayerId id : ids) {
            layers_.emplace_back(id);
        }
    }

    void set_layer_connections(UpLayerId id, std::function<bool()> func) {
        runner_.add_connection(id, std::move(func));
    }

    std::vector<UpLayerId> defaultPlan() const {
        std::vector<UpLayerId> ids;
        ids.reserve(layers_.size());
        for (const auto& layer : layers_) {
            ids.push_back(layer.getLayerId());
        }
        return ids;
    }

    void tick() { runner_.tick(context_); }

    bool addLayerConnection(UpLayerId id, std::function<bool()> connection) {

        return runner_.add_connection(id, std::move(connection));
    }

    void clearLayerConnections() { runner_.clearConnections(); }

    std::optional<UpLayerId> get_current_layer_id() const { return runner_.current_layer_id(); }

    void stop() {
        runner_.stop();
        resetResult();
    }

    const std::array<double, LegJointCount>& get_result() const { return result_; }

    Auto_Leg_Up_Stairs& load(
        const std::vector<double>& initial_parameter, const std::vector<double>& press_parameter,
        const std::vector<double>& lift_parameter,
        const std::vector<double>& lift_and_initial_parameter,
        const std::vector<double>& initial_again_parameter,
        const std::vector<double>& press_again_parameter,
        const std::vector<double>& lift_again_parameter) {
        std::vector<std::vector<double>> layer_parameters(UpLayerCount);
        layer_parameters[toIndex(UpLayerId::Initial)]        = initial_parameter;
        layer_parameters[toIndex(UpLayerId::Press)]          = press_parameter;
        layer_parameters[toIndex(UpLayerId::Lift)]           = lift_parameter;
        layer_parameters[toIndex(UpLayerId::LiftAndInitial)] = lift_and_initial_parameter;
        layer_parameters[toIndex(UpLayerId::InitialAgain)]   = initial_again_parameter;
        layer_parameters[toIndex(UpLayerId::PressAgain)]     = press_again_parameter;
        layer_parameters[toIndex(UpLayerId::LiftAgain)]      = lift_again_parameter;

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

    bool validateLayerParameters(const std::vector<std::vector<double>>& layer_parameters) {
        for (const UpLayerId id : defaultPlan()) {
            const auto& parameter = layer_parameters[toIndex(id)];
            if (parameter.size() != LegJointCount + 2) {
                RCLCPP_ERROR(
                    logger_,
                    "up one stairs load failed: layer_parameters[%zu] size=%zu expected=%zu",
                    toIndex(id), parameter.size(), LegJointCount + 2);
                return false;
            }
        }
        return true;
    }

    linear::ILayer<UpStairsRunnerContext>* resolveLayer(UpLayerId id) {
        auto it = std::find_if(layers_.begin(), layers_.end(), [id](const UpStairsLayer& layer) {
            return layer.getLayerId() == id;
        });

        return (it == layers_.end()) ? nullptr : &(*it);
    }

    rclcpp::Logger logger_;
    UpStairsPlanner planner_;
    std::array<double, LegJointCount> result_{};
    UpStairsRunnerContext context_{};
    std::vector<UpStairsLayer> layers_{
        UpStairsLayer{UpLayerId::Initial}, UpStairsLayer{UpLayerId::Press},
        UpStairsLayer{UpLayerId::Lift}};

    InputDouble theta_lf_;
    InputDouble theta_lb_;
    InputDouble theta_rb_;
    InputDouble theta_rf_;
    InputVelocity speed_;

    LayerRunner runner_;
};

} // namespace rmcs_core::controller::leg::hsm::up_stairs