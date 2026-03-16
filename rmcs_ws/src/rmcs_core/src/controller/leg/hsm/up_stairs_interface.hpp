#pragma once

#include "controller/arm/trajectory.hpp"
#include "controller/leg/hsm/linear_layer_runner.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <vector>

namespace rmcs_core::controller::leg::hsm::up_stairs {

using InputDouble = rmcs_executor::Component::InputInterface<double>;
using InputVelocity =
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>;

inline constexpr std::size_t LegJointCount = 4;
using LegJointArray = std::array<double, LegJointCount>;

enum class UpStairsMode {
    Invalid,
    OneProcess,
    TwoProcessLift,
    TwoProcessInitial,
};

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

struct UpStairsStartRequest {
    UpStairsMode mode{UpStairsMode::Invalid};
    std::vector<UpLayerId> layers{};
};

constexpr std::size_t toIndex(UpLayerId layer) {
    return static_cast<std::size_t>(layer);
}

inline constexpr std::size_t UpLayerCount = toIndex(UpLayerId::Count);

class UpStairsPlanner {
public:
    UpStairsPlanner()
        : trajectory_(LegJointCount) {}

    void load(const std::vector<std::vector<double>>& layer_parameters) {
        layer_parameters_ = layer_parameters;
        loaded_ = true;
        reset();
    }

    bool loaded() const {
        return loaded_;
    }

    void unload() {
        loaded_ = false;
        active_layer_.reset();
        reset();
    }

    void reset() {
        active_layer_.reset();
        trajectory_.reset();
    }

    bool beginLayer(UpLayerId layer, const LegJointArray& start_joints, double speed_x) {
        if (!loaded_) {
            return false;
        }
        const auto* parameter = parameterForLayer(layer);
        if (!parameter) {
            return false;
        }

        LegJointArray end_point{};
        std::copy_n(parameter->begin(), LegJointCount, end_point.begin());
        double k = (*parameter)[LegJointCount];
        double b = (*parameter)[LegJointCount + 1];

        active_layer_ = layer;
        trajectory_.reset();
        trajectory_.set_start_point(toVector(start_joints));
        trajectory_.set_end_point(toVector(end_point));
        trajectory_.set_total_step(calculateSteps(k, b, speed_x));
        return true;
    }

    bool tickLayer(UpLayerId layer, LegJointArray& out_joints) {
        if (!loaded_ || !active_layer_ || *active_layer_ != layer) {
            return false;
        }
        if (trajectory_.get_complete()) {
            return true;
        }

        const auto joints = trajectory_.trajectory();
        std::copy(joints.begin(), joints.end(), out_joints.begin());
        return false;
    }

private:
    using LegJointTrajectory =
        rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>;
    static constexpr int kMinStep = 200;
    static constexpr int kMaxStep = 4000;
    static constexpr double kVReference = 1.5;

    static std::vector<double> toVector(const LegJointArray& values) {
        return std::vector<double>{values[0], values[1], values[2], values[3]};
    }

    int calculateSteps(double k, double b, double speed_x) const {
        const double raw = k * (speed_x - kVReference) + b;
        const int unclamped = static_cast<int>(raw);
        return std::clamp(unclamped, kMinStep, kMaxStep);
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
    std::function<LegJointArray()> read_joints{};
    std::function<double()> read_speed_x{};
    LegJointArray* result{nullptr};
};

class UpStairsLayer
    : public linear::ILayer<UpStairsRunnerContext> {
public:
    explicit UpStairsLayer(UpLayerId layer_id)
        : layer_id_(layer_id) {}

    bool onEnter(UpStairsRunnerContext& ctx) override {
        if (!ctx.planner || !ctx.read_joints || !ctx.read_speed_x) {
            return false;
        }
        const LegJointArray start_state = ctx.read_joints();
        if (ctx.result) {
            *ctx.result = start_state;
        }  //safe guard
        return ctx.planner->beginLayer(layer_id_, start_state, ctx.read_speed_x());
    }

    linear::LayerTickResult onTick(UpStairsRunnerContext& ctx) override {
        if (!ctx.planner) {
            return linear::LayerTickResult::fault();
        }

        LegJointArray output_state = ctx.result ? *ctx.result : LegJointArray{};
        const bool layer_complete = ctx.planner->tickLayer(layer_id_, output_state);
        if (ctx.result) {
            *ctx.result = output_state;
        }
        if (!layer_complete) {
            return linear::LayerTickResult::running();
        }
        return linear::LayerTickResult::completed();
    }

    void onExit(UpStairsRunnerContext& /*ctx*/) override {}

private:
    UpLayerId layer_id_{UpLayerId::Initial};
};

class Auto_Leg_Up_Stairs {
public:
    using LayerRunner =
        linear::LinearLayerRunner<UpStairsRunnerContext, UpLayerId>;

    Auto_Leg_Up_Stairs(rmcs_executor::Component& component, const rclcpp::Logger& logger)
        : logger_(logger)
        , context_{
              &planner_,
              [this]() { return readCurrentLegJoints(); },
              [this]() { return readCurrentSpeedX(); },
              &result_}
        , layers_(makeLayers())
        , runner_([this](UpLayerId id) { return resolveLayer(id); }) {
        component.register_input("/leg/encoder/lf/angle", theta_lf_);
        component.register_input("/leg/encoder/lb/angle", theta_lb_);
        component.register_input("/leg/encoder/rb/angle", theta_rb_);
        component.register_input("/leg/encoder/rf/angle", theta_rf_);
        component.register_input("/chassis/control_velocity", speed_);
        resetResult();
    }

    void Set_One_Stairs() {
        mode_ = UpStairsMode::OneProcess;
    }

    void Set_Two_Stairs_Lift() {
        mode_ = UpStairsMode::TwoProcessLift;
    }

    void Set_Two_Stairs_Initial() {
        mode_ = UpStairsMode::TwoProcessInitial;
    }

    bool start(const UpStairsStartRequest& request) {
        if (!planner_.loaded()) {
            RCLCPP_WARN(logger_, "up stairs start failed: planner not loaded");
            return false;
        }

        const UpStairsMode resolved_mode =
            (request.mode == UpStairsMode::Invalid) ? mode_ : request.mode;
        if (resolved_mode == UpStairsMode::Invalid && request.layers.empty()) {
            RCLCPP_WARN(logger_, "up stairs start failed: mode not ready");
            return false;
        }

        const std::vector<UpLayerId> plan = buildPlan(request, resolved_mode);
        if (plan.empty()) {
            RCLCPP_WARN(logger_, "up stairs start failed: plan is empty");
            return false;
        }

        linear::StartRequest<UpLayerId> linear_request;
        linear_request.layers = plan;

        if (!runner_.start(linear_request, context_)) {
            RCLCPP_WARN(logger_, "up stairs start failed: runner rejected request");
            return false;
        }
        runner_.clearBreakpoints();
        if (plan.size() > 1 && plan.front() == UpLayerId::Initial) {
            (void)runner_.addBreakpoint(toIndex(UpLayerId::Initial)+1);
        }
        mode_ = resolved_mode;
        return true;
    }

    bool start() {
        UpStairsStartRequest request;
        request.mode = mode_;
        return start(request);
    }

    void tick() {
        runner_.tick(context_);
    }

    bool resume() {
        return runner_.resume(context_);
    }

    void stop() {
        runner_.stop();
        mode_ = UpStairsMode::Invalid;
        resetResult();
    }

    const LegJointArray& get_result() const {
        return result_;
    }

    Auto_Leg_Up_Stairs& load(
        const std::vector<double>& initial_parameter,
        const std::vector<double>& press_parameter,
        const std::vector<double>& lift_parameter,
        const std::vector<double>& lift_and_initial_parameter,
        const std::vector<double>& initial_again_parameter,
        const std::vector<double>& press_again_parameter,
        const std::vector<double>& lift_again_parameter) {
        std::vector<std::vector<double>> layer_parameters(UpLayerCount);
        layer_parameters[toIndex(UpLayerId::Initial)] = initial_parameter;
        layer_parameters[toIndex(UpLayerId::Press)] = press_parameter;
        layer_parameters[toIndex(UpLayerId::Lift)] = lift_parameter;
        layer_parameters[toIndex(UpLayerId::LiftAndInitial)] = lift_and_initial_parameter;
        layer_parameters[toIndex(UpLayerId::InitialAgain)] = initial_again_parameter;
        layer_parameters[toIndex(UpLayerId::PressAgain)] = press_again_parameter;
        layer_parameters[toIndex(UpLayerId::LiftAgain)] = lift_again_parameter;

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
    LegJointArray readCurrentLegJoints() const {
        return {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_};
    }

    double readCurrentSpeedX() const {
        return (*speed_)->x();
    }

    void resetResult() {
        result_ = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()};
    }

    bool validateLayerParameters(const std::vector<std::vector<double>>& layer_parameters) {
        if (layer_parameters.size() != UpLayerCount) {
            RCLCPP_ERROR(
                logger_,
                "up stairs load failed: layer_parameters size=%zu expected=%zu",
                layer_parameters.size(),
                UpLayerCount);
            return false;
        }

        for (std::size_t idx = 0; idx < UpLayerCount; ++idx) {
            const auto& parameter = layer_parameters[idx];
            if (parameter.size() != LegJointCount + 2) {
                RCLCPP_ERROR(
                    logger_,
                    "up stairs load failed: layer_parameters[%zu] size=%zu expected=%zu",
                    idx,
                    parameter.size(),
                    LegJointCount + 2);
                return false;
            }
        }
        return true;
    }

    static std::array<UpStairsLayer, UpLayerCount> makeLayers() {
        return {
            UpStairsLayer{UpLayerId::Initial},
            UpStairsLayer{UpLayerId::Press},
            UpStairsLayer{UpLayerId::Lift},
            UpStairsLayer{UpLayerId::LiftAndInitial},
            UpStairsLayer{UpLayerId::InitialAgain},
            UpStairsLayer{UpLayerId::PressAgain},
            UpStairsLayer{UpLayerId::LiftAgain},
        };
    }

    linear::ILayer<UpStairsRunnerContext>* resolveLayer(UpLayerId id) {
        const std::size_t idx = toIndex(id);
        if (idx >= layers_.size()) {
            return nullptr;
        }
        return &layers_[idx];
    }

    static std::vector<UpLayerId> defaultPlanForMode(UpStairsMode mode) {
        switch (mode) {
        case UpStairsMode::OneProcess:
            return {UpLayerId::Initial, UpLayerId::Press, UpLayerId::Lift};
        case UpStairsMode::TwoProcessLift:
            return {
                UpLayerId::Initial,
                UpLayerId::Press,
                UpLayerId::LiftAndInitial,
                UpLayerId::PressAgain,
                UpLayerId::LiftAgain};
        case UpStairsMode::TwoProcessInitial:
            return {
                UpLayerId::Initial,
                UpLayerId::Press,
                UpLayerId::Lift,
                UpLayerId::InitialAgain,
                UpLayerId::PressAgain,
                UpLayerId::LiftAgain};
        case UpStairsMode::Invalid:
            return {};
        }
        return {};
    }

    static std::vector<UpLayerId> buildPlan(
        const UpStairsStartRequest& request,
        UpStairsMode resolved_mode) {
        if (!request.layers.empty()) {
            return request.layers;
        }
        return defaultPlanForMode(resolved_mode);
    }

    rclcpp::Logger logger_;
    UpStairsPlanner planner_;
    LegJointArray result_{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};

    UpStairsRunnerContext context_;
    std::array<UpStairsLayer, UpLayerCount> layers_;
    LayerRunner runner_;
    UpStairsMode mode_{UpStairsMode::Invalid};

    InputDouble theta_lf_;
    InputDouble theta_lb_;
    InputDouble theta_rb_;
    InputDouble theta_rf_;
    InputVelocity speed_;
};

} // namespace rmcs_core::controller::leg::hsm::up_stairs
