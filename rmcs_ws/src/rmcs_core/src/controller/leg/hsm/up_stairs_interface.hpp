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
#include <map>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmcs_core::controller::leg::hsm::up_stairs {

inline constexpr std::size_t LegJointCount       = 4;
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

    void reset() { trajectory_.reset(); }

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
        decodeLayerParameter(*parameter, end_point, k, b);

        trajectory_.reset();
        trajectory_.set_start_point(toVector(start_joints));
        trajectory_.set_end_point(toVector(end_point));
        trajectory_.set_total_step(calculateSteps(k, b, speed_x));
        return true;
    }

    bool tickLayer(std::array<double, LegJointCount>& out_joints) {
        if (trajectory_.get_complete()) {
            return true;
        }

        const auto joints = trajectory_.trajectory();
        std::copy(joints.begin(), joints.end(), out_joints.begin());
        return false;
    }

private:
    static constexpr int MinStep       = 200;
    static constexpr int MaxStep       = 2500;
    static constexpr double VReference = 2.0;

    static void decodeLayerParameter(
        const std::vector<double>& parameter, std::array<double, LegJointCount>& end_point,
        double& k, double& b) {
        if (parameter.size() == FullLayerParamCount) {
            std::copy_n(parameter.begin(), LegJointCount, end_point.begin());
            k = parameter[LegJointCount];
            b = parameter[LegJointCount + 1];
        }
        if (parameter.size() == HalfLayerParamCount) {
            end_point[0] = parameter[0];
            end_point[1] = parameter[1];
            end_point[2] = parameter[1];
            end_point[3] = parameter[0];
            k            = parameter[2];
            b            = parameter[3];
        }
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
        return &(it->second);
    }

    LayerParameters layer_parameters_{};
    bool loaded_{false};
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
        const bool layer_complete = ctx.planner->tickLayer(output_state);
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

class Auto_Leg_Up_Stairs {
public:
    using LayerRunner = linear::LinearLayerRunner<UpStairsRunnerContext, LayerId>;

    Auto_Leg_Up_Stairs(
        rmcs_executor::Component& component, std::string_view name,
        std::vector<std::string_view> layer_ids)
        : context_{
              &planner_, [this]() { return readCurrentLegJoints(); },
              [this]() { return readCurrentSpeedX(); }, &result_}
        , runner_([this](LayerId id) { return resolveLayer(id); }) {
        resetResult();
        component.register_input("/leg/encoder/lf/angle", theta_lf_);
        component.register_input("/leg/encoder/lb/angle", theta_lb_);
        component.register_input("/leg/encoder/rb/angle", theta_rb_);
        component.register_input("/leg/encoder/rf/angle", theta_rf_);
        component.register_input("/chassis/control_velocity", speed_);
        plan_.clear();
        layers_.clear();
        plan_.assign(layer_ids.begin(), layer_ids.end());

        layers_.reserve(plan_.size());
        for (const auto& id : plan_) {
            layers_.emplace_back(id);
        }

        const auto load_from_component = [this, &component, &name]() {
            auto* node = dynamic_cast<rclcpp::Node*>(&component);
            if (node == nullptr) {
                resetResult();
                throw std::runtime_error{
                    "Failed to load up stairs layer parameters: component is not an rclcpp::Node."};
            }

            std::map<std::string, rclcpp::Parameter> raw;
            const std::string prefix = std::string(name) + ".params";
            const bool loaded =
                node->get_node_parameters_interface()->get_parameters_by_prefix(prefix, raw);

            if (!loaded || raw.empty()) {
                resetResult();
                throw std::runtime_error{
                    "Failed to load up stairs layer parameters: no parameters found with prefix '"
                    + prefix + "'."};
            }

            LayerParameters parsed;
            parsed.reserve(plan_.size());
            for (const auto& layer_id : plan_) {
                const auto iter = raw.find(layer_id);
                if (iter == raw.end()) {
                    resetResult();
                    throw std::runtime_error{
                        "Failed to load up stairs layer parameters: missing parameters for layer '"
                        + layer_id + "'."};
                }
                const auto parameter = (iter->second).as_double_array();
                if (parameter.size() != FullLayerParamCount
                    && parameter.size() != HalfLayerParamCount) {
                    resetResult();
                    throw std::runtime_error{
                        "failed to load up stairs layer parameters: invalid parameter size for "
                        "layer "
                        + layer_id};
                }
                parsed.emplace(layer_id, parameter);
            }
            planner_.load(parsed);
        };

        load_from_component();
    }

    void start() { runner_.start(plan_); }

    void set_layer_connections(const LayerId& id, std::function<bool()> func) {
        runner_.add_connection(id, std::move(func));
    }

    void tick() {
        if (runner_.status() == linear::RunnerStatus::Fault) {
            throw std::runtime_error{
                "haven't load planner or context correctly "};
        }
        runner_.tick(context_);
    }

    void clearLayerConnections() { runner_.clearConnections(); }

    std::optional<LayerId> get_current_layer_id() const { return runner_.current_layer_id(); }

    const std::array<double, LegJointCount>& get_result() const { return result_; }

private:
    void resetResult() {
        result_ = {
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }
    std::array<double, LegJointCount> readCurrentLegJoints() const {
        return {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_};
    }

    double readCurrentSpeedX() const { return (*speed_)->x(); }

    linear::ILayer<UpStairsRunnerContext>* resolveLayer(const LayerId& id) {
        auto it = std::find_if(layers_.begin(), layers_.end(), [id](const UpStairsLayer& layer) {
            return layer.getLayerId() == id;
        });

        return (it == layers_.end()) ? nullptr : &(*it);
    }

    UpStairsPlanner planner_;
    std::array<double, LegJointCount> result_{};
    UpStairsRunnerContext context_{};
    std::vector<UpStairsLayer> layers_{};
    std::vector<LayerId> plan_{};

    rmcs_executor::Component::InputInterface<double> theta_lf_;
    rmcs_executor::Component::InputInterface<double> theta_lb_;
    rmcs_executor::Component::InputInterface<double> theta_rb_;
    rmcs_executor::Component::InputInterface<double> theta_rf_;
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector> speed_;

    LayerRunner runner_;
};

} // namespace rmcs_core::controller::leg::hsm::up_stairs
