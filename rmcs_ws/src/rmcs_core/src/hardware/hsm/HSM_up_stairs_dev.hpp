#ifndef HSM_UP_TWO_STAIRS_HPP_H_
#define HSM_UP_TWO_STAIRS_HPP_H_

#include "hardware/device/trajectory.hpp"
#include "hardware/hsm/HSM_dev.hpp" // 注意: 使用优化后的 HSM.hpp
#include <cassert>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

using InputD = rmcs_executor::Component::InputInterface<double>*;
using InputVct =
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>*;

enum class UpStairsState { Initial, StepByOne, StepByTwo_initial, StepByTwo_lift };

enum class StepSubState {
    Wait,
    Press,
    Lift,
    LiftAndInitial,
    InitialAgain,
    Delay,
    PressAgain,
    LiftAgain
};

enum class UpStairsEventEnum {
    quit,
    stop,
    tick,
    go_to_OneProcess,
    go_to_TwoProcess_lift,
    go_to_TwoProcess_initial,
    go_to_Lift_And_Initial,
    go_to_Press,
    go_to_Lift,
    tof_already,                    // args:: double distance
    go_to_Initial_Again
};

using UpStairsEventId = std::variant<UpStairsEventEnum, std::string>;

namespace rmcs_core::hardware::hsm::up_stairs_hsm {

namespace events {
using UpStairsEvent = Event<UpStairsEventId>;
const UpStairsEvent tick{UpStairsEventId{UpStairsEventEnum::tick}, {}};

const UpStairsEvent quit{UpStairsEventId{UpStairsEventEnum::quit}, {}};

const UpStairsEvent stop{UpStairsEventId{UpStairsEventEnum::stop}, {}};

const UpStairsEvent go_to_OneProcess{UpStairsEventId{UpStairsEventEnum::go_to_OneProcess}, {}};

const UpStairsEvent go_to_TwoProcess_lift{
    UpStairsEventId{UpStairsEventEnum::go_to_TwoProcess_lift}, {}};

const UpStairsEvent go_to_TwoProcess_initial{
    UpStairsEventId{UpStairsEventEnum::go_to_TwoProcess_initial}, {}};

const UpStairsEvent go_to_Lift_And_Initial{
    UpStairsEventId{UpStairsEventEnum::go_to_Lift_And_Initial}, {}};

const UpStairsEvent go_to_Press{UpStairsEventId{UpStairsEventEnum::go_to_Press}, {}};

const UpStairsEvent go_to_Lift{UpStairsEventId{UpStairsEventEnum::go_to_Lift}, {}};

const UpStairsEvent go_to_Initial_Again{
    UpStairsEventId{UpStairsEventEnum::go_to_Initial_Again}, {}};

} // namespace events

struct UpStairsContext {
    std::unordered_map<std::string, std::any> data; // 动态存储所有依赖

    // Helper getters
    template <typename T>
    T& get(const std::string& key) {
        auto it = data.find(key);
        if (it == data.end()) {
            throw std::runtime_error("Key not found: " + key);
        }
        if (it->second.type() != typeid(std::reference_wrapper<T>)) {
            throw std::runtime_error(
                "Type mismatch for key: " + key + ", stored=" + it->second.type().name()
                + ", requested=" + typeid(T).name());
        }
        return std::any_cast<std::reference_wrapper<T>>(it->second).get();
    }

    // 示例: 获取 Trajectory
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>&
        getTrajectory(const std::string& key) {
        return get<hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>>(key);
    }

    // 获取 InputD*
    InputD& getInputD(const std::string& key) { return get<InputD>(key); }

    // 获取 InputVct*
    InputVct& getInputVct(const std::string& key) { return get<InputVct>(key); }

    // 其他: k, b, result 等
    std::vector<double>& getVec(const std::string& key) { return get<std::vector<double>>(key); }

    std::array<double, 6>& getResult() { return get<std::array<double, 6>>("result"); }

    static constexpr double v_reference = 1.5;
    bool is_one_process                 = false;
    bool is_two_process_lift            = false;
    bool is_two_process_initial         = false;
    int calculate_steps(double k, double b) {
        double raw = k * ((**getInputVct("speed"))->x() - v_reference) + b;
        return (std::max(static_cast<int>(raw), 200));
    }
};

class Auto_Leg_Up_Two_Stairs : public rclcpp::Node {
public:
    explicit Auto_Leg_Up_Two_Stairs()
        : context_()
        , up_stairs_hsm(context_)
        , Node{"auto_leg_up_stairs"} {}

    void processEvent(const Event<UpStairsEventId>& event) {
        // 处理事件ID为variant: 使用std::visit
        std::visit(
            [&](auto&& id) {
                using T = std::decay_t<decltype(id)>;

                if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                    //RCLCPP_INFO(this->get_logger(), "Processing event: %d", static_cast<int>(id));
                } else if constexpr (std::is_same_v<T, std::string>) {
                    //RCLCPP_INFO(this->get_logger(), "Processing custom event: %s", id.c_str());
                }
            },
            event.id);
        up_stairs_hsm.processEvent(event);
    }

    void start(UpStairsState initial, const EventArgs& args = {}) {
        check_context_ready();
        up_stairs_hsm.start(initial, args);
    }

    void stop() { up_stairs_hsm.stop(); }

    void check_context_ready() const {
        // 动态检查关键key是否存在
        const std::vector<std::string> required_keys = {"theta_lf", "theta_lb", "theta_rb",
                                                        "theta_rf", "speed",    "result"};
        for (const auto& key : required_keys) {
            if (context_.data.find(key) == context_.data.end()) {
                throw std::runtime_error(key + " not bound");
            }
        }
    }

    template <typename T>
    Auto_Leg_Up_Two_Stairs& bind(const std::string& key, T& value) {
        context_.data[key] = std::ref(value);
        return *this;
    }

    std::array<double, 6>& get_result() { return context_.getResult(); }

    Auto_Leg_Up_Two_Stairs& init_and_trajectory_set(
        const std::vector<double>& initial_end_point_, const std::vector<double>& press_end_point_,
        const std::vector<double>& lift_end_point_,
        const std::vector<double>& lift_and_initial_end_point_,
        const std::vector<double>& initial_again_end_point_) {

        // 绑定轨迹 (作为成员变量，但现在动态bind)
        bind("initial", up_stairs_initial)
            .bind("press", up_stairs_leg_press)
            .bind("lift", up_stairs_leg_lift)
            .bind("lift_and_initial", up_stairs_leg_lift_and_initial)
            .bind("initial_again", up_stairs_leg_initial_again);

        // 设置端点
        up_stairs_initial.set_end_point(
            {initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
             initial_end_point_[3], 0, 0});
        up_stairs_leg_press.set_end_point(
            {press_end_point_[0], press_end_point_[1], press_end_point_[2], press_end_point_[3], 0,
             0});
        up_stairs_leg_lift.set_end_point(
            {lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3], 0, 0});
        up_stairs_leg_lift_and_initial.set_end_point(
            {lift_and_initial_end_point_[0], lift_and_initial_end_point_[1],
             lift_and_initial_end_point_[2], lift_and_initial_end_point_[3], 0, 0});
        up_stairs_leg_initial_again.set_end_point(
            {initial_again_end_point_[0], initial_again_end_point_[1], initial_again_end_point_[2],
             initial_again_end_point_[3], 0, 0});

        // 注册状态使用BasicState和lambda

        // InitialState
        auto initialState =
            std::make_unique<BasicState<UpStairsState, UpStairsEventId, UpStairsContext>>(
                UpStairsState::Initial,
                [&](UpStairsContext& ctx, const EventArgs&) {
                    RCLCPP_INFO(this->get_logger(), " in initial");
                    auto& traj = ctx.getTrajectory("initial");

                    traj.reset();
                    traj.set_start_point(
                        {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                         **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                    traj.set_total_step(1000);
                },
                nullptr, // no exit
                [&](const Event<UpStairsEventId>& event,
                    UpStairsContext& ctx) -> std::optional<UpStairsState> {
                    return std::visit(

                        [&](auto&& id) -> std::optional<UpStairsState> {
                            using T = std::decay_t<decltype(id)>;
                            if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                                if (id == UpStairsEventEnum::tick) {
                                    auto& traj = ctx.getTrajectory("initial");
                                    if (!traj.get_complete()) {
                                        ctx.getResult() = traj.trajectory();
                                        return UpStairsState::Initial;
                                    } else {
                                        return std::nullopt;
                                    }
                                } else if (id == UpStairsEventEnum::go_to_TwoProcess_lift) {
                                    ctx.is_one_process         = false;
                                    ctx.is_two_process_lift    = true;
                                    ctx.is_two_process_initial = false;
                                    return UpStairsState::StepByTwo_lift;
                                } else if (id == UpStairsEventEnum::go_to_TwoProcess_initial) {
                                    ctx.is_one_process         = false;
                                    ctx.is_two_process_lift    = false;
                                    ctx.is_two_process_initial = true;
                                    return UpStairsState::StepByTwo_initial;
                                } else if (id == UpStairsEventEnum::go_to_OneProcess) {
                                    ctx.is_one_process         = true;
                                    ctx.is_two_process_lift    = false;
                                    ctx.is_two_process_initial = false;
                                    return UpStairsState::StepByOne;
                                }
                            }
                            return std::nullopt;
                        },
                        event.id);
                });
        up_stairs_hsm.registerState(std::move(initialState));

        // StepByOneState (Composite)
        auto stepByOne = std::make_unique<
            CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext>>(
            UpStairsState::StepByOne, StepSubState::Press);

        // 添加子状态动态

        stepByOne->addSubState(createPressState());
        stepByOne->addSubState(createLiftState());
        stepByOne->addSubState(createWaitState());
        up_stairs_hsm.registerState(std::move(stepByOne));

        // StepByTwoState (Composite)
        auto stepByTwo_initial = std::make_unique<
            CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext>>(
            UpStairsState::StepByTwo_initial, StepSubState::Press);

        stepByTwo_initial->addSubState(createPressState());
        stepByTwo_initial->addSubState(createLiftState());
        stepByTwo_initial->addSubState(createInitialAgainState());
        stepByTwo_initial->addSubState(createPressAgainState());
        stepByTwo_initial->addSubState(createLiftAgainState());
        stepByTwo_initial->addSubState(createWaitState());
        up_stairs_hsm.registerState(std::move(stepByTwo_initial));

        auto stepByTwo_lift = std::make_unique<
            CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext>>(
            UpStairsState::StepByTwo_lift, StepSubState::Press);

        stepByTwo_lift->addSubState(createPressState());
        stepByTwo_lift->addSubState(createLiftAndInitialState());
        stepByTwo_lift->addSubState(createPressAgainState());
        stepByTwo_lift->addSubState(createLiftAgainState());
        stepByTwo_lift->addSubState(createWaitState());
        up_stairs_hsm.registerState(std::move(stepByTwo_lift));
        return *this;
    }

private:
    // Helper functions to create sub states
    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createWaitState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::Wait,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), " in wait");
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::go_to_Lift_And_Initial) {
                                return StepSubState::LiftAndInitial;
                            } else if (id == UpStairsEventEnum::go_to_Press) {
                                return StepSubState::Press;
                            } else if (id == UpStairsEventEnum::go_to_Lift) {
                                return StepSubState::Lift;
                            } else if (id == UpStairsEventEnum::tick) {
                                return StepSubState::Wait;
                            } else if (id == UpStairsEventEnum::go_to_Initial_Again) {
                                return StepSubState::InitialAgain;
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createDelayState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::Delay,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), "in delay");
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::go_to_Lift_And_Initial) {
                                return StepSubState::LiftAndInitial;
                            } else if (id == UpStairsEventEnum::go_to_Press) {
                                return StepSubState::Press;
                            } else if (id == UpStairsEventEnum::go_to_Lift) {
                                return StepSubState::Lift;
                            } else if (id == UpStairsEventEnum::tick) {
                                return StepSubState::Wait;
                            } else if (id == UpStairsEventEnum::go_to_Initial_Again) {
                                return StepSubState::InitialAgain;
                            } else if (id == UpStairsEventEnum::go_to_Lift_And_Initial) {
                                return StepSubState::LiftAndInitial;
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createPressState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::Press,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), "in press");
                auto& traj = ctx.getTrajectory("press");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[0], ctx.getVec("b")[0]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("press");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::Press;
                                } else {
                                    if (ctx.is_one_process) {
                                        return StepSubState::Lift;
                                    } else if (ctx.is_two_process_initial) {
                                        return StepSubState::Lift;
                                    } else if (ctx.is_two_process_lift) {
                                        return StepSubState::LiftAndInitial;
                                    }
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }
    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>>
        createLiftAndInitialState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::LiftAndInitial,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), " in lift_and_initial");
                auto& traj = ctx.getTrajectory("lift_and_initial");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[3], ctx.getVec("b")[3]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("lift_and_initial");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::LiftAndInitial;
                                } else {
                                    // return StepSubState::Wait;
                                    return StepSubState::PressAgain;
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createLiftState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::Lift,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), " in lift");
                auto& traj = ctx.getTrajectory("lift");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[1], ctx.getVec("b")[1]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("lift");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::Lift;
                                } else {
                                    if (ctx.is_one_process) {
                                        return StepSubState::Wait;
                                    } else if (ctx.is_two_process_initial) {
                                        return StepSubState::InitialAgain;
                                    } else if (ctx.is_two_process_lift) {
                                        RCLCPP_INFO(
                                            this->get_logger(),
                                            "wrongly enter lift in two_process_lift");
                                    }
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>>
        createInitialAgainState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::InitialAgain,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), "in initial_again");
                auto& traj = ctx.getTrajectory("initial_again");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[2], ctx.getVec("b")[2]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("initial_again");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::InitialAgain;
                                } else {
                                    return StepSubState::PressAgain;
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>>
        createPressAgainState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::PressAgain,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), " in press again");
                auto& traj = ctx.getTrajectory("press");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[0], ctx.getVec("b")[0]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("press");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::PressAgain;
                                } else {
                                    return StepSubState::LiftAgain;
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createLiftAgainState() {
        return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
            StepSubState::LiftAgain,
            [&](UpStairsContext& ctx, const EventArgs&) {
                RCLCPP_INFO(this->get_logger(), " in lift again");
                auto& traj = ctx.getTrajectory("lift");
                traj.reset();
                traj.set_start_point(
                    {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
                     **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
                traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[1], ctx.getVec("b")[1]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                auto& traj = ctx.getTrajectory("lift");
                                if (!traj.get_complete()) {
                                    ctx.getResult() = traj.trajectory();
                                    return StepSubState::LiftAgain;
                                } else {
                                    return StepSubState::Wait;
                                }
                            }
                        }
                        return std::nullopt;
                    },
                    event.id);
            });
    }

    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_initial;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_press;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_lift;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
        up_stairs_leg_lift_and_initial;
    hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
        up_stairs_leg_initial_again;

    UpStairsContext context_;
    HSM<UpStairsState, UpStairsEventId, UpStairsContext> up_stairs_hsm;
};

} // namespace rmcs_core::hardware::hsm::up_stairs_hsm

#endif // HSM_UP_TWO_STAIRS_HPP_H_