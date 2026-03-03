#pragma once

#include "controller/arm/trajectory.hpp"
#include "controller/leg/hsm/HSM_dev.hpp" // 注意: 使用优化后的 HSM.hpp
#include <algorithm>
#include <cassert>
#include <limits>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

using InputD = rmcs_executor::Component::InputInterface<double>;
using InputVct =
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>;

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
    tof_already,                          // args:: double distance
    go_to_Initial_Again
};

using UpStairsEventId = std::variant<UpStairsEventEnum, std::string>;

namespace rmcs_core::controller::leg::hsm::up_stairs_hsm {

namespace events {
using UpStairsEvent = Event<UpStairsEventId>;
const UpStairsEvent tick{UpStairsEventId{UpStairsEventEnum::tick}, {}};

const UpStairsEvent tof_already{UpStairsEventId{UpStairsEventEnum::tof_already}, {}};

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
};

class Auto_Leg_Up_Stairs : public rclcpp::Node {
public:
    explicit Auto_Leg_Up_Stairs(rmcs_executor::Component& state_component)
        : context_()
        , up_stairs_hsm(context_)
        , Node{"auto_leg_up_stairs"} {
        state_component.register_input("/leg/encoder/lf/angle", theta_lf_);
        state_component.register_input("/leg/encoder/lb/angle", theta_lb_);
        state_component.register_input("/leg/encoder/rb/angle", theta_rb_);
        state_component.register_input("/leg/encoder/rf/angle", theta_rf_);
        state_component.register_input("/chassis/control_velocity", speed_);
        is_one_process_         = false;
        is_two_process_lift_    = false;
        is_two_process_initial_ = false;
    }

    void Set_One_Stairs() {
        is_one_process_         = true;
        is_two_process_lift_    = false;
        is_two_process_initial_ = false;
    }

    void Set_Two_Stairs_Lift() {
        is_one_process_         = false;
        is_two_process_lift_    = true;
        is_two_process_initial_ = false;
    }

    void Set_Two_Stairs_Initial() {
        is_one_process_         = false;
        is_two_process_lift_    = false;
        is_two_process_initial_ = true;
    }

    void processEvent(const Event<UpStairsEventId>& event) {
        // 处理事件ID为variant: 使用std::visit
        std::visit(
            [&](auto&& id) {
                using T = std::decay_t<decltype(id)>;

                if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                    // RCLCPP_INFO(this->get_logger(), "Processing event: %d",
                    // static_cast<int>(id));
                } else if constexpr (std::is_same_v<T, std::string>) {
                    // RCLCPP_INFO(this->get_logger(), "Processing custom event: %s", id.c_str());
                }
            },
            event.id);
        up_stairs_hsm.processEvent(event);
    }

    void start(UpStairsState initial, const EventArgs& args = {}) {
        check_mode_ready();
        if(mode_valid && load_valid) {
            up_stairs_hsm.start(initial, args);
        }
        else {
            RCLCPP_WARN(this->get_logger(), "hsm start failed!");
        }
    }

    void stop() { up_stairs_hsm.stop();
        mode_valid=false;
        result_ = {
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
    }

    void check_mode_ready() {
        int count = static_cast<int>(is_one_process_) + static_cast<int>(is_two_process_lift_)
                  + static_cast<int>(is_two_process_initial_);

        if (count != 1) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(),
                1000, // 毫秒
                "mode not ready");
         mode_valid=false;   
        }
        else {
            mode_valid=true;
        }
    }

    std::array<double, 4>& get_result() { return result_; }

    Auto_Leg_Up_Stairs& load(
        const std::vector<double>& initial_end_point_, const std::vector<double>& press_end_point_,
        const std::vector<double>& lift_end_point_,
        const std::vector<double>& lift_and_initial_end_point_,
        const std::vector<double>& initial_again_end_point_, std::span<const double> k,
        std::span<const double> b) {
        load_valid = false;

        if (initial_end_point_.size() != 4 || press_end_point_.size() != 4
            || lift_end_point_.size() != 4 || lift_and_initial_end_point_.size() != 4
            || initial_again_end_point_.size() != 4) {
            RCLCPP_ERROR(
                this->get_logger(),
                "up stairs load failed: endpoint sizes must all be 4, got initial=%zu press=%zu "
                "lift=%zu lift_and_initial=%zu initial_again=%zu",
                initial_end_point_.size(), press_end_point_.size(), lift_end_point_.size(),
                lift_and_initial_end_point_.size(), initial_again_end_point_.size());
            return *this;
        }

        if (k.size() != 4 || b.size() != 4) {
            RCLCPP_ERROR(
                this->get_logger(), "up stairs load failed: k size=%zu, b size=%zu, expected 4",
                k.size(), b.size());
            return *this;
        }

        this->k_ = k;
        this->b_ = b;
        up_stairs_initial.set_end_point(
            std::vector<double>{
                initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
                initial_end_point_[3]});
        up_stairs_leg_press.set_end_point(
            std::vector<double>{
                press_end_point_[0], press_end_point_[1], press_end_point_[2],
                press_end_point_[3]});
        up_stairs_leg_lift.set_end_point(
            std::vector<double>{
                lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3]});
        up_stairs_leg_lift_and_initial.set_end_point(
            std::vector<double>{
                lift_and_initial_end_point_[0], lift_and_initial_end_point_[1],
                lift_and_initial_end_point_[2], lift_and_initial_end_point_[3]});
        up_stairs_leg_initial_again.set_end_point(
            std::vector<double>{
                initial_again_end_point_[0], initial_again_end_point_[1],
                initial_again_end_point_[2], initial_again_end_point_[3]});

        // 注册状态使用BasicState和lambda

        // InitialState
        auto initialState =
            std::make_unique<BasicState<UpStairsState, UpStairsEventId, UpStairsContext>>(
                UpStairsState::Initial,
                [&](UpStairsContext& ctx, const EventArgs&) {
                    RCLCPP_INFO(this->get_logger(), " in initial");
                    up_stairs_initial.reset();
                    up_stairs_initial.set_start_point(
                        {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                    up_stairs_initial.set_total_step(1000);
                },
                nullptr, // no exit
                [&](const Event<UpStairsEventId>& event,
                    UpStairsContext& ctx) -> std::optional<UpStairsState> {
                    return std::visit(

                        [&](auto&& id) -> std::optional<UpStairsState> {
                            using T = std::decay_t<decltype(id)>;
                            if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                                if (id == UpStairsEventEnum::tick) {

                                    if (!up_stairs_initial.get_complete()) {
                                        const auto joints = up_stairs_initial.trajectory();
                                        std::copy(joints.begin(), joints.end(), result_.begin());
                                        return UpStairsState::Initial;
                                    } else {
                                        return std::nullopt;
                                    }
                                } else if (id == UpStairsEventEnum::tof_already) {
                                    if (is_one_process_ && !is_two_process_initial_
                                        && !is_two_process_lift_) {
                                        return UpStairsState::StepByOne;
                                    } else if (
                                        !is_one_process_ && !is_two_process_initial_
                                        && is_two_process_lift_) {
                                        return UpStairsState::StepByTwo_lift;
                                    } else if (
                                        !is_one_process_ && is_two_process_initial_
                                        && !is_two_process_lift_) {
                                        return UpStairsState::StepByTwo_initial;
                                    }
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
        load_valid = true;
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
                up_stairs_leg_press.reset();
                up_stairs_leg_press.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_press.set_total_step(calculate_steps(k_[0], b_[0]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_press.get_complete()) {
                                    const auto joints = up_stairs_leg_press.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
                                    return StepSubState::Press;
                                } else {
                                    if (is_one_process_) {
                                        return StepSubState::Lift;
                                    } else if (is_two_process_initial_) {
                                        return StepSubState::Lift;
                                    } else if (is_two_process_lift_) {
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
                up_stairs_leg_lift_and_initial.reset();
                up_stairs_leg_lift_and_initial.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_lift_and_initial.set_total_step(calculate_steps(k_[3], b_[3]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_lift_and_initial.get_complete()) {
                                    const auto joints = up_stairs_leg_lift_and_initial.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
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
                up_stairs_leg_lift.reset();
                up_stairs_leg_lift.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_lift.set_total_step(calculate_steps(k_[1], b_[1]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_lift.get_complete()) {
                                    const auto joints = up_stairs_leg_lift.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
                                    return StepSubState::Lift;
                                } else {
                                    if (is_one_process_) {
                                        return StepSubState::Wait;
                                    } else if (is_two_process_initial_) {
                                        return StepSubState::InitialAgain;
                                    } else if (is_two_process_lift_) {
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
                up_stairs_leg_initial_again.reset();
                up_stairs_leg_initial_again.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_initial_again.set_total_step(calculate_steps(k_[2], b_[2]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_initial_again.get_complete()) {
                                    const auto joints = up_stairs_leg_initial_again.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
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
                up_stairs_leg_press.reset();
                up_stairs_leg_press.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_press.set_total_step(calculate_steps(k_[0], b_[0]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_press.get_complete()) {
                                    const auto joints = up_stairs_leg_press.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
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
                up_stairs_leg_lift.reset();
                up_stairs_leg_lift.set_start_point(
                    {*theta_lf_, *theta_lb_, *theta_rb_, *theta_rf_});
                up_stairs_leg_lift.set_total_step(calculate_steps(k_[1], b_[1]));
            },
            nullptr,
            [&](const Event<UpStairsEventId>& event,
                UpStairsContext& ctx) -> std::optional<StepSubState> {
                return std::visit(
                    [&](auto&& id) -> std::optional<StepSubState> {
                        using T = std::decay_t<decltype(id)>;
                        if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
                            if (id == UpStairsEventEnum::tick) {
                                if (!up_stairs_leg_lift.get_complete()) {
                                    const auto joints = up_stairs_leg_lift.trajectory();
                                    std::copy(joints.begin(), joints.end(), result_.begin());
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

    int calculate_steps(double k, double b) {
        double raw = k * ((*speed_)->x() - v_reference) + b;
        return (std::max(static_cast<int>(raw), 200));
    }

    double v_reference = 1.5;

    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        up_stairs_initial{4};
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        up_stairs_leg_press{4};
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        up_stairs_leg_lift{4};
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        up_stairs_leg_lift_and_initial{4};
    rmcs_core::controller::arm::Trajectory<rmcs_core::controller::arm::TrajectoryType::JOINT>
        up_stairs_leg_initial_again{4};

    std::array<double, 4> result_{
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};

    rmcs_executor::Component::InputInterface<double> theta_lf_;
    rmcs_executor::Component::InputInterface<double> theta_lb_;
    rmcs_executor::Component::InputInterface<double> theta_rb_;
    rmcs_executor::Component::InputInterface<double> theta_rf_;
    rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector> speed_;

    std::span<const double> k_{};
    std::span<const double> b_{};

    bool is_one_process_;
    bool is_two_process_lift_;
    bool is_two_process_initial_;

    bool load_valid{false};
    bool mode_valid{false};

    UpStairsContext context_;
    HSM<UpStairsState, UpStairsEventId, UpStairsContext> up_stairs_hsm;
};

} // namespace rmcs_core::controller::leg::hsm::up_stairs_hsm
