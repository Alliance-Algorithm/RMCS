// #ifndef HSM_UP_STAIRS_HPP_H_
// #define HSM_UP_STAIRS_HPP_H_

// #include "hardware/device/trajectory.hpp"
// #include "hardware/hsm/HSM_plus.hpp" // 注意: 使用优化后的 HSM.hpp
// #include <cassert>
// #include <optional>
// #include <rmcs_description/tf_description.hpp>
// #include <rmcs_executor/component.hpp>
// #include <string>
// #include <type_traits>
// #include <unordered_map>
// #include <variant>
// #include <vector>

// using InputD = rmcs_executor::Component::InputInterface<double>*;
// using InputVct =rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>*;

// enum class UpStairsState {
//     Initial,
//     StepByOne,
//     StepByTwo,
// };

// enum class StepSubState { Wait, Press, Lift, PressAndLift, InitialAgain,Delay };

// enum class UpStairsEventEnum {
//     quit,
//     stop,
//     tick,
//     go_to_OneProcess,
//     go_to_TwoProcess,
//     go_to_Press_And_Lift,
//     go_to_Press,
//     go_to_Lift,
//     tof_already,                     // args:: double distance
//     go_to_Initial_Again
// };

// using UpStairsEventId = std::variant<UpStairsEventEnum, std::string>;

// namespace rmcs_core::hardware::hsm::up_stairs_hsm {

// namespace events {
// using UpStairsEvent = Event<UpStairsEventId>;
// const UpStairsEvent tick{UpStairsEventId{UpStairsEventEnum::tick}, {}};

// const UpStairsEvent quit{UpStairsEventId{UpStairsEventEnum::quit}, {}};

// const UpStairsEvent stop{UpStairsEventId{UpStairsEventEnum::stop}, {}};

// const UpStairsEvent go_to_OneProcess{UpStairsEventId{UpStairsEventEnum::go_to_OneProcess}, {}};

// const UpStairsEvent go_to_TwoProcess{UpStairsEventId{UpStairsEventEnum::go_to_TwoProcess}, {}};

// const UpStairsEvent go_to_Press_And_Lift{
//     UpStairsEventId{UpStairsEventEnum::go_to_Press_And_Lift}, {}};

// const UpStairsEvent go_to_Press{UpStairsEventId{UpStairsEventEnum::go_to_Press}, {}};

// const UpStairsEvent go_to_Lift{UpStairsEventId{UpStairsEventEnum::go_to_Lift}, {}};

// const UpStairsEvent go_to_Initial_Again{UpStairsEventId{UpStairsEventEnum::go_to_Initial_Again}, {}};

// } // namespace events

// struct UpStairsContext {
//     std::unordered_map<std::string, std::any> data; // 动态存储所有依赖

//     // Helper getters
//     template <typename T>
//     T& get(const std::string& key) {
//         auto it = data.find(key);
//         if (it == data.end()) {
//             throw std::runtime_error("Key not found: " + key);
//         }
//         return std::any_cast<std::reference_wrapper<T>>(it->second).get();
//     }

//     // 示例: 获取 Trajectory
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>&
//         getTrajectory(const std::string& key) {
//         return get<hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>>(key);
//     }

//     // 获取 InputD*
//     InputD& getInputD(const std::string& key) { return get<InputD>(key); }

//     // 获取 InputVct*
//     InputVct& getInputVct(const std::string& key) { return get<InputVct>(key); }

//     // 其他: k, b, result 等
//     std::vector<double>& getVec(const std::string& key) { return get<std::vector<double>>(key); }

//     std::array<double, 6>& getResult() { return get<std::array<double, 6>>("result"); }

//     static constexpr double v_reference = 1.5;

//     int calculate_steps(double k, double b) {
//         double raw  = k * ((**getInputVct("speed"))->x() - v_reference) + b;
//         return static_cast<int>(raw);
//     }
// };

// class Auto_Leg_Up_Stairs : public rclcpp::Node {
// public:
//     explicit Auto_Leg_Up_Stairs()
//         : context_()
//         , up_stairs_hsm(context_)
//         , Node{"auto_leg_up_stairs"} {}

//     void processEvent(const Event<UpStairsEventId>& event) {
//         // 处理事件ID为variant: 使用std::visit
//         std::visit(
//             [&](auto&& id) {
//                 using T = std::decay_t<decltype(id)>;

//                 if constexpr (std::is_same_v<T, UpStairsEventEnum>) {
//                     RCLCPP_INFO(this->get_logger(), "Processing event: %d", static_cast<int>(id));
//                 } else if constexpr (std::is_same_v<T, std::string>) {
//                     RCLCPP_INFO(this->get_logger(), "Processing custom event: %s", id.c_str());
//                 }
//             },
//             event.id);
//         up_stairs_hsm.processEvent(event);
//     }

//     void start(UpStairsState initial, const EventArgs& args = {}) {
//         check_context_ready();
//         up_stairs_hsm.start(initial, args);
//     }

//     void stop() { up_stairs_hsm.stop(); }

//     void check_context_ready() const {
//         // 动态检查关键key是否存在
//         const std::vector<std::string> required_keys = {"theta_lf", "theta_lb", "theta_rb",
//                                                         "theta_rf", "speed",    "result"};
//         for (const auto& key : required_keys) {
//             if (context_.data.find(key) == context_.data.end()) {
//                 throw std::runtime_error(key + " not bound");
//             }
//         }
//     }

//     template <typename T>
//     Auto_Leg_Up_Stairs& bind(const std::string& key, T& value) {
//         context_.data[key] = std::ref(value);
//         return *this;
//     }

//     std::array<double, 6>& get_result() { return context_.getResult(); }

//     Auto_Leg_Up_Stairs& init_and_trajectory_set(
//         const std::vector<double>& initial_end_point_, const std::vector<double>& press_end_point_,
//         const std::vector<double>& lift_end_point_,
//         const std::vector<double>& press_and_lift_end_point_,
//         const std::vector<double>& initial_again_end_point_) {

//         // 绑定轨迹 (作为成员变量，但现在动态bind)
//         bind("initial", up_stairs_initial)
//             .bind("press", up_stairs_leg_press)
//             .bind("lift", up_stairs_leg_lift)
//             .bind("press_and_lift", up_stairs_leg_press_and_lift)
//             .bind("initial_again", up_stairs_leg_initial_again);

//         // 设置端点
//         up_stairs_initial.set_end_point(
//             {initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
//              initial_end_point_[3], 0, 0});
//         up_stairs_leg_press.set_end_point(
//             {press_end_point_[0], press_end_point_[1], press_end_point_[2], press_end_point_[3], 0,
//              0});
//         up_stairs_leg_lift.set_end_point(
//             {lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3], 0, 0});
//         up_stairs_leg_press_and_lift.set_end_point(
//             {press_and_lift_end_point_[0], press_and_lift_end_point_[1],
//              press_and_lift_end_point_[2], press_and_lift_end_point_[3], 0, 0});
//         up_stairs_leg_initial_again.set_end_point(
//             {initial_again_end_point_[0], initial_again_end_point_[1], initial_again_end_point_[2],
//              initial_again_end_point_[3], 0, 0});

//         // 注册状态使用BasicState和lambda

//         // InitialState
//         auto initialState =
//             std::make_unique<BasicState<UpStairsState, UpStairsEventId, UpStairsContext>>(
//                 UpStairsState::Initial,
//                 [&](UpStairsContext& ctx, const EventArgs&) {
//                     auto& traj = ctx.getTrajectory("initial");
//                     traj.reset();
//                     traj.set_start_point(
//                         {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                          **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                     traj.set_total_step(1500);
//                 },
//                 nullptr, // no exit
//                 [&](const Event<UpStairsEventId>& event,
//                     UpStairsContext& ctx) -> std::optional<UpStairsState> {
//                     return std::visit(
//                         [&](auto&& id) -> std::optional<UpStairsState> {
//                             if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                                 if (id == UpStairsEventEnum::tick) {
//                                     auto& traj = ctx.getTrajectory("initial");
//                                     if (!traj.get_complete()) {
//                                         ctx.getResult() = traj.trajectory();
//                                         return UpStairsState::Initial;
//                                     } else {
//                                         return std::nullopt;
//                                     }
//                                 } else if (id == UpStairsEventEnum::go_to_OneProcess) {
//                                     return UpStairsState::StepByOne;
//                                 } else if (id == UpStairsEventEnum::go_to_TwoProcess) {
//                                     return UpStairsState::StepByTwo;
//                                 }
//                                 else if (id == UpStairsEventEnum::go_to_Press){
//                                     return UpStairsState::StepByTwo;
//                                 }
//                             }
//                             return std::nullopt;
//                         },
//                         event.id);
//                 });
//         up_stairs_hsm.registerState(std::move(initialState));

//         // StepByOneState (Composite)
//         auto stepByOne = std::make_unique<
//             CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext>>(
//             UpStairsState::StepByOne, StepSubState::Wait);

//         // 添加子状态动态
//         stepByOne->addSubState(createWaitState());
//         stepByOne->addSubState(createPressAndLiftState());
//         up_stairs_hsm.registerState(std::move(stepByOne));

//         // StepByTwoState (Composite)
//         auto stepByTwo = std::make_unique<
//             CompositeState<UpStairsState, StepSubState, UpStairsEventId, UpStairsContext>>(
//             UpStairsState::StepByTwo, StepSubState::Press);

//         stepByTwo->addSubState(createPressState());
//         stepByTwo->addSubState(createLiftState());
//         stepByTwo->addSubState(createWaitState());
//         stepByTwo->addSubState(createInitialAgainState());
//         up_stairs_hsm.registerState(std::move(stepByTwo));

//         return *this;
//     }

// private:
//     // Helper functions to create sub states
//     std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createWaitState() {
//         return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
//             StepSubState::Wait, nullptr, nullptr,
//             [&](const Event<UpStairsEventId>& event,
//                 UpStairsContext& ctx) -> std::optional<StepSubState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepSubState> {
//                         if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                             if (id == UpStairsEventEnum::go_to_Press_And_Lift) {
//                                 return StepSubState::PressAndLift;
//                             } else if (id == UpStairsEventEnum::go_to_Press) {
//                                 return StepSubState::Press;
//                             } else if (id == UpStairsEventEnum::go_to_Lift) {
//                                 return StepSubState::Lift;
//                             } else if (id == UpStairsEventEnum::tick) {
//                                 return StepSubState::Wait;
//                             } else if (id == UpStairsEventEnum::go_to_Initial_Again) {
//                                 return StepSubState::InitialAgain;
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

//     std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>>
//         createPressAndLiftState() {
//         return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
//             StepSubState::PressAndLift,
//             [&](UpStairsContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("press_and_lift");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(2100);
//             },
//             nullptr,
//             [&](const Event<UpStairsEventId>& event,
//                 UpStairsContext& ctx) -> std::optional<StepSubState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepSubState> {
//                         if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                             if (id == UpStairsEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("press_and_lift");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return std::nullopt;
//                                 } else {
//                                     auto& res = ctx.getResult();
//                                     res[0]    = 0.8;
//                                     res[3]    = 0.8;
//                                     return StepSubState::Wait;
//                                 }
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

//     std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createPressState() {
//         return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
//             StepSubState::Press,
//             [&](UpStairsContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("press");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(1000);
//             },
//             nullptr,
//             [&](const Event<UpStairsEventId>& event,
//                 UpStairsContext& ctx) -> std::optional<StepSubState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepSubState> {
//                         if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                             if (id == UpStairsEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("press");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return StepSubState::Press;
//                                 } else {
//                                     return StepSubState::Lift;
//                                 }
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

//     std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>> createLiftState() {
//         return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
//             StepSubState::Lift,
//             [&](UpStairsContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("lift");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(1100);
//             },
//             nullptr,
//             [&](const Event<UpStairsEventId>& event,
//                 UpStairsContext& ctx) -> std::optional<StepSubState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepSubState> {
//                         if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                             if (id == UpStairsEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("lift");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return StepSubState::Lift;
//                                 } else {
//                                     return StepSubState::Wait;
//                                 }
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

//     std::unique_ptr<IState<StepSubState, UpStairsEventId, UpStairsContext>>
//         createInitialAgainState() {
//         return std::make_unique<BasicState<StepSubState, UpStairsEventId, UpStairsContext>>(
//             StepSubState::InitialAgain,
//             [&](UpStairsContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("initial_again");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(1000);
//             },
//             nullptr,
//             [&](const Event<UpStairsEventId>& event,
//                 UpStairsContext& ctx) -> std::optional<StepSubState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepSubState> {
//                         if constexpr (std::is_same_v<decltype(id), UpStairsEventEnum>) {
//                             if (id == UpStairsEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("initial_again");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return StepSubState::InitialAgain;
//                                 } else {
//                                     return StepSubState::Wait;
//                                 }
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_initial;
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_press;
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_lift;
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
//         up_stairs_leg_press_and_lift;
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT> up_stairs_leg_initial_again;

//     UpStairsContext context_;
//     HSM<UpStairsState, UpStairsEventId, UpStairsContext> up_stairs_hsm;
// };

// } // namespace rmcs_core::hardware::hsm::up_stairs_hsm

// #endif // HSM_UP_STAIRS_HPP_H_