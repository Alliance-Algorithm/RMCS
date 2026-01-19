// #ifndef HSM_UP_ONE_STAIR_HPP_H_
// #define HSM_UP_ONE_STAIR_HPP_H_

// #include "hardware/device/trajectory.hpp"
// #include "hardware/hsm/HSM_dev.hpp" // 注意: 使用优化后的 HSM.hpp
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
// using InputVct =
//     rmcs_executor::Component::InputInterface<rmcs_description::BaseLink::DirectionVector>*;

// enum class UpOneStairState {
//     Initial,
//     Step
// };

// enum class StepState {
//     Press,
//     Lift,
//     Delay,
//     Wait
// };

// enum class UpOneStairEventEnum {
//     quit,
//     stop,
//     tick,
//     go_to_Step,
//     go_to_Press,
//     go_to_Lift,
//     tof_already     // args:: double distance
// };

// using UpOneStairEventId = std::variant<UpOneStairEventEnum, std::string>;

// namespace rmcs_core::hardware::hsm::up_one_stair_hsm {

// namespace events {
// using UpOneStairEvent = Event<UpOneStairEventId>;
// const UpOneStairEvent tick{UpOneStairEventId{UpOneStairEventEnum::tick}, {}};

// const UpOneStairEvent quit{UpOneStairEventId{UpOneStairEventEnum::quit}, {}};

// const UpOneStairEvent stop{UpOneStairEventId{UpOneStairEventEnum::stop}, {}};


// const UpOneStairEvent go_to_Press{UpOneStairEventId{UpOneStairEventEnum::go_to_Press}, {}};

// const UpOneStairEvent go_to_Lift{UpOneStairEventId{UpOneStairEventEnum::go_to_Lift}, {}};
// } // namespace events

// struct UpOneStairContext {
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
//         double raw = k * ((**getInputVct("speed"))->x() - v_reference) + b;
//         return (std::max(static_cast<int>(raw), 200));
//     }
// };

// class Auto_Leg_Up_One_Stair : public rclcpp::Node {
// public:
//     explicit Auto_Leg_Up_One_Stair()
//         : context_()
//         , up_one_stair_hsm(context_)
//         , Node{"auto_leg_up_one_stair"} {}

//     void processEvent(const Event<UpOneStairEventId>& event) {
//         // 处理事件ID为variant: 使用std::visit
//         std::visit(
//             [&](auto&& id) {
//                 using T = std::decay_t<decltype(id)>;

//                 if constexpr (std::is_same_v<T, UpOneStairEventEnum>) {
//                     RCLCPP_INFO(this->get_logger(), "Processing event: %d", static_cast<int>(id));
//                 } else if constexpr (std::is_same_v<T, std::string>) {
//                     RCLCPP_INFO(this->get_logger(), "Processing custom event: %s", id.c_str());
//                 }
//             },
//             event.id);
//         up_one_stair_hsm.processEvent(event);
//     }

//     void start(UpOneStairState initial, const EventArgs& args = {}) {
//         check_context_ready();
//         up_one_stair_hsm.start(initial, args);
//     }

//     void stop() { up_one_stair_hsm.stop(); }

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
//     Auto_Leg_Up_One_Stair& bind(const std::string& key, T& value) {
//         context_.data[key] = std::ref(value);
//         return *this;
//     }

//     std::array<double, 6>& get_result() { return context_.getResult(); }

//     Auto_Leg_Up_One_Stair& init_and_trajectory_set(
//         const std::vector<double>& initial_end_point_, const std::vector<double>& press_end_point_,
//         const std::vector<double>& lift_end_point_) {

//         // 绑定轨迹 (作为成员变量，但现在动态bind)
//         bind("initial", up_stairs_initial)
//             .bind("press", up_stairs_leg_press)
//             .bind("lift", up_stairs_leg_lift);

//         // 设置端点
//         up_stairs_initial.set_end_point(
//             {initial_end_point_[0], initial_end_point_[1], initial_end_point_[2],
//              initial_end_point_[3], 0, 0});
//         up_stairs_leg_press.set_end_point(
//             {press_end_point_[0], press_end_point_[1], press_end_point_[2], press_end_point_[3], 0,
//              0});
//         up_stairs_leg_lift.set_end_point(
//             {lift_end_point_[0], lift_end_point_[1], lift_end_point_[2], lift_end_point_[3], 0, 0});
//         // 注册状态使用BasicState和lambda

//         // InitialState
//         auto initialState =
//             std::make_unique<BasicState<UpOneStairState, UpOneStairEventId, UpOneStairContext>>(
//                 UpOneStairState::Initial,
//                 [&](UpOneStairContext& ctx, const EventArgs&) {
//                     auto& traj = ctx.getTrajectory("initial");
//                     traj.reset();
//                     traj.set_start_point(
//                         {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                          **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                     traj.set_total_step(1000);
//                 },
//                 nullptr, // no exit
//                 [&](const Event<UpOneStairEventId>& event,
//                     UpOneStairContext& ctx) -> std::optional<UpOneStairState> {
//                     return std::visit(
//                         [&](auto&& id) -> std::optional<UpOneStairState> {
//                             if constexpr (std::is_same_v<decltype(id), UpOneStairEventEnum>) {
//                                 if (id == UpOneStairEventEnum::tick) {
//                                     auto& traj = ctx.getTrajectory("initial");
//                                     if (!traj.get_complete()) {
//                                         ctx.getResult() = traj.trajectory();
//                                         return UpOneStairState::Initial;
//                                     } else {
//                                         return std::nullopt;
//                                     }
//                                 } else if (id == UpOneStairEventEnum::go_to_Step) {
//                                     return UpOneStairState::Step;
//                                 }
//                             }
//                             return std::nullopt;
//                         },
//                         event.id);
//                 });
//         up_one_stair_hsm.registerState(std::move(initialState));


//         // StepByTwoState (Composite)
//         auto Step = std::make_unique<
//             CompositeState<UpOneStairState, StepState, UpOneStairEventId, UpOneStairContext>>(
//             UpOneStairState::Step, StepState::Press);

//         Step->addSubState(createPressState());
//         Step->addSubState(createLiftState());
//         Step->addSubState(createWaitState());
//         up_one_stair_hsm.registerState(std::move(Step));

//         return *this;
//     }

// private:
//     // Helper functions to create sub states
//     std::unique_ptr<IState<StepState, UpOneStairEventId, UpOneStairContext>> createWaitState() {
//         return std::make_unique<BasicState<StepState, UpOneStairEventId, UpOneStairContext>>(
//             StepState::Wait, nullptr, nullptr,
//             [&](const Event<UpOneStairEventId>& event,
//                 UpOneStairContext& ctx) -> std::optional<StepState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepState> {
//                         if constexpr (std::is_same_v<decltype(id), UpOneStairEventEnum>) {
                           
//                        if (id == UpOneStairEventEnum::go_to_Press) {
//                                 return StepState::Press;
//                             } else if (id == UpOneStairEventEnum::go_to_Lift) {
//                                 return StepState::Lift;
//                             } else if (id == UpOneStairEventEnum::tick) {
//                                 return StepState::Wait;
//                             } 
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }

    
//     std::unique_ptr<IState<StepState, UpOneStairEventId, UpOneStairContext>> createDelayState() {
//         return std::make_unique<BasicState<StepState, UpOneStairEventId, UpOneStairContext>>(
//             StepState::Delay, nullptr, nullptr,
//             [&](const Event<UpOneStairEventId>& event,
//                 UpOneStairContext& ctx) -> std::optional<StepState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepState> {
//                         if constexpr (std::is_same_v<decltype(id), UpOneStairEventEnum>) {
//                          if (id == UpOneStairEventEnum::go_to_Press) {
//                                 return StepState::Press;
//                             } else if (id == UpOneStairEventEnum::go_to_Lift) {
//                                 return StepState::Lift;
//                             } else if (id == UpOneStairEventEnum::tick) {
//                                 return StepState::Wait;
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }


//     std::unique_ptr<IState<StepState, UpOneStairEventId, UpOneStairContext>> createPressState() {
//         return std::make_unique<BasicState<StepState, UpOneStairEventId, UpOneStairContext>>(
//             StepState::Press,
//             [&](UpOneStairContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("press");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[0], ctx.getVec("b")[0]));
//             },
//             nullptr,
//             [&](const Event<UpOneStairEventId>& event,
//                 UpOneStairContext& ctx) -> std::optional<StepState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepState> {
//                         if constexpr (std::is_same_v<decltype(id), UpOneStairEventEnum>) {
//                             if (id == UpOneStairEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("press");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return StepState::Press;
//                                 } else {
//                                         return StepState::Lift;
                                    
//                                 }
//                             }
//                         }
//                         return std::nullopt;
//                     },
//                     event.id);
//             });
//     }
   
//     std::unique_ptr<IState<StepState, UpOneStairEventId, UpOneStairContext>> createLiftState() {
//         return std::make_unique<BasicState<StepState, UpOneStairEventId, UpOneStairContext>>(
//             StepState::Lift,
//             [&](UpOneStairContext& ctx, const EventArgs&) {
//                 auto& traj = ctx.getTrajectory("lift");
//                 traj.reset();
//                 traj.set_start_point(
//                     {**ctx.getInputD("theta_lf"), **ctx.getInputD("theta_lb"),
//                      **ctx.getInputD("theta_rb"), **ctx.getInputD("theta_rf"), 0.0, 0.0});
//                 traj.set_total_step(ctx.calculate_steps(ctx.getVec("k")[1], ctx.getVec("b")[1]));
//             },
//             nullptr,
//             [&](const Event<UpOneStairEventId>& event,
//                 UpOneStairContext& ctx) -> std::optional<StepState> {
//                 return std::visit(
//                     [&](auto&& id) -> std::optional<StepState> {
//                         if constexpr (std::is_same_v<decltype(id), UpOneStairEventEnum>) {
//                             if (id == UpOneStairEventEnum::tick) {
//                                 auto& traj = ctx.getTrajectory("lift");
//                                 if (!traj.get_complete()) {
//                                     ctx.getResult() = traj.trajectory();
//                                     return StepState::Lift;
//                                 } else {
//                                     return StepState::Wait;
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
//         up_stairs_leg_lift_and_initial;
//     hardware::device::Trajectory<hardware::device::TrajectoryType::JOINT>
//         up_stairs_leg_initial_again;

//     UpOneStairContext context_;
//     HSM<UpOneStairState, UpOneStairEventId, UpOneStairContext>  up_one_stair_hsm;
// };

// } // namespace rmcs_core::hardware::hsm::up_one_stair_hsm

// #endif // HSM_UP_ONE_STAIR_HPP_H_