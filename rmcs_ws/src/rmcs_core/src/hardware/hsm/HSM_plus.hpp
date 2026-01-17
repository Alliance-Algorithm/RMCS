// #ifndef HARDWARE_HSM_HPP_H_
// #define HARDWARE_HSM_HPP_H_
// #include <any>
// #include <functional>
// #include <iostream>
// #include <memory>
// #include <mutex>
// #include <optional>
// #include <string>
// #include <type_traits>
// #include <unordered_map>
// #include <variant>
// #include <vector>

// // --------------------------- Variant / Event ---------------------------
// using EventArgs = std::vector<std::any>;

// template <typename EventId>
// struct Event {
//     EventId id;
//     EventArgs args;

//     template <typename T>
//     const T* getArg(size_t idx = 0) const {
//         if (idx >= args.size())
//             return nullptr;
//         return std::any_cast<T>(&args[idx]);
//     }
// };

// // --------------------------- IState (基础接口) ---------------------------
// template <typename StateId, typename EventId, typename Context>
// class IState {
// public:
//     virtual ~IState() = default;

//     virtual void enter(Context& ctx, const EventArgs& args) {}

//     virtual void exit(Context& ctx) {}

//     virtual std::optional<StateId> handleEvent(const Event<EventId>& event, Context& ctx) = 0;

//     virtual void onRegister(Context& ctx) {}

//     virtual StateId getStateID() const = 0;
// };

// // 前向声明 HSM
// template <typename StateId, typename EventId, typename Context>
// class HSM;

// // --------------------------- HSM（只负责当前层） ---------------------------
// template <typename StateId, typename EventId, typename Context>
// class HSM {
// public:
//     explicit HSM(Context& ctx)
//         : context(ctx) {}

//     void registerState(std::unique_ptr<IState<StateId, EventId, Context>> state) {
//         state->onRegister(context);
//         StateId id = state->getStateID();
//         std::lock_guard<std::recursive_mutex> lock(mtx);
//         states.emplace(id, std::move(state));
//     }

//     void start(StateId initial, const EventArgs& args = {}) {
//         std::lock_guard<std::recursive_mutex> lock(mtx);
//         auto it = states.find(initial);
//         if (it == states.end()) {
//             std::cerr << "[HSM] Error: initial state not found\n";
//             return;
//         }
//         currentState = it->second.get();
//         if (currentState)
//             currentState->enter(context, args);
//     }

//     void stop() {
//         std::lock_guard<std::recursive_mutex> lock(mtx);
//         if (currentState) {
//             currentState->exit(context);
//             currentState = nullptr;
//         }
//     }

//     bool processEvent(const Event<EventId>& event) {
//         std::lock_guard<std::recursive_mutex> lock(mtx);
//         if (!currentState)
//             return false;

//         auto nextId = currentState->handleEvent(event, context);

//         if (!nextId) {
//             return true;
//         }

//         if (*nextId == currentState->getStateID()) {
//             return true;
//         }

//         auto it = states.find(*nextId);
//         if (it == states.end()) {
//             std::cerr << "[HSM] Target state not found\n";
//             return true;
//         }

//         currentState->exit(context);
//         currentState = it->second.get();
//         currentState->enter(context, event.args);
//         return true;
//     }

//     StateId getCurrentStateID() const {
//         std::lock_guard<std::recursive_mutex> lock(mtx);
//         return currentState ? currentState->getStateID() : StateId{};
//     }

// private:
//     Context& context;
//     mutable std::recursive_mutex mtx;

//     std::unordered_map<StateId, std::unique_ptr<IState<StateId, EventId, Context>>> states;
//     IState<StateId, EventId, Context>* currentState = nullptr;
// };

// // --------------------------- BasicState for simplified state creation ---------------------------
// template <typename StateId, typename EventId, typename Context>
// using EnterFunc = std::function<void(Context&, const EventArgs&)>;
// template <typename StateId, typename EventId, typename Context>
// using ExitFunc = std::function<void(Context&)>;
// template <typename StateId, typename EventId, typename Context>
// using HandleFunc = std::function<std::optional<StateId>(const Event<EventId>&, Context&)>;

// template <typename StateId, typename EventId, typename Context>
// class BasicState : public IState<StateId, EventId, Context> {
// public:
//     BasicState(StateId id, EnterFunc<StateId, EventId, Context> enter, ExitFunc<StateId, EventId, Context> exit, HandleFunc<StateId, EventId, Context> handle)
//         : id_(id), enter_(std::move(enter)), exit_(std::move(exit)), handle_(std::move(handle)) {}

//     StateId getStateID() const override { return id_; }

//     void enter(Context& ctx, const EventArgs& args) override {
//         if (enter_) enter_(ctx, args);
//     }

//     void exit(Context& ctx) override {
//         if (exit_) exit_(ctx);
//     }

//     std::optional<StateId> handleEvent(const Event<EventId>& e, Context& ctx) override {
//         return handle_ ? handle_(e, ctx) : std::nullopt;
//     }

// private:
//     StateId id_;
//     EnterFunc<StateId, EventId, Context> enter_;
//     ExitFunc<StateId, EventId, Context> exit_;
//     HandleFunc<StateId, EventId, Context> handle_;
// };

// // --------------------------- CompositeState（支持动态子状态注册） ---------------------------
// template <typename ParentStateId, typename SubStateId, typename EventId, typename Context>
// class CompositeState : public IState<ParentStateId, EventId, Context> {
// public:
//     CompositeState(ParentStateId id, SubStateId initialSub)
//         : id_(id), initialSubState(initialSub), innerHsm(nullptr), is_subActive(false) {}

//     void addSubState(std::unique_ptr<IState<SubStateId, EventId, Context>> subState) {
//         subStatesToRegister.push_back(std::move(subState));
//     }

//     void onRegister(Context& ctx) override {
//         innerHsm = std::make_unique<HSM<SubStateId, EventId, Context>>(ctx);
//         for (auto& sub : subStatesToRegister) {
//             innerHsm->registerState(std::move(sub));
//         }
//         subStatesToRegister.clear(); // 清空以释放内存
//         onEnterRegister(ctx);
//     }

//     void enter(Context& ctx, const EventArgs& args) override {
//         if (innerHsm) {
//             is_subActive = true;
//             innerHsm->start(initialSubState, args);
//         }
//         onEnter(ctx, args);
//     }

//     void exit(Context& ctx) override {
//         if (innerHsm && is_subActive) {
//             innerHsm->stop();
//             is_subActive = false;
//         }
//         onExit(ctx);
//     }

//     std::optional<ParentStateId> handleEvent(const Event<EventId>& event, Context& ctx) override {
//         if (innerHsm && is_subActive) {
//             bool handledByChild = innerHsm->processEvent(event);
//             if (handledByChild) {
//                 return std::nullopt;
//             }
//         }
//         return handleComposite(event, ctx);
//     }

//     ParentStateId getStateID() const override { return id_; }

// protected:
//     virtual void onEnterRegister(Context& /*ctx*/) {}
//     virtual void onEnter(Context& /*ctx*/, const EventArgs& /*args*/) {}
//     virtual void onExit(Context& /*ctx*/) {}
//     virtual std::optional<ParentStateId> handleComposite(const Event<EventId>& /*event*/, Context& /*ctx*/) {
//         return std::nullopt;
//     }

// private:
//     ParentStateId id_;
//     SubStateId initialSubState;
//     std::unique_ptr<HSM<SubStateId, EventId, Context>> innerHsm;
//     std::vector<std::unique_ptr<IState<SubStateId, EventId, Context>>> subStatesToRegister;
//     bool is_subActive;
// };

// #endif  // HARDWARE_HSM_HPP_H_