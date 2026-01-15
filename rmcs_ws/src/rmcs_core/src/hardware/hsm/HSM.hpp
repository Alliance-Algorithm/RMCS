#ifndef HARDWARE_HSM_HPP_H_
#define HARDWARE_HSM_HPP_H_
#include <any>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <vector>

// --------------------------- Variant / Event ---------------------------
using EventArgs = std::vector<std::any>;

template <typename EventId>
struct Event {
    EventId id;
    EventArgs args;

    template <typename T>
    const T* getArg(size_t idx = 0) const {
        if (idx >= args.size())
            return nullptr;
        return std::any_cast<T>(&args[idx]);
    }
};

// --------------------------- IState (基础接口) ---------------------------
template <typename StateId, typename EventId, typename Context>
class IState {
public:
    virtual ~IState() = default;

    // Called when HSM 启动并进入该状态
    virtual void enter(Context& ctx, const EventArgs& args) {}

    // Called当 HSM 离开该状态
    virtual void exit(Context& ctx) {}

    // 事件处理：返回 std::nullopt 表示“在本层无需切换父层状态”；
    // 返回父层 StateId 表示父层状态需要切换为返回值
    virtual std::optional<StateId> handleEvent(const Event<EventId>& event, Context& ctx) = 0;

    // 当 HSM 注册这个 state 时被调用（给 state 一个机会去获得 Context 或构建子 HSM）
    // 默认实现空，复合态可 override 在这里创建子 HSM 并注册子状态
    virtual void onRegister(Context& ctx) {}

    virtual StateId getStateID() const = 0;
};

// 前向声明 HSM，用于示例中 CompositeState 持有子 HSM 的类型名
template <typename StateId, typename EventId, typename Context>
class HSM;

// --------------------------- HSM（只负责当前层） ---------------------------
template <typename StateId, typename EventId, typename Context>
class HSM {
public:
    explicit HSM(Context& ctx)
        : context(ctx) {}

    // 注册状态（需要 StateType 有默认构造）
    template <typename StateType>
    void registerState() {
        static_assert(
            std::is_base_of_v<IState<StateId, EventId, Context>, StateType>,
            "StateType must derive from IState<StateId,EventId,Context>");
        auto state = std::make_unique<StateType>();
        // 在把 state 放入 map 之前，让 state 有机会初始化内部的子 HSM（如果需要）
        state->onRegister(context);

        StateId id = state->getStateID();
        std::lock_guard<std::recursive_mutex> lock(mtx);
        states.emplace(id, std::move(state));
    }

    // 启动 HSM（进入初始态）
    void start(StateId initial, const EventArgs& args = {}) {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        auto it = states.find(initial);
        if (it == states.end()) {
            std::cerr << "[HSM] Error: initial state not found\n";
            return;
        }
        currentState = it->second.get();
        if (currentState)
            currentState->enter(context, args);
    }

    // 停止 HSM（退出当前态并清空）
    void stop() {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        if (currentState) {
            currentState->exit(context);
            currentState = nullptr;
        }
    }

    // 处理事件：返回 true 表示事件被处理（在本层或下层）
    bool processEvent(const Event<EventId>& event) {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        if (!currentState)
            return false;

        // currentState 自己决定是否把事件交给子 HSM（如果它是 CompositeState）
        auto nextId = currentState->handleEvent(event, context);

        // 如果子处理完或者当前态处理完但没有父层切换，则认为 handled = true
        if (!nextId) {
            return true;
//        return false;
    }

        // 若返回了一个父层 StateId（表示要切换本层状态）
        if (*nextId == currentState->getStateID()) {
            // 明确写出相同 ID 表示没有切换
            return true;
        }

        // 切换状态
        auto it = states.find(*nextId);
        if (it == states.end()) {
            std::cerr << "[HSM] Target state not found\n";
            return true; // 认为已处理（避免事件漏出）
        }

        currentState->exit(context);
        currentState = it->second.get();
        currentState->enter(context, event.args);
        return true;
    }

    StateId getCurrentStateID() const {
        std::lock_guard<std::recursive_mutex> lock(mtx);
        return currentState ? currentState->getStateID() : StateId{};
    }

private:
    Context& context;
    mutable std::recursive_mutex mtx;

    std::unordered_map<StateId, std::unique_ptr<IState<StateId, EventId, Context>>> states;
    IState<StateId, EventId, Context>* currentState = nullptr;
};

// --------------------------- CompositeState（模板：父 StateId + 子 StateId）
// ---------------------------
template <typename ParentStateId, typename SubStateId, typename EventId, typename Context>
class CompositeState : public IState<ParentStateId, EventId, Context> {
public:
    CompositeState()
        : innerHsm(nullptr)
        , initialSubStatePtr(nullptr)
        , is_subActive(false) {}

    // Derived 应在 override onRegister(ctx) 中：创建 innerHsm 和注册子状态
    // 示例：
    //   innerHsm = std::make_unique<HSM<SubStateId,EventId,Context>>(ctx);
    //   innerHsm->registerState<SubStateA>();
    //   innerHsm->registerState<SubStateB>();
    //   initialSubState = SubStateA;
    //
    void onRegister(Context& ctx) override {
        // 默认不创建 innerHsm，派生类应创建 innerHsm（如果需要）
    }

    // 进入复合态：激活子 HSM（如果已创建）
    void enter(Context& ctx, const EventArgs& args) override {
        if (innerHsm) {
            is_subActive = true;
            if (initialSubStatePtr) {
                innerHsm->start(*initialSubStatePtr, args);
            }
        }
        onEnter(ctx, args);
    }

    // 退出复合态：停止子 HSM（如果存在）
    void exit(Context& ctx) override {
        if (innerHsm && is_subActive) {
            innerHsm->stop();
            is_subActive = false;
        }
        onExit(ctx);
    }

    // 事件处理：优先交给子 HSM；若子 HSM 未处理，再由 derived 处理（handleComposite）
    std::optional<ParentStateId> handleEvent(const Event<EventId>& event, Context& ctx) override {
        if (innerHsm && is_subActive) {
            bool handledByChild = innerHsm->processEvent(event);
            if (handledByChild) {
                // 子层处理完（或者在子层引发了子层内部转移），父层无需变更
                return std::nullopt;
            }
            else{
                return handleComposite(event, ctx);
            }
        }
        // 子层未处理，则交给复合态自己的逻辑（由派生类实现）
        throw std::runtime_error{"Unable to get parameter update_rate<double>"};
        return std::nullopt;
    }

    // 派生类可以在这里实现复合态在子层未处理时的行为（包括返回父层要切换的 StateId）
    virtual std::optional<ParentStateId> handleComposite(const Event<EventId>& /*event*/, Context& /*ctx*/) {
        return std::nullopt;
    }

protected:
    // helper：派生类在 onRegister 中可调用这些来创建与初始化子 HSM
    void createInnerHsm(Context& ctx) {
        innerHsm = std::make_unique<HSM<SubStateId, EventId, Context>>(ctx);
    }

    template <typename SubStateType>
    void registerInnerState() {
        if (!innerHsm)
            throw std::runtime_error(
                "Inner HSM not created. Call createInnerHsm(ctx) in onRegister first.");
        innerHsm->template registerState<SubStateType>();
    }

    void setInitialSubState(SubStateId id) {
        initialSubState    = id;
        initialSubStatePtr = &initialSubState;
    }

    // optional hooks for derived classes
    virtual void onEnter(Context& /*ctx*/, const EventArgs& /*args*/) {}
    virtual void onExit(Context& /*ctx*/) {}

private:
    std::unique_ptr<HSM<SubStateId, EventId, Context>> innerHsm;
    SubStateId initialSubState;
    SubStateId* initialSubStatePtr; // pointer used only when initialSubState is set
    bool is_subActive;
};
#endif  // HARDWARE_HSM_HPP_H_
