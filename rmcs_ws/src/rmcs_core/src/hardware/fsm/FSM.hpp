#pragma once

#include <memory>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <typeinfo>
#include <typeindex>
#define up   1
#define down 2
#define initial_enter 3

template <typename S, typename E, typename C>
class FiniteStateMachine;

template <typename S, typename E, typename C>
class IState {
public:
    virtual ~IState() = default;
    virtual void enter(FiniteStateMachine<S,E,C>&, const C&) {}
    virtual void exit(FiniteStateMachine<S,E,C>&, const C&) {}
    virtual std::shared_ptr<IState<S,E,C>> handleEvent(FiniteStateMachine<S,E,C>&, const E&, C&) = 0;
    virtual S getStateID() const = 0;
};

template <typename S, typename E, typename C>
struct Transition {
    std::function<bool(const E&, const C&)> condition;
    std::function<void(const E&, C&)> action;
    std::shared_ptr<IState<S,E,C>> target;
};

template <typename S, typename E, typename C = void*>
class FiniteStateMachine {
    using StatePtr = std::shared_ptr<IState<S,E,C>>;
    
    std::unordered_map<S, StatePtr> states_;
    std::unordered_map<std::type_index, 
                      std::unordered_map<S, 
                                      std::vector<Transition<S,E,C>>>> transitions_;
    StatePtr currentState_;
    C context_;
    mutable std::mutex mtx_;
    
public:
    explicit FiniteStateMachine(C context = C{}) : context_(context) {}

    template <typename T, typename... Args>
    void registerState(Args&&... args) {
        std::lock_guard lock(mtx_);
        auto state = std::make_shared<T>(std::forward<Args>(args)...);
        states_.emplace(state->getStateID(), state);
    }

    std::shared_ptr<IState<S,E,C>> getState(S stateID) const {
        std::lock_guard lock(mtx_);
        auto it = states_.find(stateID);
        if (it != states_.end()) {
            return it->second;
        }
        return nullptr;
    }

    template <typename EventType>
    void addTransition(S from, EventType&& event,
                      std::function<bool(const EventType&, const C&)> condition,
                      std::function<void(const EventType&, C&)> action,
                      S to) {
        std::lock_guard lock(mtx_);
        auto& eventTransitions = transitions_[typeid(EventType)][from];
        eventTransitions.emplace_back(Transition<S,E,C>{
            [condition=std::move(condition)](const E& e, const C& c) {
                if constexpr (std::is_same_v<EventType, E>) {
                    return condition(static_cast<const EventType&>(e), c);
                }
                return false;
            },
            [action=std::move(action)](const E& e, C& c) {
                if constexpr (std::is_same_v<EventType, E>) {
                    action(static_cast<const EventType&>(e), c);
                }
            },
            states_.at(to)
        });
    }

    void start(S initialState) {
        std::lock_guard lock(mtx_);
        currentState_ = states_.at(initialState);
        currentState_->enter(*this, context_);
    }

    void processEvent(const E& event) {
        std::lock_guard lock(mtx_);
        if (!currentState_) return;

        auto& typeTransitions = transitions_[typeid(event)];
        auto stateTransitions = typeTransitions.find(currentState_->getStateID());
        if (stateTransitions == typeTransitions.end()) return;

        for (auto& transition : stateTransitions->second) {
            if (transition.condition(event, context_)) {
                currentState_->exit(*this, context_);
                transition.action(event, context_);
                currentState_ = transition.target;
                currentState_->enter(*this, context_);
                break; 
            }
        }
    }


    S getCurrentState() const {
        std::lock_guard lock(mtx_);
        return currentState_ ? currentState_->getStateID() : S{};
    }

    C& getContext() { return context_; }
    const C& getContext() const { return context_; }
};
