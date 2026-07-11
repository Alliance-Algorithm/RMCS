#pragma once

#include <atomic>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs_executor {

enum class InterfaceKind {
    Normal,
    Event,
};

inline const char* interface_kind_name(InterfaceKind kind) {
    switch (kind) {
    case InterfaceKind::Normal: return "Normal";
    case InterfaceKind::Event: return "Event";
    }

    return "Unknown";
}

using EventState = std::uint32_t;
constexpr EventState EVENT_DISABLED_BIT = EventState{1} << 31;
constexpr EventState EVENT_ACTIVE_MASK = ~EVENT_DISABLED_BIT;

class Component {
public:
    friend class Executor;

    struct OutputInfo {
        std::reference_wrapper<const std::type_info> type;
        InterfaceKind kind;
    };
    using OutputInfoMap = std::map<std::string, OutputInfo>;

    Component(const Component&) = delete;
    Component& operator=(const Component&) = delete;
    Component(Component&&) = delete;
    Component& operator=(Component&&) = delete;

    virtual ~Component() = default;

    virtual void before_pairing(const OutputInfoMap& output_map) { (void)output_map; }
    virtual void before_updating() {}

    virtual void update() = 0;

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class InputInterface {
    public:
        friend class Component;
        InputInterface() = default;

        InputInterface(const InputInterface&) = delete;
        InputInterface& operator=(const InputInterface&) = delete;
        InputInterface(InputInterface&&) = delete;
        InputInterface& operator=(InputInterface&&) = delete;

        ~InputInterface() {
            if (delete_data_when_deconstruct) {
                if constexpr (std::is_array_v<T>) {
                    delete[] data_pointer_;
                } else {
                    delete data_pointer_;
                }
            }
        }

        [[nodiscard]] bool active() const { return activated; }
        [[nodiscard]] bool ready() const { return data_pointer_ != nullptr; }

        template <typename... Args>
        void make_and_bind_directly(Args&&... args) {
            if (ready())
                throw std::runtime_error("The interface has already been bound to somewhere");

            data_pointer_ = new T(std::forward<Args>(args)...);
            activated = true;

            delete_data_when_deconstruct = true;
        }

        void bind_directly(const T& destination) {
            if (ready())
                throw std::runtime_error("The interface has already been bound to somewhere");

            data_pointer_ = const_cast<T*>(&destination);
            activated = true;
        }

        const T* operator->() const { return data_pointer_; }
        const T& operator*() const { return *data_pointer_; }

    private:
        void* activate() {
            activated = true;
            return reinterpret_cast<void*>(&data_pointer_);
        }

        T* data_pointer_ = nullptr;
        bool activated = false;

        bool delete_data_when_deconstruct = false;
    };

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class EventInputInterface {
    public:
        friend class Component;

        template <typename Callback>
        requires std::invocable<Callback&, const T&>
        explicit EventInputInterface(Callback&& callback)
            : callback_(std::forward<Callback>(callback)) {}

        EventInputInterface(const EventInputInterface&) = delete;
        EventInputInterface& operator=(const EventInputInterface&) = delete;
        EventInputInterface(EventInputInterface&&) = delete;
        EventInputInterface& operator=(EventInputInterface&&) = delete;

        [[nodiscard]] bool active() const { return activated; }
        [[nodiscard]] bool ready() const { return static_cast<bool>(callback_); }

    private:
        void* activate() {
            if (!ready())
                throw std::runtime_error(
                    "The event input interface requires a callback before registration");

            activated = true;
            return reinterpret_cast<void*>(&callback_);
        }

        std::function<void(const T&)> callback_;
        bool activated = false;
    };

    template <typename T>
    requires(
        !std::is_reference_v<T> && !std::is_unbounded_array_v<T>
        && std::is_nothrow_copy_constructible_v<T> && std::is_nothrow_destructible_v<T>)
    class QueuedEventInputInterface final : public EventInputInterface<T> {
    public:
        template <typename Callback>
        requires std::invocable<Callback&, T&&>
        QueuedEventInputInterface(size_t queue_depth, Callback&& callback)
            : EventInputInterface<T>([this](const T& event) { enqueue(event); })
            , user_callback_(std::forward<Callback>(callback))
            , queue_(queue_depth)
            , worker_(&QueuedEventInputInterface::worker_main, this) {}

        ~QueuedEventInputInterface() {
            stop_requested_.store(true, std::memory_order::relaxed);
            notify_event();
            if (worker_.joinable())
                worker_.join();
        }

        QueuedEventInputInterface(const QueuedEventInputInterface&) = delete;
        QueuedEventInputInterface& operator=(const QueuedEventInputInterface&) = delete;
        QueuedEventInputInterface(QueuedEventInputInterface&&) = delete;
        QueuedEventInputInterface& operator=(QueuedEventInputInterface&&) = delete;

    private:
        void enqueue(const T& event) {
            if (stop_requested_.load(std::memory_order::relaxed))
                return;

            auto guard = std::scoped_lock{enqueue_mutex_};
            if (stop_requested_.load(std::memory_order::relaxed))
                return;

            if (!queue_.push_back(event)) {
                const auto dropped_count = dropped_event_count_++;
                if (dropped_count == 0) {
                    RCLCPP_WARN(
                        rclcpp::get_logger("rmcs_executor"),
                        "QueuedEventInputInterface started dropping events because the queue is "
                        "full");
                }
                return;
            }

            const auto dropped_count = dropped_event_count_;
            dropped_event_count_ = 0;
            if (dropped_count != 0) {
                RCLCPP_WARN(
                    rclcpp::get_logger("rmcs_executor"),
                    "QueuedEventInputInterface resumed enqueueing after dropping %u events",
                    dropped_count);
            }

            notify_event();
        }

        void notify_event() {
            event_count_.fetch_add(1, std::memory_order::release);
            event_count_.notify_one();
        }

        void worker_main() {
            while (!stop_requested_.load(std::memory_order::relaxed)) {
                if (auto* event = queue_.peek_front()) {
                    try {
                        user_callback_(std::move(*event));
                    } catch (const std::exception& exception) {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("rmcs_executor"),
                            "QueuedEventInputInterface worker terminated by exception: %s",
                            exception.what());
                        return;
                    } catch (...) {
                        RCLCPP_ERROR(
                            rclcpp::get_logger("rmcs_executor"),
                            "QueuedEventInputInterface worker terminated by unknown exception");
                        return;
                    }

                    if (!queue_.pop_front([](T&&) noexcept {}))
                        std::terminate();
                    continue;
                }

                const auto old = event_count_.load(std::memory_order::relaxed);
                if (!queue_.readable() && !stop_requested_.load(std::memory_order::relaxed))
                    event_count_.wait(old, std::memory_order::acquire);
            }
        }

        std::function<void(T&&)> user_callback_;
        rmcs_utility::RingBuffer<T> queue_;
        std::atomic<bool> stop_requested_{false};
        std::uint32_t dropped_event_count_ = 0;
        std::atomic<std::uint32_t> event_count_{0};
        std::mutex enqueue_mutex_;
        std::thread worker_;
    };

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class OutputInterface {
    public:
        friend class Component;

        OutputInterface() = default;

        OutputInterface(const OutputInterface&) = delete;
        OutputInterface& operator=(const OutputInterface&) = delete;
        OutputInterface(OutputInterface&&) = delete;
        OutputInterface& operator=(OutputInterface&&) = delete;

        ~OutputInterface() {
            if (active())
                std::destroy_at(storage_pointer());
        };

        [[nodiscard]] bool active() const { return activated; }

        T* operator->() { return storage_pointer(); }
        const T* operator->() const { return storage_pointer(); }
        T& operator*() { return *storage_pointer(); }
        const T& operator*() const { return *storage_pointer(); }

    private:
        template <typename... Args>
        void* activate(Args&&... args) {
            std::construct_at(raw_storage_pointer(), std::forward<Args>(args)...);
            activated = true;
            return data_;
        }

        [[nodiscard]] T* raw_storage_pointer() { return reinterpret_cast<T*>(data_); }

        [[nodiscard]] T* storage_pointer() { return std::launder(raw_storage_pointer()); }

        [[nodiscard]] const T* storage_pointer() const {
            return std::launder(reinterpret_cast<const T*>(data_));
        }

        alignas(T) std::byte data_[sizeof(T)];
        bool activated = false;
    };

    template <typename T>
    requires(!std::is_reference_v<T> && !std::is_unbounded_array_v<T>) class EventOutputInterface {
    public:
        friend class Component;

        EventOutputInterface() = default;

        EventOutputInterface(const EventOutputInterface&) = delete;
        EventOutputInterface& operator=(const EventOutputInterface&) = delete;
        EventOutputInterface(EventOutputInterface&&) = delete;
        EventOutputInterface& operator=(EventOutputInterface&&) = delete;

        [[nodiscard]] bool active() const { return activated; }

        void emit(const T& event) {
            if (!active())
                throw std::runtime_error("The event output interface has not been activated");
            if (!try_enter_emit())
                return;

            struct LeaveEmitGuard {
                EventOutputInterface& interface;

                ~LeaveEmitGuard() { interface.leave_emit(); }
            } leave_emit_guard{*this};

            for (const auto* callback : callback_list_)
                (*callback)(event);
        }

    private:
        using EventCallback = std::function<void(const T&)>;

        void* activate() {
            activated = true;
            return this;
        }

        void add_listener(EventCallback* callback) { callback_list_.emplace_back(callback); }

        bool try_enter_emit() {
            auto state = state_.load(std::memory_order_relaxed);
            while (true) {
                if (state & EVENT_DISABLED_BIT)
                    return false;
                if ((state & EVENT_ACTIVE_MASK) == EVENT_ACTIVE_MASK)
                    throw std::runtime_error("Too many active event emissions");

                if (state_.compare_exchange_weak(
                        state, state + 1, std::memory_order_relaxed, std::memory_order_relaxed))
                    return true;
            }
        }

        void leave_emit() {
            const auto previous_state = state_.fetch_sub(1, std::memory_order_release);
            const auto new_state = previous_state - 1;
            if ((new_state & EVENT_DISABLED_BIT) != 0 && (new_state & EVENT_ACTIVE_MASK) == 0)
                state_.notify_one();
        }

        void disable() { state_.fetch_or(EVENT_DISABLED_BIT, std::memory_order_relaxed); }

        void enable() { state_.fetch_and(EVENT_ACTIVE_MASK, std::memory_order_relaxed); }

        void wait_idle() {
            auto state = state_.load(std::memory_order_acquire);
            while ((state & EVENT_ACTIVE_MASK) != 0) {
                state_.wait(state, std::memory_order_acquire);
                state = state_.load(std::memory_order_acquire);
            }
        }

        std::vector<EventCallback*> callback_list_;
        std::atomic<EventState> state_{EVENT_DISABLED_BIT};
        bool activated = false;
    };

    const std::string& get_component_name() { return component_name_; }

    template <typename T>
    void register_input(
        const std::string& name, InputInterface<T>& interface, bool required = true) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(
            name, InterfaceKind::Normal, RegistrationDirection::Input);
        input_list_.emplace_back(
            typeid(T), name, InterfaceKind::Normal, required, interface.activate(),
            &bind_input_interface<T>);
    }

    template <typename T>
    void register_input(
        const std::string& name, EventInputInterface<T>& interface, bool required = true) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(
            name, InterfaceKind::Event, RegistrationDirection::Input);
        input_list_.emplace_back(
            typeid(T), name, InterfaceKind::Event, required, interface.activate(),
            &bind_event_input_interface<T>);
    }

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...>
    void register_output(const std::string& name, OutputInterface<T>& interface, Args&&... args) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(
            name, InterfaceKind::Normal, RegistrationDirection::Output);
        output_list_.emplace_back(
            typeid(T), name, InterfaceKind::Normal, interface.activate(std::forward<Args>(args)...),
            nullptr, nullptr, nullptr, this);
    }

    template <typename T>
    void register_output(const std::string& name, EventOutputInterface<T>& interface) {
        if (interface.active())
            throw std::runtime_error("The interface has been activated");

        ensure_registration_name_is_available(
            name, InterfaceKind::Event, RegistrationDirection::Output);
        output_list_.emplace_back(
            typeid(T), name, InterfaceKind::Event, interface.activate(),
            &enable_event_output_interface<T>, &disable_event_output_interface<T>,
            &wait_event_output_interface_idle<T>, this);
    }

    template <typename T, typename... Args>
    requires std::constructible_from<T, Args...>
    std::shared_ptr<T> create_partner_component(const std::string& name, Args&&... args) {
        initializing_component_name = name;

        auto component = std::make_shared<T>(std::forward<Args>(args)...);
        partner_component_list_.emplace_back(component);

        return component;
    }

    static std::string initializing_component_name;

protected:
    Component()
        : component_name_(initializing_component_name) {}

private:
    enum class RegistrationDirection {
        Input,
        Output,
    };

    using BindFunction = void (*)(void* input_binding, void* output_binding);
    using OutputLifecycleHook = void (*)(void* output_binding);

    static const char* registration_direction_name(RegistrationDirection direction) {
        switch (direction) {
        case RegistrationDirection::Input: return "input";
        case RegistrationDirection::Output: return "output";
        }

        return "interface";
    }

    void ensure_registration_name_is_available(
        const std::string& name, InterfaceKind kind, RegistrationDirection direction) const {
        auto check_declarations = [&](const auto& declarations,
                                      RegistrationDirection existing_direction) {
            for (const auto& declaration : declarations) {
                if (declaration.name != name)
                    continue;

                if (declaration.kind != kind) {
                    throw std::runtime_error(
                        "Component [" + component_name_ + "] cannot register "
                        + interface_kind_name(kind) + " " + registration_direction_name(direction)
                        + " \"" + name + "\" because \"" + name
                        + "\" has already been registered as "
                        + interface_kind_name(declaration.kind) + " "
                        + registration_direction_name(existing_direction) + " in this component.");
                }

                throw std::runtime_error(
                    "Component [" + component_name_ + "] registered " + interface_kind_name(kind)
                    + " " + registration_direction_name(direction) + " \"" + name
                    + "\" more than once.");
            }
        };

        check_declarations(input_list_, RegistrationDirection::Input);
        check_declarations(output_list_, RegistrationDirection::Output);
    }

    template <typename T>
    static void bind_input_interface(void* input_binding, void* output_binding) {
        *reinterpret_cast<T**>(input_binding) = reinterpret_cast<T*>(output_binding);
    }

    template <typename T>
    static void bind_event_input_interface(void* input_binding, void* output_binding) {
        auto& callback = *reinterpret_cast<std::function<void(const T&)>*>(input_binding);
        reinterpret_cast<EventOutputInterface<T>*>(output_binding)->add_listener(&callback);
    }

    template <typename T>
    static void enable_event_output_interface(void* output_binding) {
        reinterpret_cast<EventOutputInterface<T>*>(output_binding)->enable();
    }

    template <typename T>
    static void disable_event_output_interface(void* output_binding) {
        reinterpret_cast<EventOutputInterface<T>*>(output_binding)->disable();
    }

    template <typename T>
    static void wait_event_output_interface_idle(void* output_binding) {
        reinterpret_cast<EventOutputInterface<T>*>(output_binding)->wait_idle();
    }

    std::string component_name_;

    struct InputDeclaration {
        const std::type_info& type;
        std::string name;
        InterfaceKind kind;
        bool required;
        void* binding;
        BindFunction bind;
    };

    struct OutputDeclaration {
        const std::type_info& type;
        std::string name;
        InterfaceKind kind;
        void* binding;
        OutputLifecycleHook enable;
        OutputLifecycleHook disable;
        OutputLifecycleHook wait_idle;

        Component* component;
    };

    std::vector<InputDeclaration> input_list_;
    std::vector<OutputDeclaration> output_list_;

    std::vector<std::shared_ptr<Component>> partner_component_list_;

    std::size_t dependency_count_ = 0;
    std::unordered_set<Component*> wanted_by_ = {};
};

} // namespace rmcs_executor
