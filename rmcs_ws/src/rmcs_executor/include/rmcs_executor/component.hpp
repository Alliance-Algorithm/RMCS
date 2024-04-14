#pragma once

#include <new>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_set>
#include <vector>

namespace rmcs_executor {

class Component {
public:
    friend class Executor;

    Component(const Component&)            = delete;
    Component& operator=(const Component&) = delete;
    Component(Component&&)                 = delete;
    Component& operator=(Component&&)      = delete;

    virtual ~Component() {
        for (auto& input : input_list_)
            *reinterpret_cast<bool*>(reinterpret_cast<size_t>(input.pointer_to_data_pointer) - 8) =
                false;
        for (auto& output : output_list_)
            *reinterpret_cast<bool*>(reinterpret_cast<size_t>(output.data_pointer) - 8) = false;
    };

    virtual void update() = 0;

    template <typename T>
    class InputInterface {
    public:
        friend class Component;

        InputInterface() = default;

        InputInterface(const InputInterface&)            = delete;
        InputInterface& operator=(const InputInterface&) = delete;
        InputInterface(InputInterface&&)                 = delete;
        InputInterface& operator=(InputInterface&&)      = delete;

        ~InputInterface() = default;

        [[nodiscard]] bool active() const { return activated; }
        [[nodiscard]] bool ready() const { return data_pointer_ != nullptr; }

        const T* operator->() const { return data_pointer_; }
        const T& operator*() const { return *data_pointer_; }

    private:
        void** activate() {
            activated = true;
            return reinterpret_cast<void**>(&data_pointer_);
        }

        bool activated   = false;
        T* data_pointer_ = nullptr;
    };

    template <typename T>
    class OutputInterface {
    public:
        friend class Component;

        OutputInterface() = default;

        OutputInterface(const OutputInterface&)            = delete;
        OutputInterface& operator=(const OutputInterface&) = delete;
        OutputInterface(OutputInterface&&)                 = delete;
        OutputInterface& operator=(OutputInterface&&)      = delete;

        ~OutputInterface() {
            if (active())
                std::destroy_at(std::launder(reinterpret_cast<T*>(&data_)));
        };

        [[nodiscard]] bool active() const { return activated; }

        T* operator->() { return reinterpret_cast<T*>(&data_); }
        const T* operator->() const { return reinterpret_cast<const T*>(&data_); }
        T& operator*() { return *reinterpret_cast<T*>(&data_); }
        const T& operator*() const { return *reinterpret_cast<const T*>(&data_); }

    private:
        template <typename... Args>
        void* activate(Args&&... args) {
            ::new (&data_) T(std::forward<Args>(args)...);
            activated = true;
            return reinterpret_cast<void*>(&data_);
        }

        bool activated = false;
        std::aligned_storage_t<sizeof(T), alignof(T)> data_;
    };

    const std::string& get_component_name() { return component_name_; }

    template <typename T>
    void register_input(const std::string& name, InputInterface<T>& interface) {
        input_list_.emplace_back(typeid(T), name, interface.activate());
    }

    template <typename T, typename... Args>
    void register_output(const std::string& name, OutputInterface<T>& interface, Args&&... args) {
        output_list_.emplace_back(
            typeid(T), name, interface.activate(std::forward<Args>(args)...), this);
    }

    static const char* initializing_component_name;

protected:
    Component()
        : component_name_(initializing_component_name) {}

private:
    std::string component_name_;

    struct InputDeclaration {
        const std::type_info& type;
        std::string name;
        void** pointer_to_data_pointer;
    };

    struct OutputDeclaration {
        const std::type_info& type;
        std::string name;
        void* data_pointer;

        Component* component;
    };

    std::vector<InputDeclaration> input_list_;
    std::vector<OutputDeclaration> output_list_;

    size_t dependency_count_                  = 0;
    std::unordered_set<Component*> wanted_by_ = {};
};

} // namespace rmcs_executor