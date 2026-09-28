#pragma once

#include <map>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_executor {

// Test-only pairing of explicitly selected plugins; no hardware or executor thread.
class Executor {
public:
    static void pair(std::span<Component*> components) {
        std::map<std::string, Component::OutputDeclaration*> outputs;
        Component::OutputInfoMap info;
        for (auto* component : components)
            for (auto& output : component->output_list_) {
                if (!outputs.emplace(output.name, &output).second)
                    throw std::runtime_error("duplicate test output " + output.name);
                info.emplace(output.name, Component::OutputInfo{output.type, output.kind});
            }
        for (auto* component : components)
            component->before_pairing(info);
        for (auto* component : components)
            for (auto& input : component->input_list_) {
                const auto output = outputs.find(input.name);
                if (output == outputs.end() && !input.required)
                    continue;
                if (output == outputs.end() || input.type != output->second->type)
                    throw std::runtime_error("unpaired test input " + input.name);
                input.bind(input.binding, output->second->binding);
            }
    }
};

} // namespace rmcs_executor
