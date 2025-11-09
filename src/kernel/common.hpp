#pragma once
#include "utility/coroutine/common.hpp"
#include "utility/serializable.hpp"

#include <yaml-cpp/yaml.h>

namespace rmcs::kernel {

using Yaml = YAML::Node;

using result_type = std::expected<void, std::string>;
using handle_type = std::coroutine_handle<co::task<result_type>::promise_type>;

namespace details {
    using Expansion = rmcs::util::Serializable;

    template <class T>
    concept has_config_trait = requires { typename T::Config; };

    template <class T>
    concept serialable_config_trait =
        requires { requires std::derived_from<typename T::Config, Expansion>; };

    template <class T>
    concept can_initialize_trait = requires(T& kernel) {
        { kernel.initialize(std::declval<typename T::Config>()) };
    };

    template <class T>
    concept has_prefix_trait = requires {
        { T::get_prefix() } -> std::convertible_to<std::string_view>;
    };
}

}
