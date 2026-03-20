#include "core/src/utility/assert.hpp"

#include <exception>
#include <iostream>
#include <print>
#include <source_location>

namespace librmcs::core::utility {

[[noreturn]] void assert_func(const std::source_location& location) {
    std::println(
        std::cerr, "Assertion failed at {}:{} in function {}", location.file_name(),
        location.line(), location.function_name());
    std::terminate();
}

} // namespace librmcs::core::utility
