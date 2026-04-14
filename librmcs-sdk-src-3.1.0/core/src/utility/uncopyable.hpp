#pragma once

namespace librmcs::core::utility {

class Uncopyable {
public:
    Uncopyable() = default;
    Uncopyable(const Uncopyable&) = delete;
    Uncopyable& operator=(const Uncopyable&) = delete;
    Uncopyable(Uncopyable&&) = default;
    Uncopyable& operator=(Uncopyable&&) = default;
    ~Uncopyable() = default;
};

} // namespace librmcs::core::utility
