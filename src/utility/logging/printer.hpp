#pragma once
#include <format>
#include <memory>

namespace rmcs {

struct Printer {
public:
    explicit Printer(const std::string&) noexcept;
    ~Printer() noexcept;

    Printer(const Printer&)            = delete;
    Printer& operator=(const Printer&) = delete;

    template <typename... Args>
    auto info(std::format_string<Args...> fmt, Args&&... args) {
        log(std::format(fmt, std::forward<Args>(args)...), Level::INFO);
    }
    template <typename... Args>
    auto warn(std::format_string<Args...> fmt, Args&&... args) {
        log(std::format(fmt, std::forward<Args>(args)...), Level::WARN);
    }
    template <typename... Args>
    auto error(std::format_string<Args...> fmt, Args&&... args) {
        log(std::format(fmt, std::forward<Args>(args)...), Level::ERROR);
    }

private:
    enum class Level { INFO, WARN, ERROR };
    struct Impl;
    std::unique_ptr<Impl> pimpl;

    auto log(const std::string&, Level) noexcept -> void;
};

}
