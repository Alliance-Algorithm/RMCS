#include "printer.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace rmcs;

struct Printer::Impl {
    rclcpp::Logger logger;

    explicit Impl(const std::string& name) noexcept
        : logger { rclcpp::get_logger(name) } { }

    auto log(const std::string& msg, Level level) const noexcept {
        switch (level) {
        case Level::INFO:
            RCLCPP_INFO(logger, "%s", msg.c_str());
            break;
        case Level::WARN:
            RCLCPP_WARN(logger, "%s", msg.c_str());
            break;
        case Level::ERROR:
            RCLCPP_ERROR(logger, "%s", msg.c_str());
            break;
        }
    }
};

Printer::Printer(const std::string& name) noexcept
    : pimpl { std::make_unique<Impl>(name) } { }

Printer::~Printer() noexcept = default;

auto Printer::log(const std::string& msg, Level level) noexcept -> void { pimpl->log(msg, level); }
