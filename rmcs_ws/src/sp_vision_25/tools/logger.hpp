#ifndef TOOLS__LOGGER_HPP
#define TOOLS__LOGGER_HPP

#include <spdlog/spdlog.h>

namespace tools
{
std::shared_ptr<spdlog::logger> logger();

}  // namespace tools

#endif  // TOOLS__LOGGER_HPP