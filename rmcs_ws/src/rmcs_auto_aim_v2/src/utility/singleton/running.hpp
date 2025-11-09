#pragma once

namespace rmcs::util {

auto get_running() noexcept -> bool;

auto set_running(bool) noexcept -> void;

}
