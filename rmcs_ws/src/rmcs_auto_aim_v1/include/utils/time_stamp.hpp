#pragma once

#include "data/time_stamped.hpp"
#include <chrono>
namespace world_exe::util::time_stamp {
    struct SteadyClock{
        data::TimeStamp operator()() const{
            return data::TimeStamp{std::chrono::steady_clock::now().time_since_epoch()};
        }
    };
}