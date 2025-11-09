#pragma once

#include "data/time_stamped.hpp"

namespace world_exe::interfaces {
/**
 * @brief 不必管他
 *
 * @tparam T
 */
template <class T> class ISyncBlock {

public:

    virtual void set_data(const T& camera_data) = 0;

    virtual std::tuple<T, bool> get_data(const data::TimeStamp& timestamp) = 0;

    virtual ~ISyncBlock() = default;
};
}