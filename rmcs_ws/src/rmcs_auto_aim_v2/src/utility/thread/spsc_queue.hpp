#pragma once
#include <boost/lockfree/spsc_queue.hpp>

namespace rmcs::util {

template <typename CopyOnly, std::size_t N>
using spsc_queue = boost::lockfree::spsc_queue<CopyOnly, boost::lockfree::capacity<N>>;

}
