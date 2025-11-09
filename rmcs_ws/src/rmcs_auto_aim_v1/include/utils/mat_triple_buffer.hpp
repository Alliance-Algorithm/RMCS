#pragma once

#include "data/mat_stamped.hpp"
#include "data/time_stamped.hpp"
#include "utils/time_stamp.hpp"

#include <atomic>
#include <concepts>
#include <opencv2/core/mat.hpp>
#include <optional>

namespace world_exe::util::memory {


template<typename F>
concept BinaryFunctor = requires(F f) {
    { f() } -> std::same_as<data::TimeStamp>;
};

template<BinaryFunctor Func>
class MatTripleBuffer {
/// SPSC only
public:
    explicit MatTripleBuffer(Func timeFunc) : func_(timeFunc) {}

    static MatTripleBuffer<time_stamp::SteadyClock> default_buffer(){
        return  MatTripleBuffer(time_stamp::SteadyClock{});
    }
    
    void set(const cv::Mat& image)
    {
        buffer[ptr_set_].Load(image,func_());
        size_t old = ptr_get_.exchange(ptr_set_, std::memory_order_acq_rel);
        data_version.fetch_add(1, std::memory_order_release);
        ptr_set_ = old;
    }
    std::optional<std::reference_wrapper<data::MatStamped>> get()
    {
        size_t current_version = data_version.load(std::memory_order_acquire);
        size_t expected = last_read_version.load(std::memory_order_acquire);

        if (current_version == expected) return std::nullopt; 

        size_t old = ptr_get_.exchange(ptr_occ_, std::memory_order_acq_rel);
        ptr_occ_ = old;
        return buffer[ptr_occ_];
    }

    const Func          func_;
    data::MatStamped    buffer[3];

    std::atomic_uint8_t ptr_occ_ = 0;
    std::atomic_uint8_t ptr_get_ = 1;
    std::atomic_uint8_t ptr_set_ = 2;
    
    std::atomic<size_t> data_version{0};
    std::atomic<size_t> last_read_version{0};
};

}