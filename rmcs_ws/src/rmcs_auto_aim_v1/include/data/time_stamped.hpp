#pragma once

#include <chrono>
#include <ctime>
namespace world_exe::data {
/**
 * @brief 时间戳，通常是SteadyClock
 */
struct TimeStamp {

public:
    
    inline TimeStamp() : stamp_(){}
    inline TimeStamp(const TimeStamp& stamp): stamp_(stamp.stamp_) {}
    inline TimeStamp(const std::chrono::nanoseconds& stamp): stamp_(stamp) {}
    inline TimeStamp(const std::chrono::seconds& stamp): stamp_(std::chrono::duration_cast<std::chrono::nanoseconds>(stamp)) {}

    inline constexpr double to_seconds() const{return std::chrono::duration_cast<std::chrono::duration<double>>(stamp_).count();}
    inline constexpr double to_nanosec() const{return stamp_.count();}

    auto operator<=>(const TimeStamp& other) const noexcept = default;
    inline TimeStamp operator- (const TimeStamp& other) const noexcept {return TimeStamp{stamp_ - other.stamp_};}
    inline TimeStamp operator+ (const TimeStamp& other) const noexcept {return TimeStamp{stamp_ + other.stamp_};}
    inline TimeStamp operator* (const double&    ratio) const noexcept {return from_nanosec(stamp_.count() * ratio);}
    inline TimeStamp operator/ (const double&    ratio) const noexcept {return from_nanosec(stamp_.count() / ratio);}
    
    template<typename T>
    static inline TimeStamp from_seconds(const T time){return TimeStamp{std::chrono::seconds(static_cast<long int>(time))};}
    template<typename T>
    static inline TimeStamp from_nanosec(const T time){return TimeStamp{std::chrono::nanoseconds(static_cast<long int>(time))};}

private:
    std::chrono::nanoseconds stamp_;

};
}