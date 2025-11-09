
#include "v1/sync/syncer.hpp"
#include "data/sync_data.hpp"
#include "data/time_stamped.hpp"
#include <chrono>
#include <interfaces/sync_block.hpp>
#include <memory>

namespace world_exe::v1 {

struct Syncer::Impl {

public:
    Impl(std::chrono::seconds time_to_hold ,long tolerable_ns) 
        :time_to_hold_(time_to_hold) 
        ,tolerable_ns_(tolerable_ns) {}

    void set_data(const data::CameraGimbalMuzzleSyncData& data) {
        
        data_queue_.emplace_back(data);
        
        auto now_time = 
            data.camera_capture_begin_time_stamp 
           - data::TimeStamp{time_to_hold_}; 

        while(!data_queue_.empty() 
            && data_queue_.front().camera_capture_begin_time_stamp < now_time) 
        data_queue_.pop_front();
    }

    std::tuple<data::CameraGimbalMuzzleSyncData, bool> get_data(const data::TimeStamp&  timestamp) {

        if (data_queue_.empty())
            return {data::CameraGimbalMuzzleSyncData{}, false};

        for(auto rit = data_queue_.rbegin(); rit != data_queue_.rend(); ++rit) 
            if (rit->camera_capture_begin_time_stamp <= timestamp) 
                return {*rit, true};
            
        return {data::CameraGimbalMuzzleSyncData{},false};
    }

    ~Impl() = default;

private:

    std::list<data::CameraGimbalMuzzleSyncData> data_queue_;

    const std::chrono::seconds  time_to_hold_;
    const long                  tolerable_ns_;
};

Syncer::Syncer(std::chrono::seconds time_to_hold, long tolerable_ns)
    : pimpl_(std::make_unique<Impl>(time_to_hold, tolerable_ns)) { }

Syncer::~Syncer() = default;

void Syncer::set_data(const data::CameraGimbalMuzzleSyncData& armor_pnp) {
    pimpl_->set_data(armor_pnp);
}
std::tuple<data::CameraGimbalMuzzleSyncData, bool> Syncer::get_data(const data::TimeStamp& timestamp) {
    return pimpl_->get_data(timestamp);
}


}