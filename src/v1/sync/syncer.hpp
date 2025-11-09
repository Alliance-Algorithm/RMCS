#pragma once

#include "data/sync_data.hpp"
#include "data/predictor_update_package.hpp"
#include <chrono>
#include <interfaces/sync_block.hpp>
#include <memory>
#include <opencv2/core/mat.hpp>

namespace world_exe::v1 {
class Syncer final : interfaces::ISyncBlock<data::CameraGimbalMuzzleSyncData> {
public:
    Syncer(std::chrono::seconds time_to_hold,long tolerable_ns = 4e6);
    ~Syncer();

    void set_data(const data::CameraGimbalMuzzleSyncData& camera_data);

    std::tuple<data::CameraGimbalMuzzleSyncData, bool> get_data(const data::TimeStamp& timestamp);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    std::shared_ptr<data::PredictorUpdatePackage> last_;
};
}