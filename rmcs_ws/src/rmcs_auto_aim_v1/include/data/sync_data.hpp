#pragma once

#include "data/time_stamped.hpp"
#include <Eigen/Eigen>
#include <ctime>
namespace world_exe::data {
struct CameraGimbalMuzzleSyncData {
    data::TimeStamp camera_capture_begin_time_stamp;
    Eigen::Affine3d camera_to_gimbal;
    Eigen::Affine3d gimbal_to_muzzle;
};
}