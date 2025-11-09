#pragma once

#include "data/sync_data.hpp"
#include "interfaces/armor_in_camera.hpp"

#include "data/time_stamped.hpp"
#include <memory>

namespace world_exe::data {

struct PredictorUpdatePackage final{
public:

    /**
     * @brief 传感器数据获取时的时间戳
     */
    const data::TimeStamp& GetTimeStamp() const{ return data1_.camera_capture_begin_time_stamp; };

    /**
     * @brief 求解好的装甲板三维信息
     *
     * @return std::shared_ptr<IArmorInCamera>
     */
    std::shared_ptr<world_exe::interfaces::IArmorInCamera> GetArmors() const{return data2_;};

    /**
     * @brief 相机坐标系到世界坐标系的仿射变换
     *
     * @return Eigen::Affine3d
     */
    Eigen::Affine3d GetCameraToWorld() const {return data1_.camera_to_gimbal;};

    // but why?
    PredictorUpdatePackage(const data::CameraGimbalMuzzleSyncData& data1, std::shared_ptr<world_exe::interfaces::IArmorInCamera> data2)
        : data1_(data1)
        , data2_(data2){
    }
    PredictorUpdatePackage()  = delete;
    ~PredictorUpdatePackage() = default;
private:
    const data::CameraGimbalMuzzleSyncData& data1_;
    const std::shared_ptr<world_exe::interfaces::IArmorInCamera> data2_;
};
}