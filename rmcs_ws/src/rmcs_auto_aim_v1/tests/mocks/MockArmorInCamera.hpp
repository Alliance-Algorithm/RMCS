#pragma  once
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <data/armor_camera_spacing.hpp>
#include <data/armor_gimbal_control_spacing.hpp>
#include <data/time_stamped.hpp>
#include <enum/car_id.hpp>
#include <interfaces/armor_in_camera.hpp>
#include <interfaces/armor_in_gimbal_control.hpp>
#include <chrono>
namespace world_exe::tests::mock{
    
class MockArmorInCamera final : public world_exe::interfaces::IArmorInCamera{

    public:
    
    Eigen::Vector3d interpolateOscillating(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double t, double period) {
        // 归一化时间到 [0, 1]
        double phase = std::fmod(t, period) / period;

        // 使用三角波函数实现往返效果
        double alpha = 2.0 * std::abs(phase - 0.5); // 从 0 到 1 再回到 0

        return (1.0 - alpha) * p1 + alpha * p2;
    }
    const enumeration::CarIDFlag armorid = enumeration::CarIDFlag::InfantryIII;
    MockArmorInCamera(double angular_speed = 1,double speed = 1, double delay = 0) 
        : time(std::chrono::steady_clock::now().time_since_epoch())
        , armor()
        , none(){
            armor.emplace_back(
                armorid,
                Eigen::Vector3d{
                    cos((time.to_seconds() + delay) * angular_speed) * 0.2,
                    sin((time.to_seconds() + delay) * angular_speed) * 0.2,
                    0}
                    + interpolateOscillating({3,0.2,0},{4,-0.2,0},time.to_seconds(), 1 / speed), 
                Eigen::Quaterniond{
                Eigen::AngleAxisd(-sin((time.to_seconds() + delay) * angular_speed) , Eigen::Vector3d::UnitZ()).toRotationMatrix()}
            );
    }

    /// 获取时间戳，标志其内容装甲板的准确时间点
    const data::TimeStamp& GetTimeStamp() const{
        return time;
    }

    /// 获取某个车辆ID的装甲板集合
    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const
    {
        if(armor_id == armorid)
            return armor;
        return none;
    }

    private:
    data::TimeStamp                                 time;
    std::vector<data::ArmorCameraSpacing>           armor;
    std::vector<data::ArmorCameraSpacing>           none;
};
}