#include "predictor_manager.hpp"
#include "car/car_predictor.hpp"
#include "car/car_predictor_ekf.hpp"
#include "data/time_stamped.hpp"
#include "enum/enum_tools.hpp"
#include "predict_armor_in_gimbal_control.hpp"
#include "util/index.hpp"
#include <memory>

namespace world_exe::v1::predictor {

class PredictorManager::Impl {
public:
    inline void Update(const std::shared_ptr<data::PredictorUpdatePackage>& data) {
        const auto time_stamp = data->GetTimeStamp();
        const auto dt         = (time_stamp - last_update_time_stamp_);

        const auto transform          = data->GetCameraToWorld();
        const auto rotation_transform = Eigen::Quaterniond { transform.linear() };

        for (int i = 0; i < 8; i++) {
            const auto& armors = data->GetArmors()->GetArmors(
                static_cast<enumeration::ArmorIdFlag>(0b00000001 << i));
            if (armors.empty()) continue;
            CarPredictEkf::ZVec input;
            // 此处只对识别到一块或两块装甲板时做处理，因为对同一辆车不可能有更多，即便有此处也不应处理，当作异常
            if (armors.size() == 1) {
                const auto tmp_armor = data::ArmorGimbalControlSpacing { armors[0].id,
                    transform * armors[0].position, rotation_transform * armors[0].orientation };
                input << util::math::get_yaw_from_quaternion(tmp_armor.orientation),
                    std::atan(tmp_armor.position.y() / tmp_armor.position.x()),
                    -std::atan(tmp_armor.position.z() / tmp_armor.position.x()),
                    tmp_armor.position.norm();
                predictors_[i].Update(input, {}, dt.to_seconds());
            } else if (armors.size() == 2) {
                // 当同时识别到两块装甲板时，优先更新近的那块，再更新远的
                const auto armor0_yaw = util::math::get_yaw_from_quaternion(armors[0].orientation);
                const auto armor1_yaw = util::math::get_yaw_from_quaternion(armors[1].orientation);

                const auto tmp_armor0 = data::ArmorGimbalControlSpacing { armors[0].id,
                    transform * armors[0].position, rotation_transform * armors[0].orientation };
                const auto tmp_armor1 = data::ArmorGimbalControlSpacing { armors[1].id,
                    transform * armors[1].position, rotation_transform * armors[1].orientation };

                if (std::abs(armor0_yaw) > std::abs(armor1_yaw)) {
                    input << util::math::get_yaw_from_quaternion(tmp_armor0.orientation),
                        std::atan(tmp_armor0.position.y() / tmp_armor0.position.x()),
                        -std::atan(tmp_armor0.position.z() / tmp_armor0.position.x()),
                        tmp_armor0.position.norm();
                    predictors_[i].Update(input, {}, dt.to_seconds());
                    // 同时识别到一辆车的两块装甲板时要调这个函数
                    predictors_[i].set_second_armor();
                    input << util::math::get_yaw_from_quaternion(tmp_armor1.orientation),
                        std::atan(tmp_armor1.position.y() / tmp_armor1.position.x()),
                        -std::atan(tmp_armor1.position.z() / tmp_armor1.position.x()),
                        tmp_armor1.position.norm();
                    predictors_[i].Update(input, {}, 0.);
                } else {
                    input << util::math::get_yaw_from_quaternion(tmp_armor1.orientation),
                        std::atan(tmp_armor1.position.y() / tmp_armor1.position.x()),
                        -std::atan(tmp_armor1.position.z() / tmp_armor1.position.x()),
                        tmp_armor1.position.norm();
                    predictors_[i].Update(input, {}, dt.to_seconds());
                    // 同时识别到一辆车的两块装甲板时要调这个函数
                    predictors_[i].set_second_armor();
                    input << util::math::get_yaw_from_quaternion(tmp_armor0.orientation),
                        std::atan(tmp_armor0.position.y() / tmp_armor0.position.x()),
                        -std::atan(tmp_armor0.position.z() / tmp_armor0.position.x()),
                        tmp_armor0.position.norm();
                    predictors_[i].Update(input, {}, 0.);
                }
            }
        }

        last_update_time_stamp_ = time_stamp;
    }

    inline std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const world_exe::enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp) {
        const auto dt = (time_stamp - last_update_time_stamp_).to_seconds();

        uint32_t id_index = static_cast<uint32_t>(enumeration::ArmorIdFlag::Hero);
        std::array<std::vector<data::ArmorGimbalControlSpacing>, 8> armors;
        for (int i = 0; i < 8; i++)
            if (enumeration::IsFlagContains(
                    id, static_cast<enumeration::ArmorIdFlag>(0b00000001 << i)))
                armors[i] = predictors_[i].get_predict_output_armor(
                    static_cast<enumeration::CarIDFlag>(0b00000001 << i), dt);

        return std::make_shared<PredictArmorInGimbalControl>(armors, last_update_time_stamp_);
    };

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) {
        auto predictor = std::make_shared<CarPredictor>();
        predictor->SetId(id);
        predictor->SetEkf(predictors_[util::enumeration::GetIndex(id)]);
        predictor->SetTimeStamp(last_update_time_stamp_);
        return predictor;
    }

private:
    data::TimeStamp last_update_time_stamp_ {};
    std::array<CarPredictEkf, 8> predictors_;

    Eigen::Affine3d transform_from_camera_to_gimbal_;
    PredictArmorInGimbalControl predictted_armors_;
};

PredictorManager::PredictorManager()
    : pimpl_(std::make_unique<Impl>()) { }

PredictorManager::~PredictorManager() = default;

std::shared_ptr<interfaces::IArmorInGimbalControl> PredictorManager::Predict(
    const world_exe::enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
};

void PredictorManager::Update(std::shared_ptr<data::PredictorUpdatePackage> data) {
    return pimpl_->Update(data);
};

std::shared_ptr<interfaces::IPredictor> PredictorManager::GetPredictor(
    const enumeration::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
};

}