#include "car_predictor_manager.hpp"

#include <cstdint>
#include <ctime>
#include <memory>
#include <unordered_map>

#include "../in_gimbal_control_armor.hpp"

#include "car_predictor.hpp"
#include "data/predictor_update_package.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "util/index.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

class CarPredictorManager::Impl {
public:
    Impl(const std::string& config_path, const double& timeout_sec)
        : targets_map_()
        , last_update_timestamp_(data::TimeStamp { })
        , config_path_(config_path) { }

    std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& flag, const data::TimeStamp& time_stamp) {
        std::vector<data::ArmorGimbalControlSpacing> result;

        for (const auto& car_id : util::enumeration::ExpandArmorIdFlags(flag)) {
            if (const auto it = targets_map_.find(car_id); it != targets_map_.end()) {
                const auto& [id, predictor] = *it;
                // if (!predictor->IsConverged()) {
                //     continue;
                // }
                // if (predictor->IsConverged()) 
                {
                    auto spacings = predictor->Predictor(time_stamp);
                    result.insert(result.end(), spacings->GetArmors(id).begin(),
                        spacings->GetArmors(id).end());
                }
            }
        }

        return std::make_shared<InGimbalControlArmor>(result, time_stamp);
    }

    std::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration::ArmorIdFlag& flag) const {
        const auto& it = targets_map_.find(flag);
        if (it == targets_map_.end()) return nullptr;
        const auto& [id, predictor] = *it;
        return std::make_shared<CarPredictor>(
            predictor->GetEkf(), predictor->GetModel(), predictor->LastSeen());
    }

    void Update(std::shared_ptr<data::PredictorUpdatePackage> data) {
        last_update_timestamp_ = data->GetTimeStamp();

        const Eigen::Affine3d transform = data->GetCameraToWorld();
        const auto armors_interface     = data->GetArmors();

        for (int i = 0; i < 8; i++) {
            auto id = static_cast<enumeration::CarIDFlag>(
                static_cast<uint32_t>(enumeration::CarIDFlag::Hero) << i);

            const auto& armors = armors_interface->GetArmors(id);
            if (armors.empty()) continue;

            for (const auto& armor : armors) {
                if (!targets_map_.contains(armor.id)) {
                    targets_map_.try_emplace(armor.id,
                        std::make_unique<CarPredictor>(armor.position,
                            util::math::quaternion_to_euler(armor.orientation, 2, 1, 0), armor.id,
                            data->GetTimeStamp()));
                } else {
                    targets_map_.at(armor.id)->Update(data->GetTimeStamp(), armor.position,
                        util::math::quaternion_to_euler(armor.orientation, 2, 1, 0),
                        util::math::xyz2ypd(armor.position));
                }
            }

            // std::erase_if(targets_map_, [](const auto& pair) { return pair.second->IsAppeared(); });
        }
    }

private:
    std::unordered_map<enumeration::ArmorIdFlag, std::unique_ptr<CarPredictor>> targets_map_;
    data::TimeStamp last_update_timestamp_;

    const std::string config_path_;
};

CarPredictorManager::CarPredictorManager(const std::string& config_path, const double& timeout_sec)
    : pimpl_(std::make_unique<Impl>(config_path, timeout_sec)) { }
CarPredictorManager::~CarPredictorManager() = default;

std ::shared_ptr<interfaces ::IArmorInGimbalControl> CarPredictorManager::Predict(
    const enumeration ::ArmorIdFlag& id, const data::TimeStamp& time_stamp) {
    return pimpl_->Predict(id, time_stamp);
}
std ::shared_ptr<interfaces::IPredictor> CarPredictorManager::GetPredictor(
    const enumeration ::ArmorIdFlag& id) const {
    return pimpl_->GetPredictor(id);
}

void CarPredictorManager::Update(std::shared_ptr<data::PredictorUpdatePackage> data) {
    return pimpl_->Update(data);
}

}
