#pragma once

#include <memory>

#include "data/predictor_update_package.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {

class CarPredictorManager final : public interfaces::ITargetPredictor {
public:
    CarPredictorManager(const std::string& config_path, const double& timeout_sec = 0.1);
    ~CarPredictorManager();

    std ::shared_ptr<interfaces ::IArmorInGimbalControl> Predict(
        const enumeration ::ArmorIdFlag& id, const data::TimeStamp& time_stamp) override;
    std ::shared_ptr<interfaces::IPredictor> GetPredictor(
        const enumeration ::ArmorIdFlag& id) const override;

    void Update(std::shared_ptr<data::PredictorUpdatePackage> data);

    CarPredictorManager(const CarPredictorManager&)                = delete;
    CarPredictorManager& operator=(const CarPredictorManager&)     = delete;
    CarPredictorManager(CarPredictorManager&&) noexcept            = default;
    CarPredictorManager& operator=(CarPredictorManager&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
