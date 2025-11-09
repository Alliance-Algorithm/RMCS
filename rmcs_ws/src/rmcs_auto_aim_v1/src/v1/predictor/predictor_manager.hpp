#pragma once

#include "data/time_stamped.hpp"
#include "data/predictor_update_package.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::v1::predictor {
class PredictorManager final : public world_exe::interfaces::ITargetPredictor {
public:
    PredictorManager();
    ~PredictorManager();

    void Update(std::shared_ptr<data::PredictorUpdatePackage> data);

    virtual std::shared_ptr<interfaces::IArmorInGimbalControl> Predict(
        const enumeration::ArmorIdFlag& id, const data::TimeStamp& time_stamp);

    std::shared_ptr<interfaces::IPredictor> GetPredictor(const enumeration::ArmorIdFlag& id) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}