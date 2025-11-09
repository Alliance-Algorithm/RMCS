#include "predict_armor_in_gimbal_control.hpp"

#include "data/time_stamped.hpp"
#include "enum/enum_tools.hpp"
#include "util/index.hpp"
#include <utility>

namespace world_exe::v1::predictor {
class PredictArmorInGimbalControl::Impl {
public:
    Impl() = default;
    Impl(const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        data::TimeStamp predict_time_stamp)
        : armors_(armors)
        , predict_time_stamp_(std::move(predict_time_stamp)) { }

    void Set(const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
        const data::TimeStamp& predict_time_stamp) {
        armors_             = armors;
        predict_time_stamp_ = predict_time_stamp;
    }

    void SetWithSingleId(const std::vector<data::ArmorGimbalControlSpacing>& armors,
        const data::TimeStamp& predict_time_stamp) {
        if (armors.empty()) return;
        armors_[util::enumeration::GetIndex(armors[0].id)] = armors;
    }

    const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) {
        output_armors_.clear();
        output_armors_.reserve(armors_.size() * 2);
        for (int i = 0; i < 8; i++)
            if (enumeration::IsFlagContains(
                    armor_id, static_cast<enumeration::ArmorIdFlag>(0b00000001 << i)))
                output_armors_.insert(output_armors_.end(), armors_[i].begin(), armors_[i].end());

        return output_armors_;
    }

    const data::TimeStamp& GetTimeStamped() const { return predict_time_stamp_; }

private:
    std::array<std::vector<data::ArmorGimbalControlSpacing>, 8> armors_;
    std::vector<data::ArmorGimbalControlSpacing> output_armors_;
    data::TimeStamp predict_time_stamp_;
};

PredictArmorInGimbalControl::PredictArmorInGimbalControl()
    : pimpl_(std::make_unique<Impl>()) { }
PredictArmorInGimbalControl::PredictArmorInGimbalControl(
    const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
    const data::TimeStamp& predict_time_stamp)
    : pimpl_(std::make_unique<Impl>(armors, predict_time_stamp)) { }

void PredictArmorInGimbalControl::Set(
    const std::array<std::vector<data::ArmorGimbalControlSpacing>, 8>& armors,
    const data::TimeStamp& predict_time_stamp) {
    return pimpl_->Set(armors, predict_time_stamp);
}

void PredictArmorInGimbalControl::SetWithSingleId(
    const std::vector<data::ArmorGimbalControlSpacing>& armors,
    const data::TimeStamp& predict_time_stamp) {
    return pimpl_->SetWithSingleId(armors, predict_time_stamp);
}

const std::vector<data::ArmorGimbalControlSpacing>& PredictArmorInGimbalControl::GetArmors(
    const enumeration::ArmorIdFlag& armor_id) const {
    return pimpl_->GetArmors(armor_id);
}

const data::TimeStamp& PredictArmorInGimbalControl::GetTimeStamp() const {
    return pimpl_->GetTimeStamped();
}

PredictArmorInGimbalControl::~PredictArmorInGimbalControl() = default;
}