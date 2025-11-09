#pragma once

#include <ctime>
#include <vector>

#include "data/armor_gimbal_control_spacing.hpp"
#include "data/time_stamped.hpp"
#include "enum/armor_id.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"

namespace world_exe::tongji::predictor {

class InGimbalControlArmor final : public interfaces::IArmorInGimbalControl {
public:
    explicit InGimbalControlArmor(const std::vector<data::ArmorGimbalControlSpacing>& all_armors,
        const data::TimeStamp& time_stamp)
        : armors_(all_armors)
        , time_stamp_(time_stamp) { }

    const std ::vector<data ::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration ::ArmorIdFlag& armor_id) const override {
        return armors_;
    }

    const data::TimeStamp& GetTimeStamp() const override { return time_stamp_; }

    InGimbalControlArmor(const InGimbalControlArmor&)                = delete;
    InGimbalControlArmor& operator=(const InGimbalControlArmor&)     = delete;
    InGimbalControlArmor(InGimbalControlArmor&&) noexcept            = default;
    InGimbalControlArmor& operator=(InGimbalControlArmor&&) noexcept = default;

private:
    data::TimeStamp time_stamp_;
    std::vector<data::ArmorGimbalControlSpacing> armors_;
};
}