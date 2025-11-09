#pragma once

#include <array>
#include <ctime>

#include "interfaces/armor_in_camera.hpp"
#include "data/time_stamped.hpp"
#include "util/index.hpp"

namespace world_exe::tongji::solver {
class SolvedArmor final : public interfaces::IArmorInCamera {
public:
    explicit SolvedArmor(const std::vector<data::ArmorCameraSpacing>& armors, data::TimeStamp when_image_come)
        : time_stamp_(when_image_come) {
        for (const auto& armor : armors) {
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
        }
    }

    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }
    const data::TimeStamp& GetTimeStamp() const override { return time_stamp_; }

private:
    std::array<std::vector<data::ArmorCameraSpacing>, 8> armors_;
    data::TimeStamp time_stamp_;
};
}
