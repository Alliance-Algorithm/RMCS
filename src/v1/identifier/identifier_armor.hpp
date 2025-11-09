#pragma once

#include "interfaces/armor_in_image.hpp"
#include "data/time_stamped.hpp"
#include "util/index.hpp"
#include <ctime>

namespace world_exe::v1::identifier {
class IdentifierArmor final : public interfaces::IArmorInImage {
public:
    IdentifierArmor() = default;
    IdentifierArmor(const std::vector<data::ArmorImageSpacing>& armors) 
    {
        for (const auto armor : armors)
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
    }

    const data::TimeStamp& GetTimeStamp() const override { return time_stamp_; }

    void SetArmors(const std::vector<data::ArmorImageSpacing>& armors) {
        for (auto& armors : armors_)
            armors.clear();
        for (const auto armor : armors)
            armors_[util::enumeration::GetIndex(armor.id)].emplace_back(armor);
    }

    const std::vector<data::ArmorImageSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        return armors_[util::enumeration::GetIndex(armor_id)];
    }

    data::TimeStamp time_stamp_ {};

private:
    std::array<std::vector<data::ArmorImageSpacing>, 8> armors_;
};
}