#pragma once
#include "interfaces/armor_in_image.hpp"
#include "data/time_stamped.hpp"

namespace world_exe::interfaces::detail{
    class ArmorInImage : public world_exe::interfaces::IArmorInImage {
    public:
        ArmorInImage(const std::vector<world_exe::data::ArmorImageSpacing>& armors_)
            : armors_(armors_) {}
        virtual ~ArmorInImage() = default;
        const data::TimeStamp& GetTimeStamp() const override;
        const std::vector<world_exe::data::ArmorImageSpacing> armors_;
        const std::vector<world_exe::data::ArmorImageSpacing>& GetArmors(const enumeration::ArmorIdFlag& armor_id) const override;
    private:
        data::TimeStamp time_stamped_{};

    };
}