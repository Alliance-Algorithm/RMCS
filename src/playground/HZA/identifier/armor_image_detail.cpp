#include "armor_image_detail.hpp"

namespace world_exe::interfaces::detail {
const std::vector<data::ArmorImageSpacing>& ArmorInImage::GetArmors(const enumeration::ArmorIdFlag& armor_id) const 
{
    static std::vector<world_exe::data::ArmorImageSpacing> result;
    result.clear();
    for(const auto& armor : armors_)
    {
        if(armor.id == armor_id)
        {
            result.push_back(armor);
        }
    }
    return result;
}

const data::TimeStamp& ArmorInImage::GetTimeStamp() const {
    return time_stamped_;
}
}