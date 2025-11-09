#include "decider.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"

namespace world_exe::tongji::identifier {

class Decider::Impl {
public:
    explicit Impl(PriorityMode mode = PriorityMode::MODE_ONE)
        : mode_(mode) { }

    enumeration::ArmorIdFlag GetBestArmor(std::vector<data::ArmorImageSpacing>& armors) const {
        if (armors.empty()) return enumeration::ArmorIdFlag::None;

        const PriorityMap& priority_map = (mode_ == PriorityMode::MODE_ONE) ? mode1 : mode2;
        std::vector<std::pair<enumeration::ArmorIdFlag, ArmorPriority>> armors_list;
        for (const auto& armor : armors) {
            armors_list.emplace_back(armor.id, priority_map.at(armor.id));
        }

        cv::Point2d img_center(1440.0 / 2, 1080.0 / 2); // TODO
        std::sort(armors.begin(), armors.end(), [&](const auto& a, const auto& b) {
            auto center_a = (a.image_points[0] + a.image_points[3]) * 0.5f;
            auto center_b = (b.image_points[0] + b.image_points[3]) * 0.5f;
            return cv::norm(center_a - img_center) < cv::norm(center_b - img_center);
        });

        std::sort(armors_list.begin(), armors_list.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        return armors.front().id;
    }

private:
    using ArmorPriority = world_exe::tongji::identifier::ArmorPriority;
    using ArmorId       = world_exe::enumeration::ArmorIdFlag;

    using PriorityMap = std::unordered_map<ArmorId, ArmorPriority>;

    const PriorityMap mode1 = {
        { ArmorId::Hero, ArmorPriority::Second },       //
        { ArmorId::Engineer, ArmorPriority::Forth },    //
        { ArmorId::InfantryIII, ArmorPriority::First }, //
        { ArmorId::InfantryIV, ArmorPriority::First },  //
        { ArmorId::InfantryV, ArmorPriority::Third },   //
        { ArmorId::Sentry, ArmorPriority::Third },      //
        { ArmorId::Outpost, ArmorPriority::Fifth },     //
        { ArmorId::Base, ArmorPriority::Fifth },        //
        { ArmorId::Unknow, ArmorPriority::Fifth }       //
    };

    const PriorityMap mode2 = {
        { ArmorId::Hero, ArmorPriority::Second },       //
        { ArmorId::Engineer, ArmorPriority::Forth },    //
        { ArmorId::InfantryIII, ArmorPriority::First }, //
        { ArmorId::InfantryIV, ArmorPriority::First },  //
        { ArmorId::InfantryV, ArmorPriority::Third },   //
        { ArmorId::Sentry, ArmorPriority::Third },      //
        { ArmorId::Outpost, ArmorPriority::Fifth },     //
        { ArmorId::Base, ArmorPriority::Fifth },        //
        { ArmorId::Unknow, ArmorPriority::Fifth }       //
    };
    PriorityMode mode_;
};

Decider::Decider(PriorityMode mode)
    : pimpl_(std::make_unique<Impl>(mode)) { }
Decider::~Decider() = default;

enumeration::ArmorIdFlag Decider::GetBestArmor(std::vector<data::ArmorImageSpacing>& armors) const {
    return pimpl_->GetBestArmor(armors);
}
}
