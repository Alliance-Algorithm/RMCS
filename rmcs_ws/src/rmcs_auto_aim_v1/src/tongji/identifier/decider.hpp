

#pragma once

#include <memory>
#include <vector>

#include "data/armor_image_spaceing.hpp"
#include "enum/armor_id.hpp"

namespace world_exe::tongji::identifier {

enum PriorityMode { MODE_ONE = 1, MODE_TWO };

enum class ArmorPriority {
    First = 1, //
    Second,    //
    Third,     //
    Forth,     //
    Fifth      //
};

class Decider {
public:
    Decider(PriorityMode mode = PriorityMode::MODE_ONE);
    ~Decider();

    enumeration::ArmorIdFlag GetBestArmor(std::vector<data::ArmorImageSpacing>& armors) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}