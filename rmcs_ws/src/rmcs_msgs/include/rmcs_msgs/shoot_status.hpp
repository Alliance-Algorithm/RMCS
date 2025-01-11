#pragma once

namespace rmcs_msgs {

struct ShootStatus {
    bool ready;
    int fired_count;
    int jammed_count;
    int dry_fed_count;
    int single_shot_cancelled_count;
};

} // namespace rmcs_msgs