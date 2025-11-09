#include "state_machine.hpp"

#include <memory>

#include "../identifier/tracker.hpp"
#include "enum/car_id.hpp"

namespace world_exe::tongji::state_machine {

class StateMachine::Impl {
public:
    Impl()
        : tracker_(std::make_unique<identifier::Tracker>())
        , target_id_(enumeration::CarIDFlag::None) { }

    const enumeration::CarIDFlag& GetAllowdToFires() const { return target_id_; }

    void Update(std::shared_ptr<interfaces::IArmorInImage> armors_in_image,
        const enumeration::CarIDFlag& invincible_armors,
        const std::chrono::milliseconds& duration_from_last_update) {

        target_id_ = enumeration::CarIDFlag::None;
        target_id_ = tracker_->SelectTrackingTargetID(
            armors_in_image, invincible_armors, duration_from_last_update);
    }

    void SetLostState() {
        target_id_ = enumeration::CarIDFlag::None;
        tracker_->SetLostState();
    }

private:
    std::unique_ptr<identifier::Tracker> tracker_;
    enumeration::CarIDFlag target_id_;
};

StateMachine::StateMachine()
    : pimpl_(std::make_unique<Impl>()) { }
StateMachine::~StateMachine() { };

const enumeration::CarIDFlag& StateMachine::GetAllowdToFires() const {
    return pimpl_->GetAllowdToFires();
}

void StateMachine::Update(std::shared_ptr<interfaces::IArmorInImage> armors_in_image,
    const enumeration::CarIDFlag& invincible_armors,
    const std::chrono::milliseconds& duration_from_last_update) {
    return pimpl_->Update(armors_in_image, invincible_armors, duration_from_last_update);
}

void StateMachine::SetLostState() { return pimpl_->SetLostState(); }
}
