#pragma once

#include <chrono>
#include <memory>

#include "interfaces/car_state.hpp"
#include "interfaces/fire_controller.hpp"
#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::fire_control {

class FireController final : public interfaces::IFireControl {
public:
    FireController(const std::string& config_path,
        std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager);
    ~FireController();

    const data ::FireControl CalculateTarget(
        const std::chrono::seconds& time_duration) const override;
    const enumeration ::CarIDFlag GetAttackCarId() const override;

    void UpdateGimbalPosition(const double& gimbal_yaw);

    FireController(const FireController&)                = delete;
    FireController& operator=(const FireController&)     = delete;
    FireController(FireController&&) noexcept            = default;
    FireController& operator=(FireController&&) noexcept = default;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}
