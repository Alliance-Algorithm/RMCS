#include "fire_controller.hpp"

#include <chrono>
#include <memory>

#include <yaml-cpp/yaml.h>

#include "../identifier/identified_armor.hpp"
#include "../state_machine/state_machine.hpp"
#include "aim_solver.hpp"
#include "data/fire_control.hpp"
#include "fire_decision.hpp"
#include "interfaces/target_predictor.hpp"
#include "tongji/predictor/car_predictor/car_predictor_manager.hpp"

namespace world_exe::tongji::fire_control {

using StateMachine        = state_machine::StateMachine;
using IdentifiedArmor     = identifier::IdentifiedArmor;
using CarIDFlag           = enumeration::CarIDFlag;
using CarPredictorManager = predictor ::CarPredictorManager;
using TimeStamp           = data::TimeStamp;

class FireController::Impl {
public:
    Impl(const std::string& config_path, std::shared_ptr<interfaces::ICarState> state_machine,
        std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
        : locked_target(CarIDFlag::None)
        , firable_(false)
        , aiming_solver_(std::make_unique<AimingSolver>(config_path))
        , fire_decision_(std::make_unique<FireDecision>(config_path))
        , state_machine_(state_machine)
        , live_target_manager_(live_target_manager) {

        auto yaml        = YAML::LoadFile(config_path);
        control_delay_s_ = yaml["control_delay_s"].as<double>();
    }

    const data ::FireControl CalculateTarget(
        const std::chrono::seconds& time_from_tracker_timepoint) const {

        if (!fire_decision_ || !state_machine_ || !live_target_manager_)
            return { .fire_allowance = false };

        const auto& lockable_target  = state_machine_->GetAllowdToFires();
        const auto& snapshot_manager = live_target_manager_->GetPredictor(lockable_target);
        if (!snapshot_manager)
            return data::FireControl { .time_stamp =
                                           data::TimeStamp { time_from_tracker_timepoint },
                .gimbal_dir = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
                .fire_allowance = false };
        // const auto& armors_in_gimbal = snapshot_manager->Predictor(time_from_tracker_timepoint);
        // TODO:这里不应该指针转换
        const auto& aim_solution = aiming_solver_->SolveAimSolution(
            snapshot_manager, time_from_tracker_timepoint, control_delay_s_);

        const auto gimbal_command = GimbalCommand { aim_solution.yaw, aim_solution.pitch };
        const auto target_pos     = Eigen::Vector3d { aim_solution.aim_point };
        auto fire_command         = aim_solution.valid
                    ? fire_decision_->ShouldFire(gimbal_yaw_, gimbal_command, target_pos)
                    : false;
        firable_                  = fire_command;

        data::FireControl result;
        result.fire_allowance = fire_command;
        result.gimbal_dir << gimbal_command.yaw, gimbal_command.pitch, 0;
        result.time_stamp = data::TimeStamp { time_from_tracker_timepoint };
        return result;
    }

    const CarIDFlag GetAttackCarId() const {
        if (firable_) return locked_target;
        return CarIDFlag::None;
    }

    void UpdateGimbalPosition(const double& gimbal_yaw) { gimbal_yaw_ = gimbal_yaw; };

private:
    double control_delay_s_;

    double gimbal_yaw_;

    CarIDFlag locked_target;
    mutable double firable_;

    std::unique_ptr<AimingSolver> aiming_solver_;
    std::unique_ptr<FireDecision> fire_decision_;
    std::shared_ptr<interfaces::ICarState> state_machine_;
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager_;
};

FireController::FireController(const std::string& config_path,
    std::shared_ptr<interfaces::ICarState> state_machine,
    std::shared_ptr<interfaces::ITargetPredictor> live_target_manager)
    : pimpl_(std::make_unique<Impl>(config_path, state_machine, live_target_manager)) { }
FireController::~FireController() = default;

const data ::FireControl FireController::CalculateTarget(
    const std::chrono::seconds& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}
const CarIDFlag FireController::GetAttackCarId() const { return pimpl_->GetAttackCarId(); }

void FireController::UpdateGimbalPosition(const double& gimbal_yaw) {
    return pimpl_->UpdateGimbalPosition(gimbal_yaw);
};

}
