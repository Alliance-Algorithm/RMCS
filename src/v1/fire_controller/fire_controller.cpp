
#include "./fire_controller.hpp"
#include "data/time_stamped.hpp"
#include "enum/enum_tools.hpp"
#include "interfaces/armor_in_gimbal_control.hpp"
#include "interfaces/predictor.hpp"
#include "trajectory.hpp"
#include <chrono>
#include <ctime>
#include <memory>

class world_exe::v1::fire_control::TracingFireControl::Impl {
public:
    explicit Impl(double control_delay_in_second, double velocity_begin, double gravity = 9.81)
        : control_delay_((static_cast<time_t>(control_delay_in_second * 1e9)))
        , velocity_begin_(velocity_begin)
        , gravity_(gravity) { }
    const enumeration::CarIDFlag GetAttackCarId() const {
        double max_dot = -2;

        enumeration::CarIDFlag ret = enumeration::ArmorIdFlag::None;
        for (auto i = enumeration::ArmorIdFlag::Hero; i < enumeration::ArmorIdFlag::Count;
            i       = static_cast<decltype(i)>((int)i << 1)) {
            if (world_exe::enumeration::IsFlagContains(tracing_, i)) {
                const auto& armors_vector = armors_->GetArmors(i);
                for (const auto& a : armors_vector) {
                    auto dot = a.position.normalized().dot(Eigen::Vector3d::UnitX());
                    if (dot > max_dot) {
                        ret     = i;
                        max_dot = dot;
                    }
                }
            }
        }
        return ret;
    }
    void SetTargetCarID(const world_exe::enumeration::CarIDFlag& id) { tracing_ = id; }

    void SetTimeStamp(const time_t& time) { time_predict_point_ = time; }

    void SetArmorsInGimbalControl(
        const std::shared_ptr<interfaces::IArmorInGimbalControl>& armors) {
        armors_ = armors;
    };
    void SetPredictor(const std::shared_ptr<interfaces::IPredictor>& predictor) {
        predictor_ = predictor;
    };

    const world_exe::data::FireControl CalculateTarget(const std::chrono::seconds& time_duration) {
        std::chrono::seconds fly_time{0};
        const auto& pre1       = predictor_->Predictor(fly_time + time_duration + control_delay_);
        const auto& pre2       = pre1->GetArmors(predictor_->GetId());
        double min_angular_dis = 1e9;
        int index = -1, index_ = 0;
        for (const auto vec : pre2) {
            const auto angular_dis =
                vec.orientation.angularDistance(Eigen::Quaterniond::Identity());

            if (angular_dis < min_angular_dis) {
                index           = index_;
                min_angular_dis = angular_dis;
            }

            index_++;
        }
        if (index == -1) return no_allow_;

        for (int i = 5; i-- > 0;) {
            const auto& armors_in_gimbal_control =
                predictor_->Predictor(fly_time + time_duration + control_delay_);
            const auto& armors = armors_in_gimbal_control->GetArmors(predictor_->GetId());
            const auto& [fly_time, dir] =
                trajectory_solver::gravity_only(armors[index].position, velocity_begin_, gravity_);
        }

        return { .time_stamp = data::TimeStamp{std::chrono::nanoseconds(fly_time + time_duration + control_delay_)}, .fire_allowance = true };
    }

private:
    world_exe::enumeration::CarIDFlag tracing_ = enumeration::CarIDFlag::None;
    time_t time_predict_point_;
    const std::chrono::seconds control_delay_;
    const double velocity_begin_;
    const double gravity_;
    const world_exe::data::FireControl no_allow_ { .fire_allowance = false };
    std::shared_ptr<interfaces::IArmorInGimbalControl> armors_;
    std::shared_ptr<interfaces::IPredictor> predictor_;
};

const world_exe::data::FireControl //
world_exe::v1::fire_control::TracingFireControl::CalculateTarget(
    const std::chrono::seconds& time_duration) const {
    return pimpl_->CalculateTarget(time_duration);
}

const world_exe::enumeration::CarIDFlag
world_exe::v1::fire_control::TracingFireControl::GetAttackCarId() const {
    return pimpl_->GetAttackCarId();
}
world_exe::v1::fire_control::TracingFireControl::TracingFireControl(
    double t, double velocity_begin, double gravity)
    : pimpl_(std::make_unique<Impl>(t, velocity_begin, gravity)) { }

void world_exe::v1::fire_control::TracingFireControl::set_armor(
    const std::shared_ptr<interfaces::IArmorInGimbalControl>& armors) {
    pimpl_->SetArmorsInGimbalControl(armors);
}

void world_exe::v1::fire_control::TracingFireControl::SetPredictor(
    const std::shared_ptr<interfaces::IPredictor>& predictor) {
    pimpl_->SetPredictor(predictor);
}

void world_exe::v1::fire_control::TracingFireControl::SetTargetCarID(
    const world_exe::enumeration::CarIDFlag& tracing_id) {
    pimpl_->SetTargetCarID(tracing_id);
}

void world_exe::v1::fire_control::TracingFireControl::SetTimeStamp(const time_t& time) {
    pimpl_->SetTimeStamp(time);
}

world_exe::v1::fire_control::TracingFireControl::~TracingFireControl() = default;