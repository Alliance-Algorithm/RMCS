#pragma once

#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "pid.hpp"

namespace ChasisController {

class ChasisControll {
private:
    /* data */
    struct ChasisHandle {

        std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback_velocity_;

        std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_effort_;
        /* data */
    };

    std::vector<std::unique_ptr<PID::PID>> ChasisPidData_;
    std::vector<ChasisHandle> ChasisFeedbackCmdData_;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

public:
    ChasisControll(size_t size);

    void BindNode(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) { node_ = node; }
    void AddChasisHandle(
        const hardware_interface::LoanedStateInterface& feedback_velocity,
        hardware_interface::LoanedCommandInterface& command_effort);
    void AddChasisPid(const PID::PID& PidData);
    void ChasisCal(double chassis_vx, double chassis_vy, double chassis_vw);
    void ChasisLimit(); // 功率限制
    void Update(double chassis_vx, double chassis_vy, double chassis_vw);
    std::size_t GetFeedbackDataSize() { return ChasisFeedbackCmdData_.size(); }
    std::size_t GetPidDataSize() { return ChasisPidData_.size(); }
    void PublishChasisData() {}
    ~ChasisControll(void);
};

} // namespace ChasisController