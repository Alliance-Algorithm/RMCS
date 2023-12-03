#include <algorithm>

#include "chasiscontroll.hpp"

namespace ChasisController {
ChasisControll::ChasisControll(size_t size) {
    ChasisPidData_.reserve(size);
    ChasisFeedbackCmdData_.reserve(size);
}

ChasisControll::~ChasisControll(void) {}

void ChasisControll::AddChasisHandle(
    const hardware_interface::LoanedStateInterface& feedback_velocity,
    hardware_interface::LoanedCommandInterface& command_effort) {

    ChasisFeedbackCmdData_.emplace_back(
        ChasisHandle{std::ref(feedback_velocity), std::ref(command_effort)});
}
void ChasisControll::AddChasisPid(const PID::PID& PidData) {
    // std::unique_ptr<PID::PID> NewPidDataPtr;
    // NewPidDataPtr.reset(new PID::PID(PidData));
    ChasisPidData_.emplace_back(std::unique_ptr<PID::PID>(new PID::PID(PidData)));
}

void ChasisControll::Update(double chassis_vx, double chassis_vy, double chassis_vw) {
    if ((ChasisPidData_.size() == 4) && (ChasisFeedbackCmdData_.size() == 4)) {
        ChasisCal(chassis_vx, chassis_vy, chassis_vw);
        for (auto i = 0; i < ChasisPidData_.size(); i++) {
            ChasisFeedbackCmdData_.at(i).command_effort_.get().set_value(
                ChasisPidData_.at(i)->update(
                    ChasisFeedbackCmdData_.at(i).feedback_velocity_.get().get_value())); // 计算pid
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("ChasisController"), "ChasisPidData_ Too Less!\n");
    }
}
constexpr double LF_CENTER = 1;
constexpr double RF_CENTER = 1;
constexpr double LB_CENTER = 1;
constexpr double RB_CENTER = 1;

void ChasisControll::ChasisCal(double chassis_vx, double chassis_vy, double chassis_vw) {

    double vt_lf = -chassis_vx - chassis_vy - chassis_vw * LF_CENTER;
    double vt_rf = -chassis_vx + chassis_vy - chassis_vw * RF_CENTER;
    double vt_lb = chassis_vx - chassis_vy - chassis_vw * LB_CENTER;
    double vt_rb = chassis_vx + chassis_vy - chassis_vw * RB_CENTER;

    ChasisLimit(); // 底盘限速
    if (ChasisPidData_.size() == 4) {
        ChasisPidData_.at(0)->setPoint(vt_lf);
        ChasisPidData_.at(1)->setPoint(vt_rf);
        ChasisPidData_.at(2)->setPoint(vt_lb);
        ChasisPidData_.at(3)->setPoint(vt_rb);
    }
}

void ChasisControll::ChasisLimit() {}
} // namespace ChasisController
