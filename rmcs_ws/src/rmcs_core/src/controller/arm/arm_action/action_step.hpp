#pragma once
#include <string>
#include <variant>
#include <vector>
#include <map>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace rmcs_core::controller::arm::Action {

enum class MotionType { Pose, Linear, Joint, OpenGripper, CloseGripper,Delay };

struct PoseTarget {
    double x, y, z, roll, pitch, yaw;
};

struct JointTarget {
    double joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;
};

struct LinearTarget {
    double dir_x, dir_y, dir_z, distance;
};

struct NoTarget {};

using Target = std::variant<NoTarget, PoseTarget, JointTarget, LinearTarget>;

struct MotionParams {
    double vel            = 0.05;
    double acc            = 0.03;
    double tolerance_pos  = 0.003;
    double tolerance_ori  = 0.2;
};

class Step {
public:
    // ---------- 构造工厂 ----------
    static Step makeJoint(const JointTarget& target,
                          const MotionParams& params,
                          const std::string& pipeline = "ompl",
                          const std::string& planner  = " ")
    {
        return Step(MotionType::Joint, pipeline, planner, target, params);
    }

    static Step makePose(const PoseTarget& target,
                         const MotionParams& params,
                         const std::string& pipeline = "ompl",
                         const std::string& planner  = " ")
    {
        return Step(MotionType::Pose, pipeline, planner, target, params);
    }

    static Step makeLinear(const LinearTarget& target,
                           const MotionParams& params,
                           const std::string& pipeline = "pilz_industrial_motion_planner",
                           const std::string& planner  = "LIN")
    {
        return Step(MotionType::Linear, pipeline, planner, target, params);
    }

    static Step makeOpenGripper() {
        return Step(MotionType::OpenGripper, "", "", NoTarget{}, MotionParams{});
    }

    static Step makeCloseGripper() {
        return Step(MotionType::CloseGripper, "", "", NoTarget{}, MotionParams{});
    }
    static Step makeDelay() {
        return Step(MotionType::Delay, "", "", NoTarget{}, MotionParams{});
    }


    // ---------- 访问器 ----------
    MotionType type() const { return motion_type_; }
    const std::string& pipelineId() const { return pipeline_id_; }
    const std::string& plannerId() const { return planner_id_; }
    const Target& target() const { return target_; }
    const MotionParams& params() const { return params_; }

private:
    Step(MotionType motion_type,
         std::string pipeline,
         std::string planner,
         Target target,
         MotionParams p)
        : motion_type_(motion_type)
        , pipeline_id_(std::move(pipeline))
        , planner_id_(std::move(planner))
        , target_(target)
        , params_(p)
    {}

    MotionType motion_type_;
    std::string pipeline_id_;
    std::string planner_id_;
    Target target_;
    MotionParams params_;
};

} // namespace rmcs_core::controller::arm::Action