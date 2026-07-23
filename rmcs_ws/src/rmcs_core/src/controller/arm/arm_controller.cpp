#include "controller/arm/arm_action/action_dictionary.hpp"
#include "controller/arm/arm_action/action_step.hpp"
#include "controller/arm/arm_action/arm_action_machine.hpp"
#include "filter/low_pass_filter.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <map>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <numbers>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <sys/cdefs.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/gripper_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/relay_mode.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/normalize_angle.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::controller::arm {

class ArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , action_dictionary_(make_action_parameter_map(get_parameter("action_profile").as_string()))
        , arm_action_machine_()
        , custom_joint_filter_(0.2) {
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch);

        register_output("/arm/mode", arm_mode_);
        register_output("/arm/enable_flag", is_arm_enable, false);

        register_input("/referee/image_transmission/custom", custom_data_);

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", theta[i]);
            register_input(joint_prefix + "/lower_limit", joint_lower_limit_[i]);
            register_input(joint_prefix + "/upper_limit", joint_upper_limit_[i]);
            register_output(joint_prefix + "/target_theta", target_theta[i], NAN);
        }
        register_input("/arm/gripper/motor/angle", gripper_angle_, NAN);
        register_input("/arm/gripper/motor/velocity", gripper_velocity_, NAN);
        register_input("/arm/gripper/motor/torque", gripper_torque_, NAN);
        register_output("/arm/gripper/target_theta", gripper_target_theta, NAN);
        register_output("/arm/gripper/calibration_done", gripper_calibration_trigger_, false);

        register_input("/arm/image_pitch/motor/angle", image_pitch_theta_, NAN);
        register_output("/arm/image_pitch/target_theta", image_pitch_target_theta_, NAN);
        register_output("/arm/custom_big_yaw", custom_big_yaw_output_, 0.0);

        register_input("/leg_back/up_stairs_step", up_stairs_layer, false);
    }

    ~ArmController() {}

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto knob         = *rotary_knob_switch;
        auto keyboard     = *keyboard_;
        auto mouse        = *mouse_;
        using namespace rmcs_msgs;

        static bool initial_check_done{false};
        if (!initial_check_done) {
            *is_arm_enable = false;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                initial_check_done = true;
            }
            reset();
            return;
        }

        if ((switch_left == Switch::DOWN && switch_right == Switch::DOWN)
            || switch_left == Switch::UNKNOWN) {
            *is_arm_enable = false;
            reset();
            return;
        } else {
            *is_arm_enable = true;
        }

        mode_selection();
        arm_control();
        gripper_control();
        image_pitch_control();

        last_switch_left_        = switch_left;
        last_switch_right_       = switch_right;
        last_rotary_knob_switch_ = knob;
        last_keyboard_           = keyboard;
        last_mouse_              = mouse;
        last_gripper_mode_       = get_gripper_mode();
        last_arm_mode_           = get_arm_mode();
    }

private:
    void set_arm_mode(rmcs_msgs::ArmMode mode, bool request_arm_action_machine = true) {
        *arm_mode_ = mode;
        if (request_arm_action_machine) {
            request_trigger_++;
        }
    }
    rmcs_msgs::ArmMode get_arm_mode() const { return *arm_mode_; }

    void set_gripper_mode(rmcs_msgs::GripperMode mode) { gripper_mode_ = mode; }
    rmcs_msgs::GripperMode get_gripper_mode() const { return gripper_mode_; }

    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        auto mouse        = *mouse_;
        auto knob         = *rotary_knob_switch;

        using namespace rmcs_msgs;
        if (switch_left == Switch::UP && switch_right == Switch::UP) {
            if (last_switch_left_ != Switch::UP || last_switch_right_ != Switch::UP) {
                set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Position, false);
            }
        } else if (switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
            if (last_switch_left_ != Switch::UP || last_switch_right_ != Switch::MIDDLE) {
                set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Orientation, false);
            }

        } else if (switch_left == Switch::DOWN && switch_right == Switch::MIDDLE) {
            if (joystick_right_->y() > 0.8) {
                set_gripper_mode(rmcs_msgs::GripperMode::Open);
            }
            if (joystick_right_->y() < -0.8) {
                set_gripper_mode(rmcs_msgs::GripperMode::Close);
            }
        }

        else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (knob != last_rotary_knob_switch_) {
                if (knob == Switch::UP) {
                    image_pitch_theta1_offset_ = 1.2;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Two_Stairs);
                } else if (knob == Switch::DOWN) {
                    image_pitch_theta1_offset_ = 0.70;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Down_Stairs,false);
                }
            }
            if (keyboard.g && !last_keyboard_.g) {
                image_pitch_theta1_offset_ = 0.56;
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Walk);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    image_pitch_theta1_offset_ = 0.70;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Down_Stairs, false);
                }
            }
            if (keyboard.b && !last_keyboard_.b) {
                image_pitch_theta1_offset_ = 0.16;
                if (!keyboard.shift && !keyboard.ctrl) {
                    image_pitch_theta1_offset_ = 1.2;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Two_Stairs);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    image_pitch_theta1_offset_ = 1.2;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_One_Stairs);
                }
            }
            if (keyboard.w && !last_keyboard_.w) {
                image_pitch_theta1_offset_ = 0.16;
                set_arm_mode(rmcs_msgs::ArmMode::Auto_Spin, false);
            }
            if (keyboard.e && !last_keyboard_.e) {
                set_arm_mode(rmcs_msgs::ArmMode::Calibration);
            }
            // A: 左前矿仓 (Left Front)
            if (keyboard.a && !last_keyboard_.a) {
                image_pitch_theta1_offset_ = 0.72;
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_LF);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LF);
                }
            }
            // S: 左后矿仓 (Left Back)
            if (keyboard.s && !last_keyboard_.s) {
                image_pitch_theta1_offset_ = 0.72;
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_LB);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LB);
                }
            }
            // D: 右后矿仓 (Right Back)
            if (keyboard.d && !last_keyboard_.d) {
                image_pitch_theta1_offset_ = 0.72;
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_RB);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RB);
                }
            }
            // F: 右前矿仓 (Right Front)
            if (keyboard.f && !last_keyboard_.f) {
                image_pitch_theta1_offset_ = 0.72;
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_RF);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RF);
                }
            }
            if (keyboard.z && !last_keyboard_.z) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_gripper_mode(rmcs_msgs::GripperMode::Open);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    set_gripper_mode(rmcs_msgs::GripperMode::Close);
                }
            }
            if(keyboard.x&&!last_keyboard_.x){
                set_gripper_mode(rmcs_msgs::GripperMode::Calibrate);
            }
            if (keyboard.r && !last_keyboard_.r) {
                image_pitch_theta1_offset_ = 0.32;
                set_arm_mode(rmcs_msgs::ArmMode::Custome, false);
            }
            if (mouse.left && !last_mouse_.left) {
                image_pitch_theta1_offset_ += 0.08;
            }
            if (mouse.right && !last_mouse_.right) {
                image_pitch_theta1_offset_ -= 0.08;
            }
        } else {
            set_arm_mode(rmcs_msgs::ArmMode::None, false);
        }
    }
    void arm_control() {
        if (request_trigger_ != last_processed_trigger_) {
            last_processed_trigger_ = request_trigger_;
            switch (get_arm_mode()) {
                using namespace rmcs_msgs;
            case ArmMode::Auto_Extract_LF:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("extract_lf"));
                break;
            case ArmMode::Auto_Extract_LB:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("extract_lb"));
                break;
            case ArmMode::Auto_Extract_RF:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("extract_rf"));
                break;
            case ArmMode::Auto_Extract_RB:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("extract_rb"));
                break;
            case ArmMode::Auto_Storage_LF:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("storage_lf"));
                break;
            case ArmMode::Auto_Storage_LB:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("storage_lb"));
                break;
            case ArmMode::Auto_Storage_RF:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("storage_rf"));
                break;
            case ArmMode::Auto_Storage_RB:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("storage_rb"));
                break;
            case ArmMode::Auto_Walk:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("auto_walk"));
                break;
            case ArmMode::Auto_Up_One_Stairs:
                arm_action_machine_.process(action_dictionary_.helper_build_chunk(
                    {"delay", "delay", "up_one_stairs_initial"}));
                break;
            case ArmMode::Auto_Up_Two_Stairs:
                arm_action_machine_.process(action_dictionary_.helper_build_chunk(
                    {"delay", "delay", "up_two_stairs_initial"}));
                break;
            case ArmMode::Calibration:
                arm_action_machine_.process(action_dictionary_.helper_find_chunk("test"));
                break;
            default: break;
            }
        }

        // every update
        switch (*arm_mode_) {
            using namespace rmcs_msgs;
        case rmcs_msgs::ArmMode::None: break;
        case ArmMode::DT7_Control_Position: execute_dt7_position(); break;
        case ArmMode::DT7_Control_Orientation: execute_dt7_orientation(); break;
        case ArmMode::Custome: execute_custom(); break;
        case ArmMode::Auto_Up_One_Stairs:
        case ArmMode::Auto_Up_Two_Stairs: {
            // Lunar rover only: switch arm actions based on stair-stage feedback from the legs.
            if (up_stairs_layer.ready() && *up_stairs_layer != last_up_stairs_layer) {
                last_up_stairs_layer = *up_stairs_layer;
                if (*up_stairs_layer == "initial_again") {
                    arm_action_machine_.process(
                        action_dictionary_.helper_find_chunk("up_two_stairs_initial_again"));
                } else if (*up_stairs_layer == "lift_again") {
                    arm_action_machine_.process(
                        action_dictionary_.helper_find_chunk("up_two_stairs_lift_again"));
                }
            }
        } // Note that there is no break here.
        default: execute_plan_request_and_trajectory_step(); break;
        }
    }

    void execute_plan_request_and_trajectory_step() {
        static int current_step_index_{0};
        static size_t step_position_index_{0};
        static uint64_t last_executed_request_id_{0};
        static std::vector<std::vector<double>> positions;
        const auto result = arm_action_machine_.get_trajectory();
        if (!result || !result->plan_success || result->step_position_map.empty())
            return;

        if (result->request_id != last_executed_request_id_) {
            current_step_index_       = 0;
            step_position_index_      = 0;
            last_executed_request_id_ = result->request_id;
        }
        if (static_cast<size_t>(current_step_index_) >= result->step_position_map.size())
            return;
        if (step_position_index_ == 0) {
            auto step = result->step_map.find(current_step_index_);
            if (step == result->step_map.end())
                return;
            if (step->second.type() == Action::MotionType::CloseGripper) {
                set_gripper_mode(rmcs_msgs::GripperMode::Close);
            } else if (step->second.type() == Action::MotionType::OpenGripper) {
                set_gripper_mode(rmcs_msgs::GripperMode::Open);
            }
            auto pos_it = result->step_position_map.find(current_step_index_);
            if (pos_it == result->step_position_map.end())
                return;
            positions = pos_it->second;
            if (positions.empty())
                return;
        } // boundary
        for (size_t i = 0; i < 6; ++i)
            *target_theta[i] = positions[step_position_index_][i];
        step_position_index_++;
        if (step_position_index_ >= positions.size()) {
            step_position_index_ = 0;
            current_step_index_++;
        }
    }

    void execute_custom() {
        struct __attribute__((packed)) CustomFrame {
            uint16_t big_yaw;
            uint16_t joint[6];
            uint16_t gripper;
        };
        const auto& custom_data = *custom_data_;
        std::array<std::uint8_t, sizeof(CustomFrame)> raw{};
        std::copy_n(custom_data.begin(), raw.size(), raw.begin());
        const auto frame  = std::bit_cast<CustomFrame>(raw);
        constexpr auto pi = std::numbers::pi;

        const auto raw_to_angle = [](std::uint16_t raw, double divisor) {
            auto angle = static_cast<double>(raw) / divisor * 2.0 * pi;
            if (angle > pi) {
                angle -= 2.0 * pi;
            }
            return angle;
        };
        Eigen::Vector<double, 6> angles;
        for (Eigen::Index i = 0; i < angles.size(); ++i) {
            const auto joint_index = static_cast<std::size_t>(i);
            double divisor         = (joint_index == 0 || joint_index == 5) ? 32768.0 : 65536.0;
            angles[i]              = raw_to_angle(frame.joint[joint_index], divisor);
            if (i == 2 || i == 5) {
                angles[i] = -angles[i];
            }
        }
        const auto filtered_angles = custom_joint_filter_.update(angles);
        for (Eigen::Index i = 0; i < filtered_angles.size(); ++i) {
            *target_theta[static_cast<std::size_t>(i)] = filtered_angles[i];
        }

        *custom_big_yaw_output_ = raw_to_angle(frame.big_yaw, 65536.0);
    }
    void execute_dt7_orientation() {
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[5] += 0.003 * joystick_left_->y();
            *target_theta[5] =
                std::clamp(*target_theta[5], *joint_lower_limit_[5], *joint_upper_limit_[5]);
        }
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[4] += 0.003 * joystick_left_->x();
            *target_theta[4] =
                std::clamp(*target_theta[4], *joint_lower_limit_[4], *joint_upper_limit_[4]);
        }
        if (fabs(joystick_right_->y()) > 0.01) {
            *target_theta[3] += 0.003 * joystick_right_->y();
            *target_theta[3] =
                std::clamp(*target_theta[3], *joint_lower_limit_[3], *joint_upper_limit_[3]);
        }
    }
    void execute_dt7_position() {
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[2] += 0.001 * joystick_left_->x();
            *target_theta[2] =
                std::clamp(*target_theta[2], *joint_lower_limit_[2], *joint_upper_limit_[2]);
        }
        if (fabs(joystick_right_->x()) > 0.01) {
            *target_theta[1] += 0.001 * joystick_right_->x();
            *target_theta[1] =
                std::clamp(*target_theta[1], *joint_lower_limit_[1], *joint_upper_limit_[1]);
        }
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[0] += 0.001 * joystick_left_->y();
            *target_theta[0] =
                std::clamp(*target_theta[0], *joint_lower_limit_[0], *joint_upper_limit_[0]);
        }
    }
    static double normalize_angle(double angle) {
        angle = std::fmod(angle + M_PI, 2 * M_PI);
        return angle < 0 ? angle + M_PI : angle - M_PI;
    }
    void gripper_control() {
        const double gripper_step         = this->get_parameter("gripper_step").as_double();
        const double gripper_open_angle   = this->get_parameter("gripper_open_angle").as_double();
        static bool initial_calibration{false};

        const auto gripper_mode  = get_gripper_mode();
        const auto stock_control = [this, gripper_step]() {
            if (std::abs(*gripper_velocity_) < 0.01 && std::abs(*gripper_torque_) > 1.0) {
                *gripper_target_theta = *gripper_angle_;
                return true;
            } else {
                *gripper_target_theta = *gripper_angle_ - gripper_step;
                return false;
            }
        };
        const auto calibrate_zero_point = [this, &stock_control]() {
            *gripper_calibration_trigger_ = false;
            if (stock_control()) {
                *gripper_calibration_trigger_ = true;
                initial_calibration           = true;
                return true;
            } else {
                *gripper_calibration_trigger_ = false;
                initial_calibration           = false;
                return false;
            }
        };

        switch (gripper_mode) {
        case rmcs_msgs::GripperMode::Open:
            if (!initial_calibration) {
                calibrate_zero_point();
            } else {
                *gripper_target_theta =
                    std::min(*gripper_angle_ + gripper_step, gripper_open_angle);
            }
            break;
        case rmcs_msgs::GripperMode::Close:
            if (!initial_calibration) {
                calibrate_zero_point();
            } else {
                stock_control();
            }
            break;
        case rmcs_msgs::GripperMode::Calibrate:
            if (calibrate_zero_point()) {
                set_gripper_mode(rmcs_msgs::GripperMode::None);
            }
            break;
        case rmcs_msgs::GripperMode::None: *gripper_target_theta = *gripper_angle_; break;
        }
    }

    void image_pitch_control() {
        double qmin = -3;
        double qmax = 3;
        this->get_parameter("image_pitch_qmin", qmin);
        this->get_parameter("image_pitch_qmax", qmax);
        if (!std::isfinite(*theta[1]) || !std::isfinite(*image_pitch_theta_)) {
            *image_pitch_target_theta_ = NAN;
            return;
        }
        const double clamped_theta = std::clamp(*theta[1] + image_pitch_theta1_offset_, qmin, qmax);
        const double shortest_error =
            rmcs_utility::normalize_angle(clamped_theta - *image_pitch_theta_);
        *image_pitch_target_theta_ = *image_pitch_theta_ + shortest_error;
    }
    void reset() {
        *is_arm_enable = false;
        for (std::size_t i = 0; i < std::size(theta); ++i) {
            *target_theta[i] = *theta[i];
        }
        *gripper_target_theta      = NAN;
        *image_pitch_target_theta_ = NAN;
        image_pitch_theta1_offset_ = 0.0;
        last_switch_left_          = *switch_left_;
        last_switch_right_         = *switch_right_;
        last_rotary_knob_switch_   = *rotary_knob_switch;
        last_keyboard_             = *keyboard_;
        last_mouse_                = *mouse_;
        custom_joint_filter_.reset();
        set_arm_mode(rmcs_msgs::ArmMode::None, false);
        set_gripper_mode(rmcs_msgs::GripperMode::None);
    }
    ActionDictionary action_dictionary_;
    ActionMachine arm_action_machine_;

    rmcs_msgs::Switch last_switch_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_switch_right_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_rotary_knob_switch_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};
    rmcs_msgs::Mouse last_mouse_{rmcs_msgs::Mouse::zero()};
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6];

    InputInterface<double> gripper_angle_;
    InputInterface<double> gripper_velocity_;
    InputInterface<double> gripper_torque_;
    OutputInterface<double> gripper_target_theta;
    OutputInterface<bool> gripper_calibration_trigger_;

    std::array<InputInterface<double>, 6> joint_lower_limit_;
    std::array<InputInterface<double>, 6> joint_upper_limit_;
    OutputInterface<double> target_theta[6];

    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;
    rmcs_msgs::ArmMode last_arm_mode_{rmcs_msgs::ArmMode::None};
    uint64_t request_trigger_{0};
    uint64_t last_processed_trigger_{0};

    rmcs_msgs::GripperMode gripper_mode_{rmcs_msgs::GripperMode::None};
    rmcs_msgs::GripperMode last_gripper_mode_{rmcs_msgs::GripperMode::None};

    InputInterface<double> image_pitch_theta_;
    double image_pitch_theta1_offset_{-0.5};
    OutputInterface<double> image_pitch_target_theta_;

    InputInterface<std::array<uint8_t, 30>> custom_data_;
    OutputInterface<double> custom_big_yaw_output_;
    filter::LowPassFilter<6> custom_joint_filter_;

    InputInterface<std::string> up_stairs_layer;
    std::string last_up_stairs_layer{"none"};
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)
