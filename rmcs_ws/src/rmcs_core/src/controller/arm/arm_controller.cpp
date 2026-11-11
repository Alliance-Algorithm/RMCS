#include "controller/arm/arm_MTC/MTC_execution.hpp"
#include "controller/arm/arm_MTC/MTC_machine.hpp"
#include "controller/arm/arm_MTC/MTC_parameter_map.hpp"
#include "controller/arm/obstacle/obstacle_coordinates_map.hpp"
#include "controller/arm/obstacle/obstacle_course.hpp"
#include "filter/low_pass_filter.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <numbers>
#include <optional>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/gripper_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/normalize_angle.hpp>

namespace rmcs_core::controller::arm {

class ArmController final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , mtc_dictionary_(MTC::make_default_dictionary())
        , mtc_recipes_(make_mtc_recipes())
        , chassis_type_(get_parameter("chassis_type").as_string())
        , mtc_machine_()
        , obstacle_coordinates_map_()
        , obstacle_course_(mtc_machine_.moveit_group_getter().get(), obstacle_coordinates_map_)
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

        // Compose every recipe once without runtime targets so unknown blocks and unbound
        // objects fail at load, not in update().
        for (const auto& [mode, recipe] : mtc_recipes_)
            mtc_dictionary_.helper_build_chunk(recipe.blocks, recipe.bindings);

        worker_thread_ = std::thread([this] { execution_worker(); });
    }

    ~ArmController() {
        requested_generation_.store(0, std::memory_order_release);
        worker_running_.store(false, std::memory_order_release);
        if (worker_thread_.joinable())
            worker_thread_.join();
        mtc_machine_.stop();
    }

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto knob = *rotary_knob_switch;
        auto keyboard = *keyboard_;
        auto mouse = *mouse_;
        using namespace rmcs_msgs;

        bool feedback_valid = true;
        for (const auto& joint : theta)
            feedback_valid = feedback_valid && std::isfinite(*joint);
        if (!feedback_valid) {
            feedback_fault_ = true;
            cancel_execution();
            *is_arm_enable = false;
            *gripper_target_theta = NAN;
            *image_pitch_target_theta_ = NAN;
            return;
        }
        if (feedback_fault_) {
            reset();
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN)
                feedback_fault_ = false;
            return;
        }

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

        last_switch_left_ = switch_left;
        last_switch_right_ = switch_right;
        last_rotary_knob_switch_ = knob;
        last_keyboard_ = keyboard;
        last_mouse_ = mouse;
        last_gripper_mode_ = get_gripper_mode();
        last_arm_mode_ = get_arm_mode();
    }

private:
    struct MtcRecipe {
        std::vector<std::string> blocks;
        MTC::ObjectBindings bindings{};
        // Evaluated on the control thread when the task is triggered, so it may read
        // controller state; the result is composed into the task on the worker.
        std::function<MTC::StageTargetOverrides()> targets{};
    };
    using MtcRecipes = std::unordered_map<rmcs_msgs::ArmMode, MtcRecipe>;

    // Automatic modes without an entry submit nothing.
    static MtcRecipes make_mtc_recipes() {
        using rmcs_msgs::ArmMode;
        return {
            {ArmMode::Auto_Extract_LF,
             MtcRecipe{
                 .blocks = {"current_start", "extra_left_front", "extra", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_left_front"}}}},
            {ArmMode::Auto_Storage_LF,
             MtcRecipe{
                 .blocks = {"current_start", "store_left_front", "store", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_left_front"}}}},
            {ArmMode::Auto_Extract_LB,
             MtcRecipe{
                 .blocks = {"current_start", "extra_left_back", "extra", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_left_back"}}}},
            {ArmMode::Auto_Storage_LB,
             MtcRecipe{
                 .blocks = {"current_start", "store_left_back", "store", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_left_back"}}}},
            {ArmMode::Auto_Extract_RF,
             MtcRecipe{
                 .blocks = {"current_start", "extra_right_front", "extra", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_right_front"}}}},
            {ArmMode::Auto_Storage_RF,
             MtcRecipe{
                 .blocks = {"current_start", "store_right_front", "store", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_right_front"}}}},
            {ArmMode::Auto_Extract_RB,
             MtcRecipe{
                 .blocks = {"current_start", "extra_right_back", "extra", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_right_back"}}}},
            {ArmMode::Auto_Storage_RB,
             MtcRecipe{
                 .blocks = {"current_start", "store_right_back", "store", "auto_walk"},
                 .bindings = {{"unit", "energy_unit_right_back"}}}},
            {ArmMode::Auto_Walk, MtcRecipe{.blocks = {"current_start", "auto_walk"}}},
            {ArmMode::Calibration, MtcRecipe{.blocks = {"current_start", "test"}}},
            // {ArmMode::Auto_Up_Two_Stairs, MtcRecipe{.blocks = {"current_start",
            // "auto_up_two_stairs"}}}, {ArmMode::Auto_Up_One_Stairs, MtcRecipe{.blocks =
            // {"current_start", "auto_up_one_stairs"}}}, {ArmMode::Auto_Down_Stairs,
            // MtcRecipe{.blocks = {"current_start", "auto_down_stairs"}}},

        };
    }

    void set_arm_mode(rmcs_msgs::ArmMode mode, bool request_mtc = true) {
        *arm_mode_ = mode;
        if (request_mtc) {
            request_trigger_++;
        }
    }
    rmcs_msgs::ArmMode get_arm_mode() const { return *arm_mode_; }

    void set_gripper_mode(rmcs_msgs::GripperMode mode) { gripper_mode_ = mode; }
    rmcs_msgs::GripperMode get_gripper_mode() const { return gripper_mode_; }

    void mode_selection() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto keyboard = *keyboard_;
        auto mouse = *mouse_;
        auto knob = *rotary_knob_switch;

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
                if (requested_generation_.load(std::memory_order_acquire) != 0)
                    cancel_execution();
                set_gripper_mode(rmcs_msgs::GripperMode::Open);
            }
            if (joystick_right_->y() < -0.8) {
                if (requested_generation_.load(std::memory_order_acquire) != 0)
                    cancel_execution();
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
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Two_Stairs);
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
                } else if (!keyboard.shift && keyboard.ctrl) {
                    image_pitch_theta1_offset_ = 1.2;
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Compensation);
                }
            }
            if (keyboard.w && !last_keyboard_.w) {
                image_pitch_theta1_offset_ = 0.16;
                set_arm_mode(rmcs_msgs::ArmMode::Auto_Spin, false);
            }
            if (keyboard.e && !last_keyboard_.e) {
                if (!keyboard.shift && keyboard.ctrl)
                    set_arm_mode(rmcs_msgs::ArmMode::Calibration);
                else if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Yaw_Close);
                }
            }
            // A: 左前矿仓 (Left Front)
            if (keyboard.a && !last_keyboard_.a) {
                image_pitch_theta1_offset_ = 0.72;
                if (!keyboard.shift && keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_LF);
                } else if (!keyboard.ctrl && keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LF);
                }
            }
            // S: 左后矿仓 (Left Back)
            if (keyboard.s && !last_keyboard_.s) {
                image_pitch_theta1_offset_ = 0.72;
                if (!keyboard.shift && keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_LB);
                } else if (!keyboard.ctrl && keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LB);
                }
            }
            // D: 右后矿仓 (Right Back)
            if (keyboard.d && !last_keyboard_.d) {
                image_pitch_theta1_offset_ = 0.72;
                if (!keyboard.shift && keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_RB);
                } else if (!keyboard.ctrl && keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RB);
                }
            }
            // F: 右前矿仓 (Right Front)
            if (keyboard.f && !last_keyboard_.f) {
                image_pitch_theta1_offset_ = 0.72;
                if (!keyboard.shift && keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Extract_RF);
                } else if (!keyboard.ctrl && keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RF);
                }
            }
            if (keyboard.c && !last_keyboard_.c) {
                image_pitch_theta1_offset_ = 0.48;
                if (!keyboard.shift && !keyboard.ctrl) {
                    if (chassis_type_ == "lunar_rover") {
                        set_arm_mode(rmcs_msgs::ArmMode::Auto_Five_Mine);
                    } else if (chassis_type_ == "climber") {
                        set_arm_mode(rmcs_msgs::ArmMode::Auto_Three_Mine_First);
                    }
                } else if (!keyboard.ctrl && keyboard.shift && chassis_type_ == "climber") {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Three_Mine_Second);
                }
            }
            if (keyboard.z && !last_keyboard_.z) {
                if (requested_generation_.load(std::memory_order_acquire) != 0)
                    cancel_execution();
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_gripper_mode(rmcs_msgs::GripperMode::Open);
                } else if (keyboard.shift && keyboard.ctrl) {
                    set_gripper_mode(rmcs_msgs::GripperMode::Calibrate);
                }
            }
            if (keyboard.x && !last_keyboard_.x) {
                if (requested_generation_.load(std::memory_order_acquire) != 0)
                    cancel_execution();
                set_gripper_mode(rmcs_msgs::GripperMode::Close);
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
        if (get_arm_mode() != last_arm_mode_
            && requested_generation_.load(std::memory_order_acquire) != 0)
            cancel_execution();

        if (request_trigger_ != last_processed_trigger_) {
            last_processed_trigger_ = request_trigger_;
            cancel_execution();
            if (const auto recipe = mtc_recipes_.find(get_arm_mode()); recipe != mtc_recipes_.end())
                submit_mtc_task(recipe->second);
        }

        switch (get_arm_mode()) {
            using namespace rmcs_msgs;
        case ArmMode::None: break;
        case ArmMode::DT7_Control_Position: execute_dt7_position(); break;
        case ArmMode::DT7_Control_Orientation: execute_dt7_orientation(); break;
        case ArmMode::Custome: execute_custom(); break;
        default: execute_mtc_step(); break;
        }
    }

    struct TaskRequest {
        std::uint64_t generation;
        const MtcRecipe* recipe; // Owned by mtc_recipes_, immutable after construction.
        MTC::StageTargetOverrides targets;
    };

    void submit_mtc_task(const MtcRecipe& recipe) {
        // Snapshot runtime targets here; the worker composes the task off the control loop.
        MTC::StageTargetOverrides targets;
        if (recipe.targets) {
            try {
                targets = recipe.targets();
            } catch (const std::exception& error) {
                fail_execution(std::string("cannot evaluate task targets: ") + error.what());
                return;
            }
        }
        auto request = std::make_shared<TaskRequest>(
            TaskRequest{++generation_counter_, &recipe, std::move(targets)});
        pending_task_.store(request, std::memory_order_release);
        requested_generation_.store(request->generation, std::memory_order_release);
    }

    struct ExecutionResult {
        std::uint64_t generation = 0;
        std::optional<MTC::PreparedExecution> execution;
        std::string error;
    };

    struct SceneJob {
        std::uint64_t generation;
        std::size_t step;
        std::shared_ptr<const ExecutionResult> plan;
    };

    struct SceneResult {
        std::uint64_t generation;
        std::size_t step;
        std::string error;
    };

    // Control-loop progress through one prepared execution.
    struct ActiveRun {
        std::shared_ptr<const ExecutionResult> result;
        std::size_t step = 0;
        std::size_t cursor = 0;
        bool entered = false;
        std::chrono::steady_clock::time_point started{};
    };

    void cancel_execution() {
        requested_generation_.store(0, std::memory_order_release);
        pending_task_.store(nullptr, std::memory_order_release);
        run_.reset();
        for (std::size_t joint = 0; joint < 6; ++joint)
            if (std::isfinite(*theta[joint]))
                *target_theta[joint] = *theta[joint];
        // The gripper keeps its mode so a held unit survives task switches; reset() releases it.
        // An in-flight scene service cannot be rolled back; its result stays in the world.
    }

    void fail_execution(const std::string& reason) {
        const auto path = run_ && run_->step < run_->result->execution->plan.steps.size()
                            ? run_->result->execution->plan.steps[run_->step].stage_path
                            : "planning";
        RCLCPP_ERROR(get_logger(), "MTC execution stopped at %s: %s", path.c_str(), reason.c_str());
        cancel_execution();
    }

    bool at_position(const std::array<double, 6>& position, double tolerance) const {
        for (std::size_t joint = 0; joint < 6; ++joint)
            if (!std::isfinite(*theta[joint])
                || std::abs(*theta[joint] - position[joint]) > tolerance)
                return false;
        return true;
    }

    void execute_mtc_step() {
        const auto generation = requested_generation_.load(std::memory_order_acquire);
        if (generation == 0)
            return;
        if (!run_) {
            const auto result = prepared_execution_.load(std::memory_order_acquire);
            if (!result || result->generation != generation)
                return;
            if (!result->execution) {
                fail_execution(result->error);
                return;
            }
            run_.emplace(ActiveRun{.result = result});
        }
        auto& run = *run_;
        const auto& execution = *run.result->execution;
        if (run.step >= execution.plan.steps.size()) {
            RCLCPP_INFO(
                get_logger(), "MTC execution completed: %s", execution.plan.task_name.c_str());
            requested_generation_.store(0, std::memory_order_release);
            run_.reset();
            return;
        }
        const auto& step = execution.plan.steps[run.step];
        const auto& motion = execution.motions[run.step];
        const auto now = std::chrono::steady_clock::now();
        if (!run.entered) {
            run.started = now;
            run.cursor = 0;
            if (const auto* command = std::get_if<MTC::GripperCommandConfig>(&step.action)) {
                set_gripper_mode(
                    command->action == MTC::GripperAction::Open ? rmcs_msgs::GripperMode::Open
                                                                : rmcs_msgs::GripperMode::Close);
            } else if (!motion) {
                scene_job_.store(
                    std::make_shared<SceneJob>(SceneJob{generation, run.step, run.result}),
                    std::memory_order_release);
            }
            run.entered = true;
        }
        const double elapsed = std::chrono::duration<double>(now - run.started).count();
        if (motion) {
            const auto position = motion->sample(elapsed, run.cursor);
            for (std::size_t joint = 0; joint < 6; ++joint) {
                if (!std::isfinite(*theta[joint]) || !std::isfinite(*joint_lower_limit_[joint])
                    || !std::isfinite(*joint_upper_limit_[joint])
                    || position[joint] < *joint_lower_limit_[joint]
                    || position[joint] > *joint_upper_limit_[joint]) {
                    fail_execution("invalid measured state or motion target outside joint limits");
                    return;
                }
            }
            for (std::size_t joint = 0; joint < 6; ++joint)
                *target_theta[joint] = position[joint];
            if (elapsed < motion->duration())
                return;
            if (!at_position(motion->positions.back(), 0.05)) {
                if (elapsed < motion->duration() + 1.0)
                    return;
                RCLCPP_WARN(
                    get_logger(),
                    "Motion at %s did not reach its endpoint within 1 second (0.05 rad); "
                    "continuing task",
                    step.stage_path.c_str());
            }
        } else if (const auto* command = std::get_if<MTC::GripperCommandConfig>(&step.action)) {
            const auto expected = command->action == MTC::GripperAction::Open
                                    ? rmcs_msgs::GripperMode::Open
                                    : rmcs_msgs::GripperMode::Close;
            if (get_gripper_mode() != expected) {
                set_gripper_mode(expected);
            }
            // Preserve the old 500-update dwell at 1000 Hz; this is not grasp feedback.
            if (elapsed < 0.5)
                return;
        } else {
            const auto result = scene_result_.load(std::memory_order_acquire);
            if (!result || result->generation != generation || result->step != run.step)
                return;
            if (!result->error.empty()) {
                RCLCPP_WARN(
                    get_logger(), "Scene update failed at %s: %s; continuing task",
                    step.stage_path.c_str(), result->error.c_str());
            }
        }
        ++run.step;
        run.entered = false;
    }

    bool apply_scene_operation(
        const MTC::SceneOperation& operation, const std::function<bool()>& can_apply) {
        return std::visit(
            [this, &can_apply](const auto& action) {
                using Operation = std::decay_t<decltype(action)>;
                if constexpr (std::is_same_v<Operation, MTC::AllowCollisionsConfig>) {
                    return obstacle_course_.set_collisions(
                        action.object, action.links, action.allow, can_apply);
                } else if constexpr (std::is_same_v<Operation, MTC::ObjectCollisionsConfig>) {
                    return obstacle_course_.set_object_collisions(
                        action.object, action.other_object, action.allow, can_apply);
                } else {
                    const auto& config = obstacle::get_obstacle_config(action.object);
                    if (config.attach_link != action.link)
                        throw std::runtime_error("execution attachment link differs from registry");
                    if constexpr (std::is_same_v<Operation, MTC::AttachObjectConfig>) {
                        if (config.touch_links != action.touch_links)
                            throw std::runtime_error("execution touch links differ from registry");
                        return obstacle_course_.attach(action.object, can_apply);
                    } else {
                        return obstacle_course_.detach(action.object, can_apply);
                    }
                }
            },
            operation);
    }

    void execution_worker() {
        std::uint64_t handled_generation = 0;
        std::uint64_t planning_request = 0;
        bool awaiting_plan = false;
        while (worker_running_.load(std::memory_order_acquire)) {
            const auto generation = requested_generation_.load(std::memory_order_acquire);
            const auto request = pending_task_.load(std::memory_order_acquire);
            if (request && request->generation == generation && generation != 0
                && generation != handled_generation) {
                handled_generation = generation;
                awaiting_plan = false;
                try {
                    if (obstacle_course_.scene_uncertain())
                        throw std::runtime_error(
                            "global scene outcome is uncertain; restart/reconcile before planning");
                    planning_request = mtc_machine_.process(mtc_dictionary_.helper_build_chunk(
                        request->recipe->blocks, request->recipe->bindings, request->targets));
                    awaiting_plan = true;
                } catch (const std::exception& error) {
                    auto result = std::make_shared<ExecutionResult>();
                    result->generation = generation;
                    result->error = error.what();
                    prepared_execution_.store(std::move(result), std::memory_order_release);
                }
            }
            if (awaiting_plan && generation == handled_generation) {
                const auto planned = mtc_machine_.get_trajectory();
                if (planned && planned->request_id == planning_request) {
                    awaiting_plan = false;
                    auto result = std::make_shared<ExecutionResult>();
                    result->generation = generation;
                    try {
                        if (!planned->plan_success || !planned->plan)
                            throw std::runtime_error(planned->diagnostic);
                        result->execution = MTC::prepare_execution(*planned->plan);
                    } catch (const std::exception& error) {
                        result->error = error.what();
                    }
                    prepared_execution_.store(std::move(result), std::memory_order_release);
                }
            }
            if (const auto job = scene_job_.exchange(nullptr, std::memory_order_acq_rel)) {
                auto result =
                    std::make_shared<SceneResult>(SceneResult{job->generation, job->step, {}});
                const std::function<bool()> can_apply = [this, generation = job->generation] {
                    return worker_running_.load(std::memory_order_acquire)
                        && requested_generation_.load(std::memory_order_acquire) == generation;
                };
                try {
                    const auto& update = std::get<MTC::SceneUpdate>(
                        job->plan->execution->plan.steps.at(job->step).action);
                    for (const auto& operation : update.operations) {
                        if (!can_apply())
                            throw std::runtime_error(
                                "scene operation cancelled before application");
                        if (!apply_scene_operation(operation, can_apply))
                            throw std::runtime_error("global scene operation failed");
                    }
                } catch (const std::exception& error) {
                    result->error = error.what();
                }
                scene_result_.store(std::move(result), std::memory_order_release);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        const auto frame = std::bit_cast<CustomFrame>(raw);
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
            double divisor = (joint_index == 0 || joint_index == 5) ? 32768.0 : 65536.0;
            angles[i] = raw_to_angle(frame.joint[joint_index], divisor);
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
        const double gripper_step = this->get_parameter("gripper_step").as_double();
        const double gripper_open_angle = this->get_parameter("gripper_open_angle").as_double();
        const double gripper_close_angle = this->get_parameter("gripper_close_angle").as_double();
        static bool initial_calibration{false};

        const auto gripper_mode = get_gripper_mode();
        const auto stock_control = [this, gripper_step]() {
            if (std::abs(*gripper_velocity_) < 0.01 && std::abs(*gripper_torque_) > 1.0) {
                *gripper_target_theta = NAN;
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
                initial_calibration = true;
                return true;
            } else {
                *gripper_calibration_trigger_ = false;
                initial_calibration = false;
                return false;
            }
        };

        switch (gripper_mode) {
        case rmcs_msgs::GripperMode::Close:
            if (!initial_calibration) {
                calibrate_zero_point();
            } else {
                *gripper_target_theta =
                    std::min(*gripper_angle_ + gripper_step, gripper_open_angle);
            }
            break;
        case rmcs_msgs::GripperMode::Open:
            if (!initial_calibration) {
                calibrate_zero_point();
            } else {
                *gripper_target_theta =
                    std::max(*gripper_angle_ - gripper_step, gripper_close_angle);
            }
            break;
        case rmcs_msgs::GripperMode::Calibrate:
            if (calibrate_zero_point()) {
                set_gripper_mode(rmcs_msgs::GripperMode::None);
            }
            break;
        case rmcs_msgs::GripperMode::None: *gripper_target_theta = NAN; break;
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
        cancel_execution();
        *is_arm_enable = false;
        for (std::size_t i = 0; i < std::size(theta); ++i) {
            if (std::isfinite(*theta[i]))
                *target_theta[i] = *theta[i];
        }
        *gripper_target_theta = NAN;
        *image_pitch_target_theta_ = NAN;
        image_pitch_theta1_offset_ = 0.0;
        last_switch_left_ = *switch_left_;
        last_switch_right_ = *switch_right_;
        last_rotary_knob_switch_ = *rotary_knob_switch;
        last_keyboard_ = *keyboard_;
        last_mouse_ = *mouse_;
        custom_joint_filter_.reset();
        set_arm_mode(rmcs_msgs::ArmMode::None, false);
        set_gripper_mode(rmcs_msgs::GripperMode::None);
    }
    MTC::Dictionary mtc_dictionary_;
    MtcRecipes mtc_recipes_;
    std::string chassis_type_;
    MTC::Machine mtc_machine_;
    obstacle::ObstacleCoordinatesMap obstacle_coordinates_map_;
    obstacle::ObstacleCourse obstacle_course_;
    std::atomic<bool> worker_running_{true};
    std::thread worker_thread_;
    std::atomic<std::uint64_t> requested_generation_{0};
    std::uint64_t generation_counter_ = 0;
    std::atomic<std::shared_ptr<const TaskRequest>> pending_task_;
    std::atomic<std::shared_ptr<const ExecutionResult>> prepared_execution_;
    std::atomic<std::shared_ptr<const SceneJob>> scene_job_;
    std::atomic<std::shared_ptr<const SceneResult>> scene_result_;
    std::optional<ActiveRun> run_;
    bool feedback_fault_ = false;

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
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)
