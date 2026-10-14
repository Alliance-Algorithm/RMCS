#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/msg/wheel_leg_identification_sample.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_recording_contract.hpp"

namespace rmcs_core::controller::identification {

// This component is a sink in the executor dependency graph: the hardware command
// partner runs before update(), and only this component's worker publishes to ROS.
class WheelLegIdentificationRecorder final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
    using Sample = rmcs_msgs::msg::WheelLegIdentificationSample;
    using SteadyClock = std::chrono::steady_clock;

public:
    WheelLegIdentificationRecorder()
        : Node{get_component_name(), node::options()}
        , queue_(4096) {
        const auto side = get_parameter_or<std::string>("side", "");
        if (side != "left" && side != "right")
            throw std::invalid_argument("Identification recording requires side: left or right");
        selected_side_ = side == "left" ? 0 : 1;
        const auto kind = get_parameter_or<std::string>("experiment_kind", "pair");
        if (kind != "pair" && kind != "wheel")
            throw std::invalid_argument("experiment_kind must be pair or wheel");
        experiment_kind_ = kind == "wheel" ? 1 : 0;
        record_on_double_middle_ = get_parameter_or("record_on_double_middle", false);
        register_input("/predefined/update_count", tick_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/identification/enable_requested", enable_requested_);
        register_input("/wheel_leg/imu/quaternion", imu_quaternion_);
        register_input("/wheel_leg/imu/angular_velocity", imu_gyro_);
        register_input("/wheel_leg/imu/last_steady_ns", imu_last_ns_);
        register_input("/wheel_leg/imu/acceleration", imu_acceleration_);
        register_input("/wheel_leg/imu/acceleration_steady_ns", imu_acceleration_ns_);
        register_input("/wheel_leg/imu/accelerometer_board_quarter_us", imu_accel_board_ticks_);
        register_input("/wheel_leg/imu/gyroscope_board_quarter_us", imu_gyro_board_ticks_);
        register_input("/wheel_leg/supply_voltage", supply_voltage_, false);
        register_input("/wheel_leg/identification/torque_preclip_api", preclip_api_, false);
        register_input("/wheel_leg/identification/repetition_id", repetition_id_, false);

        for (std::size_t i = 0; i < kAxisNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_input(prefix + "/feedback_sequence", feedback_sequence_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            register_input(prefix + "/feedback_frame_bytes", feedback_frame_[i]);
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/torque", feedback_torque_[i]);
            register_input(prefix + "/max_torque", max_torque_[i]);
            register_input(prefix + "/temperature_c", temperature_[i], false);
            register_input(prefix + "/control_torque", command_torque_[i], false);
            if (i < 4) {
                register_input(prefix + "/control_angle", command_angle_[i], false);
                register_input(prefix + "/control_velocity", command_velocity_[i], false);
            }
            register_input(prefix + "/tau_frame_api", frame_torque_[i]);
            register_input(prefix + "/tx_kind", tx_kind_[i]);
            register_input(prefix + "/tx_queued_steady_ns", tx_ns_[i]);
            register_input(prefix + "/tx_frame_bytes", tx_frame_[i]);
            register_input(prefix + "/tx_can_id", tx_can_id_[i]);
            register_input(prefix + "/tx_can_bus", tx_can_bus_[i]);
            if (i < 4) {
                register_input(prefix + "/fault_code", dm_fault_[i]);
                register_input(prefix + "/status_code", dm_status_[i]);
                register_input(prefix + "/temperature_mos", dm_mos_temp_[i]);
                register_input(prefix + "/temperature_rotor", dm_rotor_temp_[i]);
            }
        }
        for (std::size_t i = 0; i < 2; ++i)
            register_input(
                std::string{"/wheel_leg/"} + (i == 0 ? "left_wheel" : "right_wheel")
                    + "/control_velocity",
                wheel_speed_target_[i], false);
        register_input("/wheel_leg/identification/reference_model", reference_model_, false);
        register_input(
            "/wheel_leg/identification/reference_velocity_model", reference_velocity_model_, false);
        register_input(
            "/wheel_leg/identification/velocity_target_model", velocity_target_model_, false);
        register_input(
            "/wheel_leg/identification/position_error_model", position_error_model_, false);
        register_input("/wheel_leg/identification/speed_error_model", speed_error_model_, false);
        register_input(
            "/wheel_leg/identification/torque_preclip_model", torque_preclip_model_, false);
        register_input(
            "/wheel_leg/identification/torque_integral_model", torque_integral_model_, false);
        register_input(
            "/wheel_leg/identification/torque_gravity_ff_model", torque_gravity_ff_model_, false);
        register_input("/wheel_leg/identification/phase", phase_, false);
        register_input("/wheel_leg/identification/failure_reason", failure_reason_, false);
        register_input("/wheel_leg/identification/segment_id", segment_id_, false);
        register_input("/wheel_leg/identification/segment_role", segment_role_, false);
        register_input("/wheel_leg/identification/segment_waveform", segment_waveform_, false);
        register_input("/wheel_leg/identification/wheel_mode", wheel_mode_, false);
        register_input("/wheel_leg/identification/measured_model", measured_model_, false);
        register_input(
            "/wheel_leg/identification/selected/inner_knee_measured_deg", inner_knee_fk_, false);
        register_input(
            "/wheel_leg/identification/selected/inner_knee_target_deg", inner_knee_requested_,
            false);
        const std::string recording = "/wheel_leg/identification/recording/";
        register_input(recording + "protocol_version", recording_protocol_, false);
        register_input(recording + "beta_reference_deg", inner_knee_reference_, false);
        register_input(recording + "theta_measured_deg", thigh_orientation_, false);
        register_input(recording + "reference_tick", reference_tick_, false);
        register_input(recording + "segment_time_s", segment_elapsed_, false);
        register_input(recording + "center_delta_rad", center_delta_, false);
        register_input(recording + "admission", configuration_admission_, false);
        register_input(recording + "skipped_segment", skipped_arrival_, false);
        register_input(recording + "qualified_segment", qualified_arrival_, false);
        register_input(recording + "cycle", jump_cycle_, false);
        register_input(recording + "jump_phase", jump_phase_, false);

        constexpr std::array kSelectedAxes{"hip", "knee"};
        for (std::size_t i = 0; i < kSelectedAxes.size(); ++i) {
            const auto prefix =
                std::string{"/wheel_leg/identification/selected/"} + kSelectedAxes[i];
            constexpr double nan = std::numeric_limits<double>::quiet_NaN();
            register_output(prefix + "/angle_api", selected_angle_[i], nan);
            register_output(prefix + "/velocity_api", selected_velocity_[i], nan);
            register_output(prefix + "/torque_feedback_api", selected_feedback_torque_[i], nan);
            register_output(prefix + "/torque_command_api", selected_command_torque_[i], nan);
            register_output(prefix + "/torque_frame_api", selected_frame_torque_[i], nan);
            register_output(prefix + "/reference_api", selected_reference_api_[i], nan);
            register_output(prefix + "/velocity_target_api", selected_target_velocity_api_[i], nan);
            register_output(prefix + "/reference_model", selected_reference_model_[i], nan);
            register_output(
                prefix + "/velocity_target_model", selected_target_velocity_model_[i], nan);
            register_output(
                prefix + "/position_error_model", selected_position_error_model_[i], nan);
            register_output(prefix + "/speed_error_model", selected_speed_error_model_[i], nan);
            register_output(prefix + "/torque_preclip_model", selected_preclip_model_[i], nan);
        }
        register_output("/wheel_leg/identification/actuation_scope", actuation_scope_, 0);

        publisher_ = create_publisher<Sample>(
            "/wheel_leg/identification/sample", rclcpp::QoS{rclcpp::KeepLast(128)}.reliable());
        if (get_parameter_or<std::string>("trajectory_revision", "") == "pair_v6_recording_v1")
            parameter_guard_ = freeze_recording_parameters(*this);
        worker_ = std::thread{[this] { publish_queued(); }};
    }

    ~WheelLegIdentificationRecorder() override {
        stopping_.store(true, std::memory_order_release);
        wake_count_.fetch_add(1, std::memory_order_release);
        wake_count_.notify_one();
        if (worker_.joinable())
            worker_.join();
    }

    void update() override {
        Sample sample;
        sample.stamp = get_clock()->now();
        sample.control_steady_ns =
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           SteadyClock::now().time_since_epoch())
                                           .count());
        sample.tick = static_cast<std::uint64_t>(*tick_);
        sample.selected_side = selected_side_;
        sample.experiment_kind = experiment_kind_;
        sample.repetition_id = repetition_id_.ready() ? *repetition_id_ : 0;
        sample.feedback_fresh = *feedback_fresh_;
        sample.enable_requested = *enable_requested_;
        sample.dropped_samples = dropped_samples_.load(std::memory_order_relaxed);

        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        sample.q_ref_model.fill(nan);
        sample.dq_ref_model.fill(nan);
        sample.velocity_target_model.fill(nan);
        sample.position_error_model.fill(nan);
        sample.speed_error_model.fill(nan);
        sample.tau_preclip_model.fill(nan);
        sample.torque_integral_model.fill(nan);
        sample.tau_gravity_ff_model.fill(nan);
        sample.q_ref_api.fill(nan);
        sample.velocity_target_api.fill(nan);
        sample.wheel_velocity_target_api.fill(nan);
        sample.tau_preclip_api.fill(nan);
        sample.feedback_current_a.fill(nan);
        sample.imu_quaternion_xyzw.fill(nan);
        sample.imu_gyro.fill(nan);
        sample.imu_acceleration_mps2.fill(nan);
        sample.temperature_c.fill(nan);
        sample.dm_rotor_temperature_c.fill(nan);
        sample.supply_voltage_v = supply_voltage_.ready() ? *supply_voltage_ : nan;
        sample.phase = phase_.ready() ? *phase_ : -1;
        sample.failure_reason = failure_reason_.ready() ? *failure_reason_ : 0;
        sample.segment_id = segment_id_.ready() ? *segment_id_ : -1;
        sample.segment_role = segment_role_.ready() ? *segment_role_ : 2;
        sample.segment_waveform = segment_waveform_.ready() ? *segment_waveform_ : 0;
        sample.wheel_mode = wheel_mode_.ready() ? *wheel_mode_ : 0;
        sample.recording_protocol_version = recording_protocol_.ready() ? *recording_protocol_ : 0;
        sample.q_unwrapped_model.fill(nan);
        if (measured_model_.ready())
            for (std::size_t i = 0; i < 4; ++i)
                sample.q_unwrapped_model[i] = (*measured_model_)[static_cast<Eigen::Index>(i)];
        sample.inner_knee_fk_deg = inner_knee_fk_.ready() ? *inner_knee_fk_ : nan;
        sample.inner_knee_requested_deg =
            inner_knee_requested_.ready() ? *inner_knee_requested_ : nan;
        sample.inner_knee_reference_deg =
            inner_knee_reference_.ready() ? *inner_knee_reference_ : nan;
        sample.thigh_orientation_fk_deg = thigh_orientation_.ready() ? *thigh_orientation_ : nan;
        sample.reference_update_tick = reference_tick_.ready() ? *reference_tick_ : 0;
        sample.segment_elapsed_s = segment_elapsed_.ready() ? *segment_elapsed_ : nan;
        sample.reference_center_delta_rad = center_delta_.ready() ? *center_delta_ : nan;
        sample.configuration_admission =
            configuration_admission_.ready() ? *configuration_admission_ : -1;
        sample.skipped_arrival_segment = skipped_arrival_.ready() ? *skipped_arrival_ : -1;
        sample.qualified_arrival_segment = qualified_arrival_.ready() ? *qualified_arrival_ : -1;
        sample.jump_cycle_id = jump_cycle_.ready() ? *jump_cycle_ : -1;
        sample.jump_phase = jump_phase_.ready() ? *jump_phase_ : 0;

        for (std::size_t i = 0; i < 6; ++i) {
            const bool received = *feedback_sequence_[i] != 0;
            sample.feedback_sequence[i] = *feedback_sequence_[i];
            sample.feedback_steady_ns[i] = received ? *feedback_ns_[i] : 0;
            sample.q_api[i] = received ? *angle_[i] : nan;
            sample.dq_api[i] = received ? *velocity_[i] : nan;
            sample.torque_fb_api[i] = received ? *feedback_torque_[i] : nan;
            sample.max_torque_api[i] = *max_torque_[i];
            if (temperature_[i].ready() && received)
                sample.temperature_c[i] = *temperature_[i];
            sample.tau_cmd_api[i] = command_torque_[i].ready() ? *command_torque_[i] : nan;
            sample.tau_frame_api[i] = *frame_torque_[i];
            sample.tx_kind[i] = *tx_kind_[i];
            sample.tx_queued_steady_ns[i] = *tx_ns_[i];
            sample.tx_can_id[i] = *tx_can_id_[i];
            sample.tx_can_bus[i] = *tx_can_bus_[i];
            for (std::size_t byte = 0; byte < 8; ++byte) {
                sample.feedback_frame_bytes[8 * i + byte] = (*feedback_frame_[i])[byte];
                sample.tx_frame_bytes[8 * i + byte] = (*tx_frame_[i])[byte];
            }
            sample.feedback_torque_source[i] = received ? (i < 4 ? 1 : 2) : 0;
            if (received && i >= 4) {
                const auto raw = static_cast<std::uint16_t>(
                    (static_cast<std::uint16_t>(sample.feedback_frame_bytes[8 * i + 4]) << 8)
                    | sample.feedback_frame_bytes[8 * i + 5]);
                sample.feedback_current_a[i] =
                    static_cast<double>(std::bit_cast<std::int16_t>(raw)) * 20.0 / 16384.0;
            }
        }
        for (std::size_t i = 0; i < 2; ++i)
            if (wheel_speed_target_[i].ready())
                sample.wheel_velocity_target_api[i] = *wheel_speed_target_[i];
        if (preclip_api_.ready())
            for (std::size_t i = 0; i < 6; ++i)
                sample.tau_preclip_api[i] = (*preclip_api_)[static_cast<Eigen::Index>(i)];
        for (std::size_t i = 0; i < 6; ++i) {
            const double requested = sample.tau_preclip_api[i];
            const double limited = sample.tau_cmd_api[i];
            sample.torque_limited[i] = std::isfinite(requested) && std::isfinite(limited)
                                    && std::abs(requested - limited) > 1e-6;
        }
        for (std::size_t i = 0; i < 4; ++i) {
            sample.dm_fault[i] = sample.feedback_sequence[i] ? *dm_fault_[i] : -1;
            sample.dm_status[i] = sample.feedback_sequence[i] ? *dm_status_[i] : -1;
            if (sample.feedback_sequence[i]) {
                sample.temperature_c[i] = *dm_mos_temp_[i];
                sample.dm_rotor_temperature_c[i] = *dm_rotor_temp_[i];
            }
            if (command_angle_[i].ready())
                sample.q_ref_api[i] = *command_angle_[i];
            if (command_velocity_[i].ready())
                sample.velocity_target_api[i] = *command_velocity_[i];
        }
        const std::size_t selected = 2 * selected_side_, other = 2 * (1 - selected_side_);
        const bool selected_torque_frames =
            sample.tx_kind[selected] == 1 && sample.tx_kind[selected + 1] == 1;
        const bool other_position_pd = sample.tx_kind[other] == 3 || sample.tx_kind[other + 1] == 3;
        const bool other_torque_frames =
            sample.tx_kind[other] == 1 && sample.tx_kind[other + 1] == 1;
        const bool inactive_confirmed =
            sample.feedback_sequence[other] != 0 && sample.feedback_sequence[other + 1] != 0
            && sample.dm_status[other] == 0 && sample.dm_status[other + 1] == 0;
        const bool selected_confirmed =
            sample.dm_status[selected] == 1 && sample.dm_status[selected + 1] == 1;
        // On the standalone pair graph the inactive side receives disable or
        // no frames, rather than zero-effort MIT frames from an enabled drive.
        if (experiment_kind_ == 1) {
            const bool all_dm_disabled = sample.dm_status[0] == 0 && sample.dm_status[1] == 0
                                      && sample.dm_status[2] == 0 && sample.dm_status[3] == 0;
            const bool wheels_current_commanded = sample.enable_requested && all_dm_disabled
                                               && sample.tx_kind[4] == 1 && sample.tx_kind[5] == 1;
            sample.actuation_scope = wheels_current_commanded ? 3 : 0;
        } else {
            const bool opposite_commanded =
                other_position_pd || (other_torque_frames && !inactive_confirmed);
            const bool selected_only = selected_torque_frames && selected_confirmed
                                    && inactive_confirmed && !other_torque_frames;
            sample.actuation_scope = opposite_commanded ? 2 : selected_only ? 1 : 0;
        }
        *actuation_scope_ = sample.actuation_scope;

        const auto copy_optional = [](auto& destination, const auto& source) {
            if (source.ready())
                for (std::size_t i = 0; i < destination.size(); ++i)
                    destination[i] = (*source)[static_cast<Eigen::Index>(i)];
        };
        copy_optional(sample.q_ref_model, reference_model_);
        copy_optional(sample.dq_ref_model, reference_velocity_model_);
        copy_optional(sample.velocity_target_model, velocity_target_model_);
        copy_optional(sample.position_error_model, position_error_model_);
        copy_optional(sample.speed_error_model, speed_error_model_);
        copy_optional(sample.tau_preclip_model, torque_preclip_model_);
        copy_optional(sample.torque_integral_model, torque_integral_model_);
        copy_optional(sample.tau_gravity_ff_model, torque_gravity_ff_model_);
        for (std::size_t i = 0; i < 2; ++i) {
            const std::size_t axis = 2 * selected_side_ + i;
            *selected_angle_[i] = sample.q_api[axis];
            *selected_velocity_[i] = sample.dq_api[axis];
            *selected_feedback_torque_[i] = sample.torque_fb_api[axis];
            *selected_command_torque_[i] = sample.tau_cmd_api[axis];
            *selected_frame_torque_[i] = sample.tau_frame_api[axis];
            *selected_reference_api_[i] = sample.q_ref_api[axis];
            *selected_target_velocity_api_[i] = sample.velocity_target_api[axis];
            *selected_reference_model_[i] = sample.q_ref_model[axis];
            *selected_target_velocity_model_[i] = sample.velocity_target_model[axis];
            *selected_position_error_model_[i] = sample.position_error_model[axis];
            *selected_speed_error_model_[i] = sample.speed_error_model[axis];
            *selected_preclip_model_[i] = sample.tau_preclip_model[axis];
        }

        sample.imu_last_ns = *imu_last_ns_;
        if (sample.imu_last_ns != 0) {
            const auto& q = *imu_quaternion_;
            sample.imu_quaternion_xyzw = {q.x(), q.y(), q.z(), q.w()};
            for (std::size_t i = 0; i < 3; ++i)
                sample.imu_gyro[i] = (*imu_gyro_)[static_cast<Eigen::Index>(i)];
        }
        sample.imu_acceleration_steady_ns = *imu_acceleration_ns_;
        sample.imu_board_timestamp_quarter_us = {*imu_accel_board_ticks_, *imu_gyro_board_ticks_};
        if (sample.imu_acceleration_steady_ns != 0)
            for (std::size_t i = 0; i < 3; ++i)
                sample.imu_acceleration_mps2[i] =
                    (*imu_acceleration_)[static_cast<Eigen::Index>(i)];

        // SPSC queue: update() is the only producer, the ROS publisher is the only consumer.
        // There are no executor-thread ROS publishes, file operations, mutexes, or waits.
        // Do not fill an MCAP indefinitely while a failed/done controller is
        // latched in double-MIDDLE. Preserve one diagnostic terminal sample.
        // Passive capture has no phase input and remains remote-gated only.
        if (record_on_double_middle_) {
            const bool middle = *dr16_fresh_ && *switch_left_ == rmcs_msgs::Switch::MIDDLE
                             && *switch_right_ == rmcs_msgs::Switch::MIDDLE;
            const bool running = !phase_.ready() || sample.phase == 1 || sample.phase == 2;
            const bool recording = middle && running;
            const bool terminal_phase = sample.phase == -1 || sample.phase == 3;
            const bool terminal = middle && phase_.ready() && terminal_phase;
            const bool keep_final = was_recording_ && !recording;
            const bool first_failure = terminal && !terminal_recorded_;
            if (!middle)
                terminal_recorded_ = false;
            else if (first_failure)
                terminal_recorded_ = true;
            if (!recording && !keep_final && !first_failure)
                return;
            was_recording_ = recording;
        }
        if (!queue_.push_back(std::move(sample))) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        wake_count_.fetch_add(1, std::memory_order_release);
        wake_count_.notify_one();
    }

private:
    void publish_queued() {
        while (true) {
            if (auto* front = queue_.peek_front()) {
                Sample sample = std::move(*front);
                queue_.pop_front([](Sample&&) noexcept {});
                if (!rclcpp::ok()) {
                    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                try {
                    publisher_->publish(sample);
                } catch (const std::exception& error) {
                    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "Identification publish failed: %s",
                        error.what());
                } catch (...) {
                    dropped_samples_.fetch_add(1, std::memory_order_relaxed);
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "Identification publish failed");
                }
                continue;
            }
            if (stopping_.load(std::memory_order_acquire))
                break;
            const auto observed = wake_count_.load(std::memory_order_relaxed);
            if (!queue_.readable() && !stopping_.load(std::memory_order_acquire))
                wake_count_.wait(observed, std::memory_order_acquire);
        }
    }

    InputInterface<std::size_t> tick_;
    InputInterface<bool> feedback_fresh_, enable_requested_;
    InputInterface<bool> dr16_fresh_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Quaterniond> imu_quaternion_;
    InputInterface<Eigen::Vector3d> imu_gyro_;
    InputInterface<Eigen::Vector3d> imu_acceleration_;
    InputInterface<double> supply_voltage_;
    InputInterface<std::uint64_t> imu_last_ns_, imu_acceleration_ns_;
    InputInterface<std::uint32_t> imu_accel_board_ticks_, imu_gyro_board_ticks_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_sequence_, feedback_ns_, tx_ns_;
    std::array<InputInterface<std::array<std::uint8_t, 8>>, 6> feedback_frame_, tx_frame_;
    std::array<InputInterface<std::uint32_t>, 6> tx_can_id_;
    std::array<InputInterface<std::uint8_t>, 6> tx_can_bus_;
    std::array<InputInterface<double>, 6> angle_, velocity_, feedback_torque_, max_torque_;
    std::array<InputInterface<double>, 6> temperature_;
    std::array<InputInterface<double>, 6> command_torque_, frame_torque_;
    std::array<InputInterface<double>, 4> command_angle_, command_velocity_;
    std::array<InputInterface<double>, 2> wheel_speed_target_;
    std::array<InputInterface<std::uint8_t>, 6> tx_kind_;
    std::array<InputInterface<int>, 4> dm_fault_, dm_status_;
    std::array<InputInterface<double>, 4> dm_mos_temp_, dm_rotor_temp_;
    InputInterface<Eigen::Vector4d> reference_model_, reference_velocity_model_;
    InputInterface<Eigen::Vector4d> velocity_target_model_, position_error_model_;
    InputInterface<Eigen::Vector4d> speed_error_model_, torque_preclip_model_,
        torque_integral_model_;
    InputInterface<Eigen::Vector4d> torque_gravity_ff_model_;
    InputInterface<int> phase_, segment_id_, failure_reason_;
    InputInterface<std::uint8_t> segment_role_, segment_waveform_, wheel_mode_;
    InputInterface<std::uint32_t> repetition_id_;
    InputInterface<Eigen::Vector4d> measured_model_;
    InputInterface<double> inner_knee_fk_, inner_knee_requested_, inner_knee_reference_,
        thigh_orientation_;
    InputInterface<double> segment_elapsed_, center_delta_;
    InputInterface<std::uint32_t> recording_protocol_;
    InputInterface<std::uint64_t> reference_tick_;
    InputInterface<int> configuration_admission_, skipped_arrival_, qualified_arrival_, jump_cycle_,
        jump_phase_;
    InputInterface<Eigen::Matrix<double, 6, 1>> preclip_api_;

    std::size_t selected_side_ = 0;
    std::uint8_t experiment_kind_ = 0;
    bool record_on_double_middle_ = false;
    bool was_recording_ = false;
    bool terminal_recorded_ = false;
    std::array<OutputInterface<double>, 2> selected_angle_, selected_velocity_;
    std::array<OutputInterface<double>, 2> selected_feedback_torque_, selected_command_torque_;
    std::array<OutputInterface<double>, 2> selected_frame_torque_;
    std::array<OutputInterface<double>, 2> selected_reference_api_, selected_target_velocity_api_;
    std::array<OutputInterface<double>, 2> selected_reference_model_,
        selected_target_velocity_model_;
    std::array<OutputInterface<double>, 2> selected_position_error_model_,
        selected_speed_error_model_;
    std::array<OutputInterface<double>, 2> selected_preclip_model_;
    OutputInterface<int> actuation_scope_;

    rclcpp::Publisher<Sample>::SharedPtr publisher_;
    rmcs_utility::RingBuffer<Sample> queue_;
    std::atomic<std::uint64_t> dropped_samples_{0};
    std::atomic<std::uint64_t> wake_count_{0};
    std::atomic<bool> stopping_{false};
    std::thread worker_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegIdentificationRecorder, rmcs_executor::Component)
