#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class DeformableSuspensionArbiter
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableSuspensionArbiter()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        joint_base_path_ = get_parameter_or<std::string>("joint_base_path", "/chassis");
        joint_suffix_ = get_parameter_or<std::string>("joint_suffix", "_joint");
        rl_base_ = get_parameter_or<std::string>("rl_base", "/chassis/rl");

        traditional_angle_suffix_ = get_parameter_or<std::string>(
            "traditional_angle_suffix", "/traditional_target_physical_angle");
        traditional_velocity_suffix_ = get_parameter_or<std::string>(
            "traditional_velocity_suffix", "/traditional_target_physical_velocity");
        traditional_acceleration_suffix_ = get_parameter_or<std::string>(
            "traditional_acceleration_suffix", "/traditional_target_physical_acceleration");
        traditional_error_suffix_ = get_parameter_or<std::string>(
            "traditional_error_suffix", "/traditional_control_angle_error");

        rl_angle_suffix_ =
            get_parameter_or<std::string>("rl_angle_suffix", "/rl_target_physical_angle");
        rl_velocity_suffix_ =
            get_parameter_or<std::string>("rl_velocity_suffix", "/rl_target_physical_velocity");
        rl_acceleration_suffix_ = get_parameter_or<std::string>(
            "rl_acceleration_suffix", "/rl_target_physical_acceleration");
        rl_error_suffix_ =
            get_parameter_or<std::string>("rl_error_suffix", "/rl_control_angle_error");

        output_angle_suffix_ =
            get_parameter_or<std::string>("output_angle_suffix", "/target_physical_angle");
        output_velocity_suffix_ =
            get_parameter_or<std::string>("output_velocity_suffix", "/target_physical_velocity");
        output_acceleration_suffix_ = get_parameter_or<std::string>(
            "output_acceleration_suffix", "/target_physical_acceleration");
        output_error_suffix_ =
            get_parameter_or<std::string>("output_error_suffix", "/control_angle_error");

        register_input("/chassis/active_suspension/active", active_suspension_);
        register_input(rl_base_ + "/valid", valid_, false);
        register_input(rl_base_ + "/healthy", healthy_, false);
        register_input("/chassis/deformable/reset_count", reset_count_);
        register_output("/chassis/deformable/rl_suspension_authoritative", rl_authoritative_, false);

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            register_input(
                joint_path_(corner, traditional_angle_suffix_), traditional_angle_[corner]);
            register_input(
                joint_path_(corner, traditional_velocity_suffix_), traditional_velocity_[corner],
                false);
            register_input(
                joint_path_(corner, traditional_acceleration_suffix_),
                traditional_acceleration_[corner], false);
            register_input(
                joint_path_(corner, traditional_error_suffix_), traditional_error_[corner], false);
            register_input(joint_path_(corner, rl_angle_suffix_), rl_angle_[corner], false);
            register_input(joint_path_(corner, rl_velocity_suffix_), rl_velocity_[corner], false);
            register_input(
                joint_path_(corner, rl_acceleration_suffix_), rl_acceleration_[corner], false);
            register_input(joint_path_(corner, rl_error_suffix_), rl_error_[corner], false);
            register_output(joint_path_(corner, output_angle_suffix_), output_angle_[corner], nan_);
            register_output(
                joint_path_(corner, output_velocity_suffix_), output_velocity_[corner], nan_);
            register_output(
                joint_path_(corner, output_acceleration_suffix_), output_acceleration_[corner],
                nan_);
            register_output(joint_path_(corner, output_error_suffix_), output_error_[corner], nan_);
        }
    }

    void before_updating() override {
        if (!valid_.ready())
            valid_.make_and_bind_directly(0.0);
        if (!healthy_.ready())
            healthy_.make_and_bind_directly(0.0);
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (!traditional_velocity_[corner].ready())
                traditional_velocity_[corner].make_and_bind_directly(nan_);
            if (!traditional_acceleration_[corner].ready())
                traditional_acceleration_[corner].make_and_bind_directly(nan_);
            if (!traditional_error_[corner].ready())
                traditional_error_[corner].make_and_bind_directly(nan_);
            if (!rl_angle_[corner].ready())
                rl_angle_[corner].make_and_bind_directly(nan_);
            if (!rl_velocity_[corner].ready())
                rl_velocity_[corner].make_and_bind_directly(nan_);
            if (!rl_acceleration_[corner].ready())
                rl_acceleration_[corner].make_and_bind_directly(nan_);
            if (!rl_error_[corner].ready())
                rl_error_[corner].make_and_bind_directly(nan_);
        }
        last_reset_count_ = *reset_count_;
        publish_nan_outputs_();
        *rl_authoritative_ = false;
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            publish_nan_outputs_();
            *rl_authoritative_ = false;
            last_rl_authoritative_ = false;
            authority_initialized_ = false;
        }

        const bool use_rl = *active_suspension_ && *valid_ > 0.5 && *healthy_ > 0.5
                         && rl_targets_finite_();
        *rl_authoritative_ = use_rl;

        if (!authority_initialized_ || use_rl != last_rl_authoritative_) {
            authority_initialized_ = true;
            last_rl_authoritative_ = use_rl;
            RCLCPP_INFO(
                get_logger(), "suspension arbiter: source=%s", use_rl ? "rl" : "traditional");
        }

        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            if (use_rl) {
                *output_angle_[corner] = *rl_angle_[corner];
                *output_velocity_[corner] = *rl_velocity_[corner];
                *output_acceleration_[corner] = *rl_acceleration_[corner];
                *output_error_[corner] = *rl_error_[corner];
            } else {
                *output_angle_[corner] = *traditional_angle_[corner];
                *output_velocity_[corner] = *traditional_velocity_[corner];
                *output_acceleration_[corner] = *traditional_acceleration_[corner];
                *output_error_[corner] = *traditional_error_[corner];
            }
        }
    }

private:
    static constexpr std::size_t kCornerCount = 4;
    static constexpr const char* kJointName[kCornerCount] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    std::string joint_path_(std::size_t corner, const std::string& suffix) const {
        return joint_base_path_ + "/" + kJointName[corner] + joint_suffix_ + suffix;
    }

    bool rl_targets_finite_() const {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner)
            if (!std::isfinite(*rl_angle_[corner]))
                return false;
        return true;
    }

    void publish_nan_outputs_() {
        for (std::size_t corner = 0; corner < kCornerCount; ++corner) {
            *output_angle_[corner] = nan_;
            *output_velocity_[corner] = nan_;
            *output_acceleration_[corner] = nan_;
            *output_error_[corner] = nan_;
        }
    }

    InputInterface<bool> active_suspension_;
    InputInterface<double> valid_;
    InputInterface<double> healthy_;
    InputInterface<std::size_t> reset_count_;
    OutputInterface<bool> rl_authoritative_;

    std::array<InputInterface<double>, kCornerCount> traditional_angle_;
    std::array<InputInterface<double>, kCornerCount> traditional_velocity_;
    std::array<InputInterface<double>, kCornerCount> traditional_acceleration_;
    std::array<InputInterface<double>, kCornerCount> traditional_error_;
    std::array<InputInterface<double>, kCornerCount> rl_angle_;
    std::array<InputInterface<double>, kCornerCount> rl_velocity_;
    std::array<InputInterface<double>, kCornerCount> rl_acceleration_;
    std::array<InputInterface<double>, kCornerCount> rl_error_;
    std::array<OutputInterface<double>, kCornerCount> output_angle_;
    std::array<OutputInterface<double>, kCornerCount> output_velocity_;
    std::array<OutputInterface<double>, kCornerCount> output_acceleration_;
    std::array<OutputInterface<double>, kCornerCount> output_error_;

    std::string joint_base_path_;
    std::string joint_suffix_;
    std::string rl_base_;
    std::string traditional_angle_suffix_;
    std::string traditional_velocity_suffix_;
    std::string traditional_acceleration_suffix_;
    std::string traditional_error_suffix_;
    std::string rl_angle_suffix_;
    std::string rl_velocity_suffix_;
    std::string rl_acceleration_suffix_;
    std::string rl_error_suffix_;
    std::string output_angle_suffix_;
    std::string output_velocity_suffix_;
    std::string output_acceleration_suffix_;
    std::string output_error_suffix_;

    std::size_t last_reset_count_ = 0;
    bool last_rl_authoritative_ = false;
    bool authority_initialized_ = false;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableSuspensionArbiter, rmcs_executor::Component)
