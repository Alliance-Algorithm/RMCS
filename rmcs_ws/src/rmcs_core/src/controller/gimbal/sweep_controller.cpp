#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class SineSweepController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SineSweepController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , prbs_mode_(get_parameter("mode").as_string() == "prbs")
        , amplitude_(get_parameter("amplitude").as_double())
        , f_start_(prbs_mode_ ? 0.0 : get_parameter("f_start").as_double())
        , f_end_(prbs_mode_ ? 0.0 : get_parameter("f_end").as_double())
        , repeat_time_(prbs_mode_ ? 0 : static_cast<int>(get_parameter("repeat_time").as_int()))
        , output_path_(
              prbs_mode_ ? get_parameter("prbs_output_path").as_string()
                         : get_parameter("sine_output_path").as_string())
        , axis_(prbs_mode_ ? "pitch" : "yaw")
        , is_pitch_(prbs_mode_)
        , homing_angle_threshold_(
              is_pitch_ ? get_parameter("homing_angle_threshold").as_double() : 0.0)
        , homing_velocity_threshold_(
              is_pitch_ ? get_parameter("homing_velocity_threshold").as_double() : 0.0)
        , homing_kp_(is_pitch_ ? get_parameter("homing_kp").as_double() : 0.0)
        , homing_kd_(is_pitch_ ? get_parameter("homing_kd").as_double() : 0.0)
        , homing_torque_limit_(is_pitch_ ? get_parameter("homing_torque_limit").as_double() : 0.0)
        , pitch_angle_limit_(
              is_pitch_ ? get_parameter("pitch_angle_limit").as_double() : 0.0)
        , prbs_register_length_(
              prbs_mode_ ? static_cast<int>(get_parameter("prbs_register_length").as_int()) : 0)
        , prbs_bit_period_(
              prbs_mode_ ? static_cast<int>(get_parameter("prbs_bit_period").as_int()) : 0)
        , prbs_num_repeats_(
              prbs_mode_ ? static_cast<int>(get_parameter("prbs_num_repeats").as_int()) : 0)
        , prbs_sequence_length_(0)
        , lfsr_taps_mask_(0) {
        validate_parameters();
        if (prbs_mode_) {
            prbs_sequence_length_ = (1 << prbs_register_length_) - 1;
            lfsr_taps_mask_ = compute_taps_mask(prbs_register_length_);
        }

        auto path = [this](std::string_view suffix) {
            return "/gimbal/" + axis_ + "/" + std::string{suffix};
        };

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input(path("velocity"), velocity_);
        register_input(path("velocity_imu"), velocity_imu_);
        register_input(path("torque"), torque_feedback_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/timestamp", timestamp_);

        register_output(path("control_torque"), control_torque_, kNan);

        if (is_pitch_)
            register_input(path("angle"), angle_);

        open_csv();
    }

    ~SineSweepController() override { csv_.close(); }

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        using rmcs_msgs::Switch;
        if ((switch_left == Switch::DOWN && switch_right == Switch::DOWN)
            || switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN) {
            disable_all_outputs();
            state_ = State::kIdle;
            reset_sweep();
            return;
        }

        if (switch_left == Switch::MIDDLE && switch_right == Switch::MIDDLE) {
            if (is_pitch_) {
                *control_torque_ = compute_homing_torque();
                state_ = State::kHoming;
            } else {
                set_hold_outputs();
                state_ = State::kIdle;
            }
            reset_sweep();
            return;
        }

        if (switch_right != Switch::UP) {
            set_hold_outputs();
            if (state_ == State::kDone)
                state_ = State::kIdle;
            return;
        }

        if (state_ == State::kDone) {
            set_hold_outputs();
            return;
        }

        if (state_ == State::kHoming) {
            update_homing();
            return;
        }

        if (state_ == State::kIdle) {
            is_pitch_ ? enter_homing() : enter_sweeping();
            if (state_ == State::kHoming)
                return;
        }

        update_sweeping();
    }

private:
    // --- Homing (pitch only) ---

    void enter_homing() {
        state_ = State::kHoming;
        *control_torque_ = compute_homing_torque();
        RCLCPP_INFO(get_logger(), "Pitch homing: moving to horizontal...");
    }

    void update_homing() {
        *control_torque_ = compute_homing_torque();
        double angle = normalize_angle(*angle_);
        if (std::abs(angle) < homing_angle_threshold_
            && std::abs(*velocity_) < homing_velocity_threshold_) {
            RCLCPP_INFO(
                get_logger(), "Pitch homing complete (angle=%.4f, vel=%.4f). Starting sweep.",
                angle, *velocity_);
            enter_sweeping();
        }
    }

    static double normalize_angle(double angle) {
        angle = std::fmod(angle, 2.0 * std::numbers::pi);
        if (angle > std::numbers::pi)
            angle -= 2.0 * std::numbers::pi;
        else if (angle < -std::numbers::pi)
            angle += 2.0 * std::numbers::pi;
        return angle;
    }

    double compute_homing_torque() const {
        double angle = normalize_angle(*angle_);
        double torque = -(homing_kp_ * angle + homing_kd_ * (*velocity_));
        return std::clamp(torque, -homing_torque_limit_, homing_torque_limit_);
    }

    // --- Sweeping ---

    void enter_sweeping() {
        state_ = State::kSweeping;
        sweep_time_ = 0.0;
        if (prbs_mode_) {
            lfsr_state_ = 1;
            sample_in_bit_ = 0;
            bit_count_ = 0;
            repeat_count_ = 0;
            clock_lfsr();
        } else {
            current_freq_ = f_start_;
            phase_ = 0.0;
            phase_start_time_ = 0.0;
        }
        open_csv();
    }

    void update_sweeping() { prbs_mode_ ? update_prbs() : update_sine(); }

    void update_sine() {
        double dt = 1.0 / *update_rate_;
        double torque = amplitude_ * std::sin(phase_);
        *control_torque_ = torque;
        log_sample(torque, current_freq_);

        phase_ += 2.0 * std::numbers::pi * current_freq_ * dt;
        sweep_time_ += dt;

        double period_duration = 1.0 / current_freq_;
        if (sweep_time_ - phase_start_time_ >= period_duration * repeat_time_) {
            current_freq_ += frequency_step(current_freq_);
            phase_start_time_ = sweep_time_;
            if (current_freq_ > f_end_)
                finish_sweep();
        }
    }

    void update_prbs() {
        if (std::abs(normalize_angle(*angle_)) > pitch_angle_limit_) {
            RCLCPP_WARN(
                get_logger(), "Pitch angle %.2f° exceeds limit %.2f°, aborting PRBS → homing",
                normalize_angle(*angle_) * 180.0 / std::numbers::pi,
                pitch_angle_limit_ * 180.0 / std::numbers::pi);
            csv_.flush();
            enter_homing();
            return;
        }

        double dt = 1.0 / *update_rate_;
        double torque = (lfsr_output_ ? 1.0 : -1.0) * amplitude_;
        *control_torque_ = torque;
        log_sample(torque, 0.0);

        sweep_time_ += dt;

        if (++sample_in_bit_ >= prbs_bit_period_) {
            sample_in_bit_ = 0;
            clock_lfsr();
            if (++bit_count_ >= prbs_sequence_length_) {
                bit_count_ = 0;
                lfsr_state_ = 1;
                clock_lfsr();
                if (++repeat_count_ >= prbs_num_repeats_)
                    finish_sweep();
            }
        }
    }

    void log_sample(double torque, double freq) {
        double timestamp_s =
            std::chrono::duration<double>(timestamp_->time_since_epoch()).count();
        csv_ << timestamp_s << ',' << *update_count_ << ',' << sweep_time_ << ','
             << freq << ',' << torque << ',' << *torque_feedback_ << ','
             << *velocity_ << ',' << *velocity_imu_ << '\n';
    }

    void finish_sweep() {
        state_ = State::kDone;
        csv_.flush();
        RCLCPP_INFO(
            get_logger(), "%s complete. CSV saved to %s",
            prbs_mode_ ? "PRBS" : "Sweep", output_path_.c_str());
    }

    // --- LFSR ---

    void clock_lfsr() {
        lfsr_output_ = lfsr_state_ & 1u;
        lfsr_state_ >>= 1;
        if (lfsr_output_)
            lfsr_state_ ^= lfsr_taps_mask_;
    }

    static constexpr uint32_t compute_taps_mask(int n) {
        switch (n) {
        case 9: return (1u << 8) | (1u << 3);                                     // x^9+x^4+1
        case 10: return (1u << 9) | (1u << 2);                                    // x^10+x^3+1
        case 11: return (1u << 10) | (1u << 1);                                   // x^11+x^2+1
        case 12: return (1u << 11) | (1u << 10) | (1u << 9) | (1u << 3);          // x^12+x^11+x^10+x^4+1
        default: return 0;
        }
    }

    // --- Utilities ---

    void disable_all_outputs() { *control_torque_ = kNan; }

    void set_hold_outputs() { *control_torque_ = 0.0; }

    void validate_parameters() const {
        auto mode_str = get_parameter("mode").as_string();
        if (mode_str != "sine" && mode_str != "prbs")
            throw std::invalid_argument{"mode must be \"sine\" or \"prbs\", got: " + mode_str};
        if (amplitude_ <= 0.0)
            throw std::invalid_argument{"amplitude must be positive"};
        if (output_path_.empty())
            throw std::invalid_argument{"output_path must not be empty"};
        if (is_pitch_ && homing_angle_threshold_ <= 0.0)
            throw std::invalid_argument{"homing_angle_threshold must be positive for pitch"};
        if (is_pitch_ && homing_velocity_threshold_ <= 0.0)
            throw std::invalid_argument{"homing_velocity_threshold must be positive for pitch"};
        if (is_pitch_ && homing_torque_limit_ <= 0.0)
            throw std::invalid_argument{"homing_torque_limit must be positive for pitch"};
        if (is_pitch_ && pitch_angle_limit_ <= 0.0)
            throw std::invalid_argument{"pitch_angle_limit must be positive for pitch"};
        if (prbs_mode_) {
            if (prbs_register_length_ < 9 || prbs_register_length_ > 12)
                throw std::invalid_argument{"prbs_register_length must be 9-12"};
            if (prbs_bit_period_ <= 0)
                throw std::invalid_argument{"prbs_bit_period must be positive"};
            if (prbs_num_repeats_ <= 0)
                throw std::invalid_argument{"prbs_num_repeats must be positive"};
        } else {
            if (f_start_ <= 0.0 || f_end_ <= 0.0 || f_start_ > f_end_)
                throw std::invalid_argument{"f_start/f_end must be positive with f_start <= f_end"};
            if (repeat_time_ <= 0)
                throw std::invalid_argument{"repeat_time must be positive"};
        }
    }

    void open_csv() {
        if (csv_.is_open())
            csv_.close();

        auto parent = std::filesystem::path(output_path_).parent_path();
        if (!parent.empty())
            std::filesystem::create_directories(parent);

        csv_.open(output_path_);
        if (!csv_.is_open())
            throw std::runtime_error{"Failed to open CSV: " + output_path_};

        csv_ << "timestamp_s,update_count,sweep_time_s,frequency_hz,"
                "command_torque_nm,actual_torque_nm,velocity_rad_s,velocity_imu_rad_s\n";
    }

    void reset_sweep() {
        current_freq_ = 0.0;
        phase_ = 0.0;
        sweep_time_ = 0.0;
        phase_start_time_ = 0.0;
        sample_in_bit_ = 0;
        bit_count_ = 0;
        repeat_count_ = 0;
    }

    static constexpr double frequency_step(double freq) {
        return freq < 24.0 ? 0.5 : 2.0;
    }

    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

    enum class State : uint8_t { kIdle, kHoming, kSweeping, kDone };

    // Mode
    bool prbs_mode_;

    // Shared params
    double amplitude_;
    double f_start_;
    double f_end_;
    int repeat_time_;
    std::string output_path_;
    std::string axis_;
    bool is_pitch_;

    // Pitch homing & safety
    double homing_angle_threshold_;
    double homing_velocity_threshold_;
    double homing_kp_;
    double homing_kd_;
    double homing_torque_limit_;
    double pitch_angle_limit_;

    // PRBS params
    int prbs_register_length_;
    int prbs_bit_period_;
    int prbs_num_repeats_;
    int prbs_sequence_length_;
    uint32_t lfsr_taps_mask_;

    // Runtime state
    State state_ = State::kIdle;
    double sweep_time_ = 0.0;

    // Sine state
    double current_freq_ = 0.0;
    double phase_ = 0.0;
    double phase_start_time_ = 0.0;

    // PRBS state
    uint32_t lfsr_state_ = 1;
    bool lfsr_output_ = false;
    int sample_in_bit_ = 0;
    int bit_count_ = 0;
    int repeat_count_ = 0;

    // Interfaces
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> velocity_;
    InputInterface<double> velocity_imu_;
    InputInterface<double> torque_feedback_;
    InputInterface<double> update_rate_;
    InputInterface<size_t> update_count_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<double> angle_;

    OutputInterface<double> control_torque_;

    std::ofstream csv_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SineSweepController, rmcs_executor::Component)
