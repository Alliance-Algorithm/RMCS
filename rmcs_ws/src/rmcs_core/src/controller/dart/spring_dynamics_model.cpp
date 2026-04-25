#include <cmath>
#include <deque>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

// ============================================================================
// Physical Constants (from mechanical design)
// ============================================================================
namespace constants {
constexpr double SCREW_PITCH_M = 0.002;    // 2mm per revolution
constexpr double PULLEY_RATIO = 2.0;       // Fixed pulley 1:2
constexpr double PROJECTILE_MASS_KG = 0.2; // Dart mass
constexpr double ETA_LOADING = 0.95;       // Loading efficiency
constexpr double ETA_UNLOADING = 0.90;     // Unloading efficiency
constexpr double FRICTION_COEFF = 0.85;    // Friction coefficient
} // namespace constants

// ============================================================================
// Algorithm Parameters (tuned empirically, rarely changed)
// ============================================================================
namespace defaults {
constexpr double FORCE_THRESHOLD_IDLE = 5000.0;  // g (≈50N)
constexpr double FORCE_THRESHOLD_PEAK = 15000.0; // g (≈150N)
constexpr double HOLDING_TOLERANCE = 1000.0;     // g (≈10N)
constexpr int HOLDING_DURATION_TICKS = 50;
constexpr size_t MIN_CALIBRATION_SAMPLES = 100;
constexpr size_t MAX_CALIBRATION_SAMPLES = 1000;
constexpr size_t FORCE_HISTORY_SIZE = 100;
} // namespace defaults

/**
 * @brief Spring-Pulley Dynamics Model with Two-Point Calibration
 *
 * Units:
 * - Force: grams (g) - matches sensor output
 * - Displacement: meters (m)
 * - Spring stiffness: g/m (auto-calibrated)
 * - Energy: Joules (J)
 * - Velocity: m/s
 *
 * Two-Point Calibration:
 * - Point 1: Top position (preload) → (angle_top, F_preload)
 * - Point 2: HOLDING position (max force) → (angle_hold, F_max)
 * - Linear mapping: F = K1 * x + F_preload
 * - K1 = (F_max - F_preload) / (x_hold - 0)
 *
 * Physical System:
 * - Screw motor (multi-turn angle) → trigger displacement (2mm/360°)
 * - Fixed pulley: trigger displacement / 2 = spring elongation (1:2 ratio)
 * - Spring force: F = K1*x + K2*|x|^n + F_preload (all in grams)
 * - Force sensors measure total tension (ch1 + ch2, in grams)
 *
 * This provides the PRIOR for ForceKalmanFilter.
 */
class SpringDynamicsModel
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SpringDynamicsModel()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        // Input: force sensor readings (grams)
        register_input("/force_sensor/channel_1/weight", force_ch1_);
        register_input("/force_sensor/channel_2/weight", force_ch2_);

        // Input: screw motor multi-turn angle (rad)
        register_input("/dart/force_screw_motor/angle", screw_angle_);

        // Output: model predictions (force in grams)
        register_output("/dart/dynamics/spring_elongation", spring_elongation_, 0.0);
        register_output("/dart/dynamics/predicted_force", predicted_force_, 0.0);
        register_output("/dart/dynamics/measured_force", measured_force_, 0.0);
        register_output("/dart/dynamics/stored_energy", stored_energy_, 0.0);
        register_output("/dart/dynamics/estimated_velocity", estimated_velocity_, 0.0);
        register_output("/dart/dynamics/phase", phase_output_, 0);

        // Output: Two-point calibration results
        register_output("/dart/dynamics/calibrated_max_force", calibrated_max_force_, 0.0);
        register_output("/dart/dynamics/calibrated_max_angle", calibrated_max_angle_, 0.0);
        register_output("/dart/dynamics/calibrated_K1", calibrated_K1_, 0.0);
        register_output("/dart/dynamics/calibration_samples", calibration_samples_, 0);
        register_output("/dart/dynamics/calibration_valid", calibration_valid_, false);

        load_parameters();

        RCLCPP_INFO(get_logger(), "[SpringDynamicsModel] Initialized with two-point calibration");
        RCLCPP_INFO(
            get_logger(), "  Point 1 (top): angle=%.3frad, F_preload=%.0fg", screw_top_angle_rad_,
            preload_force_g_);
        RCLCPP_INFO(
            get_logger(), "  Spring: K2=%.0f g/m^%.2f, n=%.2f (K1 will be auto-calibrated)", K2_,
            n_, n_);
    }

    void update() override {
        if (!force_ch1_.ready() || !force_ch2_.ready() || !screw_angle_.ready()) {
            return;
        }

        // Measure force from sensors (grams)
        const double F_measured = (*force_ch1_) + (*force_ch2_);
        *measured_force_ = F_measured;

        // Calculate spring elongation from screw angle
        update_spring_elongation();

        // Predict force from spring model (grams)
        *predicted_force_ = predict_force_from_displacement(spring_elongation_m_);

        // Update force history for phase detection
        force_history_.push_back(F_measured);
        if (force_history_.size() > defaults::FORCE_HISTORY_SIZE) {
            force_history_.pop_front();
        }

        // Detect loading phase
        detect_loading_phase();
        *phase_output_ = static_cast<int>(phase_);

        // HOLDING phase: two-point calibration
        if (phase_ == Phase::HOLDING) {
            calibrate_holding_point(F_measured, *screw_angle_);
        }

        // Calculate stored elastic energy (Joules)
        *stored_energy_ = calculate_stored_energy();

        // Estimate projectile velocity at release (m/s)
        if (phase_ == Phase::RELEASE) {
            *estimated_velocity_ = estimate_release_velocity();
        }
    }

private:
    enum class Phase {
        IDLE = 0,    // No significant force
        LOADING = 1, // Force increasing (energy storage)
        HOLDING = 2, // Force stable at trigger position
        RELEASE = 3  // Force dropping (energy release)
    };

    // -------------------------------------------------------------------------
    // Screw kinematics: angle → spring elongation
    // -------------------------------------------------------------------------

    void update_spring_elongation() {
        const double angle_rad = *screw_angle_;

        // Trigger displacement from top position (positive = downward)
        const double trigger_disp_m =
            (angle_rad - screw_top_angle_rad_) * constants::SCREW_PITCH_M / (2.0 * M_PI);

        // Spring elongation via pulley ratio (1:2)
        spring_elongation_m_ = trigger_disp_m / constants::PULLEY_RATIO;
        *spring_elongation_ = spring_elongation_m_;
    }

    // -------------------------------------------------------------------------
    // Spring force model (force in grams)
    // -------------------------------------------------------------------------

    double predict_force_from_displacement(double x) const {
        // Two-point linear model: F = K1*x + F_preload
        // K1 is calibrated from HOLDING phase
        const double K1 = *calibration_valid_ ? *calibrated_K1_ : K1_fallback_;

        // Add nonlinear term if configured
        const double F_linear = K1 * x + preload_force_g_;
        const double F_nonlinear = K2_ * std::pow(std::abs(x), n_) * (x >= 0 ? 1.0 : -1.0);

        return F_linear + F_nonlinear;
    }

    // -------------------------------------------------------------------------
    // Phase detection
    // -------------------------------------------------------------------------

    void detect_loading_phase() {
        if (force_history_.size() < 10) {
            return;
        }

        const double F_current = force_history_.back();
        const double F_prev = force_history_[force_history_.size() - 10];
        const double dF = F_current - F_prev;

        switch (phase_) {
        case Phase::IDLE:
            if (F_current > force_threshold_idle_ && dF > 500.0) { // 500g ≈ 5N
                phase_ = Phase::LOADING;
                F_peak_ = F_current;
            }
            break;

        case Phase::LOADING:
            if (F_current > F_peak_) {
                F_peak_ = F_current;
            }
            // Detect transition to holding (force stabilizes)
            if (F_current > force_threshold_peak_ && std::abs(dF) < holding_tolerance_) {
                holding_counter_++;
                if (holding_counter_ >= defaults::HOLDING_DURATION_TICKS) {
                    phase_ = Phase::HOLDING;
                    holding_counter_ = 0;
                    // Reset calibration buffers
                    force_calibration_buffer_.clear();
                    angle_calibration_buffer_.clear();
                }
            } else {
                holding_counter_ = 0;
            }
            break;

        case Phase::HOLDING:
            // Detect release (force drops significantly)
            if (dF < -2000.0) { // 2000g ≈ 20N
                phase_ = Phase::RELEASE;
                F_release_start_ = F_current;
            }
            break;

        case Phase::RELEASE:
            // Return to idle when force drops below threshold
            if (F_current < force_threshold_idle_) {
                phase_ = Phase::IDLE;
            }
            break;
        }
    }

    // -------------------------------------------------------------------------
    // Two-point calibration in HOLDING phase
    // -------------------------------------------------------------------------

    void calibrate_holding_point(double F_measured, double angle_measured) {
        force_calibration_buffer_.push_back(F_measured);
        angle_calibration_buffer_.push_back(angle_measured);

        if (force_calibration_buffer_.size() > defaults::MAX_CALIBRATION_SAMPLES) {
            force_calibration_buffer_.pop_front();
            angle_calibration_buffer_.pop_front();
        }

        if (force_calibration_buffer_.size() >= defaults::MIN_CALIBRATION_SAMPLES) {
            // Calculate average force and angle
            double sum_force = 0.0;
            double sum_angle = 0.0;
            for (size_t i = 0; i < force_calibration_buffer_.size(); ++i) {
                sum_force += force_calibration_buffer_[i];
                sum_angle += angle_calibration_buffer_[i];
            }

            *calibrated_max_force_ =
                sum_force / static_cast<double>(force_calibration_buffer_.size());
            *calibrated_max_angle_ =
                sum_angle / static_cast<double>(angle_calibration_buffer_.size());
            *calibration_samples_ = static_cast<int>(force_calibration_buffer_.size());

            // Calculate spring elongation at HOLDING position
            const double trigger_disp_hold = (*calibrated_max_angle_ - screw_top_angle_rad_)
                                           * constants::SCREW_PITCH_M / (2.0 * M_PI);
            const double x_hold = trigger_disp_hold / constants::PULLEY_RATIO;

            // Two-point linear calibration: K1 = (F_max - F_preload) / x_hold
            if (x_hold > 0.001) { // Minimum 1mm elongation
                *calibrated_K1_ = (*calibrated_max_force_ - preload_force_g_) / x_hold;
            }
        }
    }

    // -------------------------------------------------------------------------
    // Energy and velocity estimation
    // -------------------------------------------------------------------------

    double calculate_stored_energy() {
        // E = ∫F(x)dx, but F is in grams, need to convert to Newtons for energy
        // F(N) = F(g) * 0.00981
        // E = ∫[K1*x + K2*|x|^n] * 0.00981 dx

        const double x = spring_elongation_m_;
        const double K1 = *calibration_valid_ ? *calibrated_K1_ : K1_fallback_;

        const double E_linear = 0.5 * K1 * x * x;
        const double E_nonlinear = K2_ * std::pow(std::abs(x), n_ + 1) / (n_ + 1);

        // Convert from g·m to J (multiply by 0.00981)
        const double E_total = (E_linear + E_nonlinear) * 0.00981;

        // Apply hysteresis efficiency
        double efficiency = 1.0;
        if (phase_ == Phase::LOADING) {
            efficiency = constants::ETA_LOADING;
        } else if (phase_ == Phase::RELEASE) {
            efficiency = constants::ETA_UNLOADING;
        }

        return E_total * efficiency;
    }

    double estimate_release_velocity() {
        // KE = ½m·v² = E_stored * friction_coefficient
        // v = sqrt(2 * E_stored * friction_coeff / m)
        const double E_available = *stored_energy_ * constants::FRICTION_COEFF;

        // Movable pulleys provide 2:1 mechanical advantage
        const double v_base = std::sqrt(2.0 * E_available / constants::PROJECTILE_MASS_KG);
        const double v_projectile = v_base * constants::PULLEY_RATIO;

        return v_projectile;
    }

    // -------------------------------------------------------------------------
    // Parameter loading
    // -------------------------------------------------------------------------

    void load_parameters() {
        // === Must calibrate ===
        screw_top_angle_rad_ = get_parameter("screw_top_angle_rad").as_double();
        preload_force_g_ = get_parameter("preload_force_g").as_double();

        // === Optional: nonlinear spring terms ===
        K2_ = get_parameter_or("spring_K2", 0.0); // Default: pure linear
        n_ = get_parameter_or("spring_exponent", 2.0);

        // === Optional tuning ===
        force_threshold_idle_ =
            get_parameter_or("force_threshold_idle", defaults::FORCE_THRESHOLD_IDLE);
        force_threshold_peak_ =
            get_parameter_or("force_threshold_peak", defaults::FORCE_THRESHOLD_PEAK);
        holding_tolerance_ = get_parameter_or("holding_tolerance", defaults::HOLDING_TOLERANCE);
    }

    template <typename T>
    T get_parameter_or(const std::string& name, T default_value) {
        if (has_parameter(name)) {
            return get_parameter(name).get_value<T>();
        }
        return default_value;
    }

    // Model parameters (force in grams)
    double K2_, n_;
    double K1_fallback_; // Used before calibration
    double screw_top_angle_rad_;
    double preload_force_g_;

    // Phase detection parameters (force in grams)
    double force_threshold_idle_;
    double force_threshold_peak_;
    double holding_tolerance_;

    // State variables
    Phase phase_ = Phase::IDLE;
    double spring_elongation_m_ = 0.0;
    double F_peak_ = 0.0;
    double F_release_start_ = 0.0;
    int holding_counter_ = 0;

    std::deque<double> force_history_;
    std::deque<double> force_calibration_buffer_;
    std::deque<double> angle_calibration_buffer_;

    // Interfaces
    InputInterface<int> force_ch1_;
    InputInterface<int> force_ch2_;
    InputInterface<double> screw_angle_;

    OutputInterface<double> spring_elongation_;
    OutputInterface<double> predicted_force_;
    OutputInterface<double> measured_force_;
    OutputInterface<double> stored_energy_;
    OutputInterface<double> estimated_velocity_;
    OutputInterface<int> phase_output_;
    OutputInterface<double> calibrated_max_force_;
    OutputInterface<double> calibrated_max_angle_;
    OutputInterface<double> calibrated_K1_;
    OutputInterface<int> calibration_samples_;
    OutputInterface<bool> calibration_valid_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::SpringDynamicsModel, rmcs_executor::Component)
