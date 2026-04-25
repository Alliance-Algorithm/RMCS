#include <cmath>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

/**
 * @brief Kalman Filter with Spring Dynamics Model Prior
 *
 * Units:
 * - Force: grams (g) - matches sensor and model output
 * - Force rate: g/s
 *
 * Architecture:
 * - SpringDynamicsModel provides the PRIOR (process model):
 *   - Maps screw angle → spring elongation → predicted force (g)
 *   - Detects phases (IDLE/LOADING/HOLDING/RELEASE)
 *   - Auto-calibrates K1 during HOLDING phase (two-point calibration)
 *
 * - ForceKalmanFilter provides the POSTERIOR (measurement fusion):
 *   - Fuses model prediction with sensor measurements
 *   - Reduces sensor noise while maintaining physical consistency
 *   - Outputs filtered force (g) and force rate (g/s)
 *
 * State vector: X = [F, dF/dt]^T
 * - F: total spring force (g)
 * - dF/dt: force rate of change (g/s)
 *
 * Prediction step:
 *   Uses SpringDynamicsModel's predicted_force as prior
 *   X_pred = α·F_model + (1-α)·F_kinematic
 *
 * Update step:
 *   Fuses with measured force from sensors
 *   X_post = X_pred + K·(F_measured - F_pred)
 */
class ForceKalmanFilter
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ForceKalmanFilter()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {

        // Input: SpringDynamicsModel outputs (PRIOR, in grams)
        register_input("/dart/dynamics/predicted_force", predicted_force_);
        register_input("/dart/dynamics/measured_force", measured_force_);
        register_input("/dart/dynamics/spring_elongation", spring_elongation_);
        register_input("/dart/dynamics/phase", phase_);

        // Output: Kalman filtered results (POSTERIOR, in grams)
        register_output("/dart/kalman/filtered_force", filtered_force_, 0.0);
        register_output("/dart/kalman/force_rate", force_rate_, 0.0);
        register_output("/dart/kalman/innovation", innovation_, 0.0);
        register_output("/dart/kalman/kalman_gain", kalman_gain_, 0.0);

        load_parameters();
        init_kalman();
    }

    void update() override {
        if (!predicted_force_.ready() || !measured_force_.ready()) {
            return;
        }

        const double F_model = *predicted_force_;   // grams
        const double F_measured = *measured_force_; // grams

        // Kalman predict: use model prediction as prior
        kalman_predict(F_model);

        // Kalman update: fuse with measurement
        kalman_update(F_measured);

        // Output filtered results
        *filtered_force_ = X_[0]; // grams
        *force_rate_ = X_[1];     // g/s
    }

private:
    // -------------------------------------------------------------------------
    // Kalman filter
    // -------------------------------------------------------------------------

    void init_kalman() {
        X_[0] = 0.0;
        X_[1] = 0.0;
        P_[0][0] = 1000000.0; // Initial uncertainty in force (g^2)
        P_[0][1] = 0.0;
        P_[1][0] = 0.0;
        P_[1][1] = 10000.0;   // Initial uncertainty in force rate (g^2/s^2)
    }

    void kalman_predict(double F_model) {
        // Use model prediction as prior, blended with kinematic extrapolation
        const double F_kinematic = X_[0] + X_[1] * dt_;
        const double F_pred = model_weight_ * F_model + (1.0 - model_weight_) * F_kinematic;

        // Estimate force rate from model change
        const double dF_model = (F_model - F_prev_model_) / dt_;
        const double R_pred = model_weight_ * dF_model + (1.0 - model_weight_) * X_[1];

        F_prev_model_ = F_model;

        // Covariance prediction: P = A*P*A^T + Q
        const double p00 = P_[0][0] + dt_ * (P_[1][0] + P_[0][1]) + dt_ * dt_ * P_[1][1] + Q_force_;
        const double p01 = P_[0][1] + dt_ * P_[1][1];
        const double p10 = P_[1][0] + dt_ * P_[1][1];
        const double p11 = P_[1][1] + Q_rate_;

        X_[0] = F_pred;
        X_[1] = R_pred;
        P_[0][0] = p00;
        P_[0][1] = p01;
        P_[1][0] = p10;
        P_[1][1] = p11;
    }

    void kalman_update(double z) {
        // Innovation: measurement - prediction (grams)
        const double innov = z - X_[0];
        *innovation_ = innov;

        // Innovation covariance: S = H*P*H^T + R
        const double S = P_[0][0] + R_sensor_;

        // Kalman gain: K = P*H^T / S
        const double K0 = P_[0][0] / S;
        const double K1 = P_[1][0] / S;
        *kalman_gain_ = K0;

        // State update
        X_[0] += K0 * innov;
        X_[1] += K1 * innov;

        // Covariance update: P = (I - K*H)*P
        const double p00 = (1.0 - K0) * P_[0][0];
        const double p01 = (1.0 - K0) * P_[0][1];
        const double p10 = P_[1][0] - K1 * P_[0][0];
        const double p11 = P_[1][1] - K1 * P_[0][1];

        P_[0][0] = p00;
        P_[0][1] = p01;
        P_[1][0] = p10;
        P_[1][1] = p11;
    }

    // -------------------------------------------------------------------------
    // Parameter loading
    // -------------------------------------------------------------------------

    void load_parameters() {
        dt_ = 1.0 / get_parameter_or("update_rate_hz", 1000.0);

        // Kalman noise parameters (in grams and g/s)
        Q_force_ = get_parameter("Q_force").as_double();
        Q_rate_ = get_parameter("Q_rate").as_double();
        R_sensor_ = get_parameter("R_sensor").as_double();

        // Model weight: 0=pure sensor, 1=pure model
        model_weight_ = get_parameter("model_weight").as_double();
    }

    template <typename T>
    T get_parameter_or(const std::string& name, T default_value) {
        if (has_parameter(name))
            return get_parameter(name).get_value<T>();
        return default_value;
    }

    // Kalman state and covariance
    double X_[2] = {0.0, 0.0}; // [F(g), dF/dt(g/s)]
    double P_[2][2] = {};      // Covariance

    // Noise parameters (in grams)
    double dt_;
    double Q_force_;      // Process noise: force (g^2)
    double Q_rate_;       // Process noise: force rate (g^2/s^2)
    double R_sensor_;     // Measurement noise (g^2)
    double model_weight_; // Model vs sensor weight

    // State tracking
    double F_prev_model_ = 0.0;

    // Interfaces
    InputInterface<double> predicted_force_;
    InputInterface<double> measured_force_;
    InputInterface<double> spring_elongation_;
    InputInterface<int> phase_;

    OutputInterface<double> filtered_force_;
    OutputInterface<double> force_rate_;
    OutputInterface<double> innovation_;
    OutputInterface<double> kalman_gain_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::ForceKalmanFilter, rmcs_executor::Component)
