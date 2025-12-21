#pragma once

#include <cstddef>

#include <eigen3/Eigen/Dense>

namespace rmcs_core::filter {

template <size_t m, size_t n = m>
using Matrix = Eigen::Matrix<double, m, n>;

template <size_t state, size_t measure, size_t control = 0>
class KalmanFilter {
    using State = Eigen::Vector<double, state>;
    using Measure = Eigen::Vector<double, measure>;
    using Control = Eigen::Vector<double, control>;

public:
    explicit KalmanFilter(
        const Matrix<state>& A, const Matrix<measure, state>& H, const Matrix<state>& Q,
        const Matrix<measure>& R)
        : measurement_noise_covariance_(R)
        , state_transition_(A)
        , process_noise_covariance_(Q)
        , measurement_transition_(H) {
        reset();

        set_input_transition();
        set_error_covariance();
        set_process_noise_transition();
        set_measurement_noise_transition();
    }

    void reset() {
        prior_estimate_ = State::Zero();
        posterior_estimate_ = State::Zero();

        kalman_gain_ = Matrix<state, measure>::Zero();
    }

    void set_initial_state(const State& initial_state) { posterior_estimate_ = initial_state; }

    void set_input_transition(const Matrix<state, control>& B = Matrix<state, control>::Zero()) {
        input_transition_ = B;
    }

    void set_error_covariance(const Matrix<state>& P = 1000. * Matrix<state>::Identity()) {
        posterior_error_covariance_ = P;
    }

    void set_process_noise_transition(const Matrix<state>& W = Matrix<state>::Identity()) {
        process_noise_transition_ = W;
    }

    void set_measurement_noise_transition(const Matrix<measure>& V = Matrix<measure>::Identity()) {
        measurement_noise_transition_ = V;
    }

    State update(const Measure& measurement, const Control& control_vector = Control::Zero()) {
        // Prediction
        prior_estimate_ =
            state_transition_ * posterior_estimate_ + input_transition_ * control_vector;
        prior_error_covariance_ =
            state_transition_ * posterior_error_covariance_ * state_transition_.transpose()
            + process_noise_transition_ * process_noise_covariance_
                  * process_noise_transition_.transpose();

        // Correction
        kalman_gain_ = prior_error_covariance_ * measurement_transition_.transpose()
                     * (measurement_transition_ * prior_error_covariance_
                            * measurement_transition_.transpose()
                        + measurement_noise_transition_ * measurement_noise_covariance_
                              * measurement_noise_transition_.transpose())
                           .inverse();
        prior_estimate_ += kalman_gain_ * (measurement - measurement_transition_ * prior_estimate_);

        // // Update
        posterior_error_covariance_ =
            (Matrix<state>::Identity() - kalman_gain_ * measurement_transition_)
            * prior_error_covariance_;
        posterior_estimate_ = prior_estimate_;

        return posterior_estimate_;
    }

private:
    State prior_estimate_, posterior_estimate_;

    Matrix<measure> measurement_noise_transition_, measurement_noise_covariance_;
    Matrix<state> state_transition_, process_noise_transition_, process_noise_covariance_,
        prior_error_covariance_, posterior_error_covariance_;

    Matrix<state, measure> kalman_gain_;
    Matrix<state, control> input_transition_;

    Matrix<measure, state> measurement_transition_;
};
} // namespace rmcs_core::filter