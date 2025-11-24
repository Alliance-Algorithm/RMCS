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
    KalmanFilter() {}

    KalmanFilter(
        const Matrix<state>& A, const Matrix<measure, state>& H, const Matrix<state>& Q,
        const Matrix<measure>& R, const Matrix<state>& W = Matrix<state>::Identity(),
        const Matrix<measure>& V = Matrix<measure>::Identity(),
        const Matrix<state, control>& B = Matrix<state, control>::Zero())
        : measurement_noise_transition_(V)
        , measurement_noise_covariance_(R)
        , state_transition_(A)
        , process_noise_transition_(W)
        , process_noise_covariance_(Q)
        , control_input_(B)
        , measurement_transition_(H) {
        reset();
    }

    void reset() {
        posterior_estimate_ = State::Zero();
        prior_estimate_ = State::Zero();

        posterior_error_covariance_ = 1000.0 * Matrix<state>::Identity();
        prior_error_covariance_ = posterior_error_covariance_;

        kalman_gain_ = Matrix<state, measure>::Zero();
    }

    State update(const Measure& measurement, const Control& control_vector = Control::Zero()) {
        // prediction
        prior_estimate_ = state_transition_ * posterior_estimate_ + control_input_ * control_vector;
        prior_error_covariance_ =
            state_transition_ * posterior_error_covariance_ * state_transition_.transpose()
            + process_noise_transition_ * process_noise_covariance_
                  * process_noise_transition_.transpose();

        // correction
        kalman_gain_ = prior_error_covariance_ * measurement_transition_.transpose()
                     * (measurement_transition_ * prior_error_covariance_
                            * measurement_transition_.transpose()
                        + measurement_noise_transition_ * measurement_noise_covariance_
                              * measurement_noise_transition_.transpose())
                           .inverse();
        posterior_estimate_ +=
            kalman_gain_ * (measurement - measurement_transition_ * prior_estimate_);

        // update
        posterior_error_covariance_ =
            (Matrix<state>::Identity() - kalman_gain_ * measurement_transition_)
            * prior_error_covariance_;

        return posterior_estimate_;
    }

private:
    State prior_estimate_, posterior_estimate_;

    Matrix<measure> measurement_noise_transition_, measurement_noise_covariance_;
    Matrix<state> state_transition_, process_noise_transition_, process_noise_covariance_,
        prior_error_covariance_, posterior_error_covariance_;

    Matrix<state, measure> kalman_gain_;
    Matrix<state, control> control_input_;

    Matrix<measure, state> measurement_transition_;
};
} // namespace rmcs_core::filter