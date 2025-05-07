#include <algorithm>
#include <cmath>
#include <tuple>

namespace rmcs_core::controller::chassis {

class PowerController {
    using Formula = std::tuple<double, double, double>;

public:
    struct Config {
        explicit Config() {
            this->k1            = 2.958580e+00;
            this->k2            = 3.082190e-03;
            this->no_load_power = 2.72;
            this->motor_count   = 4;
        }

        Config& set_k1(double value) { return k1 = value, *this; }
        Config& set_k2(double value) { return k2 = value, *this; }
        Config& set_no_load_power(double value) { return no_load_power = value, *this; }
        Config& set_motor_count(size_t value) { return motor_count = value, *this; }

        double k1;
        double k2;
        double no_load_power;
        size_t motor_count;
    };

    explicit PowerController(const Config& config)
        : k1_(config.k1)
        , k2_(config.k2)
        , no_load_power_(config.no_load_power)
        , motor_count_(config.motor_count) {}

    double update_power_scaling_factor(
        const double (&motor_velocities)[], const double (&motor_unrestricted_torques)[]) {
        double power_limit = power_limit_;

        double k = 0;
        double a = 0, b = 0, c = 0;

        if (std::isnan(power_limit)) {
            k = 0;
        } else {
            for (size_t index = 0; index < motor_count_; index++) {
                auto [ia, ib, ic] =
                    update_formula(motor_velocities[index], motor_unrestricted_torques[index]);
                a += ia;
                b += ib;
                c += ic;
            }
            auto predict_power = a + b + c;

            if (predict_power < power_limit) {
                k = 1;
            } else {
                c -= power_limit;
                k = (-b + std::sqrt(std::pow(b, 2) - 4 * a * c)) / (2 * a);
                if (std::isnan(k))
                    k = 0;
                else
                    k = std::clamp(k, 0.0, 1.0);
            }
        }
        return k;
    }

    void store_status(const double& power_limit) { power_limit_ = power_limit; }

private:
    Formula update_formula(double velocity, double unrestricted_torque) const {
        double max_torque = motor_max_control_torque_;
        double a = 0, b = 0, c = 0;
        if (std::isnan(unrestricted_torque))
            unrestricted_torque = 0;
        unrestricted_torque = std::clamp(unrestricted_torque, -max_torque, max_torque);

        a = k1_ * std::pow(unrestricted_torque, 2);
        b = velocity * unrestricted_torque;
        c = k2_ * std::pow(velocity, 2) + no_load_power_ / static_cast<double>(motor_count_);

        return Formula{a, b, c};
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    const double k1_, k2_, no_load_power_;
    const size_t motor_count_;
    double power_limit_;

    double motor_max_control_torque_;
};
} // namespace rmcs_core::controller::chassis
