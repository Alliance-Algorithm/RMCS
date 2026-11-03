#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware/big_yaw_position_reference.hpp"

namespace {

using rmcs_core::hardware::BigYawPositionReference;
using rmcs_core::hardware::big_yaw_feedback_continuous;

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr std::array kInvalid{kNan, kInf, -kInf};

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-10)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

void startup_requires_stationary_double_down_origin() {
    BigYawPositionReference reference;
    require(!reference.tracking_fault(), "startup must not invent a tracking fault");
    require(!reference.origin_valid(), "startup needs an explicit origin");
    require(!reference.update(false, true, 0.7, 0.0),
            "valid startup feedback without double-down must remain blocked");
    require(!reference.origin_valid(), "normal switch positions cannot establish an origin");
    require(reference.update(true, true, 0.7, 0.0),
            "stationary double-down must establish the first origin");
    require(reference.origin_valid(), "successful startup must mark the origin valid");
    near(reference.origin(), 0.7, "startup origin must use the current encoder position");
    require(reference.update(false, true, 0.9, 0.3),
            "normal motion after explicit startup must be ready");
    near(reference.origin(), 0.7, "normal motion must not move the protection origin");
}

void dropout_sequence_recovers_only_through_double_down() {
    BigYawPositionReference reference;
    require(reference.update(true, true, 0.4, 0.0), "initial protection must establish origin");
    require(reference.update(false, true, 0.9, 0.3), "normal operation must initially be ready");

    reference.mark_fault();
    require(reference.tracking_fault(), "a feedback tracking failure must latch");
    require(!reference.origin_valid(), "a tracking failure must invalidate the old origin");
    require(!reference.update(false, false, kNan, kNan), "feedback loss must block readiness");
    for (int cycle = 0; cycle < 1000; ++cycle)
        require(!reference.update(false, true, 1.2, 0.0),
                "recovered feedback without double-down must not unlock a latched fault");
    require(reference.tracking_fault(), "normal feedback alone must leave the fault latched");

    // Recovery must run despite the command path still being blocked by the fault.
    require(reference.update(true, true, 1.2, 0.0),
            "valid stationary double-down must recover the latched fault");
    require(!reference.tracking_fault(), "double-down recovery must clear the fault");
    require(reference.origin_valid(), "double-down recovery must validate the rebuilt origin");
    near(reference.origin(), 1.2, "recovery must rebuild origin rather than reuse the pre-dropout value");
    require(reference.update(false, true, 1.3, 0.4),
            "releasing double-down after recovery must allow readiness");
}

void moving_double_down_does_not_reset_fault() {
    for (const double velocity : {-100.0, -0.100001, 0.100001, 100.0}) {
        BigYawPositionReference reference;
        require(reference.update(true, true, 0.3, 0.0), "set initial origin");
        reference.mark_fault();
        require(!reference.update(true, true, 1.0, velocity),
                "double-down while moving must not clear a fault");
        require(reference.tracking_fault() && !reference.origin_valid(),
                "moving recovery attempt must preserve the latched fault and invalid origin");
        require(reference.update(true, true, 1.1, 0.1),
                "slowing to the inclusive reset threshold must allow recovery");
        near(reference.origin(), 1.1, "failed moving reset must not establish an origin");
    }
    for (const double velocity : {-0.1, 0.0, 0.1}) {
        BigYawPositionReference reference;
        reference.mark_fault();
        require(reference.update(true, true, -0.8, velocity),
                "either low-speed sign and the reset threshold must allow recovery");
        near(reference.origin(), -0.8, "low-speed reset must use the current position");
    }
}

void invalid_double_down_feedback_cannot_rearm() {
    for (const double invalid : kInvalid) {
        for (const bool invalid_angle : {false, true}) {
            BigYawPositionReference reference;
            require(reference.update(true, true, 0.3, 0.0), "set initial origin");
            reference.mark_fault();
            require(!reference.update(true, true, invalid_angle ? invalid : 1.0,
                                      invalid_angle ? 0.0 : invalid),
                    "nonfinite angle or velocity must prevent double-down recovery");
            require(reference.tracking_fault() && !reference.origin_valid(),
                    "nonfinite recovery input must leave a latched fault");
            require(!reference.update(false, true, 1.0, 0.0),
                    "subsequent normal feedback still requires explicit double-down");
            require(reference.update(true, true, 1.0, 0.0),
                    "finite stationary feedback must allow a later explicit recovery");
        }
    }
    BigYawPositionReference reference;
    reference.mark_fault();
    require(!reference.update(true, false, 1.0, 0.0),
            "finite but invalid position feedback must not recover a fault");
    require(reference.tracking_fault() && !reference.origin_valid(),
            "position-valid flag is required for recovery");
    require(reference.update(true, true, 1.4, 0.0), "valid double-down must recover afterward");
    near(reference.origin(), 1.4, "invalid recovery attempts cannot choose the future origin");
}

void valid_origin_stays_fixed_until_a_new_fault() {
    BigYawPositionReference reference;
    require(reference.update(true, true, -0.5, 0.0), "set initial origin");
    for (const double angle : {-0.4, 0.0, 0.2}) {
        require(reference.update(true, true, angle, 0.0),
                "remaining in double-down with valid tracking must stay ready");
        near(reference.origin(), -0.5, "double-down must not continually chase a moving angle");
    }
    reference.mark_fault();
    reference.mark_fault();
    require(!reference.origin_valid() && reference.tracking_fault(),
            "repeated fault marks must remain latched and invalidate origin");
    require(reference.update(true, true, 0.6, 0.0), "double-down must recover repeated fault marks");
    near(reference.origin(), 0.6, "a new fault must permit a fresh origin");
}

void invalid_live_feedback_blocks_ready_without_implicit_fault_reset() {
    BigYawPositionReference reference;
    require(reference.update(true, true, 0.25, 0.0), "set initial origin");
    require(!reference.update(false, false, 0.25, 0.0), "invalid position must block readiness");
    for (const double invalid : kInvalid) {
        require(!reference.update(false, true, invalid, 0.0), "nonfinite live angle blocks readiness");
        require(!reference.update(false, true, 0.25, invalid), "nonfinite live speed blocks readiness");
    }
    // The caller explicitly latches tracking faults; transient invalid samples
    // must never overwrite an established origin in this state helper.
    near(reference.origin(), 0.25, "invalid live samples must not overwrite origin");
    require(reference.origin_valid() && !reference.tracking_fault(),
            "only mark_fault changes the tracking fault latch");
    require(reference.update(false, true, 0.3, 0.0), "valid live feedback is ready absent a marked fault");
}

void configured_speed_allows_sparse_high_speed_can_feedback() {
    for (const double direction : {-1.0, 1.0}) {
        for (const int64_t interval_ns : {20'000'000, 25'000'000, 50'000'000}) {
            const double delta = direction * 2.25 * (interval_ns * 1e-9);
            require(big_yaw_feedback_continuous(delta, interval_ns, 2.25),
                    "configured high speed must accept valid 2.25-radian/s CAN position updates");
        }
        require(!big_yaw_feedback_continuous(direction * .1125, 50'000'000),
                "default continuity guard must retain its historical one-radian/s envelope");
        require(big_yaw_feedback_continuous(direction * .08, 50'000'000),
                "default continuity guard must retain the inclusive encoder tolerance");
        require(!big_yaw_feedback_continuous(direction * .080001, 50'000'000),
                "default continuity guard must still reject motion outside its historical envelope");
        require(!big_yaw_feedback_continuous(direction * .15, 50'000'000, 2.25),
                "configured speed must still reject an impossible angle jump");
    }
}

void configured_speed_does_not_relax_sample_timing_or_finite_checks() {
    for (const int64_t interval_ns : {-1, 0, 50'000'001})
        require(!big_yaw_feedback_continuous(0.0, interval_ns, 2.25),
                "high-speed feedback must retain positive timing and the fifty-millisecond gap bound");
    for (const double invalid : kInvalid) {
        require(!big_yaw_feedback_continuous(invalid, 20'000'000, 2.25),
                "a configured high-speed envelope must not accept nonfinite angle motion");
        require(!big_yaw_feedback_continuous(0.0, 20'000'000, invalid),
                "a continuity velocity envelope must be finite");
    }
    for (const double invalid : {-1.0, 0.0})
        require(!big_yaw_feedback_continuous(0.0, 20'000'000, invalid),
                "a continuity velocity envelope must be positive");
}

void rejected_high_speed_feedback_preserves_explicit_fault_rearming() {
    for (const auto& [delta, interval_ns] : {
             std::pair{.15, int64_t{50'000'000}},
             std::pair{.01, int64_t{50'000'001}}}) {
        BigYawPositionReference reference;
        require(reference.update(true, true, .4, 0.0), "stationary startup must establish an origin");
        require(!big_yaw_feedback_continuous(delta, interval_ns, 2.25),
                "fault sequence must begin with a rejected high-speed CAN sample");
        reference.mark_fault();
        require(!reference.update(false, true, .8, 0.0),
                "valid feedback after a continuity rejection must not silently clear the tracking fault");
        require(!reference.update(true, true, .8, 2.25),
                "double-down while moving at high speed must not rebuild the position origin");
        require(reference.update(true, true, .8, 0.0),
                "stationary double-down must still recover after a high-speed continuity fault");
        near(reference.origin(), .8, "high-speed fault recovery must capture a fresh encoder origin");
    }
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"startup requires stationary double-down", &startup_requires_stationary_double_down_origin},
        std::pair{"dropout recovers only through double-down", &dropout_sequence_recovers_only_through_double_down},
        std::pair{"moving double-down cannot reset a fault", &moving_double_down_does_not_reset_fault},
        std::pair{"invalid double-down feedback cannot rearm", &invalid_double_down_feedback_cannot_rearm},
        std::pair{"origin remains fixed until a new fault", &valid_origin_stays_fixed_until_a_new_fault},
        std::pair{"invalid live feedback blocks readiness", &invalid_live_feedback_blocks_ready_without_implicit_fault_reset},
        std::pair{"configured speed accepts sparse high-speed feedback", &configured_speed_allows_sparse_high_speed_can_feedback},
        std::pair{"configured speed retains timing and finite checks", &configured_speed_does_not_relax_sample_timing_or_finite_checks},
        std::pair{"high-speed continuity faults retain explicit rearming", &rejected_high_speed_feedback_preserves_explicit_fault_rearming}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " cases passed\n";
    return failures == 0 ? 0 : 1;
}
