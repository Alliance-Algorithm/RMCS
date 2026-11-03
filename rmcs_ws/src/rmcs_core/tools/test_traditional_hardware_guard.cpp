#ifdef NDEBUG
# undef NDEBUG
#endif
#include <array>
#include <cassert>
#include <limits>

#include "controller/wheelleg/traditional_hardware_guard.hpp"

using rmcs_core::wheelleg::traditional_relative_angles_valid;
using rmcs_core::wheelleg::traditional_start_pose_valid;
using rmcs_core::wheelleg::TraditionalHardwareGuard;

int main() {
    constexpr std::uint64_t period = 1'000'000, maximum_gap = 2'500'000;
    std::uint64_t now = 1'000'000'000;
    TraditionalHardwareGuard guard;
    TraditionalHardwareGuard::Input input{
        .arm = true,
        .remote_fresh = true,
        .control_mode = true,
        .admitted = false,
        .feedback_fresh = true,
        .attitude_valid = true,
        .power_valid = true,
        .startup_pose_valid = true,
        .drives_ready = false};
    auto step = [&] { return guard.step(now += period, input, maximum_gap, 1'000'000'000); };
    assert(step().failure == 10);
    assert(!guard.armed());
    input.admitted = true;
    auto out = step();
    assert(out.enable && !out.compute);
    input.arm = false;
    input.drives_ready = true;
    assert(step().compute);
    now += 3 * period;
    out = step();
    assert(out.failure == 17 && !out.enable && guard.faulted());
    assert(step().failure == 17);
    input.arm = true;
    assert(!step().enable);
    input.reset = true;
    assert(!step().enable && !guard.faulted());
    input.reset = false;
    assert(step().compute);
    input.schedule_valid = false;
    assert(step().failure == 17);
    input.schedule_valid = true;
    assert(!step().enable);
    input.reset = true;
    step();
    input.reset = false;
    assert(step().compute);
    for (int fault : {11, 12, 13, 14, 16, 18}) {
        input.remote_fresh = fault != 11;
        input.feedback_fresh = fault != 12;
        input.geometry_valid = fault != 16;
        input.attitude_valid = fault != 13;
        input.power_valid = fault != 14;
        input.drives_ready = fault != 18;
        out = step();
        assert(out.failure == fault && !out.enable && guard.faulted());
        input.remote_fresh = input.feedback_fresh = input.attitude_valid = input.geometry_valid =
            true;
        input.power_valid = input.drives_ready = true;
        assert(!step().enable);
        input.reset = true;
        step();
        input.reset = false;
        assert(step().compute);
    }
    input.control_mode = false;
    assert(!step().enable && !guard.armed());
    input.control_mode = true;
    input.arm = false;
    assert(!step().enable);
    input.arm = true;
    input.startup_pose_valid = false;
    assert(step().failure == 15 && !guard.faulted());
    input.startup_pose_valid = true;
    input.drives_ready = false;
    assert(step().enable);
    now += 1'000'000'001;
    assert(step().failure == 18 && guard.faulted());

    std::array<double, 54> rows{};
    rows[0] = .24;
    rows[27] = .35;
    rows[2] = .1;
    rows[4] = -.1;
    rows[29] = .8;
    rows[31] = -.8;
    std::array<double, 6> position{0., .4, 0., -.4, 0., 0.};
    assert(traditional_start_pose_valid(rows, position, .35));
    auto wound = position;
    for (std::size_t i = 0; i < 4; ++i)
        wound[i] += (i < 2 ? 4. : -6.) * std::numbers::pi;
    assert(traditional_start_pose_valid(rows, wound, .35));
    position[0] += 1.;
    position[1] += 1.;
    assert(!traditional_start_pose_valid(rows, position, .35));
    position = {0., .9, 0., -.4, 0., 0.};
    assert(!traditional_start_pose_valid(rows, position, .35));
    position[1] = std::numeric_limits<double>::quiet_NaN();
    assert(!traditional_start_pose_valid(rows, position, .35));
    const std::array<double, 2> lower{-.14089326779459305, -1.4283102010168598};
    const std::array<double, 2> upper{1.428276168918513, .14084732125908733};
    position = {1.6, 3.028272137913126, -1.6, -3.0283141661876525, 0., 0.};
    assert(traditional_relative_angles_valid(position, lower, upper, .01));
    position[0] += .5;
    position[1] += .5;
    assert(traditional_relative_angles_valid(position, lower, upper, .01));
    position[1] += .02;
    assert(!traditional_relative_angles_valid(position, lower, upper, .01));
    position = {0., .5, 0., upper[1] + .02, 0., 0.};
    assert(!traditional_relative_angles_valid(position, lower, upper, .01));
}
