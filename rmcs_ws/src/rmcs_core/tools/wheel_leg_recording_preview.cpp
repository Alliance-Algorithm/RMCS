#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "identification/wheel_leg_pair_recording_plan.hpp"

using namespace rmcs_core::controller::identification;

int main(int argc, char** argv) {
    try {
        if (argc != 3)
            throw std::invalid_argument(
                "Usage: wheel_leg_recording_preview <manifest.json> <preview.csv>; config on "
                "stdin");
        PairRecordingConfig c;
        std::cin >> c.run >> c.hip_sign >> c.hip_zero >> c.beta_min >> c.beta_max >> c.max_speed[0]
            >> c.max_speed[1] >> c.max_acceleration[0] >> c.max_acceleration[1] >> c.beta_tolerance
            >> c.arrival_speed >> c.arrival_stable_s >> c.arrival_timeout_s >> c.center_step_rad
            >> c.center_max_rad >> c.move_s >> c.dwell_s >> c.baseline_s;
        std::size_t count = 0;
        std::cin >> count;
        if (count < 3 || count > 1000)
            throw std::invalid_argument("Invalid geometry size");
        std::vector<double> beta(count), delta(count);
        for (std::size_t i = 0; i < count; ++i)
            std::cin >> beta[i] >> delta[i];
        if (!std::cin)
            throw std::invalid_argument("Invalid preview configuration");
        PairRecordingPlan plan{PairGeometry{std::move(beta), std::move(delta)}, c};
        std::ofstream manifest{argv[1]}, csv{argv[2]};
        if (!manifest || !csv)
            throw std::runtime_error("Cannot open preview outputs");
        manifest << plan.manifest_json() << '\n';
        csv << "time_s,segment_id,role,waveform,cycle,jump_phase,beta_requested_deg,q_hip_rad,q_"
               "aux_rad,dq_hip_rad_s,dq_aux_rad_s,ddq_hip_rad_s2,ddq_aux_rad_s2\n"
            << std::setprecision(17);
        for (std::size_t tick = 0;
             tick <= static_cast<std::size_t>(std::round(plan.duration() * 50)); ++tick) {
            const double t = std::min(tick / 50.0, plan.duration());
            const auto value = plan.at(t);
            const auto& segment = plan.segments().at(static_cast<std::size_t>(value.segment_id));
            csv << t << ',' << value.segment_id << ',' << value.validation << ','
                << static_cast<int>(value.waveform) << ',' << segment.cycle << ','
                << static_cast<int>(segment.jump) << ',' << value.beta / PairGeometry::rad;
            for (auto x : value.position)
                csv << ',' << x;
            for (auto x : value.velocity)
                csv << ',' << x;
            for (auto x : value.acceleration)
                csv << ',' << x;
            csv << '\n';
        }
        if (!manifest || !csv)
            throw std::runtime_error("Cannot write preview outputs");
        std::cout << c.run << ": " << plan.segments().size() << " segments, " << plan.duration()
                  << " nominal seconds\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
