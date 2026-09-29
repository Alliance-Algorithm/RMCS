#include <fstream>
#include <iomanip>
#include <iostream>
#include "identification/wheel_leg_pair_multiband_plan.hpp"
using namespace rmcs_core::controller::identification;
int main(int argc, char** argv) {
PairMultibandConfig c;
c.common_hz = {0.08,0.75,2.0,6.0};
c.relative_hz = {0.1,0.9,2.5,6.0};
c.common_amplitude = {1.0471975511965976,0.32,0.04};
c.relative_amplitude = {0.12,0.08,0.025};
c.band_s = {24.0,16.0,12.0};
c.shape_offset = 0.04;
c.eighth_turn_s = 0.8;
c.dwell_s = 1.5;
c.ramp_s = 2.0;
c.common_step = 0.35;
c.relative_step = 0.12;
c.step_rise_s = 0.22;
c.validation_s = 30.0;
c.load_offset = .2;
c.validation_relative_scale = 3;
PairLimits lim;
lim.root_min = {-100.0,-100.0,-100.0,-100.0};
lim.root_max = {100.0,100.0,100.0,100.0};
lim.max_speed = {6.0,6.0,6.0,6.0};
lim.max_acceleration = {80.0,80.0,80.0,80.0};
lim.braking_acceleration = {0.4,0.4,0.4,0.4};
lim.max_torque = {40.0,40.0,40.0,40.0};
lim.spring_min = {-1.42,-0.16};
lim.spring_max = {0.16,1.42};
lim.spring_margin = 0.005; lim.joint_margin = 0.05;

const std::array initial{1.031609, -.3510346};
PairMultibandPlan p(c, initial, -1.415, .155);
std::ofstream trace(argv[1]), manifest(argv[2]);
trace << std::setprecision(12) << "time_s,segment,role,waveform,q_hip,q_aux,dq_hip,dq_aux,ddq_hip,ddq_aux\n";
manifest << std::setprecision(12) << "[\n";
int id=0;
for (const auto& s : p.segments()) {
 if(id) manifest << ",\n";
 manifest << "{\"id\":" << id++ << ",\"name\":\"" << s.name << "\",\"start_s\":" << s.start_s
          << ",\"duration_s\":" << s.duration_s << ",\"end_s\":" << s.end_s
          << ",\"band\":" << s.band << ",\"validation\":" << (s.validation ? "true" : "false")
          << ",\"mean_drive_angle_rad\":" << s.from[0] << ",\"drive_difference_rad\":" << s.from[1]
          << ",\"relative_amplitude_cap_rad\":" << p.relative_amplitude(c.relative_amplitude[s.band], s.from[1]) << "}";
}
manifest << "\n]\n";
double speed=0,accel=0;
for (std::size_t i=0; i<=static_cast<std::size_t>(std::ceil(p.duration()/.001)); ++i) {
 double t=std::min(i*.001,p.duration()); auto s=p.at(t);
 if(check_probe_reference(lim, 0, s)!=PairFault::kNone) return 2;
 for(int j: {0,1}) { speed=std::max(speed,std::abs(s.velocity[j])); accel=std::max(accel,std::abs(s.acceleration[j])); }
 if(i%20==0) {
 trace << t << ',' << s.segment_id << ',' << s.validation << ',' << static_cast<int>(s.waveform);
 for(double v: s.position) trace << ',' << v;
 for(double v: s.velocity) trace << ',' << v;
 for(double v: s.acceleration) trace << ',' << v;
 trace << '\n';
 }
}
for (std::size_t side: {0,1}) for(double magnitude: {.8,1.38264,1.41499}) {
 const double delta=(side==0 ? -magnitude : magnitude);
 const PairMultibandPlan other(c,{1,1+delta},lim.spring_min[side]+.005,lim.spring_max[side]-.005);
 for(std::size_t i=0;i<=static_cast<std::size_t>(std::ceil(other.duration()/.001));++i) {
   const auto s=other.at(std::min(i*.001,other.duration()));
   if(check_probe_reference(lim,side,s)!=PairFault::kNone) return 3;
   for(int j: {0,1}) { speed=std::max(speed,std::abs(s.velocity[j])); accel=std::max(accel,std::abs(s.acceleration[j])); }
 }
}
std::cout << std::setprecision(12) << "{\"duration_s\":" << p.duration() << ",\"segments\":" << p.segments().size()
          << ",\"max_speed_rad_s\":" << speed << ",\"max_acceleration_rad_s2\":" << accel << "}\n";
}
