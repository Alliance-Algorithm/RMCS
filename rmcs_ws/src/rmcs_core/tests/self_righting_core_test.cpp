#include <iostream>
#include <limits>
#include <stdexcept>

#include "controller/wheelleg/recovery_geometry_v202.hpp"

using namespace rmcs_core::controller::wheelleg;

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    using Core=SelfRightingCore;
    Core::Settings settings;
    settings.stand_knee=recovery_v202::stand_knee;
    settings.stand_orbit=recovery_v202::stand_orbit;
    settings.cruise_speed=1.2;
    settings.timeout_seconds=40.;
    // All four independent front/rear combinations, not averaged per side.
    for (int combination=0; combination<8; ++combination) {
        Core core(recovery_v202::geometry,settings);
        Core::Feedback f;
        const auto& row=recovery_v202::geometry[17];
        for (std::size_t side=0; side<2; ++side) {
            const double angle=(combination&(1<<side) ? 1. : -1.)*std::numbers::pi/2.;
            for (std::size_t joint=0; joint<2; ++joint)
                f.position[side*2+joint]=row[side*2+joint+1]-(side==0 ? 1. : -1.)*angle;
        }
        auto previous=f.position;
        bool reached=false;
        for (int tick=0; tick<25000; ++tick) {
            f.spring_compensation=recovery_v202::compensation(f.position);
            const auto out=core.step(f,.001,true);
            require(out.fault==0,"Nominal reference sequence faulted");
            for (std::size_t axis=0; axis<4; ++axis) {
                require(std::isfinite(out.torque[axis]) && std::abs(out.torque[axis])<=40.,"Unbounded torque");
                require(std::abs(out.velocity[axis])<=1.20000001,"Unbounded trajectory speed");
                require(std::abs(out.position[axis]-previous[axis])<=.00120001,"Position reference jumped");
            }
            previous=out.position;
            // Also exercise segment boundaries with a small tracking lag.
            // This validates planner continuity, not native vehicle dynamics.
            for (std::size_t axis=0;axis<4;++axis) {
                const double next=combination<4 ? out.position[axis]
                    : f.position[axis]+.1*(out.position[axis]-f.position[axis]);
                f.velocity[axis]=(next-f.position[axis])/.001;
                f.position[axis]=next;
            }
            if (out.phase==Core::Complete) { reached=true; break; }
        }
        require(reached,"Reference sequence did not complete");
        auto out=core.step(f,.001,false);
        require(out.phase==Core::Disabled && out.fault==0,"Disable failed to reset");
        for (double value:out.torque) require(value==0.,"Disable retained torque");
    }
    Core core(recovery_v202::geometry,settings);
    Core::Feedback f;
    f.position={-.4050553768,.0579936793,.4050553768,-.0580227003};
    f.velocity[2]=3.14001;
    auto out=core.step(f,.001,true);
    require(out.phase==Core::Fault && out.fault==3,"Actual overspeed was accepted");
    for (double value:out.torque) require(value==0.,"Overspeed retained torque");
    f.velocity.fill(0.);
    require(core.step(f,.001,true).fault==3,"Fault did not latch");
    require(core.step(f,.001,true,false).fault==3,"Later stale input replaced the first fault");
    core.step(f,.001,false);
    require(core.step(f,.001,true,false).fault==2,"Stale feedback was accepted");
    core.step(f,.001,false);
    f.gravity[0]=std::numeric_limits<double>::quiet_NaN();
    require(core.step(f,.001,true).fault==1,"Nonfinite IMU was accepted");
    core.step(f,.001,false);
    f.gravity={0.,0.,0.};
    require(core.step(f,.001,true).fault==1,"Invalid gravity magnitude was accepted");
    bool rejected=false;
    settings.integral_gain=std::numeric_limits<double>::infinity();
    try { Core invalid(recovery_v202::geometry,settings); }
    catch (const std::invalid_argument&) { rejected=true; }
    require(rejected,"Nonfinite gains were accepted");
    std::cout<<"PASS: independent leg reference paths, speed, torque, faults, reset, invalid configuration\n";
}
