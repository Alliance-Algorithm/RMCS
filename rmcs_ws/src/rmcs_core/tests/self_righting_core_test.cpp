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
    settings.integral_gain=50.;
    // The same relative program must mirror with the lower/upper side while
    // wheel targets keep physical motor coordinates. Reset must replay it.
    for (double gy:{-1.,1.}) {
        Core programmed(recovery_v202::geometry,settings);
        Core::FlipWaypoint waypoint{{109.,43.},{-170.,-90.},2.6,30.,.2,{12.,-12.}};
        programmed.set_flip_program({waypoint},true);
        Core::Feedback side;
        side.gravity={0.,gy,0.};
        for (std::size_t i=0;i<4;++i)side.position[i]=recovery_v202::geometry[17][i+1];
        for (int replay=0;replay<2;++replay) {
            auto last=side.position;
            for (int tick=0;tick<100;++tick) {
                const auto motion=programmed.step(side,.001,true);
                require(motion.phase==Core::Flip && motion.fault==0,"Program did not run in Flip");
                require(motion.wheel_torque[0]==4.5 && motion.wheel_torque[1]==-4.5,"Wheel program coordinates changed");
                for (std::size_t i=0;i<4;++i) {
                    require(std::abs(motion.position[i]-last[i])<=.00260001,"Program reference jumped");
                    require(std::abs(motion.velocity[i])<=2.60000001,"Program reference exceeded speed");
                    last[i]=motion.position[i];
                }
            }
            rejected=false;
            try { programmed.set_flip_program({},true); }catch(const std::invalid_argument&){rejected=true;}
            require(rejected,"Running program was reconfigured");
            programmed.reset();
        }
        waypoint.speed=3.14;
        rejected=false;
        try { programmed.set_flip_program({waypoint},true); }catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Waypoint accepted an unsafe reference speed");
        waypoint.speed=2.6; waypoint.hold_seconds=5.; waypoint.wheel_velocity.fill(0.);
        programmed.set_flip_program({waypoint},true);
        for (int tick=0;tick<4000;++tick) {
            const auto motion=programmed.step(side,.001,true);
            require(motion.fault==0,"Mirrored program faulted under ideal tracking");
            for(std::size_t axis=0;axis<4;++axis) {
                side.velocity[axis]=(motion.position[axis]-side.position[axis])/.001;
                side.position[axis]=motion.position[axis];
            }
        }
        const std::size_t lower=gy>0. ? 0 : 1;
        for(std::size_t leg=0;leg<2;++leg) {
            const auto& expected=leg==lower ? recovery_v202::geometry.back() : recovery_v202::geometry[1];
            require(std::abs((side.position[leg*2+1]-side.position[leg*2])
                -(expected[leg*2+2]-expected[leg*2+1]))<1e-8,"Lower/upper knee targets were not mirrored");
        }
    }
    for (double wheel_limit:{20.,30.}) {
        Core capture(recovery_v202::geometry,settings);
        capture.configure_capture(wheel_limit,0.);
        Core::Feedback moving;
        moving.position={-.4050553768,.0579936793,.4050553768,-.0580227003};
        moving.gravity={std::sin(.02),0.,-std::cos(.02)};
        moving.wheel_velocity={25.,-25.};
        const auto command=capture.step(moving,.001,true);
        require(command.fault==0 && command.phase==Core::Capture,"Moving standing entry failed");
        require(command.wheel_torque[0]*(wheel_limit==20. ? -1. : 1.)>0.,"Wheel speed envelope was not applied");
        capture.reset();
        rejected=false;
        try {capture.configure_capture(60.,0.);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Invalid wheel speed envelope accepted");
    }
    {
        Core prepared(recovery_v202::geometry,settings);
        prepared.set_flip_program({{{109.,109.},{180.,180.},1.,3.,.1,{0.,0.}}},false);
        prepared.set_flip_fold_first(true);
        Core::Feedback side;
        side.gravity={0.,1.,0.};
        Core::Axes fold{};
        for (std::size_t leg=0;leg<2;++leg) {
            const double offset=std::numbers::pi/2.;
            for (std::size_t joint=0;joint<2;++joint) {
                const std::size_t axis=leg*2+joint;
                side.position[axis]=recovery_v202::geometry[10][axis+1]+offset;
                fold[axis]=recovery_v202::geometry[17][axis+1]+offset;
            }
        }
        const auto initial=side.position;
        Core::Axes replay_first{};
        for (int replay=0;replay<2;++replay) {
            for (int tick=0;tick<100;++tick) {
                const auto motion=prepared.step(side,.001,true);
                require(motion.phase==Core::Flip && motion.fault==0,"Fold preparation failed");
                const double fraction=(motion.position[0]-initial[0])/(fold[0]-initial[0]);
                require(fraction>0. && fraction<1.,"Fold preparation reached the swing prematurely");
                for (std::size_t axis=0;axis<4;++axis)
                    require(std::abs(motion.position[axis]-initial[axis]-fraction*(fold[axis]-initial[axis]))<1e-9,
                        "Preparation changed the independent leg orbits");
                if (tick==0) {
                    if (replay==0) replay_first=motion.position;
                    else require(motion.position==replay_first,"Reset retained preparation state");
                }
                side.position=motion.position;
                side.velocity=motion.velocity;
            }
            prepared.reset();side.position=initial;side.velocity.fill(0.);
        }
    }
    {
        Core guarded(recovery_v202::geometry,settings);
        guarded.configure_flip_handoff(15.,.8,.05);
        Core::Feedback side;
        side.gravity={0.,1.,0.};
        for (std::size_t axis=0;axis<4;++axis)side.position[axis]=recovery_v202::geometry[10][axis+1];
        require(guarded.step(side,.001,true).phase==Core::Flip,"Guarded flip did not start");
        side.gravity={std::sin(.1),0.,-std::cos(.1)};
        side.gyro[0]=1.;
        for (int tick=0;tick<100;++tick)
            require(guarded.step(side,.001,true).phase==Core::Flip,"High gyro passed handoff guard");
        side.gyro.fill(0.);
        for (int tick=0;tick<49;++tick)
            require(guarded.step(side,.001,true).phase==Core::Flip,"Handoff skipped continuous dwell");
        side.gravity={0.,1.,0.};guarded.step(side,.001,true);
        side.gravity={std::sin(.1),0.,-std::cos(.1)};
        for (int tick=0;tick<49;++tick)
            require(guarded.step(side,.001,true).phase==Core::Flip,"Interrupted dwell was accumulated");
        guarded.step(side,.001,true);
        require(guarded.step(side,.001,true).phase==Core::Fold,"Quiet handoff did not enter Fold");
        guarded.reset();
        rejected=false;
        try {guarded.configure_flip_handoff(0.,.8,.05);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Invalid handoff threshold accepted");
    }
    for (bool balance:{false,true}) {
        Core guarded(recovery_v202::geometry,settings);
        guarded.configure_flip_completion(60.,balance);
        Core::Feedback side;
        side.gravity={0.,1.,0.};
        const auto set_orbits=[&](double angle) {
            for (std::size_t axis=0;axis<4;++axis) {
                const double offset=(axis<2 ? -1. : 1.)*angle*std::numbers::pi/180.;
                side.position[axis]=recovery_v202::geometry[10][axis+1]+offset;
            }
        };
        set_orbits(100.+720.);
        require(guarded.step(side,.001,true).phase==Core::Flip,"Orbit guard did not start flip");
        side.gravity={std::sin(.1),0.,-std::cos(.1)};
        const auto motion=guarded.step(side,.001,true);
        require(motion.phase==Core::Flip,"Body upright bypassed leg-orbit guard");
        require((std::abs(motion.wheel_torque[0])>1.)==balance,"Flip wheel balance option not respected");
        bool rejected_while_active=false;
        try {guarded.configure_flip_completion(0.,false);}catch(const std::invalid_argument&){rejected_while_active=true;}
        require(rejected_while_active,"Active flip completion reconfiguration accepted");
        set_orbits(-35.-720.);
        require(guarded.step(side,.001,true).phase==Core::Fold,"Unwrapped safe orbit did not permit handoff");
        guarded.reset();
        bool rejected_limit=false;
        try {guarded.configure_flip_completion(181.,false);}catch(const std::invalid_argument&){rejected_limit=true;}
        require(rejected_limit,"Invalid orbit handoff limit accepted");
        set_orbits(100.+720.);side.gravity={0.,1.,0.};
        guarded.step(side,.001,true);side.gravity={0.,0.,-1.};
        require(guarded.step(side,.001,true).phase==Core::Flip,"Reset discarded orbit guard configuration");
    }
    {
        Core blocked(recovery_v202::geometry,settings);
        blocked.configure_flip_stall(.05);
        blocked.set_flip_program({{{109.,109.},{0.,0.},1.2,3.,0.,{0.,0.}},
            {{45.,45.},{90.,90.},1.2,3.,0.,{0.,0.}}},false);
        Core::Feedback fixed;
        fixed.gravity={0.,1.,0.};
        for(std::size_t axis=0;axis<4;++axis)fixed.position[axis]=recovery_v202::geometry[17][axis+1];
        bool stopped_target=false;Core::Axes blocked_target{};
        for(int tick=0;tick<5000;++tick) {
            const auto command=blocked.step(fixed,.001,true);
            require(command.fault==0,"Stalled flip faulted before its dwell");
            bool no_reference_motion=true;
            for(double v:command.velocity)no_reference_motion=no_reference_motion && std::abs(v)<1e-8;
            if(no_reference_motion) {stopped_target=true;blocked_target=command.position;break;}
        }
        require(stopped_target,"Blocked reference did not reach its finite target");
        for(int tick=0;tick<24;++tick)blocked.step(fixed,.001,true);
        require(blocked.stalled_waypoints()==0,"Blocked waypoint skipped its continuous dwell");
        double target_error=0.;
        for(std::size_t axis=0;axis<4;++axis)target_error=std::max(target_error,std::abs(blocked_target[axis]-fixed.position[axis]));
        for(std::size_t axis=0;axis<4;++axis)
            fixed.position[axis]+=.02*(blocked_target[axis]-fixed.position[axis])/target_error;
        for(int tick=0;tick<24;++tick)blocked.step(fixed,.001,true);
        require(blocked.stalled_waypoints()==0,"Progressing encoders did not restart stall dwell");
        fixed.velocity[0]=.7;blocked.step(fixed,.001,true);fixed.velocity.fill(0.);
        for(int tick=0;tick<48;++tick)blocked.step(fixed,.001,true);
        require(blocked.stalled_waypoints()==0,"Moving encoder did not restart stall dwell");
        fixed.gyro[2]=.6;blocked.step(fixed,.001,true);fixed.gyro.fill(0.);
        for(int tick=0;tick<48;++tick)blocked.step(fixed,.001,true);
        require(blocked.stalled_waypoints()==0,"Yawing body did not restart stall dwell");
        Core::Output rebase;
        for(int tick=0;tick<3 && blocked.stalled_waypoints()==0;++tick)
            rebase=blocked.step(fixed,.001,true);
        require(blocked.stalled_waypoints()==1,"Stopped blocked waypoint did not advance");
        require(rebase.position==fixed.position,"Stall did not rebase command state to measured encoders");
        for(std::size_t axis=0;axis<4;++axis) {
            require(rebase.velocity[axis]==0.,"Stall retained reference velocity");
            require(std::abs(rebase.torque[axis])<1e-12,"Stall retained accumulated tracking effort");
        }
        rejected=false;
        try {blocked.configure_flip_stall(0.);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Active stall policy reconfiguration accepted");
        fixed.velocity[0]=3.15;
        require(blocked.step(fixed,.001,true).fault==3,"Stall policy masked actual overspeed");
        blocked.reset();
        require(blocked.stalled_waypoints()==0,"Reset retained stalled waypoint count");
        for(double bad:{-.1,.01,2.1,std::numeric_limits<double>::quiet_NaN()}) {
            rejected=false;
            try {blocked.configure_flip_stall(bad);}catch(const std::invalid_argument&){rejected=true;}
            require(rejected,"Invalid stall dwell accepted");
        }
        blocked.configure_flip_stall(0.);fixed.velocity.fill(0.);
        Core::Output unchanged;
        for(int tick=0;tick<12000;++tick) {
            unchanged=blocked.step(fixed,.001,true);
            if(unchanged.fault)break;
        }
        require(unchanged.fault==5 && blocked.stalled_waypoints()==0,"Disabled stall policy changed legacy tracking fault");
    }
    for(bool relative:{false,true})for(double gy:{-1.,1.})
        for(const auto& angles:std::array<std::array<double,2>,4>{{{90.,90.},{-90.,-90.},{90.,-90.},{-90.,90.}}}) {
        Core held(recovery_v202::geometry,settings);
        held.configure_flip_first_orbit_hold(2);
        held.set_flip_program({{{109.,109.},{-145.,-90.},1.5,3.,.1,{0.,0.}},
            {{109.,109.},{0.,0.},1.5,3.,.1,{0.,0.}}},relative);
        Core::Feedback f;f.gravity={0.,gy,0.};Core::Axes expected{};
        for(std::size_t index=0;index<2;++index) {
            const std::size_t side=relative ? (index==0 ? (gy>0. ? 0 : 1) : (gy>0. ? 1 : 0)) : index;
            const double target=(index==1 ? angles[side] : (angles[side]>0. ? 215. : -145.));
            for(std::size_t joint=0;joint<2;++joint) {
                const auto axis=side*2+joint;const double sign=side==0 ? -1. : 1.;
                f.position[axis]=recovery_v202::geometry[17][axis+1]+sign*angles[side]*std::numbers::pi/180.;
                expected[axis]=recovery_v202::geometry.back()[axis+1]+sign*target*std::numbers::pi/180.;
            }
        }
        const auto initial=f.position;Core::Axes first{};
        for(int replay=0;replay<2;++replay) {
            bool reached=false;
            for(int tick=0;tick<6000;++tick) {
                const auto out=held.step(f,.001,true);
                require(out.fault==0 && out.phase==Core::Flip,"Initial independent-orbit hold faulted");
                if(tick==0) {
                    if(replay==0)first=out.position;
                    else require(out.position==first,"Reset discarded first-orbit hold configuration");
                }
                double error=0.;
                for(std::size_t axis=0;axis<4;++axis)error=std::max(error,std::abs(out.position[axis]-expected[axis]));
                f.position=out.position;f.velocity=out.velocity;
                if(error<1e-9) {reached=true;break;}
            }
            require(reached,"First program waypoint did not preserve the held leg's independent initial orbit");
            if(replay==0) {
                rejected=false;
                try {held.configure_flip_first_orbit_hold(0);}catch(const std::invalid_argument&){rejected=true;}
                require(rejected,"Active first-orbit hold reconfiguration accepted");
            }
            held.reset();f.position=initial;f.velocity.fill(0.);
        }
        rejected=false;
        try {held.configure_flip_first_orbit_hold(4);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Invalid first-orbit hold mask accepted");
    }
    std::cout<<"PASS: independent leg reference paths, speed, torque, faults, reset, invalid configuration, stalled waypoint policy, initial-orbit hold\n";
}
