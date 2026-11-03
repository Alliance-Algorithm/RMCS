#include <cassert>
#include <cmath>
#include <limits>

#include "controller/wheelleg/traditional_control_core.hpp"
#include "controller/wheelleg/referee_power_gate.hpp"

using namespace rmcs_core::wheelleg;

int main() {
    constexpr std::uint64_t now = 1000000000;
    assert(referee_power_budget(150.,true,true,80.,true,now-1000,now)==80.);
    assert(referee_power_budget(150.,true,true,std::numeric_limits<double>::infinity(),true,now,now)==150.);
    assert(referee_power_budget(150.,true,true,80.,true,now-500000001,now)==0.);
    assert(referee_power_budget(150.,true,true,80.,false,now,now)==0.);
    assert(referee_power_budget(150.,true,true,80.,true,now+1,now)==0.);
    assert(referee_power_budget(150.,true,false,80.,true,now,now)==0.);
    assert(referee_power_budget(150.,false,false,0.,false,0,now)==150.);
    PowerModel model;
    model.copper_w_per_nm2.fill(0.4);
    model.idle_w=2.0;
    model.efficiency=.85;
    Effort torque{20.,-20.,15.,-15.,4.,-4.}, velocity{1.,-1.,2.,-2.,30.,-30.};
    const double scale=model.allocate(torque,velocity,100.);
    assert(scale>0. && scale<1.);
    assert(model.predict(torque,velocity)<=100.+1e-9);
    Effort braking{-20.,20.,-15.,15.,-4.,4.};
    assert(model.predict(braking,velocity)>2.); // Copper loss persists during braking.
    SupportObserver support;
    for(int i=0;i<25;++i) support.step(9.81,0.,.001,3.,.5,.01);
    assert(support.state==Support::grounded);
    for(int i=0;i<25;++i) support.step(0.,0.,.001,3.,.5,.01);
    assert(support.state==Support::airborne);
    SlewState slew;
    double previous_rate=0.;
    for(int i=0;i<1000;++i) {
        slew.step(.5,1.5,4.,.001);
        assert(std::abs(slew.rate)<=1.5+1e-12);
        assert(std::abs(slew.rate-previous_rate)<=.004+1e-12);
        previous_rate=slew.rate;
    }
    TraditionalParameters p;
    PostureKnot a,b;
    a.length=.24; b.length=.35;
    a.position={0.,.1,0.,-.1}; b.position={0.,.8,0.,-.8};
    p.postures={a,b}; p.force_curve={{{.02,100.}},{{.06,500.}}};
    TraditionalControlCore core(p);
    TraditionalSensors sensors;
    sensors.enabled=sensors.fresh=true;
    sensors.position={0.,.4,0.,-.4,0.,0.};
    TraditionalCommand command{0.,0.,.3,100.};
    for(int i=0;i<100;++i) {
        sensors.time_ns=sensors.feedback_ns=i*1000000LL;
        const auto out=core.step(sensors,command);
        assert(out.failure==0);
        for(double effort:out.effort) assert(std::isfinite(effort));
    }
    sensors.reset=true;
    auto out=core.step(sensors,command);
    for(double effort:out.effort) assert(effort==0.);
    sensors.reset=false;
    sensors.pitch=std::numeric_limits<double>::quiet_NaN();
    out=core.step(sensors,command);
    assert(out.failure==1);
    for(double effort:out.effort) assert(effort==0.);

    // Capture after a recovery sweep must be invariant to independent whole
    // turns on the two leg pairs, including its velocity estimate and reset.
    for (const auto offsets : {std::array{2.0, -3.0}, std::array{-1.0, 2.0}}) {
        TraditionalControlCore nominal(p), wound(p);
        sensors.pitch = 0.01;
        sensors.position = {0., .4, 0., -.4, 0., 0.};
        for (int tick = 0; tick < 100; ++tick) {
            sensors.time_ns = sensors.feedback_ns = tick * 1000000LL;
            auto shifted = sensors;
            for (std::size_t i = 0; i < 4; ++i)
                shifted.position[i] += 2.0 * std::acos(-1.0) * offsets[i / 2];
            const auto a = nominal.step(sensors, command), b = wound.step(shifted, command);
            assert(a.failure == 0 && b.failure == 0);
            assert(std::abs(a.velocity_estimate - b.velocity_estimate) < 1e-8);
            for (std::size_t i = 0; i < 6; ++i)
                assert(std::abs(a.effort[i] - b.effort[i]) < 1e-8);
        }
        wound.reset();
        sensors.time_ns = sensors.feedback_ns = 0;
        const auto reset_result = wound.step(sensors, command);
        assert(reset_result.failure == 0);
    }
}
