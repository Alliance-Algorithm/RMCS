#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace rmcs_core::controller::wheelleg {

// All positions, speeds and torques are at the four ACTIVE output axes, in the
// authenticated CAD frame. The hardware adapter must apply its calibration.
// No ROS, simulator, wall clock, state teleport, or policy dependency.
class SelfRightingCore {
public:
    using Axes = std::array<double, 4>;
    using Row = std::array<double, 5>; // knee degrees, four zero-orbit positions
    using SupportModel = Axes (*)(const Axes&, const std::array<double,3>&);
    enum Phase { Disabled = 0, Rise = 1, Plant = 2, Fold = 3, Capture = 4,
                 Complete = 5, Flip = 7, Fault = 255 };
    struct Feedback {
        Axes position{}, velocity{}, spring_compensation{};
        std::array<double, 3> gravity{0., 0., -1.}, gyro{};
        std::array<double, 2> wheel_velocity{};
    };
    struct Output {
        Axes position{}, velocity{}, torque{};
        Axes servo_velocity{}, servo_feedforward{};
        std::array<double, 2> wheel_torque{};
        Phase phase = Disabled;
        int fault = 0;
    };
    struct FlipWaypoint {
        std::array<double,2> knee_deg{}, orbit_deg{};
        double speed=1.5, acceleration=3., hold_seconds=0.;
        std::array<double,2> wheel_velocity{}; // physical left/right output coordinates
    };
    struct Settings {
        double speed_limit = 3.14, cruise_speed = 1.5, acceleration = 3.;
        double velocity_gain = 5., position_gain = 12., torque_limit = 40.;
        double integral_gain = 50., integral_limit = 30.;
        double fold_knee = 75., flip_knee = 105., stand_knee = 90.;
        double stand_orbit = 0., timeout_seconds = 40.;
        double flip_orbit = 0., sweep_orbit = 180.;
        double balance_kp = 35., balance_kd = 5., balance_kv = 2.;
        double flip_asymmetry = 0.;
        double pitch_trim_gain = 0., yaw_rate_gain = .6;
        double support_gain = 1.;
    };

    explicit SelfRightingCore(std::vector<Row> table, Settings settings, SupportModel support=nullptr)
        : table_(std::move(table)), settings_(settings), support_model_(support) {
        if (table_.size() < 2 || !(settings_.speed_limit > 0.)
            || settings_.speed_limit > 3.14 || !(settings_.cruise_speed > 0.)
            || settings_.cruise_speed >= settings_.speed_limit
            || !(settings_.acceleration > 0.))
            throw std::invalid_argument("Invalid recovery table or speed envelope");
        for (std::size_t i = 0; i < table_.size(); ++i) {
            for (double value : table_[i])
                if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite recovery table");
            if (i && table_[i][0] <= table_[i-1][0])
                throw std::invalid_argument("Recovery knee table must increase");
        }
        for (double value : {settings_.speed_limit, settings_.cruise_speed, settings_.acceleration,
                 settings_.velocity_gain, settings_.position_gain, settings_.torque_limit,
                 settings_.integral_gain, settings_.integral_limit, settings_.fold_knee,
                 settings_.flip_knee, settings_.stand_knee, settings_.stand_orbit,
                 settings_.flip_orbit, settings_.sweep_orbit, settings_.flip_asymmetry,
                 settings_.balance_kp, settings_.balance_kd, settings_.balance_kv,
                 settings_.pitch_trim_gain, settings_.yaw_rate_gain, settings_.timeout_seconds, settings_.support_gain})
            if (!std::isfinite(value)) throw std::invalid_argument("Nonfinite recovery setting");
        if (settings_.velocity_gain<=0. || settings_.position_gain<=0.
            || settings_.integral_gain<0. || settings_.integral_limit<0.
            || settings_.torque_limit<=0. || settings_.torque_limit>40. || settings_.timeout_seconds<=0.
            || settings_.support_gain<0. || settings_.support_gain>1.
            || (settings_.flip_asymmetry!=0. && (settings_.flip_asymmetry<table_.front()[0]
                || settings_.flip_asymmetry>table_.back()[0])))
            throw std::invalid_argument("Invalid recovery gains, torque or timeout");
        for (double knee : {settings_.fold_knee,settings_.flip_knee,settings_.stand_knee})
            if (knee<table_.front()[0] || knee>table_.back()[0])
                throw std::invalid_argument("Recovery pose lies outside its geometry table");
    }

    // Optional measured-state side recovery program. When relative is true,
    // pose entries mean lower/upper leg, selected once from initial gravity.
    void set_flip_program(std::vector<FlipWaypoint> program, bool relative) {
        if (phase_!=Disabled || program.size()>64)
            throw std::invalid_argument("Configure a finite flip program while disabled");
        for (const auto& p:program) {
            for (double v:{p.knee_deg[0],p.knee_deg[1],p.orbit_deg[0],p.orbit_deg[1],
                    p.speed,p.acceleration,p.hold_seconds,p.wheel_velocity[0],p.wheel_velocity[1]})
                if (!std::isfinite(v)) throw std::invalid_argument("Nonfinite flip waypoint");
            if (p.speed<=0. || p.speed>=settings_.speed_limit || p.acceleration<=0.
                || p.hold_seconds<0. || p.hold_seconds>5.
                || std::abs(p.wheel_velocity[0])>55. || std::abs(p.wheel_velocity[1])>55.)
                throw std::invalid_argument("Flip waypoint exceeds its speed or time envelope");
            for (double knee:p.knee_deg)
                if (knee<table_.front()[0] || knee>table_.back()[0])
                    throw std::invalid_argument("Flip waypoint exceeds knee geometry");
        }
        flip_program_=std::move(program); program_relative_=relative;
    }
    void configure_capture(double value, double distance_gain) {
        if (phase_!=Disabled || !std::isfinite(value) || value<10. || value>55. || !std::isfinite(distance_gain) || distance_gain<0. || distance_gain>5.)
            throw std::invalid_argument("Configure wheel speed limit between 10 and 55 rad/s while disabled");
        balance_wheel_speed_limit_=value; balance_distance_gain_=distance_gain;
    }
    void configure_flip_handoff(double tilt_deg,double gyro,double dwell) {
        if(phase_!=Disabled || !std::isfinite(tilt_deg) || !std::isfinite(gyro) || !std::isfinite(dwell)
            || tilt_deg<1. || tilt_deg>60. || gyro<=0. || gyro>3. || dwell<0. || dwell>1.)
            throw std::invalid_argument("Invalid flip handoff conditions");
        flip_capture_gravity_z_=-std::cos(tilt_deg*std::numbers::pi/180.);
        flip_capture_gyro_=gyro;flip_capture_dwell_=dwell;
    }
    void set_flip_fold_first(bool value) {
        if (phase_!=Disabled) throw std::invalid_argument("Configure flip preparation while disabled");
        flip_fold_first_=value;
    }
    void configure_flip_completion(double orbit_limit_deg, bool balance_when_upright) {
        if (phase_!=Disabled || !std::isfinite(orbit_limit_deg) || orbit_limit_deg<0. || orbit_limit_deg>180.)
            throw std::invalid_argument("Configure flip handoff orbit limit between zero and 180 degrees while disabled");
        flip_handoff_orbit_limit_=orbit_limit_deg*std::numbers::pi/180.;
        flip_balance_when_upright_=balance_when_upright;
    }

    void reset() { phase_ = Disabled; fault_ = 0; elapsed_ = stable_ = segment_time_ = wheel_distance_ = orbit_trim_ = 0.;
                   segment_done_ = true; flip_step_ = 0; arrival_time_ = 0.; completed_once_=false;
                   reference_.fill(0.); reference_velocity_.fill(0.); integral_.fill(0.);
                   from_.fill(0.); to_.fill(0.); initial_orbit_.fill(0.); support_blend_=0.;
                   distance_=duration_=ramp_=peak_=posture_lost_time_=program_hold_time_=0.;
                   program_active_=preparing_program_=false; program_index_=0; lower_side_=0;
                   tracking_speed_=settings_.cruise_speed; segment_acceleration_=flip_capture_time_=0.; }

    Output step(const Feedback& f, double dt, bool enabled, bool fresh = true) {
        Output out;
        out.position = f.position;
        if (!enabled) { reset(); return out; }
        bool finite = std::isfinite(dt) && dt > 0. && dt <= .01;
        for (const auto& a : {f.position, f.velocity, f.spring_compensation})
            for (double value : a) finite = finite && std::isfinite(value);
        for (double v : f.gravity) finite = finite && std::isfinite(v);
        for (double v : f.gyro) finite = finite && std::isfinite(v);
        for (double v : f.wheel_velocity) finite = finite && std::isfinite(v);
        const double gravity_norm=std::sqrt(f.gravity[0]*f.gravity[0]+f.gravity[1]*f.gravity[1]+f.gravity[2]*f.gravity[2]);
        finite=finite && gravity_norm>.95 && gravity_norm<1.05;
        if (!finite || !fresh) fail(!finite ? 1 : 2);
        for (double v : f.velocity)
            if (std::abs(v) > settings_.speed_limit + 1e-6) fail(3);
        if (phase_ == Fault) { out.position.fill(0.); out.phase = Fault; out.fault = fault_; return out; }
        const bool capture=phase_==Capture || phase_==Complete;
        posture_lost_time_=capture && f.gravity[2]>-.75 ? posture_lost_time_+dt : 0.;
        if (posture_lost_time_>.1) {
            fail(6); out.phase=Fault; out.fault=fault_; return out;
        }
        elapsed_ += dt;
        if (elapsed_ > settings_.timeout_seconds && !completed_once_) fail(4);
        if (phase_ == Fault) { out.phase = Fault; out.fault = fault_; return out; }
        if (phase_ == Disabled) {
            reference_ = f.position;
            for (std::size_t s = 0; s < 2; ++s)
                initial_orbit_[s] = orbit(f.position, s);
            const double stand_angle=settings_.stand_orbit*std::numbers::pi/180.;
            const Axes standing=pose(settings_.stand_knee,
                {nearest(stand_angle,initial_orbit_[0]),nearest(stand_angle,initial_orbit_[1])});
            bool already_standing=f.gravity[2]<-.99;
            for (std::size_t i=0; i<4; ++i)
                already_standing=already_standing && std::abs(standing[i]-f.position[i])<.08;
            if (already_standing) {
                enter(Capture,standing,f); support_blend_=1.;
            } else if (f.gravity[2] < -.75) enter(Fold, pose(settings_.fold_knee, initial_orbit_), f);
            else {
                phase_ = Flip; flip_step_ = 0;
                program_active_=!flip_program_.empty();
                lower_side_=f.gravity[1]>0. ? 0 : 1;
                if (program_active_ && flip_fold_first_) {
                    preparing_program_=true;
                    begin(pose(settings_.fold_knee,initial_orbit_),reference_);
                } else if (program_active_) start_program_waypoint(f);
                else begin(pose(settings_.fold_knee, initial_orbit_), reference_);
            }
        }
        bool handoff_orbits_ready=true;
        if (flip_handoff_orbit_limit_>0.)
            for (std::size_t side=0;side<2;++side)
                handoff_orbits_ready=handoff_orbits_ready &&
                    std::abs(std::remainder(orbit(f.position,side),2.*std::numbers::pi))<=flip_handoff_orbit_limit_;
        const bool flip_quiet=phase_==Flip && handoff_orbits_ready && f.gravity[2]<flip_capture_gravity_z_
            && std::hypot(f.gyro[0],f.gyro[1])<flip_capture_gyro_;
        flip_capture_time_=flip_quiet ? flip_capture_time_+dt : 0.;
        if (flip_quiet && flip_capture_time_>=flip_capture_dwell_) {
            for (std::size_t s = 0; s < 2; ++s) initial_orbit_[s] = orbit(f.position, s);
            program_active_=false;
            enter(Fold, pose(settings_.fold_knee, initial_orbit_), f);
        }
        if (segment_done_ && phase_==Flip && program_active_) {
            if (preparing_program_) {
                preparing_program_=false;start_program_waypoint(f);
            } else {
                program_hold_time_+=dt;
                if (program_hold_time_>=flip_program_[program_index_].hold_seconds) {
                    if (++program_index_<flip_program_.size()) start_program_waypoint(f);
                    else { program_active_=false; tracking_speed_=settings_.cruise_speed; }
                }
            }
        } else if (segment_done_ && phase_ == Flip) {
            ++flip_step_;
            if (flip_step_>8) { fail(8); out.phase=Fault; out.fault=fault_; return out; }
            std::array<double, 2> angles{};
            double target = flip_step_ % 2 ? settings_.flip_orbit : settings_.sweep_orbit;
            // A body resting on a different face may not respond to the same
            // sagittal sweep. Try front/rear support before asymmetric support.
            if (flip_step_==3 || flip_step_==4) target=flip_step_==3 ? 90. : -90.;
            for (std::size_t s = 0; s < 2; ++s)
                angles[s] = nearest(target * std::numbers::pi / 180., orbit(f.position, s));
            Axes goal = pose(settings_.flip_knee, angles);
            if (settings_.flip_asymmetry != 0. && flip_step_>=5) {
                const std::size_t s = f.gravity[1] < 0. ? 0 : 1; // shorten the upper leg
                const Axes folded = pose(settings_.flip_asymmetry, angles);
                goal[s*2] = folded[s*2]; goal[s*2+1] = folded[s*2+1];
            }
            if (settings_.flip_asymmetry != 0. && std::abs(f.gravity[1])>.9 && flip_step_>=3) {
                // Exact side lying has no wheel-ground support. Hold the lower
                // leg while sweeping the upper one, so symmetric inertial
                // reactions cannot cancel. This remains a bounded candidate,
                // not a declaration that side recovery is feasible.
                const std::size_t lower = f.gravity[1]>0. ? 0 : 1;
                goal[lower*2]=reference_[lower*2];
                goal[lower*2+1]=reference_[lower*2+1];
            }
            begin(goal, reference_);
        } else if (segment_done_ && phase_ == Fold) {
            const double a = settings_.stand_orbit * std::numbers::pi / 180.;
            enter(Plant, pose(settings_.fold_knee,
                {nearest(a, orbit(f.position, 0)), nearest(a, orbit(f.position, 1))}), f);
        } else if (segment_done_ && phase_ == Plant) {
            const double a = settings_.stand_orbit * std::numbers::pi / 180.;
            enter(Rise, pose(settings_.stand_knee,
                {nearest(a, orbit(f.position, 0)), nearest(a, orbit(f.position, 1))}), f);
        } else if (segment_done_ && phase_ == Rise) phase_ = Capture;

        advance(dt, f);
        if (phase_==Fault) { out.phase=Fault; out.fault=fault_; return out; }
        out.position = reference_;
        out.velocity = reference_velocity_;
        out.phase = phase_;
        if (phase_==Flip && program_active_)
            for (std::size_t i=0;i<2;++i)
                out.wheel_torque[i]=std::clamp(.5*(flip_program_[program_index_].wheel_velocity[i]-f.wheel_velocity[i]),-4.5,4.5);
        if ((phase_ == Plant || phase_ == Rise || phase_ == Capture || phase_ == Complete
                || (phase_==Flip && flip_balance_when_upright_))
            && f.gravity[2] < -.8) {
            const double pitch = std::atan2(f.gravity[0], -f.gravity[2]);
            const double trim_rate = (phase_==Capture || phase_==Complete) && std::abs(pitch)<.35
                ? std::clamp(settings_.pitch_trim_gain*pitch, -.4, .4) : 0.;
            orbit_trim_ = std::clamp(orbit_trim_+trim_rate*dt, -.4, .4);
            for (std::size_t i = 0; i < 4; ++i) {
                const double sign = i < 2 ? -1. : 1.;
                out.position[i] += sign*orbit_trim_;
                out.velocity[i] = std::clamp(out.velocity[i]+sign*trim_rate,
                    -settings_.cruise_speed, settings_.cruise_speed);
            }
            const double velocity = .03*(f.wheel_velocity[0]-f.wheel_velocity[1]);
            // Airborne wheel rotation during the flip is not chassis odometry.
            if (phase_==Capture || phase_==Complete) wheel_distance_ += velocity*dt;
            else wheel_distance_=0.;
            const double velocity_gain=(phase_==Capture || phase_==Complete)
                ? settings_.balance_kv : std::min(2.,settings_.balance_kv);
            const double common = std::clamp(settings_.balance_kp*pitch
                + settings_.balance_kd*f.gyro[1]
                + velocity_gain*velocity + balance_distance_gain_*wheel_distance_, -4.5, 4.5);
            const double headroom=4.5-std::abs(common);
            const double yaw=std::clamp(settings_.yaw_rate_gain*f.gyro[2],-headroom,headroom);
            out.wheel_torque = {common+yaw, -common+yaw};
            // The identified wheel response reaches about 59 rad/s unloaded.
            // Keep headroom below it without clipping the acceleration needed
            // to catch a falling body after the wheels first touch the ground.
            for (std::size_t i = 0; i < 2; ++i)
                if (std::abs(f.wheel_velocity[i]) > balance_wheel_speed_limit_
                    && out.wheel_torque[i]*f.wheel_velocity[i] > 0.)
                    out.wheel_torque[i] = std::clamp(-.15*f.wheel_velocity[i], -4.5, 4.5);
            bool quiet = f.gravity[2] < -.9903 && std::hypot(f.gyro[0],f.gyro[1]) < .35
                && std::abs(f.wheel_velocity[0]) < 2. && std::abs(f.wheel_velocity[1]) < 2.;
            for (double velocity:f.velocity) quiet=quiet && std::abs(velocity)<.6;
            stable_ = quiet ? stable_+dt : 0.;
            if (phase_==Complete && !quiet) out.phase=phase_=Capture;
            if ((phase_ == Capture || phase_ == Complete) && stable_ >= .5) {
                out.phase = phase_ = Complete;
                completed_once_=true;
            }
        }
        Axes support{};
        if (support_model_ && (phase_==Rise || phase_==Capture || phase_==Complete) && f.gravity[2]<-.75) {
            support_blend_=std::min(1.,support_blend_+dt/.5);
            support=support_model_(f.position,f.gravity);
        } else support_blend_=0.;
        for (std::size_t i = 0; i < 4; ++i) {
            const double desired_velocity = std::clamp(out.velocity[i]
                + settings_.position_gain*(out.position[i]-f.position[i]),
                -tracking_speed_, tracking_speed_);
            const double error = desired_velocity-f.velocity[i];
            const double previous_integral = integral_[i];
            integral_[i] = std::clamp(integral_[i]+settings_.integral_gain*error*dt,
                                     -settings_.integral_limit, settings_.integral_limit);
            const double feedforward=f.spring_compensation[i]+settings_.support_gain*support_blend_*support[i];
            double torque = settings_.velocity_gain*error + integral_[i] + feedforward;
            if (std::abs(torque)>settings_.torque_limit && torque*error>0.) {
                integral_[i] = previous_integral;
                torque = settings_.velocity_gain*error+integral_[i]+feedforward;
            }
            // Brake before the measured-speed guard; never mask an actual overspeed.
            if (std::abs(f.velocity[i]) > settings_.speed_limit-.12
                && torque*f.velocity[i] > 0.) {
                integral_[i]=previous_integral;
                out.servo_velocity[i]=0.; out.servo_feedforward[i]=0.;
                torque=-settings_.velocity_gain*f.velocity[i];
            } else {
                out.servo_velocity[i]=desired_velocity;
                out.servo_feedforward[i]=std::clamp(integral_[i]+feedforward,
                    -settings_.torque_limit,settings_.torque_limit);
            }
            out.torque[i] = std::clamp(torque, -settings_.torque_limit, settings_.torque_limit);
        }
        // Project motor effort away from the conservative 41..109 degree
        // command envelope. Gas and passive friction remain physical loads.
        for (std::size_t side=0; side<2; ++side) {
            const std::size_t plus=side==0 ? 1 : 2, minus=side==0 ? 0 : 3;
            const double delta=f.position[plus]-f.position[minus];
            const double low=table_.front()[plus+1]-table_.front()[minus+1];
            const double high=table_.back()[plus+1]-table_.back()[minus+1];
            const double relative=out.torque[plus]-out.torque[minus];
            const double factor=std::clamp((relative>0. ? high-delta : delta-low)/.03,0.,1.);
            const double removed=(1.-factor)*relative*.5;
            out.torque[plus]-=removed; out.torque[minus]+=removed;
            out.servo_feedforward[plus]-=removed; out.servo_feedforward[minus]+=removed;
        }
        for (double& value:out.servo_feedforward)
            value=std::clamp(value,-settings_.torque_limit,settings_.torque_limit);
        return out;
    }

private:
    static double nearest(double value, double anchor) {
        return value + 2.*std::numbers::pi*std::round((anchor-value)/(2.*std::numbers::pi));
    }
    Row interpolate(double knee) const {
        knee = std::clamp(knee, table_.front()[0], table_.back()[0]);
        auto upper = std::upper_bound(table_.begin(), table_.end(), knee,
            [](double value, const Row& row) { return value < row[0]; });
        if (upper == table_.end()) return table_.back();
        if (upper == table_.begin()) return table_.front();
        const Row& a = *(upper-1); const Row& b = *upper;
        Row result{}; const double t = (knee-a[0])/(b[0]-a[0]);
        for (std::size_t j = 0; j < 5; ++j) result[j] = a[j]+t*(b[j]-a[j]);
        return result;
    }
    double orbit(const Axes& q, std::size_t side) const {
        const double relative = q[side*2+1]-q[side*2];
        double best = 1e99; const Row* match = &table_.front();
        for (const Row& row : table_) {
            double error = std::abs(std::remainder(relative-(row[side*2+2]-row[side*2+1]),
                                                  2.*std::numbers::pi));
            if (error < best) { best = error; match = &row; }
        }
        return ((*match)[side*2+1]-q[side*2])*(side == 0 ? 1. : -1.);
    }
    Axes pose(double knee, const std::array<double, 2>& angles) const {
        const Row row = interpolate(knee); Axes q{};
        for (std::size_t s = 0; s < 2; ++s)
            for (std::size_t j = 0; j < 2; ++j)
                q[s*2+j] = row[s*2+j+1] - (s == 0 ? 1. : -1.)*angles[s];
        return q;
    }
    void begin(const Axes& goal, const Axes& measured, double speed=0., double acceleration=0.) {
        tracking_speed_=speed>0. ? speed : settings_.cruise_speed;
        const double a=acceleration>0. ? acceleration : settings_.acceleration;
        from_ = measured; to_ = goal; distance_ = 0.;
        for (std::size_t i = 0; i < 4; ++i) distance_ = std::max(distance_, std::abs(to_[i]-from_[i]));
        segment_acceleration_=a;
        ramp_ = std::min(tracking_speed_/a,std::sqrt(distance_/a));
        peak_ = a*ramp_;
        duration_ = peak_ > 1e-9 ? distance_/peak_+ramp_ : 0.;
        segment_time_ = 0.; segment_done_ = distance_ < 1e-8;
        arrival_time_=0.;
    }
    void start_program_waypoint(const Feedback& f) {
        const auto& p=flip_program_[program_index_];
        Axes goal{};
        for (std::size_t index=0;index<2;++index) {
            const std::size_t side=program_relative_ ? (index==0 ? lower_side_ : 1-lower_side_) : index;
            const double angle=nearest(p.orbit_deg[index]*std::numbers::pi/180.,orbit(f.position,side));
            const Row row=interpolate(p.knee_deg[index]);
            for (std::size_t j=0;j<2;++j)
                goal[side*2+j]=row[side*2+j+1]-(side==0 ? 1. : -1.)*angle;
        }
        begin(goal,reference_,p.speed,p.acceleration); program_hold_time_=0.;
    }
    void enter(Phase phase, const Axes& goal, const Feedback& f) {
        (void)f;
        phase_ = phase; begin(goal, reference_);
    }
    void advance(double dt, const Feedback& f) {
        reference_velocity_.fill(0.);
        if (segment_done_) return;
        segment_time_ += dt;
        const double t = std::min(segment_time_, duration_);
        double distance = 0., velocity = 0.;
        if (t < ramp_) { velocity = segment_acceleration_*t; distance = .5*velocity*t; }
        else if (t <= duration_-ramp_) {
            velocity = peak_; distance = .5*peak_*ramp_+peak_*(t-ramp_);
        } else {
            const double remaining = duration_-t;
            velocity = segment_acceleration_*remaining;
            distance = distance_-.5*segment_acceleration_*remaining*remaining;
        }
        for (std::size_t i = 0; i < 4; ++i) {
            const double direction = distance_ > 0. ? (to_[i]-from_[i])/distance_ : 0.;
            reference_[i] = from_[i]+direction*distance;
            reference_velocity_[i] = direction*velocity;
        }
        if (t >= duration_) {
            reference_ = to_;
            bool arrived=true;
            for (std::size_t i=0; i<4; ++i)
                arrived=arrived && std::abs(to_[i]-f.position[i])<.04;
            arrival_time_=arrived ? arrival_time_+dt : 0.;
            segment_done_=arrival_time_>=.04;
            if (segment_time_>duration_+5.) fail(5);
        }
    }
    void fail(int reason) { if (phase_!=Fault) { phase_ = Fault; fault_ = reason; } }
    std::vector<Row> table_;
    Settings settings_;
    SupportModel support_model_=nullptr;
    double support_blend_=0.;
    Phase phase_ = Disabled;
    int fault_ = 0, flip_step_ = 0;
    double elapsed_ = 0., stable_ = 0., segment_time_ = 0., wheel_distance_ = 0., orbit_trim_ = 0.;
    double distance_ = 0., duration_ = 0., ramp_ = 0., peak_ = 0.;
    double arrival_time_=0.;
    double posture_lost_time_=0.;
    bool segment_done_ = true, completed_once_=false;
    std::vector<FlipWaypoint> flip_program_;
    bool program_relative_=true, program_active_=false;
    bool flip_fold_first_=false,preparing_program_=false;
    std::size_t program_index_=0,lower_side_=0;
    double program_hold_time_=0.,tracking_speed_=0.,segment_acceleration_=0.;
    double balance_wheel_speed_limit_=20.,balance_distance_gain_=.98;
    double flip_capture_gravity_z_=-.85,flip_capture_gyro_=2.,flip_capture_dwell_=0.,flip_capture_time_=0.;
    double flip_handoff_orbit_limit_=0.;
    bool flip_balance_when_upright_=false;
    Axes reference_{}, reference_velocity_{}, from_{}, to_{}, integral_{};
    std::array<double, 2> initial_orbit_{};
};

} // namespace rmcs_core::controller::wheelleg
