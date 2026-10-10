# Foldable-sentry motion state

`OmniWheelStatus` produces the internal executor interface `/chassis/motion_state`,
of type `rmcs_msgs::ChassisMotionState`. It observes motion continuously, including
when motor control is disabled. It does not write motor commands or change the
existing driving feedback loops. Only `foldable-sentry.yaml` enables this plugin.

## Reading the state

```cpp
#include <rmcs_msgs/chassis_motion_state.hpp>

// In an executor Component:
InputInterface<rmcs_msgs::ChassisMotionState> motion_state_;
// In its constructor:
register_input("/chassis/motion_state", motion_state_);
// In update():
if (motion_state_->usable()) {
    const auto& velocity = motion_state_->velocity;
    // velocity[0]: forward m/s; velocity[1]: left m/s;
    // velocity[2]: counterclockwise yaw rate rad/s.
}
```

The reference is the chassis centre in `base_link`. The mixed-unit velocity vector
is not a spatial vector: transform its linear part and angular part separately.
For motion at another point, account for the rotational velocity `omega cross r`
and any moving gimbal joints. `timestamp` uses the host's `steady_clock`, not ROS
time or the IMU board clock; `covariance` describes `[vx, vy, yaw_rate]` uncertainty.

| Quality | Meaning | `usable()` |
| --- | --- | --- |
| `FUSED` | All four wheel samples and chassis gyro are fresh | true |
| `WHEEL_ONLY` | All four wheels are fresh; gyro is unavailable/stale | true |
| `PREDICTED` | Brief wheel feedback gap; estimate has no complete fresh wheel observation | false |
| `INVALID` | Not initialized, feedback expired, or invalid timing | false |

Predicted states have `UNKNOWN` motion kind. Invalid states have NaN velocity and
covariance. Good states are classified as `STATIONARY`, `TRANSLATING`, `ROTATING`,
or `COMBINED` using hysteresis, not remote-control intent.

## ROS velocity topics

`foldable-sentry.yaml` enables `ValueBroadcaster` to publish the EKF estimate and
the existing wheel solver's raw velocities as `std_msgs/msg/Float64`. Publication
follows the executor's configured 1000 Hz update rate, including when control is
disabled.

| Quantity | Filtered topic | Raw topic | Unit |
| --- | --- | --- | --- |
| Forward velocity | `/chassis/motion/vx` | `/chassis/motion/raw/vx` | m/s |
| Left velocity | `/chassis/motion/vy` | `/chassis/motion/raw/vy` | m/s |
| Counterclockwise yaw rate | `/chassis/motion/wz` | `/chassis/motion/raw/wz` | rad/s |
| Translational speed | `/chassis/motion/speed` | `/chassis/motion/raw/speed` | m/s |

Each `speed` is `hypot(vx, vy)` and excludes angular velocity. Invalid EKF states
publish NaN; predicted states retain their estimate. These scalar topics do not
include the internal state's quality or covariance. The raw solver adds no
freshness checks, so it can retain old motor feedback and does not share the
EKF's quality semantics.
The raw outputs and driving controller retain the original four-wheel calculation.

```bash
ros2 topic echo /chassis/motion/vx
ros2 topic echo /chassis/motion/vy
ros2 topic echo /chassis/motion/wz
ros2 topic echo /chassis/motion/speed
ros2 topic echo /chassis/motion/raw/speed
```

## Estimation and tuning

The velocity EKF predicts with the previous cycle's final chassis target, limited
acceleration, and body-frame rotation. Intent is never treated as a velocity
measurement. A missing/disabled command does not imply zero measured motion.

With fresh chassis gyro feedback, the translation observation first subtracts
each wheel's rotational contribution. Let `k = sqrt(2) * wheel_radius` and
`L = chassis_radius_x + chassis_radius_y`; the shared rotational wheel speed is
`wrot = -L * gyro / k`. For each wheel, form `t = wheel_velocity - wrot`, then use
these opposing wheel pairs:

```text
A = signed_minabs(t_left_front, -t_right_back)
B = signed_minabs(t_left_back, -t_right_front)
vx = k / 2 * (A + B)
vy = k / 2 * (B - A)
```

`signed_minabs` selects the candidate with the smaller absolute value and keeps
that candidate's sign. The reconstructed `[vx, vy, gyro]` is fused in one joint
3D update. Its full measurement covariance retains the correlations introduced
by using the same gyro sample to remove rotation and observe yaw rate:
`R = T * diag(selected_wheel_variances, gyro_variance) * T.transpose()`, where
`T` is the reconstruction's local Jacobian for the selected wheel candidates.

A joint update requires all four wheel sequences and the gyro sequence to be new
relative to their shared last-consumed sequences. A new gyro can update yaw rate
alone while waiting for a complete batch; that same sample is not reused in a
later joint update. Initialization and the wheel-only fallback share these
consumed-sequence records. When the IMU is missing or stale, the observer uses the
original four-wheel observations and initialization with `WHEEL_ONLY` quality;
this branch does not subtract an IMU rotational contribution.

Sample reception time supplies freshness and still increases measurement
uncertainty with sample age. The observer does not perform delayed-measurement
rollback.

A partner component captures the command after this observer and the chassis
controller have updated. The observer itself therefore does not depend on the
controller; other controllers can consume its state without creating a feedback
dependency cycle in the executor.

Defaults are in `foldable-sentry.yaml`: 50 ms feedback timeout, 100 ms maximum
prediction age, 0.05/0.03 m/s translation and 0.10/0.06 rad/s rotation enter/exit
thresholds. Noise settings are standard deviations (process noise per square-root
second); response times and acceleration limits are initial estimates to be
calibrated from driving logs. `log_state` enables one diagnostic log per second.

Wheel velocities already include the hardware's sign and reduction ratio. Geometry
matches the existing controller (wheel radius 0.07 m, x/y radii 0.3 m). Pairwise
selection suppresses a single wheel's excessive speed relative to its partner;
it cannot guarantee ground-relative translation when both wheels slip together.
Position integration and changes to the auto-aim or navigation consumers are
outside this change.
