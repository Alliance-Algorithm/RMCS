# Wheel-leg integration validation

The official profile is `rmcs_bringup/config/wheel-leg-infantry-rl.yaml`.
Its components are `rmcs_core::controller::chassis::WheelLegController`,
`rmcs_rl::RlBridge` and the existing hardware, command and telemetry components.
The bridge owns YAML serialization, scaling and worker-thread inference. RMCS
owns mechanical calibration, native self-righting and 1 kHz feedback PD.

The recovery reference is `origin/dev/wheel_leg_rl` at
`0fcc9b45cd80e4dba69c03a1fbed2b848ab5092a` (2026-10-20), including low-knee
upright insertion and capture during motion with its own wheel-speed bound.
The actor remains `models/wheel_leg/recovery_candidate.onnx`, SHA256
`a00d13df1d4113fe949c2a6a15b508e5c491fd1a5e6881570bf025a8d3c29c29`.
Recovery JSON calibration now installs from `rmcs_core/config/wheel_leg`.

From a ROS Jazzy environment in `rmcs_ws`, with the reference branch fetched:

```sh
colcon build --merge-install --symlink-install --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_core
colcon test-result --test-result-base build/rmcs_core --verbose
python3 src/rmcs_core/tools/verify_wheel_leg_reference.py --reference origin/dev/wheel_leg_rl
```

The integration test loads the production executor, chassis controller, bridge
and actor with in-memory sensor snapshots. It checks the acyclic update order,
asynchronous action return, reset, rearm, sensor expiration and bounded efforts.
It never constructs the hardware component or any vehicle transport.

The reference verifier compares hardware/chassis parameters, mechanical and
recovery settings, actor/calibration bytes and hardware sources. It compiles
both versions and compares 6,400 recovery snapshots (phases, routes, failures,
references, efforts and effective action history), plus 256 actor samples.
These are software and deterministic replay checks; they do not establish
physical recovery, CAN timing, control quality or real-time acceptance.

Validation recorded on 2026-10-20: the three-package build passed, both production
component integration tests passed (4 entries including CTest wrappers), and
all 6,400 recovery snapshots plus 256 actor samples matched with maximum
absolute difference 0. Deformable retained the 8x32/four-action contract,
200 Hz inference, delayed action history, flat hold, reset and invalid-input
handling. No hardware component was instantiated by these checks.
