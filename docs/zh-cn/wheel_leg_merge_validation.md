# Wheel-leg integration validation

The official profile is `rmcs_bringup/config/wheel-leg-infantry-rl.yaml`.
Its components are `rmcs_core::controller::chassis::WheelLegController`,
`rmcs_rl::RlBridge` and the existing hardware, command and telemetry components.
The bridge owns YAML serialization, scaling and worker-thread inference. RMCS
owns mechanical calibration, native self-righting and 1 kHz feedback PD.

The recovery reference is `origin/dev/wheel_leg_rl` at
`0fcc9b45cd80e4dba69c03a1fbed2b848ab5092a` (2026-10-20), including low-knee
upright insertion and capture during motion with its own wheel-speed bound.
The original 50 Hz actor is `models/wheel_leg/recovery_candidate.onnx`, SHA256
`a00d13df1d4113fe949c2a6a15b508e5c491fd1a5e6881570bf025a8d3c29c29`.
The current profile selects the directional 200 Hz preview actor. Its adjacent
`.deployment.json` pins source/deployment hashes, unchanged graph, model spec,
YAML layout, clocks and motion limits. The loader rejects a `policy_hz` mismatch;
the controller requires the bridge frequency to divide feedback PD frequency.
Action timeout is 15 ms in the bridge and controller. PD/recovery stay at 1 kHz.
Recovery JSON calibration now installs from `rmcs_core/config/wheel_leg`.

From the dev container (with `build-rmcs` on `PATH`), with the reference branch fetched:

```sh
build-rmcs --packages-select rmcs_rl rmcs_core rmcs_bringup
colcon test --merge-install --packages-select rmcs_rl rmcs_core
colcon test-result --test-result-base build --verbose
python3 src/rmcs_core/tools/verify_wheel_leg_reference.py --reference origin/dev/wheel_leg_rl --policy-receipt src/rmcs_rl/models/wheel_leg/flat_directional200_1346_preview_20261020.deployment.json
```

The integration test loads the production executor, chassis controller, bridge
and actor with in-memory sensor snapshots. It checks the acyclic update order,
asynchronous action return, reset, rearm, sensor expiration and bounded efforts.
It never constructs the hardware component or any vehicle transport.

The reference verifier compares hardware/chassis parameters, mechanical and
recovery settings, calibration bytes and hardware sources. With `--policy-receipt`
it allows only the declared 200 Hz actor, 15 ms age and 4 rad/s command migration;
the original no-receipt mode still requires the original actor bytes. It compiles
both versions and compares 6,400 recovery snapshots (phases, routes, failures,
references, efforts and effective action history), plus 256 actor samples.
These are software and deterministic replay checks; they do not establish
physical recovery, CAN timing, control quality or real-time acceptance.

Metadata preparation uses the verified training package as evidence:

```sh
python3 src/rmcs_rl/tool/prepare_wheel_leg_model.py \
  --model src/rmcs_rl/models/wheel_leg/flat_directional200_1346_preview_20261020.onnx \
  --config src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml \
  --source-package /home/noir/Documents/workspace/isaac_wheeled_rl_train/models/flat_directional200_1346_preview_20261020 \
  --training-root /home/noir/Documents/workspace/isaac_wheeled_rl_train \
  --receipt src/rmcs_rl/models/wheel_leg/flat_directional200_1346_preview_20261020.deployment.json
```

Add `--stamp` only when preparing a deployment copy from the verified source.
The training package remains a simulation preview with `behavior_qualified=false`
and `hardware_deployment_ready=false`; metadata stamping preserves these facts.

Validation recorded on 2026-10-20: the three-package build passed, both production
component integration tests passed (4 entries including CTest wrappers), and
all 6,400 recovery snapshots plus 256 actor samples matched with maximum
absolute difference 0. Deformable retained the 8x32/four-action contract,
200 Hz inference, delayed action history, flat hold, reset and invalid-input
handling. No hardware component was instantiated by these checks.

Validation on 2026-10-21 for the 200 Hz migration: the three-package build passed;
seven GTest cases passed (ten entries including CTest wrappers), and an additional
real-component run with the original 50 Hz actor passed reset/rearm and sensor
expiry. Six offline negative fixtures rejected rate, history, observation order,
action scaling, winding and missing metadata mismatches. The stamped deployment
matched the original preview graph and 256 batch-one inferences exactly. Recovery
replay again matched all 6,400 snapshots and both model loaders matched 256 samples.
One 200 Hz component run returned 62 actions: inference p50/p95/max was
0.388/0.520/0.763 ms; maximum observed age on action acceptance was 1.020 ms.
These short, loaded-host software samples do not establish a worst-case timing
bound or qualify physical actor takeover. No vehicle was started.
