# RMCS V6 flat deployment

`policy.onnx` is the frozen `v6_flat_14020` candidate, SHA256
`4006bf79182e14074f38c3e8f573fe1870fdfeba3fcc0760bb24cc5752161e8a`.
`policy_profile` binds this identity to V6 nominal angles, 160/2.5 leg PD,
0.6 wheel speed PD, 4.5 Nm wheel output limit and 50 Hz policy / 200 Hz fresh-feedback PD.
The executor holds and sends efforts at 1 kHz.

The supplied `README.md`, original JSON sidecars and Python references retain
handoff bytes. `SHA256SUMS` and `bundle_manifest.json` additionally authenticate
the frozen V6 recovery profile and height lookup copied from source commit
`2778206b5905c2760ccc381f2592b08e1cad611a`. Descriptions of the previous RMCS V5
configuration in the handoff are historical. Run `sha256sum -c SHA256SUMS` from this directory.
The actual RMCS integration is documented in
`docs/zh-cn/wheel_leg_v6_model_deployment_20261003.md` at the repository root.

Four legs use `q_model = -q_api + [1.6,2.93,-1.6,-2.93]` in P order;
wheel scales `[1,1]` await hardware verification. Six-axis calibration and
mechanism readiness stay false. BMI088 installation is X forward, Y left,
Z up. The native V6 recovery FSM, load-corrected preparation, conditional
encoder/IMU observer and 200 ms linear torque handover are implemented.
`recovery_enabled` and `recovery_profile_ready` remain false in the hardware
YAML. Frozen geometry alone never grants hardware readiness. Jump stays disabled.

See `docs/zh-cn/wheel_leg_v6_recovery_alignment_20261003.md` for the complete
C++ simulation matrix, native parity scope and remaining failures. Legacy V5
geometry is not accepted by the V6 loader; both files are bound by SHA256.

`legacy_v5/policy.onnx` retains the previous frozen actor only for explicit
`v5_flat_12486` profiles and the V5 recovery regression. It is never selected
by the default V6 configuration.
