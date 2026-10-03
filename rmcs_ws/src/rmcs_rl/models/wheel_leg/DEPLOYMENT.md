# RMCS V6 flat deployment

`policy.onnx` is the frozen `v6_flat_14020` candidate, SHA256
`4006bf79182e14074f38c3e8f573fe1870fdfeba3fcc0760bb24cc5752161e8a`.
`policy_profile` binds this identity to V6 nominal angles, 160/2.5 leg PD,
0.6 wheel speed PD, 4.5 Nm wheel output limit and 50 Hz policy / 200 Hz fresh-feedback PD.
The executor holds and sends efforts at 1 kHz.

The supplied `README.md`, JSON sidecars, Python references and `SHA256SUMS`
are the original handoff bytes. Its descriptions of the previous RMCS V5
configuration are historical. Run `sha256sum -c SHA256SUMS` from this directory.
The actual RMCS integration is documented in
`docs/zh-cn/wheel_leg_v6_model_deployment_20261003.md` at the repository root.

Four legs use `q_model = -q_api + [1.6,2.93,-1.6,-2.93]` in P order;
wheel scales `[1,1]` await hardware verification. Six-axis calibration and
mechanism readiness stay false. BMI088 installation is X forward, Y left,
Z up. V6 recovery/jump are disabled; attempting to enable legacy recovery
under this V6 profile fails at startup.

`legacy_v5/policy.onnx` retains the previous frozen actor only for explicit
`v5_flat_12486` profiles and the V5 recovery regression. It is never selected
by the default V6 configuration.
