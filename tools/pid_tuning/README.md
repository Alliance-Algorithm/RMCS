# Friction Wheel PID Tuning

This directory contains friction wheel PID tuning utilities and configuration snippets.

## Recorder

The recorder plugin class is:

```text
rmcs_core::controller::pid::FrictionWheelPidRecorder
```

It records only when:

- `control_velocity` is finite
- `velocity` is finite
- `control_torque` is finite
- `abs(control_velocity) >= min_setpoint_abs`

Raw CSV columns:

```text
timestamp_us,wheel_id,run_id,sample_idx,setpoint_velocity,measured_velocity,control_torque,enabled
```

Default output directory:

```text
/tmp/friction_pid_logs
```

See `friction_wheel_pid_recorder.example.yaml` for a minimal configuration snippet.

## Identification

`identify_plant.py` estimates a discrete first-order-plus-dead-time model from the recorder CSV:

```text
y[k+1] = a * y[k] + b * u[k-d] + c
```

Where:

- `u` is `control_torque`
- `y` is `measured_velocity`
- `d` is the pure delay in samples

The script scans candidate delays, fits each candidate with least squares, and writes:

- `identification_summary.csv`
- one `*_model.json` per `wheel_id/run_id`
- one `*_fit.csv` per `wheel_id/run_id`

Example:

```bash
python3 tools/pid_tuning/identify_plant.py /tmp/friction_pid_logs/friction_wheel_pid_xxx.csv \
  --output-dir /tmp/friction_pid_identify
```

Common filters:

```bash
python3 tools/pid_tuning/identify_plant.py /tmp/friction_pid_logs/friction_wheel_pid_xxx.csv \
  --wheel-id first_left_friction \
  --run-id 1
```

Key options:

- `--sample-time-ms`: fallback sample interval when timestamps are unavailable
- `--max-delay-ms`: maximum delay search range
- `--min-samples`: minimum segment length
- `--allow-unphysical`: export the best fit even if `a` or `b` is not physically plausible

## Tuning

`tune_pid.py` tunes only `kp` and `kd`. `ki` is always fixed to `0` so the offline controller matches the current friction wheel constraint.

The simulated controller uses the same discrete update rule as `PidCalculator`:

```text
u[k] = clamp(kp * e[k] + kd * (e[k] - e[k-1]))
```

Where:

- `e[k] = setpoint_velocity[k] - measured_velocity[k]`
- `ki = 0`
- plant model comes from `identify_plant.py`

Supported optimization methods:

- `hybrid` (default): GA first, then GP-like Bayesian optimization refinement
- `ga`: genetic algorithm only
- `bo`: GP-like Bayesian optimization only

Trace-driven tuning with the original recorded setpoint profile:

```bash
python3 tools/pid_tuning/tune_pid.py /tmp/friction_pid_identify/first_left_friction_run_1_model.json \
  --trace-csv /tmp/friction_pid_logs/friction_wheel_pid_xxx.csv \
  --output-dir /tmp/friction_pid_tuning
```

Synthetic startup step tuning:

```bash
python3 tools/pid_tuning/tune_pid.py /tmp/friction_pid_identify/first_left_friction_run_1_model.json \
  --step-setpoint 5000 \
  --output-limit 12 \
  --method hybrid
```

Batch tuning from an identification summary:

```bash
python3 tools/pid_tuning/tune_pid.py /tmp/friction_pid_identify/identification_summary.csv \
  --trace-csv /tmp/friction_pid_logs/friction_wheel_pid_xxx.csv
```

Outputs:

- `tuning_summary.csv`
- one `tuning_result.json` per `wheel_id/run_id`
- one `candidates.csv` per `wheel_id/run_id`
- one `best_response.csv` per `wheel_id/run_id`

Common options:

- `--seed-kp/--seed-kd`: baseline gains used for comparison
- `--kp-min/--kp-max`, `--kd-min/--kd-max`: search bounds
- `--output-limit` or `--output-min/--output-max`: controller output clamp
- `--trace-scope wheel`: reuse all runs of the same wheel instead of exact `run_id`
