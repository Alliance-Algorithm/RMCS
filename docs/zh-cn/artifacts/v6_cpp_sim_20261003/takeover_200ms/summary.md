# V6 C++ dynamic simulation summary

Source run status: `complete`. Confirmed passes: 4/4.

![Trajectories](trajectory.png)

| Case | Outcome | Observed / expected sim s | Wall s | Wall sample p95 ms | Last inference p95 us |
| --- | --- | ---: | ---: | ---: | ---: |
| stand | PASS | 16.000 / 16.000 | 25.339 | 9.075 | 90.474 |
| stand__native_prepare | PASS | 16.000 / 16.000 | 25.352 | 9.176 | 88.163 |
| stand__pitch_forward | PASS | 16.000 / 16.000 | 25.286 | 9.103 | 85.955 |
| stand__pitch_backward | PASS | 16.000 / 16.000 | 25.138 | 8.801 | 86.076 |

Inference and PD timings are cached last-call durations sampled in trace rows; their row counts are not invocation counts. Wall intervals use bridge values when saved, otherwise explicitly labeled estimates from wall_s differences.

Incomplete, missing, unknown-horizon and invalid cases cannot pass. The original report and case JSON are unchanged. This nominal simulation does not qualify hardware, CAN/USB or BMI088 EKF.
