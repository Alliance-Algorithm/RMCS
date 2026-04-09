# Planner Benchmark

这个 benchmark 对应 `docs/mpc_formulation.md` 和 `docs/standard_mpc_control_chain.md` 里描述的两条公共入口：

- `Planner::plan(Target, bullet_speed)`：只覆盖 MPC 求解主路径。
- `Planner::plan(std::optional<Target>, bullet_speed)`：额外覆盖按 `decision_speed` 选择的软件延迟补偿分支，以及 `nullopt -> control=false` 的快路径。

## 运行

```bash
./build/planner_benchmark configs/standard3.yaml
```

可选参数：

```bash
./build/planner_benchmark configs/standard3.yaml --warmup=100 --steps=800 --repeat=5
```

## 内置场景

- `direct_low_speed`：直接调用 `plan(Target, ...)`，作为 solver 主路径基线。
- `optional_low_speed`：角速度低于 `decision_speed`，覆盖低速延迟分支。
- `optional_high_speed`：角速度高于 `decision_speed`，覆盖高速延迟分支。
- `optional_aggressive_close_range`：近距离高机动工况，观察约束接近饱和时的耗时和误差。
- `optional_invalid_bullet_speed`：弹速传 `0`，走公共 fallback 路径。
- `optional_intermittent_target_loss`：每 8 帧喂一次 `nullopt`，测无目标快路径和恢复后的开销。

## 输出指标

- `latency_us(all)`：所有调用的耗时统计，包含 `nullopt` 快路径。
- `latency_us(active)`：仅目标存在时的耗时统计。
- `midpoint_error_mrad`：`plan.target_yaw/pitch` 与 `plan.yaw/pitch` 的中点误差，和 docs 里 `HALF_HORIZON` 的输出定义一致。
- `yaw_acc_ratio` / `pitch_acc_ratio`：输出角加速度相对配置上限 `max_yaw_acc` / `max_pitch_acc` 的占比。
- `contract`：只检查公开行为约束：
  - 目标存在时 `control` 不应掉为 `false`
  - `nullopt` 时 `control` 不应变成 `true`
  - 输出角加速度不应超过配置上限
