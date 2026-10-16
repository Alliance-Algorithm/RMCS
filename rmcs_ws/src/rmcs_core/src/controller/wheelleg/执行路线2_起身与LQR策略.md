# 执行路线 2：起身与 LQR 策略

> 独立于 `执行路线1_数学解析.md`（那里只留几何/机械参数/质心公式），也不同于
> `执行路线2_控制律设计.md`（站立/运动主链）。本文记录**摔倒后的起身/自扶正**与**平衡用的 LQR**。
> 记号：**[实测] 已跑通**、**[实现] 代码即如此**、**[待定] 未定/未激活**。
>
> 本版对应已落地的实现：`WheelLegLqrController`、`WheelLegRecoveryController`、`WheelLegStandupController`。

## 1. 目标与两条路径

目标是让机器人**摔倒后重新站起并交棒给平衡链**。当前实现了**两条互补路径**：

| 路径 | 组件 | 适用 | 腿执行 | 轮执行 |
|---|---|---|---|---|
| A 分段起身 | `WheelLegRecoveryController` | 完整站立链已在跑（轨迹环 + LQR） | 作为 `task/*` 第二腿任务，经 `alpha_r` 混合 | 主链 LQR 全程 |
| B 自扶正 | `WheelLegStandupController` | 单独跑（仅仿真桥 + 该组件） | 自己驱动关节参考（RL 状态机） | 自己的 pitch 律 |

两条路径都遵循同一原则：**腿用时间/速率参数化轨迹**（不会瞬移），**轮是唯一的绝对 pitch 权限**，
且**只在轮子承载后才信任轮**。

## 2. LQR 平衡 / 偏航

平衡与偏航由 `WheelLegLqrController`（`wheel_leg_lqr_controller.cpp`）实现，通用至多 4 状态状态反馈：

$$u=\operatorname{clamp}\Big(\sum_i g_i\,(x_i-x_i^*),\;u_{min},u_{max}\Big)$$

- **平衡**（`wheelleg_balance_controller`）：$x=[\theta,\dot\theta,v,x]$，$g=[-17.6,-3.6,-3.0,-2.2]$，$u\in\pm4.88$。
- **偏航**（`wheelleg_yaw_controller`）：$x=[\psi-\psi^*,\dot\psi]$，$g=[-10.8,-4.3]$，$u\in\pm8$。
- 可选 `noise_*` 注入输入噪声、可选 `enable` 门（未配置）、`setpoint_*` 可选。

**为什么用 LQR 而不是 ADRC**：ADRC 的 ESO（$w_0=300$）在真机 IMU 噪声下产生极限环；
LQR 无观测器、增益即参数，线性模型在站立邻域足够。真机 classic 配置仍保留 ADRC 作备份（**同 topic，可直接互换**）。

> 线性化来源：URDF-MuJoCo 数值（`mjd_transitionFD`/有限差分）或解析降阶模型（轮式倒立摆）。
> 底部自由模态 $x_A$ 由 LQR 的 $v,x$ 项（$g_z,g_w$）阻尼，替代了旧的速度外环。

## 3. 路径 A：分段起身 `WheelLegRecoveryController`

`wheelleg_recovery_controller.cpp`（配置 `wheelleg_sim_recovery.yaml`）。它是**腿任务的第二个源**，
发布 `/wheelleg/task/{leg_length,shank,recovery}`，由 `WheelLegTrajectoryController` 按 `alpha_r` 混合。

### 3.1 分段（时间/速率参数化）

```
idle --(trigger/start_on_boot)--> PLANT --(轮接地)--> STAND --(到位+承载+小倾角)--> DONE
```

- `T1 PLANT`：从实测位姿向「轮子下探接地」位姿（`plant_leg_length=0.22, plant_shank=0.2`）移动。
- `T2 STAND`：从 PLANT 位姿向站姿（`stand_leg_length=0.334, stand_shank=2.26`）移动。
- `DONE`：保持到位姿，`alpha_r` 由 $0\to1$ 平滑交棒给平衡轨迹（`handoff_seconds=0.5`）。

### 3.2 速率受限（关键）

参考**不是**固定时长插值，而是**限速逼近目标**：

$$
L_0\leftarrow L_0+\operatorname{clamp}(L_0^*-L_0,\ \pm \dot L_0^{max}dt),\qquad
\varphi\leftarrow\varphi+\operatorname{clamp}(\varphi^*-\varphi,\ \pm \dot\varphi^{max}dt)
$$

`l0_rate=0.03 m/s`、`shank_rate=0.5 rad/s`（角度取 $\operatorname{wrap}$ 的短路径）。
若用固定时长，腿会被重力甩开、PD 需要巨大力矩追赶 → 把机器人甩飞（历史故障）。

### 3.3 交棒门与机身反馈

- `DONE` 需满足：位姿到位、轮子承载（`contact_normal > contact_force_eps`）、$|pitch|<handover\_tilt$。
- 机身 pitch 前馈到腿朝向：$\varphi_{cmd}=ref\_shank+k_\theta\,\theta$（`theta_kp`，默认 0）。
- `alpha_r=0` 期间轮无 pitch 权限，故轨迹环把 `reference/pitch` 置为**实测 pitch**（不喂平衡环一个达不到的设定）。

## 4. 路径 B：自扶正 `WheelLegStandupController`

`wheelleg_standup_controller.cpp`（配置 `wheelleg_standup.yaml`）。**照搬 RL 自扶正状态机**
（`rmcs_rl` 的 `JointReferenceRecoveryController`/`RecoveryController`）到降阶矢状模型，
不依赖 `rmcs_rl`、不用 ADRC，自持轮子用 RL 的 pitch 律。

### 4.1 位姿（冻结 V6 recovery profile）

| pose | hip（$\equiv$ raw `L_joint1`） | 内膝角 $\beta$ |
|---|---|---|
| fold | $-0.252107$ | $50.000^\circ$ |
| nominal/upright | $-0.420000$ | $67.855^\circ$ |
| thrust | $-0.749367$ | $95.000^\circ$ |

发布给桥的降阶膝参考是 $-\beta$ 分支（与轨迹环一致）。

### 4.2 相位机

```
idle -> SELECT -> FOLD -> PLANT -> PREPARE -> BLEND -> DONE
(倒扣)          -> FOLD -> ORBIT -> THRUST -> CAPTURE -> PREPARE -> ...
```

- `SELECT`：按倾角/高度选择路线——接近直立 `kStand`；底盘着地（tilt < 70°）`kPlant`；
  其余绕 `kOrbitPositive/Negative`。
- 参考用**速率受限**逼近每个相位目标（`goal_speed`），过渡由测量量（接触、倾角、高度、角度）触发。
- `kLower`（chassis 着地、腿朝上）先把腿转到下方近全伸，让轮早接地；并对轮**刹车**（`lower_brake_kp`），
  使腿摆动把机身绕轮轴翻起而非底座滑走。

### 4.3 轮 pitch 律

$$
\tau=\big(k_p\theta+k_d\dot\theta\big)\,s_{axis}-k_{damp}\,\omega,\qquad u\in\pm wheel\_torque\_limit
$$

`pitch_kp=8, pitch_kd=1.5, wheel_damping=0.2, wheel_torque_limit=4.88`。
仅在 `PLANT/PREPARE/CAPTURE/BLEND/DONE` 且承载且 $|pitch|<balance\_tilt=55^\circ$ 时平衡；
`kLower` 时退化为纯刹车。

### 4.4 双后端

同一组件由 `backend: simulation|hardware` 选择话题：仿真走 `/wheelleg/state|reference|control`，
真机走 `/wheel_leg/<axis>/{angle,velocity,control_angle,control_torque}` + IMU + recovery 观测器。
真机膝角通过共享模型 + `CanonicalCalibration` 由原始关节角反解。

## 5. 与轨迹控制器的 `alpha_r` 混合

`WheelLegTrajectoryController` 用 `/wheelleg/task/recovery` 作为混合权重 $\alpha_r$（1=平衡，0=起身）：

$$
L_0\leftarrow\alpha_r L_0^{bal}+(1-\alpha_r)L_0^{rec},\qquad
\varphi\leftarrow\alpha_r\varphi^{bal}+(1-\alpha_r)\varphi^{rec},\qquad
\theta_{ref}\leftarrow\alpha_r\,0+(1-\alpha_r)\,\theta_{meas}
$$

- 起身时腿**直接跟踪**起身源的时间参数化参考（**不经**五次多项式重规划，否则每 tick 重置导致抖动）。
- 步态加速度姿态与基座速度前馈都乘 $\alpha_r$，避免与起身姿态打架。

## 6. 接口 / 话题

| 生产者 | 话题 | 消费者 |
|---|---|---|
| `WheelLegRecoveryController` | `/wheelleg/task/{leg_length,shank,recovery}` | `WheelLegTrajectoryController` |
| `WheelLegTrajectoryController` | `/wheelleg/reference/{pitch,hip,knee,shank,wheel_velocity}` | 仿真桥 / 驱动桥 |
| `WheelLegStandupController`（仿真） | `/wheelleg/reference/{hip_angle,knee_angle,wheel_velocity}`、`/wheelleg/control/wheel_torque` | 仿真桥 |
| `WheelLegStandupController`（真机） | `/wheel_leg/<axis>/{control_angle,control_torque}` | 硬件 |
| `WheelLegLqrController` | `/wheelleg/control/{wheel_torque,wheel_torque_diff}` | `WheelLegWheelMixer` |

触发：`/wheelleg/recovery/trigger`、`/wheelleg/standup/trigger`，或 `start_on_boot: true`。

## 7. 初始位姿与仿真配置

仿真桥支持 `initial_pose`：`standing`（默认）、`motor_zero`、`legs_up`、`crouched`（RL fold 位姿）。
`wheelleg_sim_recovery.yaml` 用 `crouched` + `hold_seconds=7`（底座钉住、腿锁，先定住起始位形再起身）；
`wheelleg_pose_show.yaml` 用 `fix_base: true` + `lock_leg: true` 仅展示静态位姿。
起始扰动：`init_pitch_deg/init_hip_delta_deg/init_crank_delta_deg/init_lift_extra`（自动抬到轮触地）。

## 8. 待定项

1. 两路径合并/仲裁到统一顶层状态机（recovery 与 standup 目前各自独立）。
2. 真机自扶正验收：`backend: hardware` 的姿态符号、接触/高度观测器尚未在实机验证。
3. LQR 增益随腿长/质量的调度（现为常值）。
4. 自动摔倒检测（当前靠 `trigger` 或 `start_on_boot`）。
