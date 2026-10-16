# 真机 Bench 测试组装说明（wheel-leg-infantry-bench）

本组装用于轮腿真机的上线前逐项验证，**不使用平衡栈**，用一个自包含组件
`WheelLegBenchTestController` 顶替整条经典链路。

- 配置：`wheel-leg-infantry-bench.yaml`
- 组件：`rmcs_core/src/controller/wheelleg/wheel_leg_bench_test_controller.cpp`
- 启动：

  ```bash
  # 先编译
  colcon build --packages-select rmcs_core rmcs_bringup --merge-install
  # 再启动（robot 参数 -> config/<name>.yaml）
  ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-bench
  # 真机上：
  .script/attach-remote -r        # 重启服务并 attach
  ```

- 切换模式：改 `wheelleg_bench.mode`（**启动前**改）。

```yaml
wheelleg_bench:
  ros__parameters:
    mode: 1   # 1=轮向  2=失能解算  3=闭位姿  4=姿态噪声
```

---

## 组装组成

| 组件 | 实例名 | 作用 |
|---|---|---|
| `rmcs_core::hardware::WheelLeg` | `wheel_leg` | 真机硬件：CAN、DM/M3508、IMU、DR16；持有使能门控 |
| `WheelLegStateEstimator` | `wheelleg_state` | 生产解算器：编码器 -> 腿形/姿态（mode 2/4 的对照与数据源） |
| `WheelLegBenchTestController` | `wheelleg_bench` | 本组件：四模式逻辑、轮速环、腿 PD、打印 |
| `ValueBroadcaster` | `value_broadcaster` | 把 executor 接口转发成 ROS 话题以便录制/看曲线 |

---

## 使能与安全（所有模式）

硬件使能门（`wheel_leg_control.hpp`）：`dr16_fresh` **且** 两拨杆组合满足
（正常臂 = 左 MIDDLE + 右 MIDDLE；`allow_spin_switch: true` 时 左 MIDDLE + 右 DOWN
也算），**且** 控制器 `enable_request = true`。

- 本组装的 `/wheel_leg/enable_request` 与 `/wheel_leg/identification/wheel_only`
  **只有 `wheelleg_bench` 一个生产者**。
- **模式 1 / 3** 会请求使能；**模式 2 / 4 永不请求**（`enable_request=false`、
  `wheel_only=false`、所有力矩置 0），所以即使拨杆乱动也不会动。
- **双 DOWN 一定失能**：`middle`/`spin` 皆为假 -> `enable=false`，且下降沿会主动排
  四条 `0xFD` 失能帧。
- 风扇/云台等其它组件不在本组装内。

---

## 模式 1：轮向测试

**目的**：确定"正力矩命令 <-> 轮速符号 <-> 车前进方向"的关系。

**行为**：
- 腿全程失能（`wheel_only=true`）；两轮按右摇杆 X 同速转。

  ```
  目标轮速 = (vx_max / wheel_radius) * clamp(摇杆X, 死区)        # 物理"前进"方向
  轮力矩_i = wheel_command_sign[i] * wheel_velocity_kp
             * (目标轮速 - wheel_command_sign[i] * 实测轮速_i)
  轮力矩   = clamp(wheel_velocity_kp * (目标轮速 - 实测轮速), ±wheel_torque_limit)
  ```

**操作**：
1. `mode: 1`。
2. 两拨杆都拨到 MIDDLE（臂）。
3. 推右摇杆 X（死区 `deadzone`）。

**看**：
- `/wheel_leg/{left,right}_wheel/velocity`
- `/wheel_leg/{left,right}_wheel/control_torque`
- 车实际往哪走

**判读**：
- `wheel_command_sign: [左, 右]` 把每个轮的"物理前进"映射到它的 API 帧（两轮装配
  **镜像**，所以左右符号相反，当前值 `[1.0, -1.0]`）。若发现某轮反了就翻对应那一项。
- 先从小 `vx_max` / 小 `wheel_torque_limit` 起步。

> 注：轮辨识（`wheels-r15p62`）已确认"正命令力矩 -> 正轮速"，两轮一致；本模式补的是
> "轮速为正 = 车前进还是后退"（装配/镜像）这一环。

---

## 模式 2：全失能 + 解算验证

**目的**：验证"两同轴电机编码器角 -> 腿形"的解算（内角 + 髋-轮轴连线角）。

**行为**：
- 全失能，只读编码器并打印。
- 车身**吊起、调平**（解算在机体系，+x 为水平参考）。

**操作**：
1. `mode: 2`。
2. 手把腿固定成若干位姿（建议内角扫 40/60/80/105°，多个摆动角，左右腿都做）。

**打印**（每 `log_every_cycles` 条，默认 200 -> 5 Hz @1 kHz）：

```
[passive raw] L: q_hip=.. q_crank=.. c=.. (..deg) | R: q_hip=.. q_crank=.. c=.. (..deg)
[passive] L: inner_knee=..deg axle=..deg len=.. | R: .. (est knee=.. legL=.. legR=..)
```

- `raw` 行：`q_hip`/`q_crank` 是该侧两个电机的原始角（rad），`c = hardware_crank_sign*(q_crank-q_hip)` 是相对曲柄，用来核对零位/查表原点。

- `inner_knee` = 大小腿**内角** β（对数显角度尺）
- `axle`      = **髋->轮轴连线到水平**的角度 `atan2(Az, Ax)`（+ = 向上；正下方 ≈ -90°）
- `len`       = 髋->轮轴长度 L0
- `est ...`   = 生产解算器 `WheelLegStateEstimator` 的对应值（交叉对照）

**话题**（也可录）：

```
/wheelleg/bench/inner_knee_l, inner_knee_r
/wheelleg/bench/leg_axle_angle_l, leg_axle_angle_r
/wheelleg/bench/leg_length_l, leg_length_r
/wheelleg/bench/leg_angle_l, leg_angle_r          # reduced hip alpha
/wheelleg/state/{knee_angle, leg_angle_l, leg_angle_r, leg_length_l, leg_length_r}
```

**判读**：`inner_knee` 应等于角度尺读数；若本组件与 `est ...` 不一致，说明生产解算器
有问题。

---

## 模式 3：闭固定位姿

**目的**：验证"解算反解 + 关节映射 + 腿 PD"能否把腿驱动到指定位姿。

**行为**：
- 轮力矩 0；对四个 DM 做**纯力矩 PD**（零-Kp MIT）。

  ```
  q_hip   = hardware_hip_sign  * (alpha - hardware_hip_zero)
  crank   = q_hip + hardware_crank_sign * crank_from_knee(beta_inner)
  tau     = clamp(leg_kp*(q_target - q_meas) - leg_kd*qd_meas, ±leg_torque_limit)
  ```

- **相对、小幅、带斜坡**（最安全）：
  - **内角给绝对值** `target_inner_knee_deg`（40..105°）；
  - **整条腿角度用相对量** `delta_leg_angle_deg`（相对实测的**小幅**变化，默认 0 = 连线角不变）；
  - 从**实测位姿**起，用**五次曲线**在 `ramp_seconds` 内平滑到位（**使能后**才启动，arm 无跳变）；
  - 关节目标按多圈编码器**对齐到最近一圈**。
  - 内部由 (β, α_L) 反解 reduced hip：
    ```
    alpha_L(beta,0) = atan2(L2*sin(beta), L1 - L2*cos(beta))
    alpha           = alpha_L(beta,0) - alpha_L_ref
    ```

**操作**：
1. `mode: 3`。
2. 设 `target_inner_knee_deg`（内角目标）与 `delta_leg_angle_deg`（腿角相对变化，先 0）。
3. 两拨杆 MIDDLE 使能；腿应**平滑**驱动到位（打印 `[pose] armed: knee ..->.. deg, axle-line ..`）。

**看**：腿是否平滑到位、内角与 `delta_leg_angle` 是否正确、有无抖动；调 `leg_kp / leg_kd / leg_torque_limit`。
**安全**：先小 `delta_leg_angle_deg`（0）+ 单边小内角变化起步；纯台架无平衡，**必须支撑车身**。

---

## 模式 4：姿态噪声测试

**目的**：测 **pitch / roll / yaw** 三轴的噪声（IMU）。

**行为**：
- 全失能，只读 `/wheelleg/state/{pitch,roll,yaw}`，按 `stats_period_cycles` 窗口
  （默认 1000 -> 1 Hz @1 kHz）算 mean/std 并打印/发布。

**操作**：
1. `mode: 4`；机器人**放稳别动**。

**打印**：

```
[attitude] std (n=1000): pitch=.. roll=.. yaw=.. rad | pitch=.. roll=.. yaw=.. deg
[attitude] mean: pitch=.. roll=.. yaw=.. rad
```

**话题**：

```
/wheelleg/state/{pitch, roll, yaw}
/wheelleg/bench/{pitch_std, roll_std, yaw_std}
```

**离线分析**：把 `value_broadcaster.update_rate` 从 `100.0` 调到 `1000.0`，录
`/wheelleg/state/{pitch,roll,yaw}` 几分钟，再做 FFT / Allan。
> 注意：`yaw` 来自陀螺积分，**无界、含低频漂移**，看纯噪声时把 `stats_period_cycles`
> 调小（如 200）。

---

## 模式 5：摇杆 teleop（点动）

**目的**：用**左摇杆**手动点动腿，安全地看每个关节/整腿的响应。

**行为**：
- 轮力矩 0；**上电起 target 就跟踪实测位姿**（第一个 target = 机器当前值，不是默认常量），
  **使能后锁定并从它积分**两个目标：
  - **左摇杆 X** → 大小腿**内角** β（每步 `teleop_sensitivity × teleop_knee_sign × 摇杆X`）；
  - **左摇杆 Y** → **髋→轮轴连线角** α_L（每步 `teleop_sensitivity × teleop_leg_sign × 摇杆Y`）。
  - `teleop_sensitivity` 是**每次 update(1 kHz) 的变化量**（默认 `0.006` → 满杆 ≈ 6 rad/s）。
  - 内角 clamp 到 [40,105]°，连线角 clamp 到 `teleop_leg_min_deg..teleop_leg_max_deg`。
- `teleop_side`：`0=左腿 / 1=右腿 / 2=双腿`；**未选中的腿保持实测位姿**。
- 目标经 `(β, α_L) → reduced α` 反解，交给腿 PD（多圈对齐）。

**操作**：
1. `mode: 5`；设 `teleop_side`（先 `0` 单腿）、`teleop_sensitivity`（先小，如 0.003）。
2. 两拨杆 MIDDLE 使能；**推左摇杆**点动，看方向对不对。
3. 方向反了就翻 `teleop_knee_sign` / `teleop_leg_sign`。

**看**：腿是否按摇杆方向平缓运动、有无抖动；调 `teleop_sensitivity`、`leg_kp/kd`。
**安全**：先单腿、小灵敏度；纯台架无平衡，**必须支撑车身**。

---

## 频率与参数速查（`wheelleg_bench:` 段）

| 参数 | 默认 | 含义 |
|---|---|---|
| `mode` | 1 | 1/2/3/4 |
| `log_every_cycles` | 200 | 模式 2 打印周期（200 -> 5 Hz @1 kHz） |
| `stats_period_cycles` | 1000 | 模式 4 统计窗口（1000 -> 1 Hz @1 kHz） |
| `wheel_velocity_kp` | 0.3 | 模式 1 轮速 P |
| `wheel_torque_limit` | 4.88 | 模式 1 轮力矩限 |
| `vx_max` | 0.5 | 满杆车体 m/s（/wheel_radius -> 轮 rad/s） |
| `deadzone` | 0.08 | 摇杆死区 |
| `wheel_command_sign` | [1.0, -1.0] | 模式 1 各轮物理前进符号 [左, 右] |
| `leg_kp` / `leg_kd` / `leg_torque_limit` | 160 / 2.5 / 40 | 模式 3 腿 PD |
| `target_inner_knee_deg` | 90.0 | 模式 3 内角**绝对**目标（40..105） |
| `delta_leg_angle_deg` | 0.0 | 模式 3 腿角**相对**变化（deg，0=连线角不变） |
| `ramp_seconds` | 3.0 | 模式 3 五次曲线斜坡时长（s） |
| `teleop_side` | 2 | 模式 5 遥控哪条腿：0=左/1=右/2=双腿 |
| `teleop_sensitivity` | 0.006 | 模式 5 每次 update 的变化量（rad，1 kHz 下 ≈×1000 → rad/s） |
| `teleop_knee_sign` / `teleop_leg_sign` | 1.0 / 1.0 | 模式 5 摇杆 X/Y 方向符号 |
| `teleop_leg_min_deg` / `teleop_leg_max_deg` | -170 / -20 | 模式 5 连线角 clamp |
| `hardware_hip_sign/zero`、`hardware_crank_sign` | 见 yaml | 硬件 -> reduced 映射 |

---

## 硬件备注

- **CAN 路由**（写在 `wheel-leg.cpp`，本组装不改）：

  | 电机 | CAN | 说明 |
  |---|---|---|
  | 轮 M3508 | `kCan0` | CAN ID `0x200` |
  | 髋 DM | `kCan1` | |
  | 膝 / crank DM | `kCan2` | |

- **DR16** 不是独立组件，由 `WheelLeg` 硬件内部经板载 DBus UART 读取；使能门用的是
  设备原始 `dr16_.switch_*()`，与 ROS 的 `/remote/*` 话题无关。
- **日志可见性**：本组件用 `RCLCPP_INFO`，rclcpp/rcutils 默认写 **stderr**；launch 的
  `output="log"` 会把子进程 stderr 转发到屏幕 -> `attach-remote` 能直接看到。
  若看不到，把 `rmcs.launch.py` 的 `output="log"` 改为 `"screen"`。
