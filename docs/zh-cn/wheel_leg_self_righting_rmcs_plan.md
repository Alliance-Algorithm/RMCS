# 自起接入 RMCS：按现有风格的实施计划

> 2026-10-03 V6 接入后：本页为 **V5 自起实现与验证历史记录**。当前默认模型已是 V6 flat_14020，旧 V5 移至 `models/wheel_leg/legacy_v5/`。现行配置、遥控/滚轮高度及未验收门控以 [V6 部署说明](wheel_leg_v6_model_deployment_20261003.md) 为准；下文 V5 标定开关、40–110° 轨迹与成功率不适用于 V6。

> 2026-10-03 更新：当前代码、传感器提交证据、参考标定与 V5/V6 边界见 [自起部署说明](wheel_leg_self_righting_deployment_20261002.md)，最新 C++/Python 对照见 [对齐复测报告](artifacts/self_righting_alignment_20261002/README.md)。本文保留当时设计/验证记录；实现状态以新说明和当前代码为准。

前置阅读：[wheel_leg_self_righting_rmcs.md](wheel_leg_self_righting_rmcs.md)（当前代码落点与缺口）。本文只回答“怎么接入才符合 RMCS 风格”，并给出可执行计划。

## 1. RMCS 风格是什么（从现有代码归纳）

- **一个组件一个文件、一个职责**：`rmcs_core/src/` 下每个 controller 都是独立 `Component + Node`，构造里 `register_input/register_output`，`update()` 做一步无阻塞推进，不把多个职责塞进一个类。
- **一切数据走注册接口**：跨组件只通过 `/a/b/c` 强类型接口收发（`double`/`Eigen::Vector3d`/自定义 struct），不共享裸全局、不互相 `#include` 实现。`ValueBroadcaster` 只把已选 double 转 ROS 话题，不承担控制通路。
- **独占输出、单一拥有者**：接口名重复注册会抛异常（`rmcs_executor/component.hpp:469`）。同一个 `/wheel_leg/left_hip_joint/control_torque` 只能有一个输出者——目前是 `RlController`。
- **频率靠接口年龄与分频，不靠额外时钟**：executor 统一按 `update_rate` 调 `update()`；需要分频的组件自己数 tick（`rl_controller.cpp:344` 的 `pd_divisor_`）。`/predefined/timestamp` 是**预定**时刻，年龄判断用 `steady_clock::now()`。
- **安全优先、故障锁存**：`rl_controller.cpp` 的非有限值、软限位越界直接 `fault_latched_` 回 IDLE；硬件 `wheel-leg.cpp` 用 `feedback_fresh()`+拨杆决定是否输出力矩。不可信状态一律回零，不降级运行。
- **参数必须显式标定**：`get_parameter_or` + 构造时整组校验（`calibration_ready` 等门禁），默认关，标定后才开，占位值不允许静默启用。
- **插件注册两处**：类在 `plugins.xml`，实例在 `config/*.yaml` 的 `components:` 列表；包内新增源文件在 `CMakeLists.txt`（`rmcs_core` 用 `GLOB_RECURSE` 自动收，`rmcs_rl` 需显式列）。
- **命名**：全小写下划线；路径按设备域前缀（`/wheel_leg/...`、`/chassis/...`）；命令类接口后缀 `/control_*`。

## 2. 接入设计（遵守上述约束）

保持「`RlController` 是六轴力矩唯一拥有者」，自起恢复放在它的 **PREPARE 阶段**；新增组件只提供恢复所需的**目标与观测**，绝不注册第二个 `control_torque`。

数据流（全部是注册接口）：

```text
WheelLeg ──逐轴 angle/velocity/torque/status/stamp, imu/*, accel, fresh ──▶
MechanismModel ──q/dq(连续)/inner_knee/spring/轮高/余量 ──▶
SupportEstimator ──每侧 Support + height(valid/confidence) + 冲击 ──▶
RecoveryController ──六轴目标(模型侧) + phase/handover_ready/failure ──▶
RlController ──PREPARE 内 PD/混合 + 统一限幅 + Jᵀ ──▶ 六轴 control_torque
```

- `RecoveryController` 只做纯状态机与参考推进，不访问 CAN/ONNX，输入是带 `stamp/valid/confidence/annual` 的不可变快照，输出**目标角**。它只写“恢复目标”接口，不写力矩。
- `RlController` 在 `update()` 里按现有外层状态机推进：PREPARE 时改用恢复目标；混合门禁通过后按 200 ms 权重把自己的 RL 目标与恢复目标混合。这样“力矩拥有者”身份不变，且不需要另一组件仲裁同名接口。
- 观测估计（连续角、闭链、轮支撑、高度）放在独立组件，`RlController` 只消费其输出，保持它自身是纯控制、可离线对表。

### 2.].1 组件清单与接口

| 组件（放 `rmcs_core/src/controller/chassis/` 或 `rmcs_rl/src/`） | 输入 | 输出 | 职责 |
|---|---|---|---|
| `WheelLeg`（改） | CAN | 现有 + 每轴 `feedback_stamp`/`sample_index`/`raw_angle`、`imu/acceleration_body`、`joint_control_ready` | 采样新鲜度与驱动就绪语义 |
| `WheelLegMechanismModel` | 四轴 angle+stamp、配置的闭链 LUT | `q_continuous`、`inner_knee_rad`、`spring_compression_m`、`wheel_relative_height_m`、`geometry_valid` | 解缠 + 闭链 FK 查表 |
| `WheelLegSupportEstimator` | IMU accel/gyro、轮 angle/velocity、已发轮力矩、上面几何输出 | 每侧 `support_state`、`height_m`(`valid`/`confidence`)、`impact` | 只输出证据，不输出伪 Fz |
| `WheelLegRecoveryController` | 支撑估计 + q/dq + 姿态 + 请求 | `recovery_target[6]`、`phase`、`failure`、`handover_ready` | 纯状态机/参考 |
| `RlController`（改） | 策略目标 + 恢复目标 + 支撑 | 六轴 `control_torque`（不变） | 唯一力矩拥有者 |

配置签名（沿用现有参数风格，默认关闭）：

```yaml
recovery:
  enabled: false
  profile: v5_232mm_40_110_capture_v1
  feedback_hz: 200
  policy_hz: 50
  active_timeout_s: 8.0
  ...
```

硬件/估计器各自 `*_ready` 门禁保持默认 `false`，验证后由标定记录开启；这些参数不接受仿真数值冒充。

## 3. 实施计划（分阶段，每阶段可独立验证）

阶段 0 起就不改 `rmcs_rl` 现有 ONNX 通路，避免破坏已验证的平地策略。

### 阶段 A：驱动层观测与使能联锁（改 `wheel-leg.cpp`、`wheel_leg_control.hpp`、RL YAML）
1. DM 在 `match_then_store_status`（收到新 CAN 帧）时记录 `stamp`、自增 `sample_index`、保存 `raw_angle`；`update_status()` 不再更新序号<tspan>。驱动状态用 `status_code` 判定 enabled。
2. 新增 `/wheel_leg/imu/acceleration_body`（比力 `m/s²`，与仿真同参考端）及 `imu_stamp`/`accel_stamp`。
3. 新增 `/wheel_leg/dm_control_ready`：仅当清错/使能帧序列完成并首次发送 MIT 帧后为真，供恢复计时起点。
4. RL 图设 `require_enable_request: true`；`RlController` 输出 `/wheel_leg/enable_request`（标定门禁通过且请求恢复/RL 时为真）。
5. 验证：硬件在环观察拨杆→使能→力矩链路；反馈超时/双下立即回零失能；`/wheel_leg/calibrate` 仅允许双下、失能时触发。

### 阶段 B：状态适配 + 机构模型（新增两个组件 + CMake/plugins/YAML）
6. 逐轴解缠（真实周期/饱和语义 + 时间戳 + 速度上限），再按已标定的对角映射与零位换到模型角。
7. 闭链 LUT + 导数：由 `q_aux-q_hip` 得内角、气簧压缩、轮心相对 base 坐标、机械余量；越界/分支不明/残差超界 → `geometry_valid=false`。
8. 验证：与训练仓库 `tests/test_v5_activation.py`、`tests/test_recovery_observer.py` 的回绕/共同圈数/有效域用例逐项对表；仿真真值适配器先跑通控制数学。

### 阶段 C：支撑估计（新增组件）
9. 按 `recovery_observer.py` 组合 IMU 比力/冲击、轮相对高、轮速/机身 gyro、已发轮力矩，输出每侧三态与条件高度；不输出假 `Fz`。
10. 加小力矩双向探测，带静置/驻留窗口；探测幅值、响应界限、驻留时间必须台架重标定，不用仿真值。
10. 验证：离线回放仿真轨迹对照 `probe_confirmed`/`settled`；台架单侧轮承重、空转、壳体支撑、托举四类。

### 阶段 D：恢复状态机（新增组件）
12. 在外层 `State::kPrepare` 内实现 `FOLD/PLANT/PREPARE/WAIT_GROUND/ORBIT/THRUST/SIDE_SWING/CAPTURE/BLEND/FAILED`，含倒扣至多一次重选路、8 s 预算、PLANT 2.5 s、ORBIT ≤1 圈、侧摆有界。
13. 从**当前连续实测角**出发；高度无效时不得假定已站立；侧翻选腿只在可靠几何下锁存。
14. 只写恢复目标接口 + 诊断 phase/failure；控制数学按部署文档 §5–7 与冻结数值附件对表。
14. 验证：C++ 与 Python 逐 200 Hz 对照阶段、参考角、接管时刻；再逐案跑原 10 姿态×5 次。

### 阶段 E：控制仲裁与交接（改 `rl_controller.*`）
16. 拆 `recovery_ref` 与 `policy_targets`；50 Hz 推理改成 PREPARE 内可影子运行，影子期不写上一动作、不发力矩。
17. 通过支撑门禁后，模型侧每 5 ms 同时算脚本六轴与 RL 六轴力矩，`alpha=clamp((now-blend_start)/0.2,0,1)` 混合**力矩**；仅 BLEND/RL 更新裁剪动作历史，BLEND→RL 不清历史/推理时钟。
18. 统一输出链：模型/机械保护 → ±40 Nm 腿限幅 → 实测轮包络 → `Jᵀ` → 驱动限幅。轮 15.8（现车）与 11（训练先验）参考端不同，先做功率/电流实测校核。
18. 验证：固定 fixture 对照混合权重 0/中间/1；`tests/test_v5_activation.py` 的 `leg_reference_delta`、`pitch_reaction_effort` 用例；饱和区顺序一致性。

### 阶段 F：调度与日志
20. 1000 Hz 只做检查/重发；200 Hz 推进恢复；50 Hz 推理。用**实际**单调时间记录快照间隔与跳周期，漏 tick 不用旧反馈补跑。
20. 环形日志：200 Hz 参考/力矩/有效位，50 Hz 观测/动作，阶段/失效原因；实时线程不压缩落盘，经 `ValueBroadcaster` 或后台线程接 TensorBoard。

## 4. 需要实物回填的标定（阶段 B–E 的阻塞项）

每轴 CAN/反馈端与 **105°** 设零姿态对应的四个模型角、符号；DM 角度饱和/回绕规律；实测电流与转矩上限；真实气簧与闭链 LUT 残差；轮侧 15.8 对 11 输出端标定；IMU 外参及加速度时间戳/延迟；轮探测响应与支撑估计误差。它们以版本化配置进入 `*_ready` 门禁，缺失时不启用自动交接。

## 5. 验收顺序

1. 离线：固定 fixture 的 35D→6D→裁剪→目标→力矩逐项对表；启动校验 ONNX SHA 与合同，保留来源/运行机械两份 manifest。
2. 仿真替换：同资产下 C++ 控制数学替换 Python 控制器，再替换传感器估计，跑原能力矩阵（仅传感器闭环末尾严格通过 45/50，真值控制 44/50，均不代表实机）。
3. 台架：四轴标定 → 限能量单侧动作 → 有明确支撑的小倾角准备/交接 → 侧翻/倒扣；记录失败，不用平均值掩盖单类问题。
