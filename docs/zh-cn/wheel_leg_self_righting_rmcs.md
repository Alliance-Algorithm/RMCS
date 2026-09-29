# 串联腿 V5 自起接入 RMCS：当前代码落点与实施顺序

核对范围：本工作树的 `rmcs_ws/src/rmcs_rl`、`rmcs_core`、`rmcs_bringup`，以及相邻训练仓库 `../robot_rl/isaac_wheeled_rl_train` 的 12486 模型合同、传感器闭环自起实现和零位文档。本文是针对**当前 RMCS 工作树**的实施设计；源仓库的硬件文件正在重构，以当前文件名为准。

## 已有能力与实际缺口

- `rmcs_ws/src/rmcs_rl/models/wheel_leg/policy.onnx` 的 SHA-256 为 `ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d`，与 `models/v5_flat_12486/policy.onnx` 相同。训练包 `infer_example.py` 的固定 35D→6D、目标和力矩用例已通过。上线时仍须校验**安装后实际加载的文件**。
- `rmcs_rl/src/observation.cpp` 已按 P 顺序组 35D；`action.cpp` 已按腿 ±3、轮 ±9 裁剪，采用 PC 侧腿 60/2 和轮 0.2 控制；`wheel-leg-infantry-rl.yaml` 使用 DM `joint_control_mode: torque`。模型不输出翻身动作，自起是脚本阶段加最终的 normal RL 平衡。
- `action.cpp` 轮力矩只按驱动报告的固定 `max_torque` 裁剪，尚未实现训练的随轮速减小的包络；`onnx_policy.cpp` 检查输入输出名字和形状，但没有校验加载模型的 SHA。
- `rl_controller.cpp::update_prepare_()` 目前每 **1 kHz** 逐轴把目标按最短角推向 actor nominal，只检查倾角/关节误差/速度，未做路线、支撑估计或恢复超时；`enter_()` 在非 RL 状态清空上一动作，首次进 RL 强制重推理，尚不能实现 PREPARE 内 200 ms 双路力矩混合。
- `observation.cpp::read_model_state_()` 直接做 `J*q_api+offset`，未对 DM 新反馈解缠。`dm_motor.hpp::update_status()` 每执行器 tick 解码缓存的 CAN 帧；缓存帧可能在多个 tick 重复，不能把 tick 当作新样本。DM `P_MAX` 是 MIT 量化区间，不能假设它是角度回绕周期。
- `hardware/wheel-leg.cpp` 当前只有聚合 `/wheel_leg/feedback_fresh`（六轴和 IMU 各距接收时间小于 50 ms）。`/wheel_leg/imu/sensor_gravity/*` 来自四元数投影，**不是原始加速度**；BMI088 加速度只进入 EKF，未发布带采样时间的独立观测。传感器闭环的冲击/主动探测还缺少必要输入。
- RL 配置没有设置 `require_enable_request: true`，硬件端默认 `false`，双中即请求使能 DM；此时 RL 的三个 `*_ready` 门禁只能令其输出零力矩，不能阻止驱动使能。`WheelLegDmCommandScheduler` 在使能切换后发清错和约 100 周期使能帧，再开始 MIT 帧：恢复计时及参考推进必须等到真实驱动就绪。`/wheel_leg/calibrate` 目前能直接发送四个 DM 设零帧，运行期间须互锁。
- YAML 中 `calibration_ready`、`soft_limits_ready`、`imu_alignment_ready` 均为 `false`，`leg_motor_to_model`/闭链系数仍为占位；这些门禁不能靠填仿真数字绕过。现行膝线性限位近似在冻结仿真 45 点上最大误差约 2.66°，大于现有 `hinge_margin=0.03 rad`，不能作为自起全行程保护。

## 执行器图与数据契约

保持 RL YAML 的三个核心组件：`WheelLeg` 发布反馈，`WheelLegChassisController` 发布双下→双中请求，**只有 `RlController` 注册六个 `/wheel_leg/*/control_torque` 输出**。参考保位图 `wheel-leg-infantry-reference.yaml` 的 `WheelLegReferenceController` 仅用于单独的标定程序，不能与 RL 图合并。参考图的 `position_pd` 与 RL 图的 `torque` 是两种独立配置。

在 `hardware/wheel-leg.cpp` 逐轴发布最新 CAN 帧的**接收时间、单调样本序号、原始/已反向 API 角度、速度、驱动状态/故障**；收到匹配 CAN 帧时更新序号，而不是在 `update_status()` 重复解码时更新。IMU 发布原始比力（与仿真 `m/s²` 一致，BMI088 当前加速度换算输出是 g）、gyro、姿态，以及各自样本时间/有效位。诊断可以另行广播标量，但 RL 控制应通过组件输入读取同一次采样快照，不依赖 50 Hz ROS 话题或通用 `ValueBroadcaster` 转发四元数/布尔值。

在 `rmcs_rl/src/` 中定义纯数据结构（下例为接口形状，不是现有头文件）：

```cpp
enum class Support { Unknown, NoContact, Candidate };
struct TimedValidity {
    std::chrono::steady_clock::time_point stamp;
    bool valid = false;
    double confidence = 0.0;
};
struct RecoverySnapshot {
    TimedValidity imu, geometry, left_wheel, right_wheel, height;
    Eigen::Matrix<double, 6, 1> q, dq; // P 顺序，腿角连续
    Eigen::Vector3d g_base, omega_base, accel_base;
    std::array<Support, 2> wheel_support{Support::Unknown, Support::Unknown};
    double height_if_grounded = 0.0; // 只有 height.valid 才允许参与门禁
    double relative_wheel_height = 0.0;
};
```

P 顺序：`[L_joint1, LL_joint1, R_joint1, RR_joint1, L_joint3, R_joint3]`；当前 DM 参数 D 顺序：`[left_hip, right_hip, left_knee_drive, right_knee_drive]`。每个 DM 在 API 角度端先按**实测反馈语义**解缠，再应用 `q_model[i] = sign[i] * e_continuous[i] + q_at_encoder_zero[i]`（等价于已校准的对角 `J` 加偏置），力矩用 `tau_api = J.transpose() * tau_model`。API 已被 `DmMotor::reversed` 处理，不要再次按电机安装位置猜符号；1:1 外链不等于被动膝内角 1:1。只有 **105°** 内膝设零这一条信息**无法确定**四根主动轴的模型角；需要大腿绝对姿态、闭链装配分支和每轴方向。解缠需要真实回绕周期/饱和语义、时间戳与物理速度上限；跨圈不唯一、设零或重启后使坐标失效并退出恢复，而非用 `2*P_MAX` 猜周期。

实测左右闭链 FK/单调 LUT 提供 `inner_knee=F(q_aux-q_hip)`、气簧压缩及导数、轮心相对 base 的坐标与机械余量。仿真附件的 45 点表只适合先做 C++ 数学对照，不可当成实车标定。平地**确认轮接触时**才能使用 `h_base = r + g_B·p_wheel,B`；两轮高度分歧、轮子空转、车壳触地/被托起、姿态卡常值要输出 `Unknown` 或无效，不能填高度 0.305 m 或伪造牛顿制法向力。

## 代码改造顺序

1. **驱动使能/失效链**（`hardware/wheel-leg.cpp`、`wheel_leg_control.hpp`、RL YAML）：让 RL 独立输出 `/wheel_leg/enable_request`，并把 RL 图的 `require_enable_request` 设为 `true`；硬件端用“双中且 DR16 新鲜 + 请求 + 六轴/IMU 新鲜 + 无故障 + 输出新鲜”决定允许力矩/使能。双下、离开双中、失联、反馈超时或 RL 故障立刻发零/失能并锁存，重新双下→双中才可重试。发布 DM 实际状态与 `kMit` 首次可用事件，不能把“已发使能请求”当“已可控”。设零命令仅允许在双下、失能且未恢复的标定路径，随后清空角度连续状态。先用硬件在环验证这一层。
2. **独立状态适配与机构模型**（新增 `rmcs_rl/src/model_state_adapter.*`、`mechanism_model.*`）：先完成逐样本解缠、P/D 顺序映射、IMU 外参、闭链 LUT 的有效域/导数及输出限位；所有输入按同一控制周期形成快照。原 RL 的角度观测仍包角，恢复参考使用连续角。同侧返回普通目标时由髋轴选择共同整圈：`k=round((q_goal_hip-q_ref_hip)/(2π))`，两轴同时减 `2πk`；定向 `ORBIT/THRUST/SIDE_SWING` 不包角。精确半圈需与 Python/Torch ties-to-even 对表。
3. **支撑观察器**（新增 `support_estimator.*`）：按 `recovery_observer.py` 将 IMU 比力/冲击、闭链 FK 相对轮高、轮速度/机身 gyro、已发送轮力矩汇成每侧三态证据；只有几何、姿态、机械余量和速度允许时做小力矩双向探测，额外保留静置和驻留窗口。仿真每轮 ±0.18 Nm ×15 ms、探测响应界限及驻留时间仅为参考，不能直接用于实机。`ORBIT→THRUST` 可用较早且受限的相位证据；`BLEND` 必须两侧探测完成、几何/速度一致并保持严格驻留。未知时维持有界动作直至超时退出。
4. **纯恢复状态机**（新增 `recovery_controller.*`）：在外层 `State::kPrepare` 内实现 `FOLD/PLANT/PREPARE/WAIT_GROUND/ORBIT/THRUST/SIDE_SWING/CAPTURE/BLEND/FAILED` 及一路至多一次倒扣重选路；双下→双中且输入有效时从**当前连续实测角**开始，不复制仿真 ZERO/释放窗口。初始近正立路径要求有效的轮接触条件高度；高度无效时不能盲选已站立。8 s 主动预算、PLANT 2.5 s、ORBIT 最多 1 圈、侧摆有界往返；每个阶段单独输出原因码与门禁状态。控制数学/静态目标按 `V5_SELF_RIGHTING_DEPLOYMENT.md` §5–7 和冻结数值附件先离线对表，再以实测配置替换仿真参数。
5. **控制仲裁与策略交接**（`rl_controller.hpp/.cpp`、`observation.cpp`、`action.cpp`）：拆 `recovery_ref` 与 `policy_targets`，将 50 Hz 推理移成 PREPARE 内可影子运行的函数，恢复期强制 normal 上下文、`vx=vy=yaw=0,h=0.305`，影子期不写上一动作。通过有效支撑门禁后在模型侧每 5 ms 同时计算脚本六轴力矩与 RL 六轴力矩，以 `alpha=clamp((now-blend_start)/0.2,0,1)` 混合；仅 BLEND/RL 时更新裁剪动作历史，BLEND→RL 不清空历史/推理时钟。统一执行模型/机械保护、±40 Nm 腿限幅、实测轮包络、`Jᵀ` 和驱动限幅。轮训练包络的 11 与现车 M3508 配置 15.8 不是相同参考端，先做功率/电流实测校核再定参数。由 RL 一个组件写六路力矩，DM 内部 Kp/Kd 保持 0。
6. **调度与日志**：1000 Hz 检查拨杆、硬件故障、输入/指令年龄并重发有效力矩；200 Hz 才推进恢复状态、轨迹和双路 PD；50 Hz 才组 35D/推理。用单调**实际**时间记录快照间隔/跳周期（executor 的 `/predefined/timestamp` 是预定 tick 时间），漏 tick 不用旧反馈补跑多个控制步。环形日志记录 200 Hz 参考/力矩/有效位、50 Hz 观测/动作、所有阶段与失效原因；实时线程不写压缩文件。

建议 `RecoveryController` 只接受不可变快照、实际 `dt` 和配置，返回 `phase/failure/script_tau_model[6]/handover_ready`，不持有 CAN 或 ONNX；`SupportEstimator` 独立维护驻留与探测证据。`RlController::update()` 中的控制路径可以按如下边界组织：

```text
每个执行器 tick：检查双中/复位/新反馈年龄/电机故障/输出年龄；异常立即清六轴、撤销 enable_request、锁存原因
驱动尚在清错/使能帧阶段：保持恢复时钟和参考冻结；首次可发 MIT 后以当前 q 重建恢复参考
每个 200 Hz tick：构造同步快照 → 估计支撑/机构余量 → 推进 PREPARE 内恢复状态
每个 50 Hz tick：从同一快照组装固定零速/0.305 m 的 35D → ONNX → 缓存独立策略目标
每个 200 Hz tick：脚本和策略分别用当前 q/dq 算模型侧力矩 → 按 phase 仲裁/混合 → 统一机械保护和限幅 → Jᵀ → 输出六轴力矩
```

目标角只在 PC 内供模型侧 PD 使用。脚本普通腿为 `clip(Kp*(q_ref-q_cont)-Kd*dq, ±40 Nm)`（回转类 120/4，近正立路径 80/2）；`CAPTURE` 按实测轴符号叠加俯仰阻尼后再次限幅，气簧抵消只在经校核的指定分支加且再限幅。脚本轮的零轮力矩阶段须真正输出 0，不能沿用现有 PREPARE 的 `-0.2*dq_wheel` 制动。`BLEND` 混合的是两套模型侧**力矩**而不是角度目标；限位和电机驱动侧包络应用于混合后的统一输出链。

## 验证顺序及当前阻塞项

1. 离线复现 `models/v5_flat_12486/io_fixture.json`：35D→原始6D→裁剪历史→P/C目标→脚本/RL/混合模型侧力矩；启动校验 ONNX SHA 与合同，保留来源机械 manifest 和恢复用机械 manifest 两份身份。训练侧 `tests/test_v5_activation.py`、`tests/test_recovery_observer.py` 提供共同圈数、回绕、支撑门禁的边界用例。
2. 在同一冻结资产上让 C++ 控制数学替换 Python 控制器，逐 200 Hz 对照阶段、连续角、接管时刻、力矩；再替换成传感器估计，逐案跑原 10 姿态×5 次。仅传感器仿真末尾严格通过分别为 45/50（无附加扰动）与 45/50（指定扰动），**不代表实机通过**；真值控制是 44/50。
3. 台架先完成四轴 `q_at_encoder_zero`/API 方向/反馈回绕/力矩参考端、真实闭链 40–110°范围及轮/IMU 朝向的标定，然后限能量单侧动作→有明确支撑的小倾角准备/交接→侧翻/倒扣。卡死轮、低摩擦滑动、壳体支撑和外部托举在仅 IMU+编码器下不可普遍判别；无法确认时拒绝接管并有界退出。

当前需要实物回填的数据：每轴 CAN/反馈端与设零姿态对应的四个模型角/符号、DM 角度饱和或回绕规律、实测电流与转矩上限、真实气簧与闭链 LUT 残差、轮侧 15.8 对 11 的输出端标定、IMU 外参及加速度时间戳/延迟、轮探测响应/支撑估计误差。它们应以版本化配置进入运行时的 `*_ready` 门禁；在这些数据和控制数学对照缺失时不启用自起自动交接。

权威参考：相邻训练仓库 `docs/V5_SELF_RIGHTING_DEPLOYMENT.md`、`docs/DEPLOYABLE_SELF_RIGHTING_SENSING.md`、`docs/V5_ZERO_TRANSMISSION_ALIGNMENT_20260926.md`、`docs/ONNX_INTEGRATION_V5_20260925.md`、`models/v5_flat_12486/policy.onnx.contract.json`，算法入口 `scripts/inspect_v5_activation.py --sensor-only` 与 `src/wheeled_tasks/chassis/recovery_observer.py`。
