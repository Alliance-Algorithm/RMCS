# IMU 与电机反馈自起部署：2026-10-02

> 2026-10-03 V6 接入后：本页为 **V5 自起实现与验证历史记录**。当前默认模型已是 V6 flat_14020，旧 V5 移至 `models/wheel_leg/legacy_v5/`。现行配置、遥控/滚轮高度及未验收门控以 [V6 部署说明](wheel_leg_v6_model_deployment_20261003.md) 为准；下文 V5 标定开关、40–110° 轨迹与成功率不适用于 V6。

状态：在 `dev/wheel_leg_rl` 实现与离线验证；本次未连接或驱动实机。硬件仅有 BMI088 IMU、四台 DM 主动关节和两台 M3508 轮电机反馈，不依赖触地开关、力传感器或测距。2026-10-03 用户确认参考分支的实际硬件配置与电机零点可复用，BMI088 +X 朝车体前方；已据此设置 `calibration_ready=true`、`imu_alignment_ready=true`。`soft_limits_ready`、`recovery_profile_ready` 和 `recovery_enabled` 仍为 `false`，本机机构配置与自起响应验收尚未完成。

## 实现位置与 RMCS 接入

| 文件 | 职责 |
| --- | --- |
| `rmcs_ws/src/rmcs_core/src/hardware/wheel-leg.cpp` | CAN/IMU callback 快照、status 发布、command 编码提交及硬件使能门禁 |
| `rmcs_ws/src/rmcs_core/src/hardware/device/wheel_leg_sensor_snapshot.hpp` | 固定三槽 SPSC 快照；值、序号、时间整体交接，无控制循环分配 |
| `rmcs_ws/src/rmcs_rl/src/recovery_sensor_guard.hpp` | 六轴及 IMU 两条流的 freshness、顺序与采样跨度检查 |
| `rmcs_ws/src/rmcs_rl/src/recovery_observer.{hpp,cpp}` | 标定闭链/壳体/气簧几何、条件支撑、实际轮探测响应 |
| `rmcs_ws/src/rmcs_rl/src/recovery_controller.{hpp,cpp}` | 纯 C++ 多路线动作与插值、失败退出、200 ms 接管权重 |
| `rmcs_ws/src/rmcs_rl/src/controller_recovery.cpp` | 通过 Component 输入把硬件快照送入 observer，再推进状态机 |
| `rmcs_ws/src/rmcs_rl/src/{configuration,observation,action,rl_controller}.cpp` | 参数、策略观测/历史、PD/力矩混合和唯一六轴输出 |
| `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml` | RMCS 组件图、标定参考、探测阈值及诊断转发 |

沿用 RMCS `Component::register_input/register_output` 和硬件 status/command 分层。控制器只从注册接口读取，不通过 ROS topic 回读自己的传感器或力矩；`RlController` 唯一拥有六轴 `control_torque`。ROS broadcaster 只用于诊断和录包。

脚本控制/PD 为 200 Hz，ONNX 50 Hz，executor/CAN 门控及 held effort 为 1 kHz。这是当前部署 **V5** 策略的合同，不是 V6 训练的 1 kHz PD 合同。进入 BLEND 仅清一次 previous action，继续持有最近 shadow targets，沿用全局 50 Hz 推理时钟；BLEND→RL 保留动作历史与时钟。混合在模型力矩侧完成，随后统一机械/速度/累计峰值预算限幅，再按 `Jᵀ` 变换成设备力矩。

## 参考分支标定如何复用

来源：[Alliance-Algorithm/RMCS 的 dev/rmcs_rl](https://github.com/Alliance-Algorithm/RMCS/tree/8149afda1de0f0e79dac07a6444fc3741759d5c2)，核对文件 `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml` 及对应 DM 驱动。参考分支四轴均 reversed，旧 V5 模型角为 `offset - raw`，P 顺序的 offset 是 `[-1.6, -2.93, 1.6, 2.93] rad`。我们的核心驱动 reversed、bias=0，API 角为 `-raw`，因此给当前 V5 actor 的换算是：

```text
q_v5 = I * q_api + [-1.6, -2.93, 1.6, 2.93]
dq_v5 = dq_api
tau_api = tau_v5
```

2026-10-03 再次通过 Git 获取参考分支，HEAD 仍为 `8149afda1de0f0e79dac07a6444fc3741759d5c2`。用户确认沿用其实际硬件与零点标定后，RL YAML 中 `calibration_ready=true`；四轴零偏继续只在模型侧加一次，不在驱动中重复添加，不向电机发送重新设零。轮输出已经由设备的 15.8 减速比换算，策略侧不再乘减速比。DM 命令/反馈 ID、reversed 方向与该分支一致，MIT 量程沿用其 `P_MAX=12.5 rad`、`V_MAX=45 rad/s`、`T_MAX=54 Nm`；控制峰值仍限于 40 Nm。

BMI088 的安装配置沿用参考分支 `Bmi088Ekf::Config{.body_to_sensor = Eigen::Matrix3d::Identity()}`，对应已确认的 +X 前、+Y 左、+Z 上车体系；`imu_to_base=I`、`imu_alignment_ready=true`。姿态、陀螺仪和比力统一使用 Eigen，策略观测、PREPARE、自起观察器及失姿保护共用 `world_base_orientation_()`：`q_world_base = q_world_imu * Quaternion(R_base_imu.transpose())`。YAML 矩阵以 Eigen 的行优先 `Map` 读取，避免把安装旋转的逆矩阵读成正矩阵；不重复应用 CAD 导出旋转。

这些就绪值记录的是用户确认的既有安装/零点事实，不表示本次新做了 CAN/USB/EKF 实机测试。当前仍被缺失机构软限位与完整自起 profile 门禁阻止输出控制力矩。

V6 录包标定文件 `.script/identification/calibration/v6_pair_from_dev_rmcs_rl.json` 使用与旧 V5 相反的模型轴：`J=-I`、offset `[1.6,2.93,-1.6,-2.93]`。不能把这套换算直接放入当前 V5 actor。参考分支单圈 wrapped 角与本分支 DM 连续原始反馈还存在表示差异；回转路线需要核验共同圈数、反馈量化端点及实际工作域。

当前 `rmcs_rl/models/wheel_leg/policy.onnx` 是 `v5_flat_12486`，SHA256 `ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d`。Isaac 多路线旧证据使用 V5 232 mm / 40–110° 资产，manifest SHA256 `dbec8e586cf29d9540db7040b133b3ff1170c1c700ff05eda64d94d8561c3dec`。当前 V6 105° 资产的 manifest SHA256 是 `875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba`。这次实现不把旧自起成功率迁移为 V6 成绩，也不更换 ONNX。

## 双下、双中与自起触发

每次有效的 **双下→双中** 都是一个新的控制会话，启动时用当时的传感器重新选路线；不是仅上电第一次触发，也不是持续双中每周期重新启动动作。`reset_count` 也会在 UNKNOWN/失联复位时递增，不能单独作为自起触发；实际入口是合法 arm sequence 发布 `control_state=3`，RL 从 Idle/Init 进入 Prepare。

| 情况 | 行为 |
| --- | --- |
| 上电直接双中 | 没有新鲜双下解锁，保持失能，不启动 |
| 双下 | 当 tick 清六轴力矩、撤销 enable；硬件直接排队四台 DM 的 `0xFD`，两轮零电流；即使此前从未使能也发送失能 |
| 双下后异步拨到双中 | 中间 DOWN/MIDDLE 状态不使能；最终双中才请求控制，先清错/使能/零 MIT，并等 `dm_control_ready` |
| 有效双中、正立 | 建立传感器基线，走 stand/PREPARE、探测支撑及稳定接管，不做翻身轨迹 |
| 有效双中、倒地 | 依据当时倾角和方向选择 PLANT、ORBIT 或 SIDE 路线，自起后再接管 RL |
| 自由落体或支撑证据不足 | WAIT_GROUND 或 PREPARE 中等待；不能直接把条件几何当真实着地并交给 RL，超时有界退出 |
| 运行中的 RL 跌倒 | 当前恢复会话 RL 倾角超过45°会清力矩、撤使能并锁存，不自行再跑一遍自起；重新双下→双中才允许新会话 |
| DR16 失联 | 底盘用 `dr16_fresh` 把缓存开关置为 UNKNOWN，清掉 armed/pending；恢复连接后仅双中无法沿用旧双下，必须重新双下 |

双下的失能帧排在所有中性 MIT 帧之前，不以零力矩 MIT 代替 `0xFD`。软件提交语义与 CAN 送达 ACK 的区别同下节说明。当前真机图保持 `require_enable_request=true` 和 `auto_enter_rl=false`；自动入口只是原有仿真功能，不用于这套遥控部署流程。

现有 IMU/电机不能独立识别“静止被吊起”的真实接触；此时条件姿态可能先进入 PREPARE，但实际双向探测响应没有通过就不会接管 RL。

## 传感器与支撑证据

每轴 `feedback_sequence/feedback_steady_ns/feedback_frame_bytes` 与 angle/velocity/torque 同属一次接收快照。IMU `/sequence` 与 `/last_steady_ns` 对应同一次 orientation/gyro 输出；`/acceleration_sequence` 与 `/acceleration_steady_ns` 对应加速度快照。重复/倒序 board tick 不刷新姿态或 freshness，正常 32 bit 回绕保留。硬件断连、板时钟重新启动后应重启组件重新初始化时基。

默认六轴/gyro 年龄上限 20 ms，加速度 30 ms，八个样本整体跨度上限 10 ms；序号与时间必须一致推进。检查在每次 executor 更新以及推理后输出前进行。缓存样本在有效年龄内可以持有，但不能积累轮探测响应或伪造新陀螺仪导数。即使 DM 已就绪，首次观测也只建立传感器导数基线，保持零力矩；下一有效观测才选择动作路线。失效会撤销使能、清六轴力矩并锁存；需要双下复位，恢复反馈不会自动重新启动。

为避免 command→controller→command 循环，两轮 status 发布前一发送周期的 `/last_submitted_torque`、`/last_submitted_kind`、`/last_submitted_steady_ns`。这些是编码后 API 轮端 Nm 及 host queue time；SDK 不提供 CAN delivery ACK，因此提交记录必须再由后续新 CAN 电流力矩响应佐证。

当前探测默认 ±0.18 Nm，每轮每方向15 ms，连续时钟推进，包含暂不满足探测条件的周期。脉冲至少有5 ms已提交时间，随后一个合格的新轮速/IMU样本即可提供加载响应证据；不要求连续多个低加速度样本。正负两个方向、左右两个轮仍要求有效提交kind=1、正确方向、时间顺序及反馈同向力矩分量 `>=max(0.025 Nm, 0.25*abs(submitted))`；提交幅值至少0.14 Nm，提交年龄与反馈间隔最多20 ms，轮加速度与新gyro样本导数分别小于100/90 rad/s²。门控成零、无电流响应、重复缓存或错方向都不能确认。四方向证据保留最多0.6 s，两次被测响应不合格或几何支撑丢失50 ms后清除；还需几何驻留100 ms、停止探测50 ms及控制器接管稳定判定。

15 ms时序与原Python仿真对齐。延迟较大的实测CAN链需要单独配置脉冲长度；9.4 ms错相反馈回归测试使用25 ms。当前电流阈值和时延仍需实测。配置均为启动时加载；运行中 `ros2 param set` 会拒绝，避免参数显示与控制器缓存值不一致。

条件高度来自 IMU 姿态、闭链轮心与轮接地假设，不是独立测距；壳体离地也属于几何推断。这些传感器不能独立证明绝对高度、地面平整或真实接触力。

世界轮速制动现已接入逐LUT的被动膝轴、轮轴、被动膝方向和BMI088姿态。重建叠加机体陀螺仪、髋轴转速、由两主动轴差推算的被动膝转速及轮编码器转速，再用姿态投影到世界Y，保持原Python制动定义。部署profile必须提供这些标定；只有轮心位置不足以恢复轴。纯observer接口缺少轴或姿态时仍保持 `world_wheel_omega_valid=false`。

## 动作路线与参数

已保留FOLD、PLANT、定向ORBIT、THRUST、SIDE_SWING、CAPTURE、WAIT_GROUND、PREPARE、BLEND等路线及有界失败。2026-10-03补齐条件高度选路、普通FOLD无需回转跟踪门控、共同圈数跟踪误差、ORBIT/CAPTURE阶段条件、壳体落地候选、标定轴投影和气簧节点导数。CAPTURE用于抑制动量，可以在支撑尚未确认时进入；RL接管仍必须通过稳定支撑判据。进入BLEND后不再被8 s脚本预算打断，保留200 ms完整转交。左右主动轴仍共享圈数，不独立选两个最短弧。

参考分支零偏不包含下面完整机构配置。启用前，在 RL 参数中填写：

- `recovery_dm_feedback_position_max` 四值及实测 `recovery_above_rated_budget_s`；四个 `recovery_{orbit,side,rollover,capture}_speed`。
- 八组 `recovery_{fold,thrust,side_extended,stand,upright,support_extended,upright_support_extended,capture_extended}_p4`，须与选用资产、轮心几何、膝工作域匹配。
- 两侧 `recovery_{left,right}_{delta_rad,inner_knee_deg,slider_m,wheel_at_hip_zero_m,hip_origin_m,hip_axis,spring_compression_at_zero_m}`。
- 两侧 `recovery_{left,right}_{knee_axis_at_hip_zero,wheel_axis_at_hip_zero,passive_knee_sign,slider_slope_m_per_rad}`；轴表与角差LUT逐行对应，节点导数与原气簧补偿表匹配。
- `recovery_shell_points_body_m` 至少四个 XYZ 点，`recovery_spring_stroke_m` 和 `recovery_spring_force_n` 四系数。
- `hinge_*`、IMU 外参及上述探测响应参数的实测验收；LUT 不能覆盖超出当前机械止挡的动作。

现有 constructor 会拒绝缺失、非有限、非单调或越界机构数据。动作与力矩保护使用所填 LUT 的实际内膝域及最多 2° 保护带；不再硬编码 42–108° 保护域。只有验收后才能开启对应 readiness、自起使能，默认配置不会尝试自起。

## 部署、诊断和录包

在开发容器构建与检查：

```bash
cd /workspaces/RMCS
.script/build-rmcs --packages-select rmcs_msgs rmcs_core rmcs_rl rmcs_bringup
source /opt/ros/jazzy/setup.bash
source /workspaces/RMCS/rmcs_ws/install/setup.bash
cd /workspaces/RMCS/rmcs_ws
colcon test --merge-install --packages-select rmcs_core rmcs_rl
colcon test-result --verbose
```

使用既有 `.script/sync-remote` / `.script/wait-sync` 同步构建产物和 profile；具体双终端 SSH 流程见 `wheel_leg_v6_recording_20260930.md`。机器人端启动命令为下面三行，须先退出已有 executor，保持只有一个进程占用 CAN。它与 V6 identification 是两套配置；`.script/host/rmcs` 是宿主机进入开发容器的工具，不是机器人启动器。

```bash
source /opt/ros/jazzy/setup.bash
source /rmcs_install/local_setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-rl
```

启动前保持双下；完成全部标定并选择匹配的动作/策略合同后，AUTO 模式、摇杆回中，双下复位→双中才申请恢复，DM 使能就绪前保持零力矩。失败后重新双下，不会自行重试。

`/wheel_leg/rl/recovery/` 下诊断由 ValueBroadcaster 转发：`phase`、`failure`、`sensors_valid`、`sensor_issue`、`sensor_invalid_mask`、`motor_age_ms`、`imu_age_ms`、`acceleration_age_ms`、`contact_candidate`、`support_confirmed`、`geometry_valid`、`height_if_grounded`、`blend`、`motion_hold`。位图 bit0..5 顺序为 LH,LK,RH,RK,LW,RW，bit6 gyro/orientation，bit7 acceleration。sensor_issue 为 0正常、1缺失、2未来时间、3过期、4倒序、5序号/时间不一致、6采样跨度过大；具体信号无效还应结合 failure 和原始反馈判断。`blend` 表示策略的实际力矩权重，恢复后 RL 阶段保持 1.0；闲置时为 0。

可在另一个 SSH 终端用 `ros2 bag record -s mcap -a -o <本次运行目录>` 同时录下原有 `/wheel_leg/identification/sample` 的 1 kHz 六轴/CAN/IMU 接口快照及自起诊断。标量诊断按 broadcaster 50 Hz；微小探测脉冲及真实发送顺序以 1 kHz typed sample 中的时间/序号/原始帧为准，不能只靠50 Hz图判断。每次保留 profile、模型/资产 SHA、机器人固件及硬件标定版本。

## 验证边界

新增测试覆盖传感器快照并发一致性、重复/倒序与32 bit时钟回绕、全八路freshness、方向/时序/电流门控、1 kHz 提交与200 Hz observer错相/延迟、轮导数实际时间、双轮双方向支撑证据、轨迹数值与跨±π行为。生产 `RlController` + 真实 ONNX 的合成传感器集成测试还验证了完整 PREPARE→双轮双方向实发/响应→BLEND→RL，覆盖 shadow target 保持、previous action 只清一次、50 Hz 时钟不移相，以及 RL 100% actor effort。该测试不模拟机械动力学。

开发容器编译通过。2026-10-03复查 `rmcs_core` 106条、`rmcs_rl` 108条，共 **214条gtest通过**；覆盖LUT节点边界、合成轮角速度、气簧节点导数、CAPTURE入口和预算边界，原双下失能与双中重新选路测试继续通过。

随后按用户确认复用参考分支硬件与零点，并统一 Eigen 姿态转换：`rmcs_core`、`rmcs_rl`、`rmcs_bringup` 编译通过；新增四项回归分别验证参考零偏只加一次、X前安装的俯仰/侧倾及gyro方向、行优先非单位阵安装转换、已确认IMU/电机但缺失机构限位时仍保持失能。当前 `rmcs_core` 106条、`rmcs_rl` 112条，共 **218条gtest全部通过**。参考提交、配置状态、源码SHA及验证记录见 [硬件标定复用记录](artifacts/self_righting_alignment_20261002/hardware_reference_reuse_20261003.json)。本次没有新运行实机动作，也没有更换策略模型。

部署 `RecoveryController`、`RecoveryObserver` 直接编译进Isaac Sim 6.0（本机包6.0.0.1）测试桥。2026-10-03使用相同V5 232 mm资产、ONNX及 **16姿态×5次**条件重跑：Python参考末尾严格通过 **62/80**，最终C++ **58/80**，修改前C++为18/80；曾稳定RL满1 s分别为73/80、66/80、26/80。14姿态单次筛查也从6/14恢复到13/14，与Python参考一致。

8°仍为机体相对正立的总倾角；脚本腿PD和轮力矩先扶正，RL在BLEND后才参与输出。本次没有放宽该门槛。相同输入回放的162827条有效记录中，Python/C++条件高度最大差6.26e-8 m、合成轮角速度最大差2.36e-5 rad/s。此项证据验证运动学计算，不代表全部状态机逐位一致。

最终22次未通过包括：6次机构角差越出LUT保护域、8次CAPTURE回弹/超时、8次曾稳定RL但末尾连续稳定时间不足1 s。仰躺双腿朝上C++为0/5，Python也只有1/5。机械保护保留，所有失败保留分母，实机仍不满足启用条件。

完整轨迹、源码/SHA、参数、失败明细和复跑命令见[最新对齐报告](artifacts/self_righting_alignment_20261002/README.md)，历史结果见[修改前闭环报告](artifacts/self_righting_sim_20261002/README.md)。入口 `.script/simulation/inspect_recovery_isaac.py` 使用理想提交与PhysX已施加轮力矩反馈；接触力和根位置仅用于评分，没有接入控制器。没有验证完整Component硬件图、CAN/USB/BMI088 EKF或热预算；运行快照中各readiness均为false。其后按用户确认复用了参考分支零点和安装方向，当前就绪状态见本文开头。

还没有把这版 RMCS C++ 控制链重新接到 V6 105° Isaac 资产做全部倒地案例闭环，也没有录制或执行实机自起。
