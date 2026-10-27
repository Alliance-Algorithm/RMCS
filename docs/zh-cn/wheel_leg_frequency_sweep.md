# wheel_leg 扫频与气簧辨识

四个独立入口位于 `rmcs_ws/src/rmcs_bringup/config`，都由硬件、
`WheelLegFrequencySweepController`、`ValueCollector` 构成，没有 RL 推理组件。
执行器和主机力矩反馈环为 1000 Hz，目标每五个控制周期更新一次（200 Hz）。
本实现的离线测试不加载硬件插件。实车实验由操作员启动。

## 启动

在检出 `feat/wheel_leg_sweep` 的开发容器中运行：

```bash
cd /workspaces/RMCS
export RMCS_PATH="$PWD"
./.script/build-rmcs --packages-select rmcs_core rmcs_bringup
source rmcs_ws/install/setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-wheels-sweep
# 只测轮子高速：robot:=wheel-leg-wheels-high-speed-sweep
# 或：robot:=wheel-leg-joints-sweep
# 或：robot:=wheel-leg-joint-limit-probe
```

使用独立工作区时，将上述目录换成该工作区在容器内的路径，
将 `RMCS_PATH` 指向此目录并运行该目录的 `.script/build-rmcs`；
本工作区生成独立的 build/install/log。首次建工作区时可先用同一脚本构建
`fast_tf rmcs_msgs rmcs_utility rmcs_description rmcs_executor rmcs_rl`，
避免原工作区切换分支造成消息头或插件的共享 install 漂移。

修改关节或探测 YAML 中 `frequency_sweep.ros__parameters.side: left/right` 选择侧别。
另一侧髋膝始终失能，两个轮子持续零电流。轮子 YAML 中四个关节始终失能。
装夹条件为机身固定、腿的转动平面水平、气簧安装到位、轮子可自由转动。
`board_serial` 及 IMU 安装参数沿用当前硬件配置。

## 遥控操作

遥控部分直接参考 `ChassisController::update()`：每周期读取当前档位，
丢控/UNKNOWN 或双下立即关闭，其余档位直接处理，不需要先经过双下或双中。

| 左 / 右 | 关节扫频 | 轮子 / 端点探测 |
|---|---|---|
| 中 / 中 | 使能、零力矩待机 | 同左 |
| 上 / 中 | 立即进入模式 1：固定腿形，共同转动 | 立即开始实验 |
| 中 / 上 | 立即进入模式 2：固定髋角，改变膝内角 | 立即开始实验 |
| 上 / 上 | 立即进入模式 3：共同转动并改变开合 | 立即开始实验 |
| 下 / 下 | 立即失能并复位 | 同左 |
| 其他有效的含下档组合 | 零输出待机 | 立即开始实验 |
| UNKNOWN、丢控 | 立即失能；恢复有效档位后自动重新判断 | 同左 |

默认没有启动顺序、300 ms 去抖、驱动确认超时或故障/完成锁存：
同一个模式持续运行时保留轨迹进度；切换到另一个关节模式立即开始新模式。
返回双中立即零输出。每轮完成当周期零力矩；如果下一周期仍在扫频档，立即开始下一轮。
启动最初的 UNKNOWN 帧不会再使后续一直停在 `Fault`。

所有扫频配置均以**当前有效拨杆状态**为准，已移除 `latch_run` 与旧档位重放。
从双中拨到双上时，先经过上/中或中/上只会短暂启动相应模式；
一旦当前状态为双上，当周期重新锚定当前实测关节位置并进入模式 3，不会锁在过渡模式。
双中立即零输出待机，双下立即失能复位，丢控/UNKNOWN 当周期关闭；
同档持续运行保留进度，完成后仍保持扫频档会重新开始下一轮。
关节的失能、清错、使能命令使用现有驱动调度器；不要求双中才可清错。

轮子实验只使用两个轮子的角度/速度反馈，四个关节状态、IMU、
总反馈新鲜度和额外扫频心跳仅作诊断，不参与轮子发送门控。
关节实验使用所选侧的两个电机反馈；另一侧持续失能、轮子零电流。
活动电机数据缺失或非有限时当周期零输出，数据恢复后自动进入当前模式；
保留驱动电流/力矩编码上限。生产配置的门控规则保持原样。

轮子开始后先执行 **3 s 零电流基线段**，随后先做高速实验。
`wheel-leg-wheels-high-speed-sweep` 只运行高速序列，约 **691 s**；
`wheel-leg-wheels-sweep` 在同样高速序列后保留原低速平台/扫频，总计约 **1619 s**。
两者都依次测试左/右单轮正反转，非活动轮始终零电流，四个髋膝关节失能。
控制台启动标识为 `remote=level-v2`，可据此核对部署版本。
`Sweep live`、`Sweep TX level-v2` 周期诊断打印的调用已注释；
拨杆、参考/实测速度、请求/编码力矩、CAN 帧及驱动反馈继续写入 CSV。
编码字节非零也不代表电机已经收到或执行，分析 CSV 时仍需同时检查反馈速度。

## 轨迹和探测

- 模式 1：先用 5 s 平滑过渡到 β=70°；以进入时整腿角为零，
  分别用 **0.2、0.6、1.5、π rad/s** 执行 0→720°→0；速度起停各平滑 1 s。
  π rad/s 的恒速平台对应 **2 s/圈**，单向两圈总用时 5 s，其中恒速 3 s。
  低速正反转用于辨别黏性阻尼、库仑摩擦及方向差异。模式 1 约 213.3 s。
- 模式 2（v4）：髋角参考保持进入值。先用 5 s 平滑进入 β=45°，
  在 **45°–100°连续往返**，不再每 5°停车。单程用时 4/2/1 s，对应往返 8/4/2 s，
  五次平滑曲线的参考峰值速度约 25.8/51.6/103.1°/s；每档先执行三次往返，
  再用不同力矩探针相位执行三次独立验证，`waveform=7`。
  随后在 50/75/95°各执行 20 s、0.15–3 Hz、幅值 4°的局部扫频，
  再执行不同相位的 20 s 验证段。模式 2 总用时 **224 s**。
  上述动态段在 PD 后叠加膝关节独立力矩探针，髋探针为零；两层最终限幅仍为 40 Nm。
  45°–100°是参考范围，气簧下实际角度存在偏差，不能作为实际行程或硬限位证明。
  水平悬空慢扫分析与 v4 设计依据见 `reports/leg_sweep_debug/slow_horizontal_20261027/analysis.md`。
- 模式 3：在 50、75、100°各执行 30 s 局部多正弦及独立力矩探针，再执行
  不同相位的 30 s 独立验证段；两段均围绕固定整腿工作点，供局部耦合模型辨识。
  随后整腿正转两圈、回转两圈，每向 5 s，恒速平台为 2 s/圈；
  另外在 60、90°执行整段高速验证。模式 3 约 260 s。
  旋转段与局部段均叠加整腿总幅值 3°和 β 总幅值 2°的多正弦。
  整腿频率为 0.17、0.43、0.79、1.31、2.17、3.73 Hz；
  β 为 0.23、0.61、1.07、1.73、2.41、3.91 Hz，并使用不同相位。
  局部段额外在 **PD 之后、最终限幅之前** 加两轴独立力矩激励，每轴总幅值最多 1 Nm。
  髋频率为 0.37、0.83、1.91、3.17、5.23、8.71 Hz，膝频率为
  0.29、0.97、1.57、2.83、4.61、9.13 Hz。局部段的 `waveform=6`，探针为模型力矩坐标。
  激励段首尾的位置扰动、速度、加速度及力矩探针包络归零。
- 轮子：新增 `wheel_high_speed_v1`，最高**目标** 65 rad/s（约 621 rpm），
  电流命令上限 20 A，按现有转矩常数和 15.62 减速比对应约 4.88 Nm 的轮轴力矩。
  该轮子限值与髋膝 40 Nm 是两个不同的驱动契约。高速序列如下，参数和供电依据见下一节。
  全频段入口仍保留 `wheel_multiband_v2` 的 0.15/0.3/0.6 rad/s 低速、
  1.5/3/6/12/20 rad/s 平台、台阶、0.2–2 Hz / 2–8 Hz 扫频及双轮验证。
- 端点：髋由主机 PD 保持在进入角（主动保持，不是机械锁死）；膝目标角在模型关节坐标里
  从 `probe_angle_start_deg` 单向慢速扫到 `probe_angle_end_deg`（默认 0°→360°、0.5°/s，约 12 min），
  纯位置闭环。**不做停点判定、不回撤**：目标一直走到底，机械顶死时只有 20 Nm 力矩限幅兜底
  （主机 `joint_torque_limit_nm` 与驱动 `dm_control_torque_max` 同为 20 Nm）。
  两端到达时各记录一次当时的真实膝角与 FK β（`probe/{low,high}/model_angle|beta_estimate`，`result=1`），
  控制台同时打印该端点髋相对进入角的偏移。
  真实限位从 CSV 里读：`reference` 还在走、`model_angle` 停住的那一点就是该侧的角度限位；
  β 超出 FK 表（30°–120°）时为 NaN，原始六电机 `angle` 始终有效。

连续角由每个新反馈帧的相位差累加，初始髋膝分支按已装配 CAD 闭链选择。
主机模型坐标为 `q_model = model_sign * q_motor + model_offset`；
`q_knee = q_hip + g_side(β)`。四关节标定顺序是 LH/LK/RH/RK，
硬件 `dm_*` 参数顺序是 LH/RH/LK/RK。
几何表来自 `v6_pair_upstream_zero_geometry_v1`（`dev/rmcs_rl@8149afda`）。
β 是 CAD 闭链 FK 估计，不是独立角度传感器数据。超出表域时记录 NaN 和无效标志，
保留电机角和连续角；不外推、不因此退出。

扫频配置关闭实测角度、相对角、误差、速度、加速度和刹停估计触发的退出功能。
这些量只记录，不自动减幅、投影目标或跳过工况。
双下、丢控立即关闭；活动电机反馈超时/无效当周期零输出，恢复后自动重试。
IMU、非活动电机及驱动状态不锁存扫描退出；保留电流/力矩编码上限。
关节扫频主机和驱动均限 **40 Nm**；端点入口两层限幅为 20 Nm。
关节入口现使用外环位置 P + 内环速度 P（`joint_control_law: cascade_pp`），
髋/膝的 `joint_position_kp=[40,40]` s⁻¹、`joint_velocity_kp=[4,4]` Nm·s/rad。
两环均在 1000 Hz 反馈周期计算；不加积分和内环速度微分。
速度目标为 `clip(dq_ref + 40*(q_ref-q), -8.5, 8.5)` rad/s，
模型力矩为 `4*(dq_target-dq) + probe`，经标定符号转换后最终限幅 ±40 Nm。
这项速度目标限幅属于级联控制器，不是实测速度越界停机；轨迹本身不自动减幅。
不触发速度目标限幅时相当于位置 PD 160/4，保留原轨迹速度前馈。
这些是基于原 120/4 数据的保守起始值，尚未实机验证最优性，也不能消除气簧的稳态误差。
原单环可显式设 `joint_control_law: pd` 并配置 `joint_kp`、`joint_kd`。
CSV 的 `host/joint_control_law` 为 1 时表示级联；为 0 时表示单环。
新增每轴 `velocity_target`、`velocity_error`、`velocity_target_limited` 及两环增益记录。
详见 `reports/wheel_leg_sweep_identification_20261027/leg_cascade_pp.md`。
高速参数可通过 `joint_turn_speeds_rad_s`、`joint_fast_turn_period_s` 修改，
模式 2 的参考范围及单程时间由 `joint_knee_beta_deg`、`joint_knee_half_periods_s` 配置。
局部力矩激励通过 `joint_probe_torque_nm` 修改。每轮启动时打印 `Joint excitation-v4` 参数摘要；
每秒关节参考/实测位置、速度、误差、力矩、探针、饱和状态和 β 的控制台打印已关闭，CSV 采集保留。
每轮正常完成时打印一次 `Sweep complete`，包含实验类型、侧别、模式、运行号和用时，
完成当周期六轴控制力矩归零。执行器 `Component 5s` 和 `Update/Skipped` 周期统计日志的调用已注释。
控制台保留扫频开始、轨迹阶段切换、完成及故障提示。

## 24 V 轮子高速实验

本车轮子在代码中是 M3508 去除原减速箱、外置 15.62:1 减速；本轮已确认供电 24 V。
[DJI 官方规格](https://www.robomaster.com/zh-CN/products/components/general/M3508?from=online-store&position=download)
给出原配减速箱在 24 V 的空载转速 482 rpm。
结合驱动代码中的原配减速比 3591/187，换算外置 15.62 后空载速度约 62 rad/s。
因此设置 65 rad/s 作为覆盖预计空载最高转速的目标上限，**不保证实测能达到 65**；
实际平台由反馈速度判断。供电压降、轮胎/传动阻力等会改变实际值。

每个轮子、每个方向依次执行：

| 段 | 参考及用时 | 用途 |
|---|---|---|
| 最高转速 | 8 s 平滑升到 ±65 rad/s，保持 8 s | 记录实际可达高速平台与电流饱和情况 |
| 自由滑行 | 直接零电流 10 s，再用 8 s 零速参考恢复 | 看高速下的减速曲线；释放之前没有速度制动 |
| 阻力平台 | ±35.75、±48.75 rad/s 各用 8 s 过渡、8 s 保持 | 实测稳态电流力矩与实际速度关系 |
| 高速动态 | 8 s 进入 ±55.25 rad/s；20 s 速度扫频，20 s 力矩探针 | 留出速度/电压余量，避免把顶速饱和当成延迟 |
| 独立验证 | 再执行不同相位的 20 s 速度扫频、20 s 力矩探针 | 按完整段验证，不能随机拆行 |
| 结束滑行 | 零电流 10 s，随后 8 s 零速参考 | 复核高速阻力及恢复 |

速度扫频为 0.3–8 Hz、总幅值 2.6 rad/s，轮子波形号 `11`；
独立力矩探针为 0.37/1.13/2.71/5.17/9.13 Hz 多正弦，叠加在速度 P 之后、最终电流限幅之前，
总幅值不超过 0.35 Nm，波形号 `9`。高速平台波形号 `10`。
训练/验证相位分别为 0.31/1.27，`validation` 明确区分。
`wheel_high_speed` 标识高速段，`wheel_high_speed_target_rad_s` 记录设定的最高目标。
第一段最高平台从运行第 11 s 开始；后续左右轮、正反向逐项执行，不会同时把两轮升到最高速。

高速阻力分析使用**反馈速度和电流估计力矩**，不能用目标速度代替；
未稳定或电流饱和的片段不当作稳态阻尼样本。
零电流滑行给出阻力/惯量的比值；没有惯量信息时，不能从减速率单独得到绝对阻尼。
延迟以编码提交时间到新电流反馈时间拟合，并用不同相位段验证；
包含主机提交、传输、驱动和测量链路影响，不能拆成纯 CAN 延迟或直接叫位置跟踪延迟。
若实际速度不足、轨迹被拨杆中断、探针因饱和被削掉，保留数据并拒绝相应辨识结果。

## CSV 字段

默认写入 `/tmp/wheel_leg_{wheels,wheels_high_speed,joints,limits}_<timestamp>.csv`。
`write_interval: 1` 记录所有执行周期，`flush_interval: 1000` 每千行刷新一次；
待机、扫描、完成、故障均记录。`strict_signals: true` 避免缺列仍继续实验。

| 字段 | 含义 |
|---|---|
| `sweep/state` | 0 初始化、1 双下关闭、2 待机、3 扫描、4 本轮完成、5 历史中断状态、6 当前输入无效 |
| `sweep/stop_reason` | 0 无、1 双下、2 丢控/UNKNOWN、3 无效档、4 双中结束、5 切换模式、6 正常完成、7 反馈超时、8 非有限数据、9 驱动故障、10 实际使能丢失/未确认 |
| `sweep/experiment`, `side`, `mode`, `run_id` | 1 轮子/2 关节/3 端点；侧别 -1/0/1=轮子/左/右；扫描模式；单进程递增运行号 |
| `stage`, `waveform`, `validation` | 轨迹段、波形及训练/验证分组。关节波形 0 准备/1 平移或保持/2 整周/3 扫频/4 旋转多正弦/5 探测/6 局部多正弦与力矩探针/7 连续开合与膝力矩探针 |
| `excitation_revision`, `stage_elapsed_s`, `stage_duration_s` | 关节激励版本 4、轮子高速版本 5；版本 3 为历史关节慢扫。记录当前段参考时间及完整段时长，用于剔除中断片段 |
| `enable_request`, `identification/enable_requested` | 控制器请求与硬件门控后的请求；均不代替实际 DM `status_code` |
| `drives_ready`, `reference_active`, `completed` | 实际启停组合是否就绪、PD 是否活动、上一次扫描是否正常完成 |
| `feedback_diagnostic` | 全六电机和 IMU 的诊断原因，沿用 stop_reason 编号；不会阻止活动电机的扫描 |
| 每电机 `angle`, `velocity`, `torque` | 原有驱动 API 坐标；反馈力矩来自驱动电流估计，不是独立力矩传感器 |
| `model_angle`, `model_velocity`, `model_acceleration` | 连续模型坐标及导数；轮子保持轮轴坐标 |
| `reference*`, `tracking_error` | 关节为位置参考及导数；轮子使用 `reference_velocity`，误差单位 rad/s |
| `torque_preclip`, `control_torque`, `saturated` | API 坐标限幅前命令、限幅后命令及标志；非活动轴为零 |
| 每电机 `probe_torque_model` | 模型坐标中叠加在 PD 后的独立力矩激励；waveform=6 使用两轴，v4 模式 2 的 waveform=3/7 仅使用膝轴；轮子 waveform=9 为独立力矩探针 |
| `host/{hip,knee}_{kp,kd,torque_limit_nm}`, `host/wheel_*` | 实际主机 PD 增益及命令力矩上限；硬件 `joint_position_kp/kd` 在本入口的纯力矩模式下不参与控制 |
| `tau_frame_api`, `tx_kind`, `tx_queued_steady_ns` | 编码帧的 API 力矩估计、命令类型、主机提交时间；非力矩帧可能为 NaN |
| `feedback_sequence`, `feedback_steady_ns` | 新反馈帧序号及主机接收时间 |
| `control_steady_ns`, `control_end_steady_ns`, `control_dt_s`, `scheduled_steady_ns` | 控制实际起止时刻、实际间隔及执行器计划时刻 |
| `reference_sequence`, `reference_elapsed_s`, `run_elapsed_s` | 200 Hz 参考更新序号、参考采样时间和实际运行时间 |
| `diagnostic_flags` | 位 0 绝对角、1 速度、2 加速度、3 跟踪误差、4 β 超范围/无效；只诊断 |
| `estimated_stopping_distance` | `dq²/(2*diagnostic_braking_rad_s2)`，只作诊断估计 |
| `probe/{low,high}/result` | 1 = 目标端点已到达（0 未到达）；保留到达时的连续模型膝角（`model_angle`）及 FK β；原始驱动角见六电机 `angle` 列 |

另记录 DM 实际状态/故障、可用温度、IMU 姿态/角速度/加速度及各自时间戳。
提交时间不代表 CAN 线上发送时刻或电机执行时刻，命令及反馈力矩也不能代替外部标定。
重回双下会清除轨迹、参考、连续角历史和探测窗口，保留 `run_id` 作为日志编号。
故障/丢控恢复后按当前档位重新开始；默认电平模式完成后保持扫描档会开始下一轮。
没有完成锁存；返回双中可保持使能、零输出待机。

## 阻尼、延迟与 PD 候选

2026-10-27 提供的 `不合适的腿扫频数据.csv` 有 66196 条完整记录、66.4749 s，
全程 `state=6`、`stop_reason=2`、`run_id=0`、`reference_active=0`。
`dr16_fresh=0`、左右拨杆均为 UNKNOWN，使能请求也为零；末尾另有一条不完整记录。
因此它不能证明 PD 偏小，也不能估计阻尼或延迟；需要先恢复遥控有效输入，
在新 CSV 确认 `state=3`、`reference_active=1`、运行号和轨迹段持续推进。

采集左右腿分别运行模式 1、2、3，保留原始 CSV。模式 1 的低速正反转
补充摩擦/阻尼和方向差异，高速整圈覆盖速度相关效应；模式 2 补充不同 β 的开合速度；
模式 3 的局部段提供双轴独立激励，高速段提供延迟复核。各模式并非同一种数据模型，
不能把整圈数据直接塞入只有局部线性载荷的回归器。

离线分析入口（Python 需要 NumPy；拟合另需 SciPy）：

```bash
python3 .script/identification/analyze_joint_sweep.py <CSV路径> \
  --fit-local --output reports/leg_sweep_debug/joint_identification.json

python3 -m unittest discover -s .script/identification -p joint_sweep_analysis_test.py -v
```

工具保留整数纳秒时间戳，以真实控制、提交及 CAN 主机接收时刻分析，
按新反馈序号去重。中断轨迹段、饱和、过期/不同步反馈不会进入局部参数回归；
验证按完整轨迹段划分，不能随机拆行来掩盖时序相关性。

- **局部机械候选**：仅在 `waveform=6`、50/75/100°附近拟合
  `M·q̈ + B·q̇ + Fc·tanh(30q̇) + l + K·(q−q0) = τ_feedback_model`。
  M 为对称正定，B 为对称半正定；保留髋膝耦合项，并检查回归条件数、完整验证段误差及
  150 ms 不重置状态的短时轨迹复现。`l,K` 合并了局部重力/气簧/静态载荷，不能解释为独立测得的气簧参数。
- **延迟候选**：对编码后 `tau_frame_api` 到电流估计反馈 `torque`，拟合纯延迟加一阶滞后、增益和偏置。
  局部段与实际速度达到高速的 `waveform=4` 段分别拟合/独立验证；不把位置跟踪滞后直接叫作执行器纯延迟。
  结果包含传输、驱动及反馈测量影响，不能拆分成单独的 CAN 或驱动执行延迟。
- **PD 搜索**：只有局部模型和局部延迟通过验证，才在耦合模型上搜索对角 PD，
  沿用当前纯 PD 公式，包含拟合延迟/一阶滞后、静态载荷造成的稳态偏差及 CSV 记录的力矩限幅（当前为 40 Nm），
  以两轴 0.05 rad 阶跃误差和饱和比例选候选。
  输出 `auto_applied=false`，不覆盖生产 YAML。采集初值 120/4 与辨识后的候选分开记录。

输出始终保留 `real2sim_ready=false`：这里新增的是**局部有效模型、等效延迟和离线 PD 候选**。
整圈真实闭链模型中的变惯量、科氏项、重力、气簧，以及落地接触需要进一步仿真复现和独立实车验证；
驱动电流估计的力矩还受力矩常数误差影响。不能把局部 B 矩阵直接全部填成 URDF 单轴 damping。

## 离线验证

```bash
build-rmcs --packages-select rmcs_core rmcs_bringup
cd rmcs_ws
colcon test --merge-install --packages-select rmcs_core
colcon test-result --test-result-base build/rmcs_core --verbose
```

新增纯状态机/轨迹/探测/DM 帧测试，以及三份实际 YAML 和右侧配置的执行器集成测试。
集成测试使用内存反馈，验证日志完整性、每控制周期一行、约 1 kHz 采集、
遥控停机、故障原因、禁用运动保护后的诊断，以及非选侧零命令。
这些测试不代表 CAN 总线时序、装夹/气簧参数或实车轨迹已验证。

2026-10-26 当前版本离线结果：两包构建通过，12 个 CTest 目标、46 个 GTest 用例通过。
三份 YAML 插件链及 228 个唯一日志信号检查通过；正式 CSV 为 236 列（含 index），
集成测试额外加入执行器计数列，为 237 列。采集约 1000 Hz，逐周期无漏行。
加速探测仅用于完成状态测试，不改变正式 YAML。

2026-10-27 激励版本 3 增加高速/低速双向平台、局部力矩探针、完整段元数据、
独立验证及离线参数/PD 候选工具。关节 CSV 新增 7 个信号，正式输出为 243 列；
轮子/端点的 CSV 格式不变。现有遥控、复位、插件和纯 MIT 力矩模式沿用当前配置。

回归覆盖启动 UNKNOWN 后直接扫描、即时模式切换、同模式进度保持、完成自动再扫、
双下关闭、活动反馈恢复，以及关节/IMU 无效时轮子发送侧仍通过非零命令。
轮子测试保持正式 YAML 的 3 s 基线段，越过该段后检查大于 0.05 Nm 的命令和发送字段。
这些结果均为内存反馈离线验证，不代表已经部署或完成实车运动验证。
