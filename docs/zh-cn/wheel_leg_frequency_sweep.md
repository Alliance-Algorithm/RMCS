# wheel_leg 扫频与气簧辨识

三个独立入口位于 `rmcs_ws/src/rmcs_bringup/config`，都由硬件、
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

关节扫频与端点探测这两份配置额外打开两个选项（`latch_run: true`、`remote_timeout_s: 0.25`），
这是长时程辨识所必需的：

- **锁存启动**：出现一次有效档位即开始，之后松手回到中档、或换到别的有效档位都不会中断本轮，
  只有**双下**才是显式中止；本轮完成后停在 `Complete`，需要双下复位后再选档才会开始下一轮。
  这样 12 分钟的端点扫描或 2 分钟的扫频段不会被一次拨杆抖动打断、并从当前位置重新锚定。
- **短时丢控容错**：`dr16_fresh` 中断不超过 0.25 s 时保持最后一次有效档位继续运行；
  超过 0.25 s 仍按丢控失能（`state=6`、`stop_reason=2`）。接收机掉线更久时必须修复链路，
  软件只容忍短抖动。

轮子配置不使用这两个选项，仍是原来的电平触发语义。
注意：锁存期间操作员松手不会停机，**紧急停止只有双下**。
关节的失能、清错、使能命令使用现有驱动调度器；不要求双中才可清错。

轮子实验只使用两个轮子的角度/速度反馈，四个关节状态、IMU、
总反馈新鲜度和额外扫频心跳仅作诊断，不参与轮子发送门控。
关节实验使用所选侧的两个电机反馈；另一侧持续失能、轮子零电流。
活动电机数据缺失或非有限时当周期零输出，数据恢复后自动进入当前模式；
保留驱动电流/力矩编码上限。生产配置的门控规则保持原样。

轮子开始后先执行 **3 s 零电流基线段**，然后从左轮 0.15 rad/s 的低速平台开始。
控制台启动标识为 `remote=level-v2`，硬件发送日志为 `Sweep TX level-v2`，可据此核对部署版本。
每秒打印拨杆、新鲜度、状态/模式/运行号、轨迹段、参考和实测速度、请求力矩、
活动电机反馈年龄、驱动状态及故障码。
硬件日志同时打印 `wheel_allowed`、门控位图、编码后的力矩和 CAN0 `0x200` 的四个电流字节。
`block_mask` 的位值为：1 遥控过期、2 UNKNOWN、4 双下、8 未请求使能、16 非轮子入口。
`request=1 wheel_allowed=1 tau_req!=0` 而编码值为零时，可进一步定位电流量化或编码；
编码字节非零也不代表电机已经收到或执行，反馈速度仍需同时检查。

## 轨迹和探测

- 模式 1：先用 5 s 平滑过渡到 β=70°；以进入时整腿角为零，
  分别用 **0.2、0.6、1.5、π rad/s** 执行 0→720°→0；速度起停各平滑 1 s。
  π rad/s 的恒速平台对应 **2 s/圈**，单向两圈总用时 5 s，其中恒速 3 s。
  低速正反转用于辨别黏性阻尼、库仑摩擦及方向差异。模式 1 约 213.3 s。
- 模式 2：髋角锁定进入时的连续角。先平滑进入 β=45°，在 45°–105°间双向逐 5°移动，
  每步停 1 s；三档移动段为五次平滑曲线，峰值速度分别为 3、12、30°/s。
  随后在 50、75、100°工作点分别执行三段 30 s 局部线性扫频：
  0.1–0.8 Hz / 4°、0.8–3 Hz / 2°、3–10 Hz / 0.6°。模式 2 约 463.3 s。
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
- 轮子：复用 `wheel_multiband_v2` 的左右单轮及双轮速度平台、速度台阶、
  0.2–2 Hz / 2–8 Hz 扫频、多正弦验证和零电流滑行，按 200 Hz 取样。
  默认速度平台最高 20 rad/s、10 A 命令上限；C620 电流与原 15.62 减速比标定不变。
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

扫频配置关闭角度、相对角、误差、速度、加速度和刹停估计的限幅/退出功能。
这些量只记录，不自动减幅、投影目标或跳过工况。
双下、丢控立即关闭；活动电机反馈超时/无效当周期零输出，恢复后自动重试。
IMU、非活动电机及驱动状态不锁存扫描退出；保留电流/力矩编码上限。
关节扫频主机和驱动均限 **40 Nm**；端点入口两层限幅为 20 Nm。
关节入口的采集初值为主机 `joint_kp=[120,120]`、`joint_kd=[4,4]`，
这是尚待实测验证的采集增益；不把它标记为辨识优化结果。
高速参数可通过 `joint_turn_speeds_rad_s`、`joint_fast_turn_period_s` 修改，
局部力矩激励通过 `joint_probe_torque_nm` 修改。控制台版本标识为 `Joint excitation-v3`，
每秒打印关节参考/实测位置、速度、误差、命令/反馈力矩、探针、饱和状态和 β。
每轮正常完成时打印一次 `Sweep complete`，包含实验类型、侧别、模式、运行号和用时，
完成当周期六轴控制力矩归零。执行器 `Component 5s` 组件耗时占比日志的调用已注释。

## CSV 字段

默认写入 `/tmp/wheel_leg_{wheels,joints,limits}_<timestamp>.csv`。
`write_interval: 1` 记录所有执行周期，`flush_interval: 1000` 每千行刷新一次；
待机、扫描、完成、故障均记录。`strict_signals: true` 避免缺列仍继续实验。

| 字段 | 含义 |
|---|---|
| `sweep/state` | 0 初始化、1 双下关闭、2 待机、3 扫描、4 本轮完成、5 历史中断状态、6 当前输入无效 |
| `sweep/stop_reason` | 0 无、1 双下、2 丢控/UNKNOWN、3 无效档、4 双中结束、5 切换模式、6 正常完成、7 反馈超时、8 非有限数据、9 驱动故障、10 实际使能丢失/未确认 |
| `sweep/experiment`, `side`, `mode`, `run_id` | 1 轮子/2 关节/3 端点；侧别 -1/0/1=轮子/左/右；扫描模式；单进程递增运行号 |
| `stage`, `waveform`, `validation` | 轨迹段、波形及训练/验证分组。关节波形 0 准备/1 平移或保持/2 整周/3 扫频/4 旋转多正弦/5 探测/6 局部多正弦与力矩探针 |
| `excitation_revision`, `stage_elapsed_s`, `stage_duration_s` | 关节激励版本 3、当前段参考时间及完整段时长，用于剔除中断片段 |
| `enable_request`, `identification/enable_requested` | 控制器请求与硬件门控后的请求；均不代替实际 DM `status_code` |
| `drives_ready`, `reference_active`, `completed` | 实际启停组合是否就绪、PD 是否活动、上一次扫描是否正常完成 |
| `feedback_diagnostic` | 全六电机和 IMU 的诊断原因，沿用 stop_reason 编号；不会阻止活动电机的扫描 |
| 每电机 `angle`, `velocity`, `torque` | 原有驱动 API 坐标；反馈力矩来自驱动电流估计，不是独立力矩传感器 |
| `model_angle`, `model_velocity`, `model_acceleration` | 连续模型坐标及导数；轮子保持轮轴坐标 |
| `reference*`, `tracking_error` | 关节为位置参考及导数；轮子使用 `reference_velocity`，误差单位 rad/s |
| `torque_preclip`, `control_torque`, `saturated` | API 坐标限幅前命令、限幅后命令及标志；非活动轴为零 |
| 每关节 `probe_torque_model` | 模型坐标中叠加在 PD 后的独立力矩激励；仅 waveform=6 非零 |
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
开启 `latch_run` 的关节/端点配置完成后等待退出完成锁存，不自动重复。

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
