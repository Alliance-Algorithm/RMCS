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

上电、丢控或故障后，先双下并等到四个 DM 反馈为失能，再双中。
已识别双下复位后，允许两只拨杆先后到中档；到达双中前始终失能、零输出。
双中仅使能并发送零力矩，不保持位置；随后选模式，稳定 300 ms 后启动。

| 左 / 右 | 关节扫频 | 轮子 / 端点探测 |
|---|---|---|
| 中 / 中 | 使能、零力矩待机；运行时结束扫描 | 同左 |
| 上 / 中 | 模式 1：固定腿形，共同转动 | 零输出待机 |
| 中 / 上 | 模式 2：固定进入时的髋角，改变膝内角 | 零输出待机 |
| 上 / 上 | 模式 3：共同转动并改变开合 | 开始实验 |
| 下 / 下 | 立即失能并复位 | 同左 |
| UNKNOWN、丢控 | 立即失能，重新双下才能恢复 | 同左 |
| 其他含下档组合 | 已使能时立即失能，重新双下才能恢复；双下复位后的过渡过程保持失能等待 | 同左 |

运行时换模式立即结束当前扫描，必须重新经过双中才能启动下一次。
自然完成锁定 `Complete`，继续使能并发送零力矩，不再计算位置 PD。
停在原模式档不会重扫。轮子完成后为零电流自由滑行。
DM 使用 Kp=Kd=0 的 MIT 命令；12 bit 力矩编码可能产生不超过一个量化步长的零点残差。
硬件实验档位仅在新 YAML 的 `identification_remote_control: true` 下启用，
同时需要控制器使能请求和 20 ms 内的控制心跳；生产 YAML 默认行为保持原样。

轮子入口在 `Running` 开始后先执行 **3 s 零电流基线段**，然后从左轮
0.15 rad/s 的低速平台开始，右轮稍后进入自己的实验段。双上需要持续保持；
双中待机和基线段轮子不转均属预期。不要只保持双上不到 1 s 就判断启动失败。
控制台现在按事件打印 `ResetReady`、`Standby`、`Running`、`Fault` 等状态，
以及轮子段名、持续时间和是否零电流滑行。
故障行包含原因、拨杆、反馈新鲜度及驱动就绪状态；不会每毫秒重复刷屏。

## 轨迹和探测

- 模式 1：先用 5 s 平滑过渡到 β=70°；以进入时整腿角为零，
  分别用 0.2、0.5、1.0 rad/s 执行 0→450°→0；速度起停各平滑 1 s。
- 模式 2：髋角锁定进入时的连续角。先平滑进入 β=35°，在 35°–105°间双向逐 5°移动，
  每步停 4 s；三档移动段为五次平滑曲线，峰值速度分别为 3、6、12°/s。
  随后在 40、60、80、100°工作点分别执行 60 s 局部线性扫频：
  0.1–0.5 Hz / 5°、0.5–2 Hz / 2°、2–6 Hz / 0.5°。
- 模式 3：在 40、60、80、100°及验证工作点 50、70、90°，
  每次 120 s 正转 450°、120 s 回转，叠加整腿总幅值 3°和 β 总幅值 2°的多正弦。
  整腿频率为 0.17、0.43、0.79、1.31、2.17、3.73 Hz；
  β 为 0.23、0.61、1.07、1.73、2.41、3.91 Hz，并使用不同相位。
  最后整周正反转与 β=35°→105°→35°的大范围开合同步进行。
  激励段首尾的值、速度和加速度包络归零。
- 轮子：复用 `wheel_multiband_v2` 的左右单轮及双轮速度平台、速度台阶、
  0.2–2 Hz / 2–8 Hz 扫频、多正弦验证和零电流滑行，按 200 Hz 取样。
  默认速度平台最高 20 rad/s、10 A 命令上限；C620 电流与原 15.62 减速比标定不变。
- 端点：髋保持进入角，默认 0.5°/s、5 Nm，先寻找低端，再回撤，再寻找高端、回撤。
  每 2 s 窗口内，膝驱动命令持续 ≥4 Nm 且膝电机位移 <0.15°，记录疑似边界并回撤约 3°。
  反馈 β 有效时按 FK 曲线精确计算回撤目标；无效时在原始电机坐标中回撤。
  达到参考搜索端且停留一个窗口时，记录“参考范围已走完”，不宣称找到物理限位。

连续角由每个新反馈帧的相位差累加，初始髋膝分支按已装配 CAD 闭链选择。
主机模型坐标为 `q_model = model_sign * q_motor + model_offset`；
`q_knee = q_hip + g_side(β)`。四关节标定顺序是 LH/LK/RH/RK，
硬件 `dm_*` 参数顺序是 LH/RH/LK/RK。
几何表来自 `v6_pair_upstream_zero_geometry_v1`（`dev/rmcs_rl@8149afda`）。
β 是 CAD 闭链 FK 估计，不是独立角度传感器数据。超出表域时记录 NaN 和无效标志，
保留电机角和连续角；不外推、不因此退出。

扫频配置关闭角度、相对角、误差、速度、加速度和刹停估计的限幅/退出功能。
这些量只记录，不自动减幅、投影目标或跳过工况。
保留双下、丢控、CAN/IMU 超时、非有限传感器/控制数据、驱动故障及电流/力矩上限。
关节默认主机和驱动均限 20 Nm；端点入口两层均限 5 Nm。

## CSV 字段

默认写入 `/tmp/wheel_leg_{wheels,joints,limits}_<timestamp>.csv`。
`write_interval: 1` 记录所有执行周期，`flush_interval: 1000` 每千行刷新一次；
待机、扫描、完成、故障均记录。`strict_signals: true` 避免缺列仍继续实验。

| 字段 | 含义 |
|---|---|
| `sweep/state` | 0 待复位、1 已双下、2 待机、3 扫描、4 完成、5 中断、6 故障 |
| `sweep/stop_reason` | 0 无、1 双下、2 丢控/UNKNOWN、3 无效档、4 双中结束、5 切换模式、6 正常完成、7 反馈超时、8 非有限数据、9 驱动故障、10 实际使能丢失/未确认 |
| `sweep/experiment`, `side`, `mode`, `run_id` | 1 轮子/2 关节/3 端点；侧别 -1/0/1=轮子/左/右；扫描模式；单进程递增运行号 |
| `stage`, `waveform`, `validation` | 轨迹段、波形及训练/验证分组。关节波形 0 准备/1 平移或保持/2 整周/3 扫频/4 多正弦/5 探测 |
| `enable_request`, `identification/enable_requested` | 控制器请求与硬件门控后的请求；均不代替实际 DM `status_code` |
| `drives_ready`, `reference_active`, `completed` | 实际启停组合是否就绪、PD 是否活动、上一次扫描是否正常完成 |
| 每电机 `angle`, `velocity`, `torque` | 原有驱动 API 坐标；反馈力矩来自驱动电流估计，不是独立力矩传感器 |
| `model_angle`, `model_velocity`, `model_acceleration` | 连续模型坐标及导数；轮子保持轮轴坐标 |
| `reference*`, `tracking_error` | 关节为位置参考及导数；轮子使用 `reference_velocity`，误差单位 rad/s |
| `torque_preclip`, `control_torque`, `saturated` | API 坐标限幅前命令、限幅后命令及标志；非活动轴为零 |
| `host/{hip,knee}_{kp,kd,torque_limit_nm}`, `host/wheel_*` | 实际主机 PD 增益及命令力矩上限；硬件 `joint_position_kp/kd` 在本入口的纯力矩模式下不参与控制 |
| `tau_frame_api`, `tx_kind`, `tx_queued_steady_ns` | 编码帧的 API 力矩估计、命令类型、主机提交时间；非力矩帧可能为 NaN |
| `feedback_sequence`, `feedback_steady_ns` | 新反馈帧序号及主机接收时间 |
| `control_steady_ns`, `control_end_steady_ns`, `control_dt_s`, `scheduled_steady_ns` | 控制实际起止时刻、实际间隔及执行器计划时刻 |
| `reference_sequence`, `reference_elapsed_s`, `run_elapsed_s` | 200 Hz 参考更新序号、参考采样时间和实际运行时间 |
| `diagnostic_flags` | 位 0 绝对角、1 速度、2 加速度、3 跟踪误差、4 β 超范围/无效；只诊断 |
| `estimated_stopping_distance` | `dq²/(2*diagnostic_braking_rad_s2)`，只作诊断估计 |
| `probe/{low,high}/result` | 0 未获得、1 疑似边界、2 参考范围走完；保留对应连续模型膝角（`model_angle`）及 FK β；原始驱动角见六电机 `angle` 列 |

另记录 DM 实际状态/故障、可用温度、IMU 姿态/角速度/加速度及各自时间戳。
提交时间不代表 CAN 线上发送时刻或电机执行时刻，命令及反馈力矩也不能代替外部标定。
重回双下会清除轨迹、参考、连续角历史和探测窗口，保留 `run_id` 作为日志编号。
故障/丢控后不会自动续扫。

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

2026-10-25 本独立工作区的离线结果：两包构建通过，12 个 CTest 目标、
40 个 GTest 用例通过。三份 YAML 的插件链、227 个唯一日志信号及标定表与来源校验通过。
正式 CSV 为 235 列（含 index）；测试额外加入执行器计数列，因此为 236 列。
内存集成测试采集约 1000 Hz，检查逐周期无漏行、完成锁存、实际使能丢失、
两圈连续反馈及运动诊断不退出。加速探测只用于完成状态测试，不改变正式 YAML。

启动过渡回归还覆盖双下后先左中、先右中两种顺序，验证进入双中前保持失能、
失联后不自动续扫，以及使能后出现含下档组合立即停机。
轮子集成测试保持正式 YAML 的 3 s 基线段，越过该段后检查非零命令和对应 CSV 行；
只执行开头零电流段不能证明后续激励已运行。

2026-10-26 启动问题修复：机器人 CSV 确认双下到双中之间约 14 ms 的中/下过渡
被旧状态机锁为故障。修复在复位等待阶段接受该过渡，补充状态和轨迹段日志。
修复后独立工作区两包构建通过，12 个 CTest 目标、42 个 GTest 用例通过；
内存集成测试覆盖所有入口和两种到中档顺序，CSV 保持约 1 kHz、逐周期无漏行。
此结果不代表修复已经部署到机器人或完成实车验证。
