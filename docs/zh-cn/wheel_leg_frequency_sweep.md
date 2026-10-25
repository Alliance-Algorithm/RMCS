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

没有启动顺序、300 ms 去抖、驱动确认超时或故障/完成锁存。
同一个模式持续运行时保留轨迹进度；切换到另一个关节模式立即开始新模式。
返回双中立即零输出。每轮完成当周期零力矩；如果下一周期仍在扫频档，立即开始下一轮。
启动最初的 UNKNOWN 帧不会再使后续一直停在 `Fault`。
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
双下、丢控立即关闭；活动电机反馈超时/无效当周期零输出，恢复后自动重试。
IMU、非活动电机及驱动状态不锁存扫描退出；保留电流/力矩编码上限。
关节默认主机和驱动均限 20 Nm；端点入口两层均限 5 Nm。

## CSV 字段

默认写入 `/tmp/wheel_leg_{wheels,joints,limits}_<timestamp>.csv`。
`write_interval: 1` 记录所有执行周期，`flush_interval: 1000` 每千行刷新一次；
待机、扫描、完成、故障均记录。`strict_signals: true` 避免缺列仍继续实验。

| 字段 | 含义 |
|---|---|
| `sweep/state` | 0 初始化、1 双下关闭、2 待机、3 扫描、4 本轮完成、5 历史中断状态、6 当前输入无效 |
| `sweep/stop_reason` | 0 无、1 双下、2 丢控/UNKNOWN、3 无效档、4 双中结束、5 切换模式、6 正常完成、7 反馈超时、8 非有限数据、9 驱动故障、10 实际使能丢失/未确认 |
| `sweep/experiment`, `side`, `mode`, `run_id` | 1 轮子/2 关节/3 端点；侧别 -1/0/1=轮子/左/右；扫描模式；单进程递增运行号 |
| `stage`, `waveform`, `validation` | 轨迹段、波形及训练/验证分组。关节波形 0 准备/1 平移或保持/2 整周/3 扫频/4 多正弦/5 探测 |
| `enable_request`, `identification/enable_requested` | 控制器请求与硬件门控后的请求；均不代替实际 DM `status_code` |
| `drives_ready`, `reference_active`, `completed` | 实际启停组合是否就绪、PD 是否活动、上一次扫描是否正常完成 |
| `feedback_diagnostic` | 全六电机和 IMU 的诊断原因，沿用 stop_reason 编号；不会阻止活动电机的扫描 |
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
故障/丢控恢复后按当前档位重新开始，完成后保持扫描档会开始下一轮。

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

2026-10-26 当前版本离线结果：两包构建通过，12 个 CTest 目标、42 个 GTest 用例通过。
三份 YAML 插件链及 228 个唯一日志信号检查通过；正式 CSV 为 236 列（含 index），
集成测试额外加入执行器计数列，为 237 列。采集约 1000 Hz，逐周期无漏行。
加速探测仅用于完成状态测试，不改变正式 YAML。

回归覆盖启动 UNKNOWN 后直接扫描、即时模式切换、同模式进度保持、完成自动再扫、
双下关闭、活动反馈恢复，以及关节/IMU 无效时轮子发送侧仍通过非零命令。
轮子测试保持正式 YAML 的 3 s 基线段，越过该段后检查大于 0.05 Nm 的命令和发送字段。
这些结果均为内存反馈离线验证，不代表已经部署或完成实车运动验证。
