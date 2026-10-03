# V6 平地模型接入与硬件映射审计：2026-10-03

本次接入对象为 `v6_flat_candidate_14020_20261003`，是完成 13,000 次平地 actor 更新、累计消费 14,020 次的研究候选。ONNX 导出与软件数值一致性可以离线验证；硬件方向、机构多点标定、六轴响应和真实倒地自起仍没有验收。本文件记录接入合同及源码能证明的边界，三个 ROS 包已构建通过，242 项 C++ 回归及原生 ABI 对照全部通过，记录见文末。

## 冻结输入

原交接目录：`/home/yukikaze/Documents/workspace/robot_rl/isaac_wheeled_rl_schedule/models/v6_flat_candidate_14020_20261003/`。原交接说明为训练仓库的 `docs/V6_FLAT_DEPLOYMENT_HANDOFF_20261003.md`。本次只读取训练源码、模型和资产，没有启动、停止或修改 Kaiser 训练、Isaac GUI、TensorBoard 或相关服务。

| 输入 | SHA-256 |
| --- | --- |
| `policy.onnx` | `4006bf79182e14074f38c3e8f573fe1870fdfeba3fcc0760bb24cc5752161e8a` |
| 原始 PT | `7e3644f595ade15f8d43315f9e5e61e4db528fd8fac8df71bcc0441e0868891f` |
| 平地物化合同 | `666f7bdbba8c305d4bedc464fd1a42ea8725245461c49d186b75ed37fe58aae7` |
| V6 232 mm / 105° manifest | `875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba` |
| 原生 200 Hz 控制合同 | `8073540d1cec0b32c6216699863b3ef53064a2d385b7110abe291d9da0e9b1ac` |
| 已匹配 manifest 的 `robot.urdf` | `12488abd510c1b6e395b88a786e9f5b542f929d10d4a3f1ce1f7b68699e35635` |

ONNX 输入为 `obs`、float32、35D，输出为 `actions`、float32、6D；使用确定性输出、batch=1，无额外运行均值归一化。策略顺序 P 为 `[LH,LK,RH,RK,LW,RW]`，LK/RK 是第二主动电机，不是被动内膝铰链。普通平地上下文为 `[1,0,0,0,0,0,0]`。交接的 `abi_test_vectors.json` 用于输入/输出一致性；建议 `atol=rtol=1e-5`。

## 模型与设备坐标

四腿参考由 `deployment/v6_pair_from_dev_rmcs_rl.json` 给出，它绑定上游 `dev/rmcs_rl` 的 `8149afda1de0f0e79dac07a6444fc3741759d5c2` 配置和闭链表。设备 API 已经过 `DmMotor` 的 reversed，bias=0；因此四腿采用：

```text
q_v6    = -I * q_api + [1.6, 2.93, -1.6, -2.93]
dq_v6   = -I * dq_api
tau_api = (-I)^T * tau_v6
nominal = [-0.42, 0.13742282595395358, 0.42, -0.1374155762580851, 0, 0]
```

这些数据复用旧编码器零点参考；不能对 API 反馈再应用一次 DM reversed，也不能重新向已经在 105° 参考姿态内部设零的电机发送设零命令。源文件仍为 `hardware_multipoint_g0_verified=false`，训练资产仍为 `active_motor_mapping_verified=false`，交接合同仍为 `hardware_motor_mapping_verified=false`。用户确认旧硬件零点和安装结构，不能替代 V6 全行程独立几何测量。

冻结 V6 `coordinate_mapping.json` 的 `joint_sign_from_v5_232mm` 规定四个腿主动轴为 −1，两个轮轴 `L_joint3/R_joint3` 为 +1。按照冻结 URDF/model_spec 递归变换关节轴，名义姿态左侧髋、辅助轴及轮轴均接近车体 +Y，右侧均接近 −Y；这是模型轴证据。

当前 WheelLeg 与上游 `8149afda` 都对两个 M3508 执行 `.set_reversed().set_reduction_ratio(15.8)`。`DjiMotor` 已将原始电机 rpm 转换为 API 轮输出侧 rad/s，将电流转换为轮输出侧 N·m。因而 `wheel_model_scale=[1,1]` 可以作为保留既有 V5 API 轮符号的源码候选，不能证明实车与 V6 模型的逐轮正方向一致。交接明确 `v6_wheel_signs_verified=false`，必须保留这个事实。本次采用 `calibration_ready=false` 明确覆盖四腿与两个轮的全部映射，不新增单独轮侧开关；四腿许可不得解释为六轴已通过。

轮速和轮力矩不重复乘除 15.8，也不向真实硬件额外注入仿真的 armature、静摩擦、粘滞阻力。左右轮在模型中的轴向相反，不能仅根据“两个驱动均 reversed”或车体 X 向前就宣布两轮符号通过。

## 40–105° 机构边界

V6 40–105° 是大小腿投影内夹角的机械域，不是四个主动输出轴的绝对角域。主动轴为 continuous；相对膝约束应保留同侧两主动轴的共同圈数。

交接的闭链参考中，可以将现有普通 RL 的线性软约束定义为主动轴差值坐标，而不将这个差值称为被动内膝角：

```text
d_left  = q_LK - q_LH
d_right = q_RH - q_RK
hinge_coefficients = [-1, +1, +1, -1]
hinge_bias         = [0, 0]
d_at_40_deg        = -0.140638855
d_at_105_deg       =  1.329574849
hinge_margin       =  0.03  # 主动轴差值坐标 rad
```

上述 min/max 按源 JSON 存储的完整九位小数保留，不补造额外实测精度。源文件为交接 `deployment/v6_pair_from_dev_rmcs_rl.json`，SHA-256 为 `da5cffb42b4760eb7bd858a7c294305469539862bd288c611666905c42684e35`。左侧取 `delta(40°)/delta(105°)`，右侧取对应 `-delta`，得到两侧同一组归一差值界。

两侧归一后的差值随内膝角单调增大，名义差值约为 0.55742 rad，处于参考表 65–70° 区间。用差值上下界约束目标与向外力矩，可以复用现有 `process_action_` 和 `apply_soft_limits_` 的投影语义；内膝角诊断仍需查单调闭链 LUT。参考表包含 30–120° 的诊断样本，并不授权命令越过 40–105° 物理域；不做外推。

这组差值边界由上游 CAD 闭链参考推导，尚非独立硬件测量。实际 106/107° 误差涉及编码器零偏与有效止挡，不能同时当作两份独立误差重复随机化或据此放宽物理边界。`soft_limits_ready` 仍应在实测确认前为 false。

## 控制与旧 V5 自起的隔离

本次原生模型使用腿 PD 160/2.5、腿端控制限幅 40 N·m，轮速度 P=0.6、轮端限幅 4.5 N·m。50 Hz 更新并持有策略目标，200 Hz 根据新反馈计算 PD，executor/CAN 以 1 kHz 持有重发力矩。训练的 200 Hz 近似没有证明与真正 1 kHz 新反馈 PD 等价。动作仍为腿裁剪 ±3 后乘 0.25 rad、轮裁剪 ±9 后乘 10 rad/s；腿目标按与当前反馈最近的整圈计算。previous action 保存裁剪后的原始动作。

平地高度命令域为 0.23–0.43 m；训练命令参考斜率为 vx 1.5 m/s²、yaw 4 rad/s²。本次 DR16 滚轮分段线性映射 `[-1,0,1] → [0.23,0.305,0.43] m`，左杆 y 为 yaw 角速度、右杆 x 为平动，右杆 y 横向命令关闭。本模型没有完成高速、跳跃、台阶和非常规倒地专项，跳跃请求保持关闭。本模型未加显式气簧/重力前馈，不能默认再加旧自起脚本补偿。

现有 `RecoveryController` 默认四腿参考姿态、根轴方向和几何属于 V5；改变 ONNX、四腿符号和 nominal 后，不得沿用 V5 自起参数。本次 V6 平地合同在 constructor 中直接拒绝 `recovery_enabled=true`，直到有独立 V6 profile、40–105° LUT/轮轴/气簧/壳体标定和倒地接管验收。旧 V5 模型及身份约束仅留作 Legacy V5 回归；这些回归不提升为 V6 自起证据。`recovery_enabled=false` 只关闭自起入口，不代表可以删除普通 PREPARE/RL 的机械软约束。

BMI088 通道沿用 `WheelLeg` 的真实回调快照和 RMCS 原生 Component 输入。名义 IMU 外参为 X前/Y左/Z上单位阵；只补偿真实安装旋转，不重复 CAD 导出旋转。安装外参 readiness、六轴反馈 freshness、双下失能、双中重新申请使能及 DM 纯力矩出口应继续保留原生语义。

## 验证范围

机器可读的源文件身份、轴向计算、四腿/轮侧映射推导及未验收项见 `artifacts/v6_flat_deployment_20261003/hardware_mapping_audit.json`。本次审计读取冻结源文件并核对 SHA、模型轴和解析公式，不进行实机动作。ONNX ABI/观测/目标/PD/API 力矩回归由部署实现的测试记录补充；静态轴向推导不属于 CAN、USB、BMI088 动态响应或实机支撑验收。

真实轮侧验收至少应记录逐轮已知物理正方向与 API 轮速符号，并核对同方向指令、电流力矩反馈和输出侧速度响应。四腿独立多点几何、物理止挡及连续角语义尚待确认。默认门控保留 false，源交接的候选/未评估身份不提升为硬件发布资格。

## 当前配置与操作

实际配置是 `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml`；
模型是 `rmcs_ws/src/rmcs_rl/models/wheel_leg/policy.onnx`。
`policy_profile: v6_flat_14020` 绑定已冻结 SHA、名义角和控制参数。
启动时校验文件 SHA 和 nominal；错用同为35D→6D的旧模型也会拒绝启动。
修改参数后重新构建安装并重启组件，参数不支持运行中热改。

已配置的操作映射如下；只有硬件标定门控完成后才产生实际控制请求。

| 操作 | 命令/状态 |
| --- | --- |
| 双下 | 四台 DM 下发 `0xFD`，轮电流清零，清会话/模式/历史 |
| 双下后双中 | 新控制会话；先驱动就绪和正立 PREPARE，再进入 RL |
| 右杆前后 | 前后平动，最大 ±0.5 m/s |
| 右杆左右 | 通用 vy 接口；当前平地模型 `vy_max=0`，无横移/转向作用 |
| 左杆左右 | yaw 角速度，最大 ±1 rad/s；左杆前后无命令作用 |
| 左中右下组合上升沿 / C 按下沿 | AUTO 与纯原地 SPIN 切换；进入方向交替，±1 rad/s，平动为零 |
| 已启动会话中左中右下持续保持 | 保持控制；上电直接此组合不会启动 |
| DR16 滚轮 | 低端0.23 m、中位0.305 m、高端0.43 m；中心死区0.08重标 |
| W/S、A/D | 前后/横向通用平动；本模型A/D横向关闭 |
| R/F 按下沿 | 高度微调 ±0.01 m，有界并在失能时清除 |
| V / Shift+V | 当前禁用跳跃 |
| 遥控失鲜、非法开关组合、传感器失效 | 清请求并失能；重新双下→双中才重试 |

SPIN 使能 opt-in 只适用于正常 RL 纯力矩图，辨识/单轮录包图保持原开关条件。
50 Hz 命令前后斜率1.5 m/s²、yaw斜率4 rad/s²；进入纯SPIN立即将平动参考清零。
高度命令经现有6 s smoothstep平滑后送入actor；这是部署侧输入整形，不能称为实测高度反馈。
BMI088及全部六轴序号/时间参与20/20/30 ms年龄、10 ms跨度检查，推理返回后输出前再次检查。
普通V6运行倾角超过45°锁存退出，默认不尝试自起。

构建与启动入口：

```bash
.script/build-rmcs --packages-select rmcs_core rmcs_rl rmcs_bringup --parallel-workers 1
source /opt/ros/jazzy/setup.bash
source rmcs_ws/install/setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-rl
```

以上启动默认只验证组件/模型并保持门控，不会绕过 `calibration_ready=false`
或 `soft_limits_ready=false`。V6轮轴逐轮方向和多点机构验收记录完成后，才更新对应参数。
新V6腿端160/2.5动态响应仍需同轨迹bag核验；不把录包PD候选参数改成策略PD参数。

## 本次软件验证回执

[完整验证记录](artifacts/v6_flat_deployment_20261003/validation.json)：
三个包构建通过；core 117、rl 125，共242项gtest零失败。
25组原ONNX对照满足atol=rtol=1e-5；七组原生Torch快照通过三个Component回归，
逐槽观测/裁剪动作按1e-5对照，双精度PD相对原生float32使用2e-4 Nm绝对误差容限。
另有120项主机辨识脚本测试通过、9项注明环境/旧PD排除的skip，
ROS容器中补验bag/实际C++ preview 46项通过，同步脚本10项通过。
本次没有新增Isaac动力学试验或实机动作；上述数值一致性不构成动态能力与硬件资格验收。

## 后续动态验证：2026-10-03

以上 242 项及“没有新增 Isaac 动力学试验”保留为初次接入阶段的历史回执。
后续已经通过生产 C++ chassis/RL/ONNX 驱动原生 V6 Isaac 自由底盘，完成261项gtest、
33,770帧actor parity、10项完整控制矩阵（9通过，430 mm高度项未通过），
以及普通正立/工程扰动/原生几何prepared端点的0/100/200 ms接管比较。
完整运行、精确结果与未验收边界见[生产C++闭环仿真记录](wheel_leg_v6_cpp_simulation_20261003.md)。
这些新增结果不提升为实机方向/多点机构标定、BMI088动态验收或完整V6倒地自起证据；
硬件门控继续保持未验收状态，V6完整自起仍禁用。
