# V6 自起控制与完整 C++ 闭环验证：2026-10-03

本轮按 `v6_flat_14020` 模型对应的冻结训练合同对齐自起路径，包括轴方向、闭链几何、动作参考、PD、轮子平衡、支撑判断、接管和动作历史。冻结脚本已经移植；动态成功率以本文关联的完整倒地矩阵为准。当前 ONNX 是平地候选，不能因脚本数值对齐而认定所有倒地姿态已具备恢复能力。

## 身份与实现入口

| 对象 | 固定身份 / 路径 |
| --- | --- |
| 分支 | `dev/wheel_leg_rl` |
| 原生训练源码 | `2778206b5905c2760ccc381f2592b08e1cad611a` |
| ONNX SHA256 | `4006bf79182e14074f38c3e8f573fe1870fdfeba3fcc0760bb24cc5752161e8a` |
| 自起 profile SHA256 | `5b09a3bb27285ab7571bd133151d7118091997309ccc9d59c6b1259836dc1eef` |
| 闭链 LUT SHA256 | `c3e215187828fae328e3df3af1728897feb6695616ec8b9bdd4322433ea00282` |
| V6 资产 manifest SHA256 | `875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba` |

生产源码位于 `rmcs_ws/src/rmcs_rl/src/`：

- `v6_recovery_profile.{hpp,cpp}`：启动时验证两个 JSON 的 SHA、资产、轴序、40–105° 机构域及载荷修正目标。文件在 `models/wheel_leg/deployment/`，随 ROS 包安装。
- `v6_recovery_controller.{hpp,cpp}`：原生阶段、共同圈数动作插值、固定 200 ms 线性接管、1 s 运动命令解锁判据及有效动作历史。
- `v6_recovery_observer.{hpp,cpp}`：Eigen 闭链 FK、条件高度、连续编码器整数圈数、IMU 判据和双向轮子响应探测。
- `configuration.cpp / controller_recovery.cpp / rl_controller.cpp / observation.cpp / action.cpp`：RMCS Component 传感器端口、50/200/1000 Hz 调度、ONNX 和唯一六轴力矩出口。

普通 V6 启动也加载相同的冻结准备目标。旧 V5 自起代码保留独立回归，V6 不复用旧几何、轴符号、8 个参考姿态或力矩投影。运行时参数只在启动时读取；V6 明确设置非 `160/2.5` 的准备 PD 会被拒绝。

## 坐标、增益与动作

外部六轴统一使用 P 顺序 `[LH,LA,RH,RA,LW,RW]`；LA/RA 是第二主动轴。训练内部 C 顺序 `[LH,LA,LW,RH,RA,RW]` 只在冻结文件与接口边界转换。

四腿继续使用参考标定映射 `q_model = -q_api + [1.6,2.93,-1.6,-2.93]`，速度、模型力矩同样按 Jacobian 转换；轮侧使用输出轴单位，不重复乘 15.8。BMI088 的 X 前、Y 左、Z 上与模型机体系一致，安装外参为 Eigen 单位旋转。

| 控制项 | V6 固定值 |
| --- | --- |
| 关节 PD | `Kp=160 N·m/rad`，`Kd=2.5 N·m·s/rad`，每轴先限 ±40 N·m |
| 轮速 P | `0.6 N·m·s/rad`，轮输出侧先限 ±4.5 N·m |
| 姿态轮平衡 | pitch `8`，gyro Y `1.5`，轮速阻尼 `0.2` |
| 平衡有效倾角 | PREPARE <55°；CAPTURE <85° |
| 轨迹根轴符号 | `[+1,+1,-1,-1]` |
| 轮平衡轴符号 | `[+1,-1]`，与驱动 API 标定符号不同 |
| 自起参考 / 新反馈 PD | 200 Hz，5 ms |
| actor / 持有目标 | 50 Hz，20 ms |
| RMCS executor / 重发 | 1000 Hz，持有最近 PD 力矩 |

准备时使用冻结静载修正 `q_pd = q_geometry + hold_torque / 160`：

```text
PREPARE P4 = [-0.3207126102216724, 0.09491855918261657,
               0.32070064924300457, -0.09490058680048478]
actor q_nom = [-0.42, 0.13742282595395358, 0.42, -0.1374155762580851]
```

这两个参考用途不同。不额外向控制器注入气弹簧前馈；气弹簧由机械体/Isaac 物理提供。两主动轴以髋轴选出的同一圈数到达轨迹目标，初始第二主动轴分支由 LUT 解析。PD 使用外部连续编码器坐标，FSM 自身的参考累积角独立保留，避免反复 wrap 带来的误差积累。

## 状态如何判断和接管

每次双下失能后的双中控制会重新建立会话，从当前编码器和 IMU 选择路线，不限第一次启动。进入自起时需要中立运动命令，恢复期间保持零平移、零转向及 305 mm 高度命令。双下仍走真实硬件的失能出口。

| 原生 SELECT 判据 | 路线 |
| --- | --- |
| 倾角 <15°，条件高度 >0.20 m | PREPARE |
| 倾角 <70°，未满足上述条件 | FOLD → PLANT |
| 倾角 ≥70°，`abs(gravity.y)>0.7` | FOLD → SIDE |
| 其余大倾角 | 按 pitch 正负选择 FOLD → ORBIT → THRUST → CAPTURE |

倾角是 `acos(-gravity.z)`，pitch 是 `atan2(gravity.x,-gravity.z)`。它们由同一 IMU 姿态转换到模型机体系获得。轮速直接读取两轮电机输出轴反馈。

支撑估计先用闭链 FK、单位重力和轮圆半径得到两侧条件高度；它不是测距，也不能单凭 FK 证明触地。原生 plausible 条件是两侧有效、最低高度 >0.12 m、高度差 <35 mm、加速度模长在 6–16 m/s²、gyro 模长 <8 rad/s。脚本期轮探测以 ±0.18 N·m、每方向 3 个参考 tick 轮流作用，比较编码器加速度 <100 rad/s² 和 gyro 加速度 <90 rad/s²；双轮双向证据 TTL 为 0.6 s。

接管候选需支撑及 body-clear 判据成立、倾角 <8°、gyro <0.75 rad/s、条件高度 0.27–0.36 m、轮速对应线速度 <0.25 m/s、腿速 <2 rad/s，并满足参考到达/误差条件持续 100 ms。脚本的轮姿态闭环参与这个阶段。随后立即开始固定 200 ms 的线性力矩混合：脚本与 actor 各自限幅，再混合。actor 已在脚本期间每 20 ms 推理，没有等到接管后才启动。

纯 RL 接管后，1 s 原生稳定条件只用于解锁运动命令；它不会延迟 RL 输出。探测停止后证据可以过期，原生 BLEND/RL 的 body-clear 使用阶段与 plausible 的组合，这不等于新增接触传感器。

50 Hz 推理/动作解码之后，执行 200 Hz reference → 更新后阶段的 probe → 当前反馈 PD → 混合 → 保存有效动作历史。下一次 35D 观测的 22–27 槽读取上个策略区间末的有效参考。原生 BLEND 不清空历史。

部署的 8 s 脚本预算从双中激活计时。训练注入场景的 8 s 则包含 reset 后预设的被动释放时间；实机任意时长的双下等待不能计入新的控制会话。释放案例会分别记录这两个时间口径，不能把超出训练绝对截止时间的部署完成当作原训练预算通过。

普通 `recovery_enabled=false` 的直立 capture 仍有独立的工程入口：默认立即接管，可配置 0–300 ms smoothstep。它不改变原生自起 FSM 固定 200 ms 的线性混合。

## 验证与复现

`test/v6_recovery_reference.json` 从冻结 Torch 源码生成，覆盖 13 个传感器序列、8,445 个阶段 tick、895 个数值快照。阶段每 tick 完全一致；最大目标误差 `9.54e-7 rad`、力矩误差 `2.39e-7 N·m`、有效历史误差 `9.54e-7`。`rmcs_core` 117 项、`rmcs_rl` 163 项，共 280 项 C++ 测试通过；回执为 [unit_validation.json](artifacts/v6_cpp_recovery_20261003/unit_validation.json)。另有 720 个时刻的编码器分支、FK、支撑与探测对照，以及真实 RMCS 组件的策略节拍、history、BLEND、解锁、失联和重新选路测试。合成传感器对照不能替代动态验收。

完整动态入口：

```bash
cd /home/yukikaze/Documents/workspace/RMCS
.script/simulation/run_v6_cpp_isaac.sh --headless --recovery-matrix --repeats 5 \
  --output docs/zh-cn/artifacts/v6_cpp_recovery_20261003/matrix_16x5
```

它在 RMCS build 目录导出只读冻结训练 runtime，模型及 CAD 经 SHA 验证；后来的单环境平地 playback 编排作为独立 sidecar 记录。每次试验重置完整 18 DOF 闭链、姿态、速度，按真实双下→双中进入，包含释放落地案例。运行期不重写机器人状态，不加牵引、不固定底盘。已有 Isaac GUI、训练和 TensorBoard 不受影响。

控制器只接收 IMU、六电机反馈及遥控输入；root 高度、实际车速、接触力、闭合残差只用于评分。严格 PASS 要求完整时长且末尾连续 1 s：有效传感器、使能、纯 RL、运动命令已释放、倾角 <10°、305±20 mm、平动速度 <0.2 m/s、gyro <0.5 rad/s、两轮接触力模长均 >2 N、非轮接触 <5 N、机构有效。提前 fault 不从分母剔除。

动态结果与失败分析在正式矩阵完成后写入下节。

## 硬件边界

现有 BMI088、DM 电机、M3508 的反馈与控制端口继续复用 RMCS 硬件链；本轮只运行本机模拟传感器闭环，未 SSH 驱动机器人、发送 CAN 或做新的实机标定。V6 的 `calibration_ready`、`soft_limits_ready`、`recovery_enabled`、`recovery_profile_ready` 在实机 YAML 仍为 false。

部署保留反馈 age/skew、DM ready、API 位置连续性、实测 hinge 边界及硬件 `max_torque` 裁剪。这些是部署附加保护，并不冒充训练中的电机辨识曲线。原生轮探测未使用已提交扭矩或电流响应证明接触；其真实 CAN 时序/响应仍须验证。LUT 越界会令支撑无效，不会自动把轻微物理止挡穿透误判成 actuator fault；实测边界保护独立生效。
