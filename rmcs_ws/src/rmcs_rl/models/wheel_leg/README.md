# V6 平地模型部署交接：2026-10-03

交付对象是 **V6 平地研究候选**，本域完成 13,000 次 actor 更新，累计消费 14,020 次（包含历史课程与 critic 适应）。不是六任务集成模型，也没有硬件发布资格。Kaiser 已从这个模型完整继承 actor、critic、Adam，进入独立的恢复课程；本文件交付的平地文件不会随继续训练改变。

## 1. 模型与证据位置

本机模型目录：

```text
/home/yukikaze/Documents/workspace/robot_rl/isaac_wheeled_rl_schedule/models/v6_flat_candidate_14020_20261003/
```

| 文件 | 用途 |
| --- | --- |
| `policy.onnx` | 实际推理模型；确定性 actor，35D 输入、6D 输出 |
| `policy.onnx.json` | 模型、原检查点、资产、控制合同 SHA 与学习时钟 |
| `policy.onnx.contract.json` | 训练时原始物化合同，原字节保存 |
| `deployment/interface.json` | 本次交接的 ABI、名义角、动作解码、频率及适用范围 |
| `deployment/abi_test_vectors.json` | 观测→ONNX 原始输出对照；用于部署侧数值一致性测试 |
| `deployment/v6_pair_from_dev_rmcs_rl.json` | 既有 V6 编码器映射参考，保留未完成的硬件核验状态 |
| `deployment/manifest.json` | 本次 V6 232 mm / 105° 资产身份与名义关节位置 |
| `deployment/v6_wheel_response_control_200hz_v1.json` | 本次实际使用的原生控制合同 |
| `deployment/scut_observation.py`、`v5_control.py` | 冻结训练源码中的观测组装与动作解码参考 |
| `bundle_manifest.json`、`SHA256SUMS` | 包内每个交接文件的 SHA-256 |

完整交接压缩包是同目录的 `v6_flat_candidate_14020_20261003.tar.gz`。压缩包旁的 `.sha256` 验证压缩包；解压后在模型目录执行 `sha256sum -c SHA256SUMS` 验证内容。压缩包不包含已训练完成的恢复/跳跃模型。

Kaiser 原始学习状态：

```text
/home/kaiser/robot-rl-sim60/experiments/v6-task-campaign-20261001T110045Z-282358/train/flat/attempt_007/train/model_final.pt
/home/kaiser/robot-rl-sim60/experiments/v6-task-campaign-20261001T110045Z-282358/train/flat/attempt_007/train/policy.onnx
```

| 身份 | SHA-256 |
| --- | --- |
| ONNX | `4006bf79182e14074f38c3e8f573fe1870fdfeba3fcc0760bb24cc5752161e8a` |
| 原始 PT | `7e3644f595ade15f8d43315f9e5e61e4db528fd8fac8df71bcc0441e0868891f` |
| 平地物化合同 | `666f7bdbba8c305d4bedc464fd1a42ea8725245461c49d186b75ed37fe58aae7` |
| V6 资产 manifest | `875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba` |
| 控制合同 | `8073540d1cec0b32c6216699863b3ef53064a2d385b7110abe291d9da0e9b1ac` |

生成这个平地模型的源码是 `2778206b5905c2760ccc381f2592b08e1cad611a`。目前自动课程运行源码是 `dc2bc475e1a16ce3a7b0e2271699cd8765687ddb`；后者只修改调度，六份 v6 worker 合同完全保留。实际部署回执在 `reports/v6_task_campaign_20261001/deploy/launch.json`，首恢复 PPO 证据在同目录 `v7_first_ppo_verification.json`。

## 2. 当前能力与适用范围

最近平地完整 100 更新窗口（全强度延迟随机化、带探索）的前/后 0.5 m/s 速度 MAE 约 3.2/2.7 cm/s，正/反 yaw **角速度** MAE 约 0.085/0.081 rad/s。230/250/305/430 mm 高度 MAE 约 1.6/2.0/1.3/5.6 mm；停车均速约 2.2 cm/s。完整 10 s 静止窗口的峰值位移均值约 8–9 cm，少量长尾仍有 37–51 cm。这些是训练曲线统计，不能替代固定确定性验收。

本机已通过原生 ONNX 导出数值检查；原生校验最大绝对误差 `5.662441253662109e-7`。此前 13,500 消费快照完成了 12 s CPU PhysX 自由底盘回放；本次最终模型已经在本机 Isaac GUI 实时运行。GUI 使用单环境、名义评测参数和 box 平地，无外部牵引，不能据此宣称随机化全域或实车通过。

本次交付适用于平地静站、升降、正反平移、旋转、启停的接入与回放。高度训练域为 0.23–0.43 m。高速、台阶、跳跃、非常规倒地自起尚没有被这个导出模型训练完；平地 ONNX 的跳跃请求应保持关闭。与 V5.3 的资产、控制合同和评测域不同，尚无同条件结果支持“整体一定更强”。

## 3. 35D 观测 ABI

ONNX 输入名 `obs`，类型 `float32`，形状 `[batch,35]`；输出名 `actions`，形状 `[batch,6]`。部署使用 batch=1，不使用 critic 的 81D，不堆历史帧，不再追加运行均值归一化。

**策略顺序 P**：`[LH,LK,RH,RK,LW,RW]`，对应 `[L_joint1,LL_joint1,R_joint1,RR_joint1,L_joint3,R_joint3]`。这里 LK/RK 是并联机构的第二主动电机，不是被动大小腿铰链；不能把观测角替换成实际内膝角。

| 0 起始索引 | 内容与缩放 |
| --- | --- |
| 0–2 | `[vx_cmd,vy_cmd,yaw_rate_cmd]`；普通平地 `vy_cmd=0`，单位 m/s、m/s、rad/s |
| 3 | `height_cmd * 5`；是高度命令，不是实测高度 |
| 4–6 | 机体坐标系 gyro XYZ，rad/s，乘 0.5 |
| 7–9 | 单位重力方向在机体系的投影；正立应接近 `[0,0,-1]` |
| 10–15 | P 顺序角偏差：前四轴 `atan2(sin(q-q_nom),cos(q-q_nom))`，两个轮角固定 0 |
| 16–21 | P 顺序六轴输出侧速度，rad/s，乘 0.1 |
| 22–27 | 上一策略步的 **已裁剪原始动作**，P 顺序；不是角目标、速度目标或力矩 |
| 28–34 | 命令上下文；本平地模型普通模式固定 `[1,0,0,0,0,0,0]` |

所有输入最终 clip 到 ±100。部署入口 reset 清 previous action；普通 50 Hz 推理不重复清零。未来任务的上下文要随对应模型交接，目前不要用接触相位、轮力或台阶真值填这些槽。

机体系为 X 前、Y 左、Z 上；IMU 使用部署侧既有安装外参统一旋转。重力方向使用单位向量，不填 m/s² 加速度。轮速为减速箱输出侧，不再乘/除 15.8。绝对车速、地面真值高度、被动铰链、气簧位移和接触力都不在 actor 输入中。

## 4. 6D 动作与下层控制

策略输出顺序仍为 P。训练内部控制顺序 C 是 `[LH,LK,LW,RH,RK,RW]`；`C=P[[0,1,4,2,3,5]]`。部署使用 P 顺序时不要再额外置换一次。

P 顺序名义位置（rad）：

```text
[-0.42, 0.13742282595395358, 0.42, -0.1374155762580851, 0.0, 0.0]
```

四个腿原始动作裁剪至 ±3；两个轮原始动作裁剪至 ±9。

```text
desired_leg = nominal_leg + 0.25 * clipped_leg_action
leg_target  = current_leg + atan2(sin(desired_leg-current_leg), cos(desired_leg-current_leg))
wheel_target_rad_s = 10.0 * clipped_wheel_action

tau_leg_model = clamp(160*(leg_target-q_leg) - 2.5*dq_leg, -40, 40)  # N·m
tau_wheel_model = clamp(0.6*(wheel_target_rad_s-dq_wheel), -4.5, 4.5) # 轮输出侧 N·m
```

目标每 20 ms 更新并持有，下层 PD 使用当前反馈。目标角使用连续电机输出轴坐标和最近共同圈数，不能独立把髋、第二主动轴重新归零。±90 rad/s 是动作解码上限，不代表这个平地模型已学会该速度。

气簧是仿真中的被动机械力；本模型未使用预览重力/气簧前馈。部署侧不能默认再加一份气簧补偿，否则控制合同改变。轮端 4.5 N·m 和腿端 40 N·m 是本次训练的控制上限，不是 MIT 协议量程或电机额定参数。

| 层 | 本次训练/回放 | 部署交接事实 |
| --- | --- | --- |
| ONNX/策略目标 | 50 Hz | 50 Hz |
| 仿真物理与新反馈 PD | 200 Hz，dt=0.005 s | 明确的计算近似，未证明与 1 kHz 等价 |
| 现有 RMCS V5 控制 | 不属于本训练运行 | 200 Hz 算 PD、1 kHz 持有并重发力矩 |
| 用户目标/录包 | raw bag 1 kHz | 若改成真正 1 kHz 新反馈 PD，需另作同轨迹响应核验，不能仅把 CAN 发送频率称为 PD 频率 |

已辨识轮侧附加 armature、静摩擦和粘滞阻尼用于仿真；它们不是要重复注入实车的补偿力矩。腿侧 bag 尚未 qualified，四腿响应仍为显式工程先验；160/2.5 是使用的闭环参数，不能称为已完成腿电机响应辨识。

## 5. 与当前 RMCS 的差异及接入顺序

截至本次源码核对，RMCS 的 `rmcs_ws/src/rmcs_rl/models/wheel_leg/policy.onnx` 仍是 **V5 flat_12486**，SHA `ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d`。其 YAML 使用 V5 正负轴、旧名义角和旧自起几何。本次没有覆盖那个文件、修改 RMCS 配置或驱动实车。

V6 编码器参考来自 `.script/identification/calibration/v6_pair_from_dev_rmcs_rl.json`，四轴 P 顺序：

```text
q_v6  = -I * q_api + [1.6,2.93,-1.6,-2.93]
dq_v6 = -I * dq_api
tau_api = (-I)^T * tau_v6
```

这是 RMCS 驱动 **已经 reversed、bias=0 后的 API 反馈** 到 V6 的映射；驱动 raw 角不是这个 API 角，不能重复应用 reversed。参考文件的 `hardware_multipoint_g0_verified=false` 和本训练 `hardware_motor_mapping_verified=false` 保持原事实。轮侧沿用设备 15.8 输出单位；最终两个轮轴符号需与 V6 实际模型逐轴对照后填写。

部署侧需要在既有 `configuration.cpp / observation.cpp / action.cpp / policy.hpp` 内适配，继续使用 RMCS 原生 Component status/command 接口和唯一六轴力矩出口：

1. 读取交接文件、检查 SHA、35→6 接口及 `abi_test_vectors.json`；数值对照用 `atol=rtol=1e-5`。输出使用确定性均值，不随机抽样。
2. 更新模型身份约束、V6 名义角、四轴 J/offset、轮侧符号与高度域。当前 RMCS `assemble_observation_` 仍拒绝 0.35 m 以上命令；当前普通 vx 参考斜率 0.6 m/s²，而训练 `command_slew` 为 1.5 m/s²、yaw 为 4 rad/s²，需显式对齐。不能只换 ONNX。
3. 用同一 IMU/编码器快照同时产生 Python 原生 35D 与 RMCS 35D，逐槽比较；再核对六输出动作、角/轮速目标、PD 模型力矩与 API 力矩变换。优先保持原生实现，不另造传感器回读或控制输出链。
4. 相对膝止挡采用本次 V6 40–105° 的闭链/LUT定义；105° 是机械参考，106/107° 实测误差要作为标定误差处理。主动大腿轴 continuous，不按被动膝上限裁剪髋角。现有 V5 40–110° 自起轨迹/成功率不能直接转作 V6 证据。
5. 正常入口、双下 `0xFD` 失能和既有传感器 freshness 保持原生语义。先记录模型/配置身份与同轨迹响应；接管前脚本姿态及 blend 必须与 V6 机构匹配。本平地交接不启用跳跃，也不把尚未完成的恢复专项标成已验证自起。

RMCS 当前实现参考：`/home/yukikaze/Documents/workspace/RMCS/docs/zh-cn/wheel_leg_self_righting_deployment_20261002.md`。本次交接明确指出差异，不替代该文档里的硬件和完整自起验证结果。

## 6. 本机回放与后续训练

本机已启动 systemd user unit `v6-flat-wasd-14020-20261003.service`，复用 `scripts/play_v5_grounded.py` 与原生 `KeyboardCommand`。点击 viewport 获取焦点：W/S 平移、A/D 旋转、Q/E 或 T/G 升降、Space 停车、P 暂停、R 重置。默认键盘上限 0.5 m/s、1 rad/s；可调滑条，但超出训练命令域的操作不作为能力验收。此平地模型不显示 J/K 跳跃按钮。

```bash
cd /home/yukikaze/Documents/workspace/robot_rl/isaac_wheeled_rl_schedule
OPENBLAS_NUM_THREADS=1 LD_LIBRARY_PATH=/home/yukikaze/.local/lib/compat \
  TMPDIR=/home/yukikaze/.cache/kit-tmp OMNI_KIT_ACCEPT_EULA=YES \
  /home/yukikaze/isaacsim60-venv/bin/python -B scripts/play_v5_grounded.py \
  --onnx models/v6_flat_candidate_14020_20261003/policy.onnx \
  --device cpu --floor-boxes --output reports/v6_flat_manual_replay_$(date +%Y%m%dT%H%M%S)
```

自动课程为 **flat → recovery → speed → terrain → jump → stepjump**，即平地基础 → 自起/非稳态恢复 → 平地高速平移/旋转 → 地形及普通上台阶 → 原地/行进跳跃 → 跳台阶专项。高速明确在恢复后、跳跃和台阶前。flat actor=13,000 已封存，其余各 15,000 actor 更新，20k 每域消费上限及总 121,000 保留。每 500 保存，每域常驻 PPO，不等待 45 项 gate。critic 适应、脚本/blend 转移、消费与 actor 时钟分别记账；正常结束标为 unassessed，各域独立保存模型，不能称六能力已经集成。

TensorBoard 服务仅在 Kaiser；浏览器入口 `http://127.0.0.1:6006/#scalars`。新恢复 run 为 `v6-task-domains-d073fe4/recovery/attempt_000/train`；旧 flat/attempt_007 是已封存记录。定时 Codex heartbeat 仍 PAUSED，tmux 原生自动课程独立运行。读取最新 `deploy/launch.json` 确认身份，不因旧停止回执重复启动。
