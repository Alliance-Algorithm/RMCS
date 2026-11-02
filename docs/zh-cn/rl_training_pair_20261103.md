# 2026-11-03 两路轮腿训练部署

配置对应本机 standard-getup + standing-bridge 的 ProneMotion-v4 训练，以及 Kaiser
`full_flat_1024_20000_200hz_balanced_seed119_011` 的 WheelBipe foundation-v2 训练。
训练来源和接口合同固定在
[`wheel_leg_training_pair_20261103.json`](../../rmcs_ws/src/rmcs_bringup/config/contracts/wheel_leg_training_pair_20261103.json)。

| 项目 | 本机自起 ProneMotion-v4 | Kaiser WheelBipe |
| --- | --- | --- |
| 配置 | [`wheel-leg-infantry-prone-v4.yaml`](../../rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-prone-v4.yaml) | [`wheel-leg-infantry-wheelbipe.yaml`](../../rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-wheelbipe.yaml) |
| 策略 / host PD | 100 / 200 Hz | 50 / 200 Hz |
| 腿 P / D | 160 / 2.5 | 60 / 2 |
| 电机气簧力矩补偿 | 开启，gain=1 | 关闭，gain=0 |
| 实体气簧 | 需要，与训练机械模型一致 | 需要，与训练机械模型一致 |
| 起始姿态 | 自起由策略控制六轴，无额外脚本 | 支撑直立启动；没有完整自起能力验收 |
| 策略关节顺序 | 左髋、左膝驱动、右髋、右膝驱动、左轮、右轮 | 左髋、右髋、左膝驱动、右膝驱动、左轮、右轮 |
| 腿 / 轮动作缩放 | 0.25 / 10 | 0.5 / 10 |
| 输入归一化 | ONNX 内嵌训练统计量；14/15/31 保留固定物理尺度 | 无经验归一化 |
| 观测 14/15/31 | 站立局部位移 ×5/m、有效位；捕获 50 ms | 0 / 0 / 0；尾部为正常模式 `[1,0,0,0,0,0,0]` |
| 模型 | `models/wheel_leg/prone_motion_v4_100hz.onnx` | `models/wheel_leg/wheelbipe_v202_50hz.onnx` |

两者都使用 1 kHz RMCS 调度器，每 5 ms 重算一次反馈 PD，在间隔内保持力矩。
轮速度 P=0.5，腿/轮力矩上限为 40/4.5 N·m。WheelBipe 的原始动作历史保持未限幅值，
实际轮速目标单独限制至 ±90 rad/s；腿角观测不做模 2π 处理。自起配置沿用训练的
`paired_branch107_v1` 目标投影，WheelBipe 使用完整 40°–110° 的逐侧目标投影。
机械/编码器参考均保留 40°–110° 机构范围及 110° 置零校准。

这些差异同时需要对应的控制器和 `rmcs_rl` 代码。本次已补齐新 profile、观测、动作历史、
模型元数据校验和当前几何表；部署时应一起构建，不能只把 YAML 拷入旧二进制。
接口校验会拒绝不同观测合同、频率、目标绕组或归一化方式的模型。

## 构建和选择入口

在 RMCS 开发容器中执行：

```sh
build-rmcs --packages-select rmcs_core rmcs_rl rmcs_bringup
source /workspaces/RMCS/rmcs_ws/install/setup.bash
```

选择本机自起配置：

```sh
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-prone-v4
```

选择 WheelBipe 配置：

```sh
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-wheelbipe
```

也可用 `set-robot wheel-leg-infantry-prone-v4` 或 `set-robot wheel-leg-infantry-wheelbipe`
选择后再运行 `launch-rmcs`。本次没有修改默认机器人选择或启动硬件。
两路均要求遥控器先双 DOWN 复位/准备，再双 MIDDLE 请求进入策略，并等待驱动就绪；
双 DOWN、遥控器断开、CAN/IMU 超时及推理失败的处理继续生效。WheelBipe 当前配置
按训练早期命令范围限制至 ±0.5 m/s、±0.5 rad/s；更高课程阶段须连同被评测模型重新选定范围。

## 模型快照和验证边界

随配置提供的是部署接口候选快照：本机 `model_00500.pt`（完成 500 次 PPO 参数更新），
WheelBipe `model_500.pt`（保存索引 500，实际完成 501 次参数更新）。Kaiser 文件按源端
SHA-256 `03508ceb13805ce7bdb167d01fa548779027a993c5633652164f11b6c4240be3`
校验，只读复制后导出；该候选尚不能作为 WheelBipe 已学成的策略。
两路训练总预算分别为 10000 / 20000 次更新，当前均未完成。
每个 ONNX 旁的 `.deployment.json` 记录来源、更新数、SHA-256 和数值误差，
明确标记 `behavior_qualified=false`、`hardware_tested=false`。

更新模型使用
[`export_wheel_leg_training_pair.py`](../../rmcs_ws/src/rmcs_rl/tool/export_wheel_leg_training_pair.py)，
传入对应运行的已落盘 checkpoint；Kaiser 文件须先在源端读取 SHA-256，再只读复制并提供
`--wheelbipe-checkpoint-sha256` 与 `--wheelbipe-source-path`。导出器将训练归一化一起冻结，
检查 1024 组 ONNX 推理与训练 actor 一致性，不修改训练进程或学习状态。

离线回放工具为
[`verify_wheel_leg_training_pair.py`](../../rmcs_ws/src/rmcs_core/tools/verify_wheel_leg_training_pair.py)：

```sh
python rmcs_ws/src/rmcs_core/tools/verify_wheel_leg_training_pair.py \
  --training-root /home/noir/Documents/workspace/isaac_wheeled_rl_train
```

它覆盖 4778 帧自起观测、256 帧 WheelBipe 观测、1201 组气簧力矩，以及生产 C++ ONNX
推理路径；本地回执在 `reports/wheel_leg_training_pair_20261103/parity.json`。
两路 YAML 还经过实际组件调度、内存传感器及线程推理集成检查，包括复位和超时故障。
这些验证只确认配置和部署代码匹配训练接口。车辆轮向、IMU 安装矩阵和杆臂仍需实车核对，
气簧实测区间外的曲线及右侧对称继续属于模型假设；本次没有进行 CAN 驱动或实车动作验收。
