# 小 yaw 开环辨识

本配置为 `small_yaw_identification.yaml`，与旧的 `gimbal_controller_identification.yaml` 大 yaw 辨识不同。大 yaw 电机控制关闭，需机械固定大 yaw；joint2 沿用已调好的抬升约 30° 与角度支撑，小 yaw 只输出辨识波形，角度保持和运动阻尼均关闭。当前只是辨识配置与操作说明，不代表已完成实机辨识。

开始前确认小 yaw 有转动空间，机械固定大 yaw，并准备托住机械臂。先双下确认，再解除双下；joint2 会抬升约 30°。辨识启动与运行只检查两项运动条件：joint2 达到稳定状态；小 yaw 输出轴角速度不超过 5 rad/s。joint2 稳定由目标误差不超过 10°并连续满足 1 s 判断，不再检查 joint2 速度。最近 250 ms 的 joint2 角度峰峰值仅作诊断显示，不限制启动或运行。满足稳定条件后自动进入 `settling` 并开始 1 s 零力矩基线，无需摇杆触发。小 yaw 速度由角度反馈估算；电机原生速度是电机轴数据，不参与这个输出轴限速判断。遥控/反馈失效、joint2 失去许可或控制周期异常仍会撤掉力矩。

自动开始后勿手扶运动中的小 yaw。序列先记录 1 s 零力矩基线，再依次使用 2、3.5、5、6 Nm；每档按 `+、−、−、+` 输出四个脉冲。每个脉冲 0.20 s，其后零力矩间隔 0.7 s，共 16 个脉冲；随后追加峰值 6 Nm、频率 0.15→0.60 Hz、持续 16 s 的线性扫频正弦。正弦后追加 `+4.5 Nm` 恒力矩 2 s、0.5 s 归零、`−4.5 Nm` 恒力矩 2 s，再做两组 `0→+5→0→−5→0` 的线性扫力矩，每条线性腿 4 s（原来的三分之二）。单轮总时长约 67.9 s。配置中的 `small_yaw_identification_command_sign: -1` 会反转辨识波形的下发符号；脉冲、正弦、恒力矩和扫力矩都使用该极性，电流反馈仍按实际符号估算。脉冲最大档为 6 Nm，正弦峰值为 6 Nm，恒力矩峰值为 4.5 Nm，线性扫力矩峰值为 5 Nm；较长序列会增加轴运动量，需保证运动范围内无碰撞风险。

运行中若小 yaw 输出轴角速度超过 5 rad/s 或 joint2 离开稳定条件，会中止并撤掉辨识力矩；遥控/反馈失效、joint2 失去许可、控制周期异常也会撤力。没有额外的小 yaw 相对角、摇杆位置或大 yaw 速度触发条件。查看 `small_yaw_identification_state` 与 `small_yaw_block_reason` 定位状态。完成后状态为 `identification_complete`，小 yaw 力矩为 0，不会自动拉回起始角度；重新双下确认后，joint2 稳定且小 yaw 速度低于 5 rad/s 时会再次自动开始。

**双下会同时撤掉 joint2 支撑力矩，切回双下前应托住机械臂。** 联锁失效也可能撤掉支撑，应按现场机械支撑方式操作。

Foxglove 使用 Foxglove WebSocket 连接 `ws://192.168.1.114:8765`，导入同目录 `small_yaw_identification_foxglove.json`。主要观察相对角、原始 `motor/angle`、电机速度与编码器估速、请求/下发/电流折算反馈力矩和状态。反馈力矩现通过 `/arm/joint_1/motor/torque_estimate` 明确标为估算值；旧 `/arm/joint_1/motor/torque` 保留为兼容别名。两者都由电流乘换算系数得到，不是独立力矩传感器测量；没有已知外加载荷的校准数据时，换算系数保持 3.6 Nm/A。小 yaw 辨识沿用 `small_yaw_damping_*` 力矩与速度话题名称，这不表示阻尼控制已开启。新版本另录制 `/gimbal/feedback/small_yaw_can_sample`，将同一 CAN 帧的接收时间、角度和电流打包，分析时优先使用该原子采样。

若停在 `waiting_for_joint2_stability`，同时查看 joint2 目标误差、稳定计时和以下诊断项。角度跨度及窗口状态只供观察，不参与稳定判定，也不会在运行中触发中止。

| 话题（前缀 `/gimbal/test/small_yaw_identification_`） | 类型与单位 | 含义 |
| --- | --- | --- |
| `joint2_angle_span` | Float64，rad | 最近 250 ms 角度峰峰值，仅作诊断显示，无角度跨度上限 |
| `joint2_stable_elapsed_s` | Float64，s | 连续满足 joint2 目标误差条件的计时，达到 1 s 后允许自动开始 |
| `joint2_window_ready` | Bool | 是否已有完整的 250 ms 角度历史，仅表示诊断数据已覆盖完整窗口，不是辨识门槛 |

三项诊断均已加入 YAML 的 `forward_list`、Foxglove 白名单和布局，也会随该配置的话题列表录制。修改运行配置后，应正常停止旧录制，再启动新录制以保存对应配置快照。

要得到可交叉验证的模型，至少录下 **3 轮完整辨识**；参数区间建议录 **5 轮或更多**。不要只看状态完成或净角度。先启动录制并确认 `status` 为 `recording`，再让一轮完整结束；每轮包含脉冲、正弦、恒力矩和两组线性扫力矩。需要重新双下复位时，双下会撤掉 joint2 支撑力矩，期间必须有可靠机械支撑；解除双下后等待下一轮自动开始。目标轮数完成且每轮都显示 `identification_complete` 后，先正常停止录制，再对完整 bag 运行 `.script/analyze-small-yaw-identification`。分析会给出每个阶段的角度、速度、实际编码命令和电流估算力矩；拟合同时使用软件实际下发力矩和电机反馈电流折算力矩，并用原始角度对应的 CAN 接收时间戳重建样本。轮内采样窗口彼此相关，不能拿窗口数量冒充独立重复次数。

分析结果仍是表观模型：脉冲峰值为 6 Nm，正弦峰值为 6 Nm，恒力矩峰值为 4.5 Nm，线性扫力矩峰值为 5 Nm，实际编码值经过电流量化；电机力矩反馈也由电流换算，没有独立力矩传感器。分析器只剔除拟合窗口前后各 125 ms 内大 yaw 角度跨度超过 0.01 rad 的窗口，同一轮其他时间段仍可参与拟合。`small_yaw_current_bias_amp`、`small_yaw_torque_scale_nm_per_amp` 和 `small_yaw_torque_bias_nm` 可在固定电机或已知负载下标定后填写。分析同时给出正反向摩擦差异诊断，但不会自动用于控制。辨识参数用于描述这次固定姿态下的运动响应，不能仅凭训练误差或单轮结果宣称为真实惯量/摩擦参数。

在开发容器仓库目录启动独立录制，明确指定当前远端配置与单独的保存根目录：

```sh
.script/record-gimbal start --config /rmcs_install/share/rmcs_bringup/config/small_yaw_identification.yaml --root /root/rmcs_recordings/small_yaw_identification
.script/record-gimbal status --root /root/rmcs_recordings/small_yaw_identification
.script/record-gimbal stop --root /root/rmcs_recordings/small_yaw_identification
```

录制会保存所选 YAML 原文、配置路径、SHA256、话题列表及每 30 s 滚动的 MCAP。`status` 和 `stop` 必须使用相同 `--root`；同一根目录已有活跃录制时，`start` 返回原录制，不会替换其配置。录制 helper 不切换运行配置，也不触发电机。完整读取当前写入的 MCAP 前先正常停止录制。
