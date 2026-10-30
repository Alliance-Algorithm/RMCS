# 轮腿训练代码 v4 的 RMCS 部署同步

入口为 `wheel-leg-infantry-rl`。这里的 v4 指 `WheelLeg-ProneMotion-v4`
训练代码和观测语义，模型仍为单帧 35 维输入、6 维输出。
当前冻结基准为训练提交 `fd73050348e0b20a529e7f93b4044582d5d46e1c`。
逐文件 SHA256、观测合同、几何、动作和控制参数保存在
`rmcs_ws/src/rmcs_bringup/config/contracts/wheel_leg_prone_motion_v4.json`。
按用户确认，本合同同时同步整车 v2.0.2 的训练派生版本：21 个刚体、
21.44174 kg、可动云台的模型身份、独立名义腿角、269 点编码器几何和277 点气弹簧几何。
质量、惯量和碰撞由真实机构承担，无需复制到车上的控制器。云台仍由实车已有云台
控制器负责；训练中的随机 yaw/pitch 扫描是扰动，不加入这套底盘部署配置。

## 观测和时钟

| 项目 | 部署约定 |
| --- | --- |
| 策略 / 观测 | 100 Hz，每 10 个 executor tick 提交一份完整观测 |
| 底层 PD / executor | 1000 Hz，host torque，腿 P160 / D2.5 |
| 轮速控制 | P0.5，轮端速度 rad/s，力矩帽 4.5 Nm，驱动减速比 15.62 |
| 槽位 14、15 | 左右轮式静站位移，米乘以 20 |
| 槽位 31 | 当前参考有效为 1，否则为 0 |
| 动作 | 左髋、左膝驱动、右髋、右膝驱动、左轮、右轮 |
| 腿动作 / 轮动作 | raw ±16 / ±9，经 0.25 / 10 缩放为 rad 偏移 / rad/s |

驱动在 CAN 接收侧展开连续多圈角，并完成减速比及方向换算；
`LocalStandingOdometry` 先用 double 减去本次零速请求起点，之后才转 float32。
累计角度不会直接写入网络，也不在 100 Hz 推理端展开单圈编码器。
局部角与腿编码器、IMU 使用同一份策略采样数据，不人为添加仿真的随机噪声或延迟。

静站捕获要求三轴指令绝对值均小于 0.001、策略控制有效、IMU 在直立 15 度内、
两侧编码器估计支撑高度均超过 0.20 m，并且 gyro 范数小于 0.5 rad/s、
四腿速度绝对值均小于 3 rad/s、轮速均小于 2.5 rad/s，连续满足 0.2 s。
使用训练的 40–107 度闭链几何、测量端点投影和 IMU 俯仰补偿，捕获一次锚点。
位移限幅 ±1 m，再以时间常数 50 ms、alpha=1/6 低通。

变高度、漂移、输出限幅和短暂无效测量均保留原锚点；无效时三个槽位归零。
运动请求、禁用、显式 reset 或严重倾倒（归一化 gravity z > -0.5）释放参考。
轮角跳变/非有限轮角使本次请求失效并锁存，须结束请求或显式 reset 后重建。
同一策略 tick 的重复读取不会再次增加捕获时间或更新滤波。
这些规则只产生观测；没有新增位置 PID、控制接管或自起脚本。

双 DOWN / 遥控丢失、CAN/IMU 超时、清错流程、非有限/过期推理停机、
扭矩饱和和髋膝相对力矩软限位继续生效。
奖励和仿真真值不进入部署代码。

## 模型合同与归一化

本次修改部署代码和配置，没有替换现有 ONNX。配置中的
`models/wheel_leg/prone_motion.onnx` 是后续所选 checkpoint 的导出目标。
运行前需要把该 checkpoint 的 **eval 观测归一化层及冻结统计量一同导出**；
只有 MLP 权重不能复现训练推理。当前 RSL-RL 归一化表达式为
`(obs - mean) / (std + eps)`，eps 应来自训练模块，不能擅自改为另一个值。
归一化嵌入 ONNX 后，桥仅裁剪原始观测，不再从 metadata 重复标准化。

模型应声明以下 metadata；缺项或不匹配会在启动时拒绝：

```text
policy_hz = 100
rmcs_observation_version = wheel-leg-prone-motion-v4
rmcs_observation_contract_sha256 = 6062baf37967d853446a95317aae4a0b315c4267da7387ab7ae020f37a021e35
rmcs_obs_normalization = embedded
leg_winding_version = paired_branch107_v1
leg_target_model_spec_sha256 = 7f62e0b47546a047219c078bdaa44fc2d82553af04b36b72ec1a94be29a22e18
```

嵌入归一化的模型不能同时声明外部 `rmcs_obs_mean` / `rmcs_obs_std`。
导出与 metadata 写入完成后再计算最终 ONNX SHA256，随所选 checkpoint 归档。
旧 50/200 Hz 模型不能直接套用本配置。
`wheel-leg-infantry-unified-rl` 继续保留原 200 Hz 候选合同；历史 50 Hz 路径的
兼容性测试使用固定测试 YAML，不再依赖当前部署入口。

## 离线验证

只用仓库的 `build-rmcs` 构建，测试不会构造硬件组件：

```bash
build-rmcs --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
cd rmcs_ws
colcon test --merge-install --packages-select rmcs_rl rmcs_core --return-code-on-test-failure
colcon test-result --test-result-base build/rmcs_core --verbose
```

在有 torch / ONNX / ONNX Runtime / PyYAML 的 Python 环境执行跨语言核对：

```bash
python rmcs_ws/src/rmcs_core/tools/sync_prone_motion_contract.py \
  --training-root /path/to/isaac_wheeled_rl_train --training-revision fd730503 --check
python rmcs_ws/src/rmcs_core/tools/verify_prone_motion_parity.py \
  --training-root /path/to/isaac_wheeled_rl_train --training-revision fd730503 \
  --container rmcs_devcontainer-rmcs-develop-1
```

核对程序只读提取指定训练版本，向训练 Python 和生产 C++ 输入同一组传感器记录。
记录覆盖长时累计轮角 ±1e8 rad、漂移、几何补偿变高度及俯仰、倾倒、重复 tick、
跳变锁存、禁用、运动/自旋和物理几何端点。
另外用明确标记的合成网络核对嵌入归一化后的六维动作、错误合同拒绝，
并以同组编码器角核对入场动作历史及新整车气弹簧补偿的双精度力矩/膝角；
并以 100 Hz 运行生产 controller/bridge/executor 的内存传感器集成测试。

回执默认写入 `reports/wheel_leg_prone_motion_v4/parity.json`，包含记录和测试图的 SHA256。
合成 ONNX 仅为测试数据，不安装为车辆策略。
这些结果证明离线接线和数值一致性；没有测试新训练权重、真实传感器录制、
CAN 实际时序或实车站立/自起能力。

本次离线结果：构建通过，51 项 C++ 测试、18 项 Python 观测测试通过。
4778 帧共同记录的观测最大绝对误差为 1.90735e-6，参考有效位及上下文逐值一致；
入场动作历史最大误差 1.19210e-7，合成归一化网络的端到端动作最大误差
5.96047e-8。1201 组双精度气弹簧记录的力矩最大误差为 7.10543e-15 N·m。
100 Hz 生产控制链路集成和七种错误模型合同拒绝均通过。
开发容器的 bash/zsh 配置已用 `set-robot wheel-leg-infantry-rl` 选定入口；未运行 launch。
