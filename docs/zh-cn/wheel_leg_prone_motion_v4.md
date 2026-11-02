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

## 110° 编码器标定与 40–110° 机械限位

2026-10-30 用户纠正：四个 DM 驱动原有内部零点对应的大小腿内夹角为
**110°**，旧 105° 说明和对应软件偏置错误。当前 `wheel-leg-infantry-rl`
已使用 `config/contracts/wheel_leg_calibration110.json`，不发送重新置零命令。
绝对髋参考继续沿用左 `+1.6 rad`、右 `-1.6 rad`；按整车 v2.0.2 的闭链
几何求出左右辅助驱动零点，得到：

```text
q_model = -q_api + [1.6, 3.0282761689183353, -1.6, -3.0283102010168]
```

这是被动膝夹角到主动驱动角的非线性换算，不能直接给电机偏置加减 5°。
四路 API 角均为零时，左右膝几何均还原为 110°。
40–110° 对应的两侧主动角差分别为：

| 侧 | 主动角差定义 | 40° 下界 / rad | 110° 上界 / rad |
| --- | --- | --- | --- |
| 左 | `q_LK - q_LH` | -0.14089326779459427 | 1.4282761689183354 |
| 右 | `q_RH - q_RK` | -0.14084732125908661 | 1.4283102010167998 |

`hinge_min/max` 使用这些机械边界，保留主动角差 `0.03 rad` 的软限位缓冲；
进入缓冲区或越界时仅去掉继续向外的相对力矩，向内回收和共同旋转仍有效。
反馈绕圈恢复保留真实 110° 测量，不把它夹到动作目标边界。

气弹簧原几何止于 108.5°；部署用相同冻结 CAD 求解器补上 108.75–110.5°
的 8 个点，形成 285 点表，覆盖 110° 标定姿态和完整机械限位。
原 277 点和测得力曲线逐项保留，未测区仍采用原有端点趋势估计。
生成工具同时记录 CAD/求解器 SHA256、闭环位置和速度映射残差。

策略训练合同的 `paired_branch107_v1` 动作投影及静站观测有效域仍为原来的
40–107°。110° 是编码器标定姿态和机械上界，不代表策略已经训练到该范围；
超出训练几何有效域时，位移观测及有效标志按原合同清零。
训练资产和模型 metadata 不随这次部署标定修正而改写。

可独立复算部署标定：

```bash
python rmcs_ws/src/rmcs_core/tools/sync_wheel_leg_calibration.py \
  --training-root /path/to/isaac_wheeled_rl_train --check
```

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

上述数值为标定修正前的 v4 同步回执。110° 标定修正的复核另存于
`reports/wheel_leg_calibration110_20261030/`，包含原训练域的数值对照、扩展部署
几何的独立对照，以及零反馈 110° / 下界 40° 的生产控制链路测试。
该修正构建通过，53 项 C++ 用例通过；100 Hz 内存传感器链路及 4778 帧观测
对照通过，新增 401 组补偿范围对照的最大力矩误差为 1.38778e-16 N·m。
这是离线结果，未启动实车或发送驱动置零命令。
