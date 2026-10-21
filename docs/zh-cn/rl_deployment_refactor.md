# RL 部署整合

RMCS 通过 `rmcs_ws/src/rmcs_rl` 子模块加载通用策略接口。车辆的标定、
自起恢复和电机控制由 `rmcs_core` 负责，车辆配置在 `rmcs_bringup/config`。

## 控制链

wheel-leg 使用 `WheelLegChassisController` 提供遥控命令及双下复位；
`WheelLegController` 维护标定、航向保持、自起恢复及 1000 Hz 反馈 PD，
通过 `register_output` 发布未归一化的模型坐标、命令和实际执行动作历史。
`RlBridge` 按 YAML 的 `observation_terms` 组装并序列化观测，以模型约定的频率
提交给进程内推理线程，再按 `action_terms` 缩放和裁剪返回动作。

动作发布由桥创建的 RMCS 伙伴组件执行，调度关系为：动作发布 →
底盘控制 → 观测组装/提交。伙伴组件使用前次已提交请求的推理结果，
从而保持有向无环的组件依赖；执行器线程不等待 ONNX 推理。
`/chassis/reset_count` 同时进入动作发布和观测组装，旧代次结果被拒绝。
第一帧推理尚未返回时保持 PREPARE 并输出零力矩；超时失效后停止控制。

自起逻辑跟随 `dev/wheel_leg_rl` 的 recovery 配置，包括低膝插入流程、
运动中的捕获、独立恢复轮速限制和 paired-107 绕组。恢复的实际动作历史
经 YAML 配置的各电机尺度转换后回送；策略接管继续受标定、铰链限位、
IMU/电机反馈时效及力矩边界约束。

Deformable 继续通过同一 `RlBridge` 的通用线程接口交给
`DeformableRlSuspension` / `DeformableSuspensionArbiter`，保留原有动作及历史契约。

## 配置与换模型

wheel-leg 的正式配置为 `wheel-leg-infantry-rl.yaml`。
两种底盘通过 `rl_bridge.rl_model_path` 选模型；同时核对观测、动作、频率和机械契约后重启。
相对模型路径以
`rmcs_rl` 的安装 share 目录为基准。轮腿恢复 JSON 标定文件随
`rmcs_core/config/wheel_leg` 安装，路径以 `rmcs_core` 的 share 为基准。

`rl_bridge` 声明模型、观测和动作布局、历史长度及推理时效；
`wheel_leg_controller` 声明机械标定、恢复、准备姿态和反馈 PD。
桥同时发布每个动作的尺度和限幅，控制器据此处理执行历史，避免重复缩放。
启动时检查模型张量维度、试推理、paired-107 模型与恢复标定契约。
桥和控制器的动作时效须一致；配置仅在启动时读取。

当前轮腿配置使用 `flat_directional200_1346_preview_20261020.onnx`：策略 200 Hz，
执行器/PD/恢复 1000 Hz，35 维单帧观测和 6 路动作；动作时效统一 15 ms。
直线速度上限仍为 3 m/s，旋转、航向纠偏及自转命令上限均为 4 rad/s。
控制器读取桥的策略频率，并要求它整除反馈 PD 频率；模型包含 `policy_hz` 时，
桥会拒绝与 YAML 不同的频率。未带频率元数据的原 50 Hz 模型仍可显式配置使用。

新候选导出文件缺失 paired-107 元数据，使用 `rmcs_rl/tool/prepare_wheel_leg_model.py`
核对原始预览包、训练计划/合同、机械模型 SHA 和 YAML 布局后补齐。
该工具只改部署副本元数据，保留训练包、权重与计算图，生成相邻的
`.deployment.json` 回执；再次不带 `--stamp` 运行可验证文件和 256 个单帧输入。
该候选仍为未通过部署验收的预览模型，自起代码回放一致不代表新 actor 的实车接管通过。

## 离线验证

```sh
colcon build --merge-install --symlink-install --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_rl rmcs_core
python3 src/rmcs_core/tools/verify_wheel_leg_reference.py --reference origin/dev/wheel_leg_rl --policy-receipt src/rmcs_rl/models/wheel_leg/flat_directional200_1346_preview_20261020.deployment.json
```

验证覆盖无硬件的实际组件调度、线程推理与复位，以及参考分支的硬件/
恢复参数、模型字节和恢复轨迹回放。它不代表 CAN 时序、实车运动或自起验收。
发布时先推送子模块提交，再更新并推送 RMCS 的 gitlink。
