# RL 部署整合

RMCS 通过 `rmcs_ws/src/rmcs_rl` 子模块加载通用策略接口。车辆标定、
电机控制和执行边界由 `rmcs_core` 负责，车辆配置在 `rmcs_bringup/config`。
轮腿自起交给 RL 策略，宿主端不再执行单独的自起脚本。

## 控制链

wheel-leg 使用 `WheelLegChassisController` 提供遥控命令及双下复位；
`WheelLegController` 维护标定、航向保持及反馈 PD，通过 `register_output`
发布未归一化的模型坐标、命令和策略动作历史。
`RlBridge` 按 YAML 的 `observation_terms` 组装并序列化观测，以模型约定的频率
提交给进程内推理线程，再按 `action_terms` 缩放和裁剪返回动作。

动作发布由桥创建的 RMCS 伙伴组件执行，调度关系为：动作发布 →
底盘控制 → 观测组装/提交。伙伴组件使用前次已提交请求的推理结果，
保持有向无环的组件依赖；执行器线程不等待 ONNX 推理。
`/chassis/reset_count` 同时进入动作发布和观测组装，旧代次结果被拒绝。

双下复位后，双中启动策略。PREPARE（状态 2）只等待驱动就绪及第一帧有效动作，
期间输出零力矩；显式请求状态 2 也不会主动调整腿姿。第一帧有效动作到达后
直接进入 RL（状态 3），六个执行器均使用策略目标。宿主端不再运行收腿、
插入、旋转、支撑探测、直立捕获或恢复混合流程，也不再用直立门槛限制策略启动，
或因机身倾斜超过 45° 中断策略。

标定就绪、paired-107 绕组、实测铰链限位、IMU/电机反馈时效、驱动就绪、
动作时效及力矩边界仍然约束执行。失效后停止控制并锁存故障，须双下复位。
复位清除目标、动作历史、连续角度反馈状态、航向保持及输出；
`auto_enter_rl` 不会在手动复位后自行重新使能。

反馈诊断发布在 `/wheel_leg/rl/feedback/*`，故障锁存为
`/wheel_leg/rl/fault_latched`。原 `/wheel_leg/rl/recovery/*` 和
`/wheel_leg/rl/v6_takeover/*` 接口、恢复 JSON、轨迹生成及专用回放工具已移除。
历史 `reports/` 保留当时的验证记录，不作为当前接口说明。

Deformable 继续通过同一 `RlBridge` 的通用线程接口交给
`DeformableRlSuspension` / `DeformableSuspensionArbiter`，保留原有动作及历史契约。

## 配置与换模型

wheel-leg 的配置为 `wheel-leg-infantry-rl.yaml`。两种底盘通过
`rl_bridge.rl_model_path` 选模型；相对路径以 `rmcs_rl` 的安装 share 目录为基准。
`rl_bridge` 声明模型、观测和动作布局、历史长度及推理时效；
`wheel_leg_controller` 声明机械标定、反馈时效和反馈 PD。
桥同时发布每个动作的尺度和限幅，控制器据此处理动作历史，避免重复缩放。
启动时检查模型张量维度、试推理及 paired-107 模型契约。
桥和控制器的动作时效须一致；配置仅在启动时读取。

当前 YAML 仍指向已有的 `recovery_candidate.onnx`，50 Hz 策略、1000 Hz PD、
35 维单帧观测、6 路动作和 60 ms 动作时效。本次清理未替换 ONNX，
也未证明该已有模型具备新的 RL 自起能力。接入新策略时须一起核对观测、动作、
频率、历史及机械契约，并验证自起行为。

`rmcs_rl/tool/prepare_wheel_leg_model.py` 可核对已有定向预览包的训练合同、
机械模型 SHA 和 YAML 布局后补齐部署元数据，生成相邻 `.deployment.json` 回执；
它只验证对应模型的部署合同，不代表新策略的行为验收。

## 离线验证

在开发容器内构建，并在 `rmcs_ws` 目录运行测试：

```sh
build-rmcs --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_rl rmcs_core
colcon test-result --test-result-base build/rmcs_core --verbose
colcon test-result --test-result-base build/rmcs_rl --verbose
```

测试使用内存传感器及实际组件调度、线程推理，检查倾倒/倒置姿态直接接入策略、
首帧等待、复位、反馈过期和铰链越界停机。它不代表物理仿真、CAN 时序、
实车运动或自起行为验收。发布时先推送子模块提交，再更新并推送 RMCS 的 gitlink。
