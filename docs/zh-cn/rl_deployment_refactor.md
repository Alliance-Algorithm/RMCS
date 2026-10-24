# RL 部署整合

RMCS 通过 `rmcs_ws/src/rmcs_rl` 子模块加载通用策略接口。车辆标定、
电机控制和执行边界由 `rmcs_core` 负责，车辆配置在 `rmcs_bringup/config`。
轮腿自起交给 RL 策略，宿主端不再执行单独的自起脚本。

## 控制链

wheel-leg 使用 `WheelLegChassisController` 提供遥控命令及双下复位；
`WheelLegController` 维护标定、命令及反馈 PD，通过 `register_output`
发布模型坐标和动作历史。旧配置由 `RlBridge` 按 YAML 组装观测；
新的 unified 配置由控制器生成完整 35 维帧，桥只路由这帧观测、提交线程推理，
再按 `action_terms` 缩放和裁剪返回动作。

动作发布由桥创建的 RMCS 伙伴组件执行，调度关系为：动作发布 →
底盘控制 → 观测组装/提交。伙伴组件使用前次已提交请求的推理结果，
保持有向无环的组件依赖；执行器线程不等待 ONNX 推理。
`/chassis/reset_count` 同时进入动作发布和观测组装，旧代次结果被拒绝。

双下复位后，双中启动策略。PREPARE（状态 2）只等待驱动就绪及第一帧有效动作，
期间输出零力矩；显式请求状态 2 也不会主动调整腿姿。第一帧有效动作到达后
直接进入 RL（状态 3），六个执行器均使用策略目标。宿主端不再运行收腿、
插入、旋转、支撑探测、直立捕获或恢复混合流程，也不再用直立门槛限制策略启动，
或因机身倾斜超过 45° 中断策略。

运行时只保留输出力矩限幅和大小腿相对夹角软限位：动作超过输出范围直接钳位，
实测夹角越界只投影掉继续越界的相对力矩，保留向内恢复及两轴共同转动，RL 保持接管。
编码器绝对角、按速度推断的角度跳变、传感器序号/时差、加速度样本时效和控制周期
不再触发停机；有限的速度/高度指令按配置范围钳位，不因超范围而退出。
保留启动标定及模型合同检查、遥控双下和失联、CAN/IMU 接收超时、非有限控制数据、
推理失效及动作过期处理。这些失效仍清零输出并锁存，须双下复位。
驱动未就绪时保持零力矩及使能请求，沿用 clear-error/enable 重试，不设一秒锁死条件。
复位清除目标、动作历史、连续角度反馈状态、航向保持及输出；
`auto_enter_rl` 不会在手动复位后自行重新使能。

反馈诊断发布在 `/wheel_leg/rl/feedback/*`，故障锁存为
`/wheel_leg/rl/fault_latched`。原 `/wheel_leg/rl/recovery/*` 和
`/wheel_leg/rl/v6_takeover/*` 接口、恢复 JSON、轨迹生成及专用回放工具已移除。
历史 `reports/` 保留当时的验证记录，不作为当前接口说明。

Deformable 继续通过同一 `RlBridge` 的通用线程接口交给
`DeformableRlSuspension` / `DeformableSuspensionArbiter`，保留原有动作及历史契约。

## 配置与换模型

旧 wheel-leg 配置为 `wheel-leg-infantry-rl.yaml`，新自起策略使用
`wheel-leg-infantry-unified-rl.yaml`。两种底盘通过
`rl_bridge.rl_model_path` 选模型；相对路径以 `rmcs_rl` 的安装 share 目录为基准。
`rl_bridge` 声明模型、观测和动作布局、历史长度及推理时效；
`wheel_leg_controller` 声明机械标定、反馈时效和反馈 PD。
桥同时发布每个动作的尺度和限幅，控制器据此处理动作历史，避免重复缩放。
启动时检查模型张量维度、试推理及 paired-107 模型契约。
桥和控制器的动作时效须一致；配置仅在启动时读取。

旧配置保留 `recovery_candidate.onnx` 及其 50 Hz、35 维单帧合同。
新配置选择 `rmcs_rl/models/wheel_leg/self_wake_V1.0.onnx`，200 Hz 策略、
1000 Hz host torque PD、35 维单帧观测和 6 路动作，动作时效沿用包内的 60 ms。
模型 SHA256 为 `48d4880f2b928512be54bea0e4b24f09fc3c47057c3c36dba87066ab34d7e443`。
默认指令高度改为 0.305 m，关闭额外航向保持，不叠加动作低通。

## unified 自起模型的观测与动作

观测维度没有增加；零起始槽位 14、15 从恒零改为相对站立轮坐标位移：

| 槽位 | 内容 |
| --- | --- |
| 0–3 | vx、vy、yaw 指令，高度 ×5 |
| 4–9 | 机体系角速度 ×0.5，单位重力方向 |
| 10–13 | 四腿相对名义角，atan2(sin,cos) 绕回 ±π |
| 14–15 | 相对站立轮坐标位移，米 ×20 |
| 16–21 | 六轴模型角速度 ×0.1 |
| 22–27 | 上次实际进入 PD 的 raw action |
| 28–34 | 固定 [1,0,0,0,0,0,0] |

位移计算使用轮侧连续多圈角、腿角、IMU 重力和冻结的闭链几何表。
轮驱动已启用多圈反馈，并已处理 15.62 减速比；控制器不能再次换算减速比，
也不能把轮角直接填进观测。`standing_model_geometry.hpp`、
`standing_reference_v3.hpp`、`unified_observation.hpp` 原样移植自导出包指定实现。
位移参考按训练静站门捕获，150 ms 滤波、±0.25 m 裁剪；reset、进入 RL、
退出静止请求或严重倾倒时清除，几何无效时输出零并保留参考。
50 ms 捕获比较沿用训练 float32 运算，首次可用为第 11 个策略周期，即 55 ms。

完整帧通过 `/wheel_leg/rl/input/frame` 和单调递增的 `frame_sequence` 发布。
桥将数组观测的标量调试接口命名为
`/wheel_leg/rl/observation/frame/element_0` 至 `element_34`，CSV 及话题转发使用相同名称。
数组下标带 `element_` 前缀，以满足 ROS 话题每段不能以数字开头的要求。
桥每个序列只提交一次，重复帧不能刷新动作时间；序列倒退锁存拒绝，须会话复位。
进入 RL 前，用测得的可行成对腿角播种历史，两轮历史为零；
新推理结果实际执行 PD 后才更新历史。复位清空整帧、历史及站立参考。

为支持自起动作，四腿 raw 限幅从 ±3 放宽到模型要求的 ±16，乘 0.25 后为
±4 rad 目标偏移；轮子仍为 raw ±9、乘 10 后为 ±90 rad/s。
目标继续按 paired-107 成对绕组及闭链范围处理，腿 PD 为 160/2.5，
腿/轮力矩帽仍为 40/4.5 N·m。力矩饱和或夹角越界都不再退出 RL；
两侧分别按 YAML 中的 hinge_min/hinge_max/hinge_margin 限制向外相对力矩。
不同电机力矩帽通过同比缩放保持软限位方向，避免单轴截断重新产生向外分量。
普通配置切换到 motor position PD 时，软限位介入的周期转发同一组已限幅 host 力矩；
连续目标超出 MIT 位置编码范围也使用 host 力矩，不因编码范围退出。
车辆零位、轴向及 40 ms 的 CAN/IMU 接收超时仍保持生效，独立加速度样本仅作诊断。
倾倒和倒置可直接进入 RL；状态 2 仅握手并输出零力矩。

模型及预处理绑定观测合同 SHA256：
`db0fc6121c73121dcd39eef0524ecfeab4b3aa69aa7f5700d4205499af2402a9`。
启动拒绝错误模型合同、频率、历史、整帧路径、动作缩放或旧预处理模式，
也拒绝 unified 模式下的航向保持和 motor position PD。
本次是在已清理脚本的当前代码上合并观测与桥接实现，
回执记录当前源码哈希，不声称代码提交与导出包的隔离工作区完全一致。

构建并加载当前工作区后，车辆入口为：

```sh
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-unified-rl
```

新模型是实验候选，导出包仍标记 `behavior_qualified=false`、`hardware_tested=false`。
本次仅完成部署及离线接口验证，尚未进行 CAN、物理仿真或实车自起验收。

`rmcs_rl/tool/prepare_wheel_leg_model.py` 可核对已有定向预览包的训练合同、
机械模型 SHA 和 YAML 布局后补齐部署元数据，生成相邻 `.deployment.json` 回执；
它只验证对应模型的部署合同，不代表新策略的行为验收。
新模型使用 `rmcs_core/tools/verify_wheel_leg_unified.py`，直接核对导出包、
当前 YAML、ONNX 元数据、256 组冻结推理输入及三个冻结预处理文件，
不修改 ONNX，也不创建硬件组件。例如使用已安装 NumPy、ONNX Runtime、PyYAML 的 Python：

```sh
python rmcs_ws/src/rmcs_core/tools/verify_wheel_leg_unified.py \
  --source-package /home/noir/Documents/workspace/isaac_wheeled_rl_train/models/unified_getup115_test_200hz_20261023 \
  --receipt /tmp/wheel_leg_unified_deployment.json
```

## 离线验证

在开发容器内构建，并在 `rmcs_ws` 目录运行测试：

```sh
build-rmcs --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_rl rmcs_core
colcon test-result --test-result-base build/rmcs_core --verbose
colcon test-result --test-result-base build/rmcs_rl --verbose
```

测试使用内存传感器及实际组件调度、线程推理，检查倾倒/倒置姿态直接接入策略、
首帧等待、复位、反馈超时停机，以及饱和力矩、上下夹角越界、多圈反馈和高速度下
RL 持续接管及软限位力矩方向；也覆盖超过旧一秒阈值的驱动就绪等待。
unified 集成测试同时加载真实
`ValueBroadcaster`，验证部署配置中观测话题的创建。它不代表物理仿真、CAN 时序、
实车运动或自起行为验收。发布时先推送子模块提交，再更新并推送 RMCS 的 gitlink。

2026-10-24 限幅处理修改后，`rmcs_core`、`rmcs_bringup` 构建通过；本次选定的
五组轮腿测试共 24 项通过，普通配置的电机位置 PD 模式另运行 1 项集成测试通过。
导出侧 12 项合同测试及 256 组 ONNX 冻结输入校验通过，最大动作误差 1.91e-6。
源码哈希、测试 XML 和日志见
[`reports/wheel_leg_soft_limits_20261024/`](../../reports/wheel_leg_soft_limits_20261024/README.md)。

当前移植另行回放了训练侧 35,158 帧冻结观测（35,137 帧有效），
完整 35 维最大误差 4.77e-7，首帧测量历史最大误差 1.91e-6，
无效帧拒绝完全一致。256 组 ONNX 冻结输入最大动作误差为 1.91e-6。
实际控制器/桥/执行器集成测试使用新 ONNX，验证整帧序列化与已执行历史，
不会因缺少可选环境变量而跳过。回执见 `reports/wheel_leg_unified115_deployment/`。
