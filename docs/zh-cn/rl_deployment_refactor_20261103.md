# 轮腿部署职责整理（2026-11-03）

> 本文保留当时的实现与验证记录。当前接口、入口及源码结构以[轮腿重构说明](wheel_leg_refactor.md)为准；文中的退役入口不再可用。

本次在两路训练部署配置上整理 RMCS 和 `rmcs_rl` 的职责。参考仓库中的
`OmniWheelController`、`SteeringWheelController` 及现有硬件 partner 组件写法：
接口由构造函数直接注册，生命周期函数组织流程，私有函数处理具体控制步骤，
相关状态使用小型结构表达。硬件仍由原有 `WheelLeg` 组件负责。

## RMCS

- [`deployment_contract.hpp`](../../rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/deployment_contract.hpp)
  集中描述四种已支持的部署合同：观测版本、模型身份、目标绕组、PD 频率、动作缩放与限幅、
  气簧前馈要求。`PolicyProfile` 继续表示名义姿态、控制增益及命令范围。
- [`configuration.cpp`](../../rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/configuration.cpp)
  负责参数加载和桥接合同校验；`before_updating()` 调用它完成启动检查。
  [`wheel_leg_controller.cpp`](../../rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/wheel_leg_controller.cpp)
  保留状态切换、故障锁存和执行流程，将首次控制时钟配置提取为具体函数。
- [`unified_observation.hpp`](../../rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/unified_observation.hpp)
  用一个 `Version` 代替三个可独立修改的版本开关。选择版本时清空参考状态并同步里程计规则，
  旧 v4 与里程计 v2 的捕获时长不会互相残留。
- 两路新 YAML 使用完整的 35 维帧和显式动作索引，删除未被这些路径使用的
  `joint_names`、`joint_base_path`、`default_joint_pos`。关节顺序仍由控制器帧定义及
  `action_terms` 固定；机械校准、遥控参数和记录接口保持原值。

## rmcs_rl

- [`config.hpp`](../../rmcs_ws/src/rmcs_rl/include/rmcs_rl/rl_bridge/config.hpp)
  保存普通 `PolicyConfig`。加载配置只解析参数，模型和推理线程由 `RlBridge` 直接拥有的
  `PolicyWorker` 创建和销毁，消除配置结构对运行中 worker 的共享所有权。
- [`rl_bridge.cpp`](../../rmcs_ws/src/rmcs_rl/src/rl_bridge.cpp)
  中的 `PolicyActionPublisher` 继续作为 partner 组件解决执行图的先后关系；它自己负责
  动作映射、NaN/Zero/Hold 输出和诊断发布，提供 `publish()` / `reset()`。
  `RlBridge` 通过这两个接口操作它，移除 friend 和对其私有输出的逐项写入。
- 接受的推理结果使用一个 `PolicyWorker::Result` 保存动作、序号、generation、上下文及
  时间戳，替代多组并行成员和一次动作复制。历史反馈继续使用可执行的映射动作。
- 删除已搬入 publisher 的自由输出函数、未使用的完成时间戳、动作缓冲和类型别名。
  同步策略执行、ONNX 校验及其它机器人模型保持原有入口。

## 验证

构建使用仓库的 `build-rmcs`，构建包为 `rmcs_core rmcs_rl rmcs_bringup`。
测试采用内存传感器和实际 executor / controller / bridge / ONNX 路径，未构造硬件组件。

| 验证 | 结果 |
| --- | --- |
| CTest | RMCS 10/10，rmcs_rl 2/2 |
| GoogleTest 用例（不重复计 CTest 包装项） | RMCS 47/47，rmcs_rl 15/15 |
| 重构前后观测回放 | 4 种模式，每种 4778 帧，文本输出逐字节一致 |
| Prone 观测相对训练实现 | 4778 帧，最大绝对误差 1.78814e-7 |
| WheelBipe 观测相对训练实现 | 256 帧，最大绝对误差 0 |
| 气簧力矩相对训练实现 | 1201 组，最大误差 3.03989e-8 N·m |
| 生产 C++ ONNX 相对 Python ONNX | Prone 239 组，最大误差 2.38419e-7；WheelBipe 256 组，9.53674e-7 |

新增的两路 YAML 已注册到同一集成测试程序，日后 `colcon test` 会自动覆盖它们。
既有 V6 和 legacy 控制器集成测试也通过。新增用例覆盖版本切换后的捕获规则，以及
配置解析不加载模型/启动线程。现有用例继续检查驱动准备、复位、异常输入、超时、力矩上限、
关节软限位和推理合同拒绝。

旧 `wheel-leg-infantry-rl.yaml` 是 100 Hz 策略 / 1 kHz PD 的 v4 导出模板，引用的
`models/wheel_leg/prone_motion.onnx` 当前未提供。尝试扩展其运行集成检查时因缺少该文件失败；
该模板未加入本次自动运行回归。其观测算法已通过上述逐字节比较，完整模板运行仍需匹配模型。

本地机器可读结果：

- `reports/wheel_leg_deployment_refactor_20261103/verification.json`
- `reports/wheel_leg_deployment_refactor_20261103/observation_equivalence.json`
- `reports/wheel_leg_deployment_refactor_20261103/parity.json`

数值比较基线为 RMCS `62c1d6bb9bf824de50a9e9ab7a95cb1e8a27c4c4` /
rmcs_rl `91443cdd5bf01c27b6e5be6537b6a2a24e295da0`。两份 ONNX 与训练合同未更换。
Prone 维持 100/200 Hz、PD160/2.5 和气簧前馈；WheelBipe 维持 50/200 Hz、PD60/2 和关闭前馈。
遥控复位、驱动握手、失联保护及动作时序沿用原逻辑。本次没有改变训练进程、启动 CAN
或执行实车动作；测试确认部署接口与代码一致性，策略能力和实车资格仍按模型回执记录。
