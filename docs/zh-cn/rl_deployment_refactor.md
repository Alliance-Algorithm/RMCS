# RL 部署整合

目标分支为 RMCS `dev/rmcs_rl`，基于 `main`；独立 `rmcs_rl` 在
`integrate/wheel-leg` 分支上基于其 `main` 整理历史。RMCS 通过
`rmcs_ws/src/rmcs_rl` 子模块引用部署包。

## 控制链

- wheel-leg：`WheelLeg` → `RlController` → 力矩接口 → 硬件命令伙伴组件。
  `WheelLegChassisController` 提供遥控命令及复位。策略 50 Hz，反馈 PD
  和恢复调度 1000 Hz，保留实车分支的关节顺序、坐标转换和控制参数。
- deformable：硬件/悬挂保护 → `RlBridge` → `policy_server` →
  `DeformableRlSuspension` → `DeformableSuspensionArbiter` → 关节控制器。
  保留 V3 物理角动作转换、目标限速和四腿整体失效逻辑。

## 换模型

wheel-leg 修改 `rl_controller.model_path`；deformable 修改
`policy_server.rl_model_path`，然后重启。相对路径以独立包安装 share
目录为基准，也可填写绝对路径。

不需要盖章、文件 hash 或 expected model ID。启动时检查输入输出类型、
数量、形状及试推理。deformable 的观测/动作/历史维度由桥组件通过
接口交给 launcher，不重复维护 policy_server 维度配置。启动握手失败
会抛出带路径及期望/实际维度的错误；运行中重新拉起使用非阻塞就绪检查。

输入输出数量兼容的模型内部网络变化只需更换路径。更改观测语义、动作
语义、多输入或循环状态时需要相应配置，不能只按元素数量推断兼容。

## 重构与清理

- wheel-leg 部署实现迁入独立包的 `src/wheel_leg`，保留原许可证。
- 遥控组件在构造函数直接注册接口；推理/PD/恢复调度组织为结构体。
- 双下复位即使已处于 Idle，也强制清理内部状态及输出。
- 删除旧 `WheelLegRlConsumer`、过时单测/姿态查看器、模型盖章工具及
  合成模型资源，同步清理构建、CI 和文档引用。
- 保留用于实车采样/辨识的控制器和消息，它们是运行功能。

## 验证

在 Jazzy 开发容器构建 `rmcs_msgs`、`rmcs_rl`、`rmcs_core`、
`rmcs_bringup`。离线 executor 仅加载桥和 launcher，验证正确模型
就绪、错误维度向 RMCS 启动层传播。直接 wheel-leg 推理验证 6 个
有限值动作以及不兼容 rank-3 模型拒绝。完整实车执行仍需车辆连接。

独立包修改尚未提交时，父仓库 gitlink 只能指向已存在的历史提交。
发布时应先提交并推送独立包，再提交 RMCS 子模块引用与集成修改。
