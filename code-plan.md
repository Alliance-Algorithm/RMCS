# Code Plan：WheelDemoController 接入与 Target 几何改造

## 总目标

围绕 `deformable infantry v2` 现有控制链，先将 `WheelDemoController` 接入实际 V2 YAML，再把关节 target 几何从“只有 target angle”逐步升级为一条连续的 physical target 轨迹链，使关节在变形过程中能够正确影响舵向与轮速参考。

本文件关注代码落地顺序，不重复展开完整 Jacobian 推导。理论总纲见 `plan.md`。

---

## 已完成 Checklist

### 1. 控制器接入

目标：将 `WheelDemoController` 接入 V2 正式 YAML，而不是停留在独立 demo 配置。

- [x] 确认本轮主改造对象为 `WheelDemoController`
- [x] 将 `deformable-infantry-v2.yaml` 中的底盘控制器切换为 `WheelDemoController`
- [x] 删除临时 demo YAML，统一收敛到 V2 正式 YAML

结果：后续验证直接发生在实际 bringup 配置上。

### 2. `DeformableChassis` target 输出语义修正

目标：为正式链路准备 `target_physical_angle` 输出，并修复非对称目标被覆盖的问题。

- [x] 在 `deformable_chassis.cpp` 中新增 `/chassis/*_joint/target_physical_angle` 输出
- [x] 输出语义统一为 physical angle，单位为 rad
- [x] 修复 `update_lift_target_toggle()` 中非对称目标在函数末尾被统一覆盖的问题

结果：正式链路具备 `target_physical_angle` 基础输出，非对称目标不再被误覆盖。

### 3. `WheelDemoController` target 输入接入

目标：让 `WheelDemoController` 同时消费 feedback joint state 与 target physical 几何。

- [x] 新增 `/chassis/*_joint/target_physical_angle` 输入
- [x] 新增 `/chassis/*_joint/target_physical_velocity` 输入
- [x] 新增 `/chassis/*_joint/target_physical_acceleration` 输入

结果：控制器已具备完整 target 几何输入通道。

### 4. joint state 结构重构

目标：将 feedback 几何与 target 几何彻底分离。

- [x] 将 joint state 拆分为 `JointFeedbackStates` 与 `JointTargetStates`
- [x] 抽取共享几何 helper，统一从 `alpha / alpha_dot / alpha_ddot` 计算 `radius / radius_dot / radius_ddot`

结果：reference 链、观测链、约束层、回退策略都有了清晰的数据结构落点。

### 5. feedback 状态更新

目标：保持底盘速度观测继续基于真实 joint feedback。

- [x] 实现 `update_joint_feedback_states_()`
- [x] 读取 `/chassis/*_joint/physical_angle`
- [x] 读取 `/chassis/*_joint/physical_velocity`
- [x] 使用反馈速度差分估计 `alpha_ddot`
- [x] 计算 feedback `radius / radius_dot / radius_ddot`

结果：`calculate_chassis_velocity(...)` 继续使用真实 feedback，不受 target 轨迹链影响。

### 6. target 状态更新

目标：让 `WheelDemoController` 能基于 provider 提供的 target physical 几何构造 target joint state。

- [x] 实现 `update_joint_target_states_()`
- [x] 优先读取 `target_physical_angle`
- [x] 优先读取 `target_physical_velocity`
- [x] 优先读取 `target_physical_acceleration`
- [x] velocity 缺失时可回退 angle 差分
- [x] acceleration 缺失时先保持 0
- [x] 计算 target `radius / radius_dot / radius_ddot`

结果：控制器已能消费连续 target 几何，target 导数不再只能由 consumer 猜测。

### 7. reference 链切换到 target 几何

目标：解决关节在移动时，舵向参考仍基于旧 feedback 几何导致慢半拍的问题。

- [x] `calculate_chassis_status_expected(...)` 改为使用 target joint state
- [x] `calculate_steering_control_torques(...)` 改为使用 target joint state
- [x] `calculate_chassis_velocity(...)` 继续使用 feedback joint state

结果：形成“target 驱动参考，feedback 负责观测”的基本结构。

### 8. 接入 `target.radius_dot` 与 `target.radius_ddot`

目标：让关节运动导数真正参与舵向前馈，而不只是改静态 target angle。

- [x] `calculate_steering_control_torques(...)` 已通过 target joint state 使用 `radius`
- [x] `calculate_steering_control_torques(...)` 已通过 target joint state使用 `radius_dot`
- [x] `calculate_steering_control_torques(...)` 已通过 target joint state使用 `radius_ddot`
- [x] `ChassisTestV2Controller` 已输出 `target_physical_velocity`
- [x] `ChassisTestV2Controller` 已输出 `target_physical_acceleration`

结果：舵向前馈已具备响应关节 target 导数的能力。

### 9. V2 physical 角语义修正

目标：消除 `/angle` 与 `/physical_angle` 语义混用，确保 target 几何与 feedback 几何基于同一 physical-angle 定义。

- [x] 确认 `/chassis/*_joint/angle` 是电机原始角
- [x] 确认 `/chassis/*_joint/physical_angle` 是与地面夹角、可直接用于轮距计算的物理角
- [x] 在 `ChassisTestV2Controller` 中新增 `/chassis/*_joint/physical_angle` 输入
- [x] `target_physical_angle` 改为真正基于 `physical_angle`
- [x] `/chassis/*_joint/target_angle` 保持 raw motor angle 语义，并通过 `motor_angle = 62.5deg - physical_angle` 转换得到

结果：joint 控制链和底盘 target 几何链的双语义边界已明确分开。

### 10. `ChassisTestV2Controller` 连续 target 轨迹

目标：避免 target angle 阶跃，输出连续的 `target_physical_angle / velocity / acceleration`。

- [x] 增加 `target_physical_angle_state_`
- [x] 增加 `target_physical_velocity_state_`
- [x] 增加 `target_physical_acceleration_state_`
- [x] 保留 `current_target_angle_rad_` 作为目标终点
- [x] `左中右上` 上升沿只切换目标终点
- [x] 每帧通过 `update_joint_target_trajectory()` 做限速限加速度推进
- [x] 发布连续 target 到 `/target_physical_angle`
- [x] 发布连续 target 到 `/target_physical_velocity`
- [x] 发布连续 target 到 `/target_physical_acceleration`
- [x] 同步转换回 `/target_angle` 供 joint ADRC 使用

当前轨迹参数：

- [x] `target_physical_velocity_limit`
- [x] `target_physical_acceleration_limit`

结果：当前 target 不再是单纯阶跃，`WheelDemoController` 能在关节移动过程中看到非零 target 导数。

### 11. 坐标语义对齐

目标：与旧控制器的底盘坐标与舵向角零位定义保持一致，避免整体偏转 `45 deg`。

- [x] `calculate_steering_status()` 中减去 `pi / 4`
- [x] `calculate_chassis_control_velocity()` 中对平移量施加 `Rotation2Dd(-pi/4)`

结果：已与旧 V2 控制链的坐标约定对齐。

### 12. 约束层部分迁移到 target 几何

目标：避免 reference 链已使用 target 几何，而约束层仍停留在 feedback 几何。

- [x] `constrain_chassis_control_acceleration(...)` 改为接收 `JointTargetStates`
- [x] `calculate_ellipse_parameters(...)` 优先使用 target 半径
- [x] target 无效时回退到 `vehicle_radius_`

结果：约束层已部分迁移到 target 几何，但力矩分配层仍未完全收敛。

### 13. V2 target 状态机语义确认

目标：明确 `ChassisTestV2Controller` 在 V2 下的 joint 逻辑，避免误判 target 缺失为 bug。

- [x] 确认 `switch 双 down` 时 joint 应无力矩
- [x] 确认 `switch 双 down` 时 target 链失活并输出 `NaN`
- [x] 确认从 `double down` 恢复时应重新对齐当前 feedback
- [x] 确认 `左中右上` 上升沿切换目标终点

结果：当前 target 缺失在 `double down` 场景下是设计行为，不是异常。

---

## 待完成 Checklist

### 1. `WheelDemoController` target 缺失回退策略

目标：当 target 缺失或失活时，控制器应清晰回退，而不是进入半退化状态。

- [ ] 为 `target_physical_angle` 缺失设计完整回退策略
- [ ] 评估是否在 target 缺失时整体回退到 feedback 几何
- [ ] 明确 `double down` 期间 `WheelDemoController` 应是“安全回退运行”还是“整体停机输出”
- [ ] 避免 `calculate_chassis_status_expected(...)` 在 target 缺失时直接失去 wheel velocity reference
- [ ] 避免 `calculate_steering_control_torques(...)` 在 target 缺失时直接返回全零 torque

### 2. 力矩分配层收敛到 target 几何

目标：避免约束按 target 判定、分配按 feedback 执行。

- [ ] 评估 `calculate_wheel_control_torques(...)` 是否应迁移到 target 几何
- [ ] 明确快速变形时 wrench map 应基于 feedback 还是 target
- [ ] 统一 reference 链、约束层、分配层的几何语义

### 3. `ChassisTestV2Controller` 非对称 target 扩展

目标：从对称升降测试扩展到更复杂的 target 场景。

- [ ] 支持前高后低
- [ ] 支持前低后高
- [ ] 支持四轮独立 physical target
- [ ] 评估共享单轨迹状态是否需要扩展为四轮独立轨迹状态

### 4. 正式链路收敛

目标：减少 `ChassisTestV2Controller` 的过渡性质，将 target 语义补回正式控制链。

- [ ] 将 `target_physical_velocity` 补到 `DeformableChassis`
- [ ] 将 `target_physical_acceleration` 补到 `DeformableChassis`
- [ ] 评估是否应由正式链路直接生成连续 target trajectory

### 5. 参数与配置落盘

目标：避免关键行为依赖代码默认值。

- [ ] 将 `target_physical_velocity_limit` 写入 `deformable-infantry-v2.yaml`
- [ ] 将 `target_physical_acceleration_limit` 写入 `deformable-infantry-v2.yaml`
- [ ] 明确这些参数与真实 joint 跟踪能力的匹配关系

### 6. 编译与运行验证

目标：完成代码层之外的实际验证闭环。

- [ ] 在有 `colcon` 的环境中完成编译验证
- [ ] 验证 `WheelDemoController` plugin 在 V2 launch 中正常工作
- [ ] 验证 target 接口全部成功连边

---

## 风险点记录

### 高优先级

- [ ] `WheelDemoController` 的 target 缺失回退策略不完整
说明：`target_physical_angle` 缺失时，`joint_target.valid` 会失败，导致参考链和 steering torque 进入半退化状态。典型场景包括启动早期、`double down` 时 target 被主动清空、以及从 `double down` 恢复时 feedback 未齐。

### 中优先级

- [ ] `ChassisTestV2Controller` 当前只支持对称 target
说明：当前 target 轨迹是四轮共享一个 physical target 状态，只适合对称升降测试与 target 驱动舵向补偿验证。

- [ ] 轨迹参数尚未显式写入 YAML
说明：当前依赖默认值 `target_physical_velocity_limit = 180 deg/s`、`target_physical_acceleration_limit = 720 deg/s^2`，不利于调试和复现实验。

- [ ] 力矩分配层仍使用 feedback 几何
说明：当前 reference 链与约束层部分使用 target 几何，但 wheel torque 分配仍使用 feedback 几何，快速变形时可能出现几何不一致。

### 低优先级

- [ ] 坐标语义仍需运行验证
说明：虽然已补上 `-pi/4` 变换，但仍需通过实机或回放确认纯前进、纯左移、原地旋转工况是否方向正确。

---

## 待做记录

### 行为验证

- [ ] 启动后 target 是否立即 finite
- [ ] `double down` 时是否无 joint 力矩
- [ ] 从 `double down` 恢复后是否正确重新对齐当前 feedback
- [ ] `左中右上` 上升沿时 `target_physical_velocity` 是否非零
- [ ] `左中右上` 上升沿时 `target_physical_acceleration` 是否非零
- [ ] 变形过程中舵向是否同步变化，而不是慢半拍

### 运动学验证

- [ ] 纯前进工况
- [ ] 纯左移工况
- [ ] 原地旋转工况
- [ ] 变形与旋转叠加工况

### 配置与构建验证

- [ ] `deformable-infantry-v2.yaml` 全部接口连边成功
- [ ] `WheelDemoController` 启动无 target 缺失异常
- [ ] `rmcs_core` 编译通过

---

## 当前结论

本轮已经完成了从“静态 target angle”到“连续 target physical trajectory”的主链改造，并将 `WheelDemoController` 接入 V2 正式 YAML。

当前最关键的剩余问题不是 target 是否已经接通，而是：

- target 缺失时的回退策略是否足够安全
- target provider 是否需要进一步扩展到非对称四轮目标
- 约束层与力矩分配层是否最终统一到同一套 target 几何
