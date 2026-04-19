# Deformable Infantry Omni 悬挂调试记录

## 目标

- 让腿先用正常闭环快速到 `min_angle`
- 到 `min_angle` 附近后进入悬挂模式
- 悬挂在接地附近要有明显抓地/支撑效果
- 不能在 `min/max` 之间来回跳，也不能过硬导致没有悬挂行程

## 当前工作树状态

- 已修改文件：
  - `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`
  - `rmcs_ws/src/rmcs_bringup/config/deformable-infantry-omni.yaml`
- 没有新增 `.md` 设计文档改动

## 已完成步骤

### 1. 先回到无悬挂基线

- 将 `deformable-infantry-omni.yaml` 对齐到 `deformable-infantry-v2` 分支的无悬挂基线
- 目的：确认问题是不是由新悬挂链路引入

**效果**
- 基线正常，说明“无悬挂”链路可工作

### 2. 恢复有悬挂链路

- `omni` 配置切回：
  - `DeformableChassis`
  - `DeformableJointController`
  - `/chassis/*/suspension_mode`
  - `/chassis/*/suspension_torque`
- 使用当前代码支持的悬挂参数集：
  - `active_suspension_Kz/Kp/Dp/Kr/Dr/D_leg/...`

**效果**
- 有悬挂链路后，最初出现严重在 `min/max` 之间来回跳的问题

### 3. 修正物理角闭环符号

- 根据硬件定义：
  - `physical_angle = const - motor_angle`
  - `physical_velocity = -motor_velocity`
- 将 `omni` 关节控制参数改为：
  - `b0 = -0.60`
  - `suspension_torque_feedforward_gain = -1.0`

**效果**
- “来回跳限位”现象明显缓解/恢复正常
- 说明物理角控制坐标系下原先符号不对

### 4. 修改悬挂接管时机

- 在 `DeformableChassis::update_active_suspension()` 中加入悬挂接管阈值：
  - `active_suspension_enter_deploy_tolerance_deg`
  - `active_suspension_exit_deploy_tolerance_deg`
  - `active_suspension_activation_velocity_threshold_deg`
- 当前配置：
  - `enter = 1.0 deg`
  - `exit = 2.0 deg`
  - `velocity = 8.0 deg/s`

**效果**
- 逻辑改成：先靠正常闭环快速到 `min_angle` 附近，再进入悬挂
- 避免半路提前接管

### 5. 修改非对称姿态切回对称的状态机

- 在 `update_lift_target_toggle()` 中修复切换逻辑
- 现在从非对称模式切回对称时，会直接切到“下一个对称状态”

**效果**
- 修复了“先展开到最低点，再站起来”的错误状态切换

### 6. 重写悬挂主力项为每条腿独立抓地

- 旧逻辑：
  - 依赖 `average_angle`
  - 公共 `support_force`
- 新逻辑：
  - 每条腿独立计算
  - 当前公式：

```cpp
leg_force = gravity_force_per_wheel
         + active_suspension_Kz_ * (alpha - nominal_angle)
         + active_suspension_D_leg_ * alpha_dot
```

**效果**
- 比旧的“公共平均角托举”更接近抓地逻辑
- 但在接地附近仍然偏软

### 7. 只改 `D_leg` 符号验证方向

- 将 `D_leg` 项由
  - `- active_suspension_D_leg_ * alpha_dot`
  改为
  - `+ active_suspension_D_leg_ * alpha_dot`

**效果**
- 轮子在接地附近更愿意贴地
- 说明 `D_leg` 原先方向大概率有问题

### 8. 调整悬挂参数

- 当前 `omni` 关键参数：

```yaml
min_angle: 20.0
max_angle: 68.0

active_suspension_enable: true
active_suspension_Kz: 100.0
active_suspension_Kp: 0.0
active_suspension_Dp: 0.0
active_suspension_Kr: 0.0
active_suspension_Dr: 0.0
active_suspension_D_leg: 5.0
active_suspension_gravity_comp_gain: 1.00
active_suspension_control_acceleration_limit: 0.0
active_suspension_preload_angle_deg: 8.0
active_suspension_enter_deploy_tolerance_deg: 1.0
active_suspension_exit_deploy_tolerance_deg: 2.0
active_suspension_activation_velocity_threshold_deg: 8.0
active_suspension_torque_limit: 40.0
```

- 当前悬挂模式 ADRC 参数：

```yaml
b0: -0.60
suspension_td_r: 12.0
suspension_eso_w0: 80.0
suspension_k1: 6.0
suspension_k2: 3.0
suspension_u_min: -35.0
suspension_u_max: 35.0
suspension_output_min: -35.0
suspension_output_max: 35.0
suspension_torque_feedforward_gain: -1.0
```

### 9. 预载方案已实现，并修正接管角/支撑角混用问题

- 已在 `DeformableChassis` 中新增：
  - `active_suspension_preload_angle_deg`
- 当前实现不是直接把 preload 写进接管阈值，而是拆成两套参考角：

```cpp
deploy_angle = deg_to_rad(min_angle_);
support_nominal_angle = deploy_angle - active_suspension_preload_angle_;
```

- 含义：
  - `deploy_angle` 只负责判断“什么时候进入悬挂”
  - `support_nominal_angle` 只负责提供预紧后的支撑力

**效果**
- 修复了“加了 preload 后腿到 `min_angle` 反而进不了悬挂”的逻辑错误
- 现在腿到 `min_angle` 附近后，可以正常进入悬挂，同时带预紧力

## 已观察到的效果

### 正向效果

- 无悬挂基线正常
- 符号修正后，不再严重在 `min/max` 之间跳
- `D_leg` 改符号后，轮在接地附近更愿意贴地
- 接管阈值加入后，更符合“先闭环到底，再进悬挂”

### 负向效果 / 失败尝试

- 把静态托举和预载加得太大时，悬挂变得很硬，几乎没有悬挂效果
- 当时具体尝试过：
  - `gravity_comp_gain = 1.0`
  - `preload_angle_deg = 8.0`
- 结果：
  - 很硬
  - 但没有明显悬挂行程
- 最初版本里，这组改动还伴随一个实现错误：
  - preload 被直接并进了接管参考角
  - 结果导致腿到 `min_angle` 时反而不进入悬挂
- 这个逻辑错误已经修正，但“太硬、悬挂感弱”的主观问题仍然存在

## 当前仍存在的问题

### 1. 接地附近还是偏软

- 轮子在接地附近虽然更愿意贴地
- 但腿整体还是偏软
- 车底盘有触底风险

### 2. 悬挂行程感觉太短

- 进入 `min_angle` 附近后，悬挂能工作
- 但不是“刚接地就有足够预紧力”
- 更像是接地后支撑慢慢起来

### 3. 预紧后又容易变得太硬

- 加 preload 和提高 `gravity_comp_gain` 后，接地瞬间不再那么软
- 但会出现另一种问题：
  - 预紧很明显
  - 但悬挂“可压缩的感觉”变少
  - 主观上像直接闭环卡在 `min_angle`

### 4. 当前问题已经从“没有预载”转成“预载和悬挂感的平衡”

- 当前公式里：
  - 接管判断用 `deploy_angle`
  - 支撑判断用 `support_nominal_angle`
- 这解决了“到 `min_angle` 进不了悬挂”的逻辑错误
- 但在当前“先闭环到 `min_angle` 再进悬挂”的策略下，可感知悬挂行程本来就偏短

## 已经明确的结论

- 当前问题不主要是 `Kz` 方向错误
- 当前已经确认：
  - `D_leg` 原始方向有问题，改符号后接地更愿意贴地
  - preload 必须和接管参考角解耦，否则会出现“到 `min_angle` 不进悬挂”
- 当前主要矛盾已经从“没有预载”变成：
  - 预载一小就软塌
  - 预载一大又很硬、像没悬挂效果
- 文档目标是“尽量抓地”，当前实现已经比最初更接近这个目标，但还没完全达到
- 在当前策略下：
  - 先正常闭环到 `min_angle`
  - 再进入悬挂
  会天然压缩可感知悬挂行程
- 所以“软硬”和“可感知行程”在当前策略下是部分冲突的，不是完全独立参数

## 下一步建议

### 当前已实现的方案：加预载 + 拆分接管角/支撑角

- 在 `DeformableChassis` 增加：
  - `active_suspension_preload_angle_deg`
- 公式改成：

```cpp
deploy_angle = deg_to_rad(min_angle_);
support_nominal_angle = deploy_angle - preload_angle;

leg_force = gravity_force_per_wheel
         + active_suspension_Kz_ * (alpha - support_nominal_angle)
         + active_suspension_D_leg_ * alpha_dot;
```

**预期效果**
- 机械目标还是 `min_angle`
- 但虚拟零力点更低
- 腿到 `min_angle` 时，`Kz` 项已经是正的
- 接地瞬间立刻有支撑

### 下一步真正要解决的问题

- 当前不再优先继续加大 preload / gravity_comp / Kz
- 下一步更应该研究：
  - 如何保留“到 `min_angle` 先有预紧力”
  - 同时让悬挂进入后还有一段明显可压缩的工作区

可选方向：
- 方向 A：
  降一点 preload 和 gravity_comp，再通过更合适的 `Kz` 曲线保留中后段支撑
- 方向 B：
  让悬挂在接近 `min_angle` 前就进入“软接管”，而不是等完全到底才切入
- 方向 C：
  把当前线性 `Kz` 改成分段或非线性刚度，做到“初段软、中后段硬”

## 备注

- 当前还没做真实编译验证
- 在当前终端环境里缺少 ROS/Jazzy 编译链，只做过：
  - YAML 语法检查
  - `git diff --check`
  - 代码静态阅读和定点修改

## 最近一次整理：Joint Controller / Chassis 职责边界规范化

### 步骤

1. 重新梳理 `DeformableChassis` 和 `DeformableJointController` 的现有职责。
   - 确认 `DeformableJointController` 实际上已经更接近“关节局部执行器”：
     - 输入 `measurement_angle / setpoint_angle / mode_input / suspension_torque`
     - 输出 `control_torque`
     - 内部只做 ADRC 模式切换、局部前馈和限幅
   - 确认真正职责混杂的是 `DeformableChassis`：
     - 既做遥控/底盘模式状态机
     - 又做关节目标轨迹生成
     - 又做悬挂接管与悬挂辅助力矩输出
     - 还夹带了 `scope_motor_control`

2. 重构 `DeformableJointController`，但不改外部接口和运行语义。
   - 新增 `ModeConfig`，把 normal / suspension 两套 ADRC 参数打包
   - 新增 `InputSnapshot`，明确 joint-local servo 每周期真正使用的输入
   - 把 `update()` 拆成清晰阶段：
     - `register_interfaces_()`
     - `load_mode_configs_()`
     - `read_inputs_()`
     - `update_mode_selection_()`
     - `effective_output_limits_()`
     - `initialize_if_needed_()`
     - `run_joint_servo_()`
     - `publish_control_output_()`
   - 内部把 `suspension_torque` 输入语义标注为“joint torque feedforward input”，避免误读成悬挂状态机本体

3. 重构 `DeformableChassis`，把高层 joint intent pipeline 显式化。
   - 新增 `JointFeedbackFrame`，把 motor / physical / torque feedback 聚合
   - 新增 `SuspensionOutputHandles`，集中管理每腿 `suspension_mode / suspension_torque` 输出句柄
   - 把 `update()` 改成更清晰的高层流程：
     - `update_mode_from_inputs_()`
     - `update_velocity_control()`
     - `update_lift_target_toggle()`
     - `run_joint_intent_pipeline_()`
   - 把原来散在 `update_lift_angle_error()` 里的工作拆出来：
     - `read_joint_feedback_frame_()`
     - `refresh_requested_joint_targets_from_deploy_state_()`
     - `apply_suspension_intent_()`
     - `run_joint_intent_pipeline_()`
   - 在代码注释里明确：
     - `DeformableChassis` 负责 high-level joint intent
     - `DeformableJointController` 负责 joint-local servo execution

4. 整理 `deformable-infantry-omni.yaml` 参数分组与注释。
   - `chassis_controller` 参数按：
     - deploy geometry / chassis-owned joint intent
     - joint intent trajectory limits
   - `*_joint_controller` 参数按：
     - joint-local servo inputs
     - normal ADRC servo mode
     - suspension ADRC servo mode
     - joint-local feedforward / limit shaping

### 解决的问题

- 解决了“看代码时不容易分清谁在做什么”的问题。
  - 现在 `DeformableJointController` 的定位更明确：
    - 不负责接地判据
    - 不负责底盘姿态/遥控模式
    - 不负责悬挂状态机
    - 只负责把上游 joint intent 执行成 `control_torque`

- 解决了 `DeformableChassis` 内部高层逻辑流程不够清晰的问题。
  - 以前 `update()` 到 `update_lift_angle_error()` 这一段像“所有事情都在一起做”
  - 现在已经能明确读出：
    - 输入模式处理
    - joint feedback 归一化
    - deploy target 生成
    - suspension intent 覆盖
    - joint target 发布

- 解决了配置文件参数语义不容易一眼看懂的问题。
  - 现在 YAML 中能直接看出：
    - 哪些参数属于 chassis 高层意图生成
    - 哪些参数属于 joint controller 的局部执行

- 保留了现有 DAG 接口和现有行为语义，没有把职责整理变成一轮功能重写。

### 遇到的问题

- 当前代码历史上已经把多层职责揉在一起，所以这轮只能先做“文件内职责收口”，还没有拆成新的独立组件。
  - 尤其是 `DeformableChassis` 仍然比较大
  - 只是现在内部职责边界比之前清楚了

- `DeformableJointController` 虽然已经被规范成 joint-local servo，但它仍然保留了 `mode_input` 和 `suspension_torque` 两个上游语义输入。
  - 这在职责上是合理的
  - 但名字上仍然容易让人误解成“joint controller 内部在做悬挂状态机”
  - 这次通过注释和内部命名做了缓解，没有改外部接口名

- `DeformableChassis` 中的悬挂逻辑目前仍然和 deploy target / joint trajectory 紧耦合。
  - 这次没有继续拆出独立 `SuspensionCoordinator` 组件
  - 主要是为了保持现有 YAML 和行为不变

- 当前环境依然无法做真实编译验证。
  - `.script/build-rmcs` 仍然失败
  - 原因是当前终端缺少 `/opt/ros/jazzy/setup.bash`
  - 且脚本期望的工作区路径是 `/workspaces/RMCS/rmcs_ws`
  - 所以这轮职责整理只做了：
    - `clang-format`
    - `git diff --check`
    - 静态代码检查
