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
