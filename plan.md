# 计划：Deformable Infantry V2 完整 Jacobian 解算推导

## 目标

将当前 `deformable_wheel_controller` 从“刚性舵轮公式替换 `R -> R_i` 并补一个 `v_mech`”的工程近似，升级为一套完整的、可推导的、适用于可变轮距舵轮底盘的统一模型。

## 总目标

将 `deformable infantry v2` 的关节目标、底盘 target 几何、舵向前馈与约束层统一到同一套 physical-angle 语义下，形成一条完整、连续、可退化运行的控制链。

具体要求如下：

- `physical_angle` 作为唯一用于轮距计算与 target 几何建模的关节角语义
- `target_physical_angle`、`target_physical_velocity`、`target_physical_acceleration` 作为底盘 target 几何的标准输入
- joint 电机控制链仍保留 raw motor angle 语义，即 `/chassis/*_joint/target_angle` 继续服务于关节电机控制器
- 关节目标切换时输出连续的 target 轨迹，而不是角度阶跃
- 变形过程中，`WheelDemoController` 能基于 target 几何实时修正舵向与轮速参考
- 当 target 缺失或失活时，控制器应具备清晰、安全的回退策略，而不是进入半退化状态
- 先打通对称 target 切换链路，再逐步扩展到四轮独立 target 与完整 Jacobian/约束模型

## 已知条件与前提

本计划针对 `deformable infantry v2` 底盘。

已知机械与控制前提如下：

- `deformable infantry v2` 是在舵轮底盘基础上，融合关节分速度与可变轮距实现解算的底盘
- 当前分析与后续重构的核心对象是 `DeformableWheelController`
- 基础舵轮解算可参考 `rmcs_notebook/chapters/algorithm/steering_wheel.typ`
- 本计划的目标不是继续沿用 rigid swerve 的闭式公式做局部修补，而是在现有舵轮模型基础上推导完整的可变轮距 Jacobian 模型

四个轮模块的固定射线方向约定如下：

- 模块顺序为：左前、左后、右后、右前
- 四个关节各自所在射线都与底盘坐标轴成 `45 deg`
- 因此四个模块相对底盘中心的几何方位可建模为四条固定对角射线

关节角的物理语义约定如下：

- `alpha_i` 表示第 `i` 个关节的物理角
- `alpha_i` 的物理意义为“连杆与地面的夹角”
- 关节运动范围上限为 `62.5 deg`
- 关节运动范围下限为 `8 deg`

在上述角度定义下，本计划采用如下轮距模型：

```text
R_i(alpha_i) = R_0 + L cos(alpha_i)
```

其物理含义为：

- 当 `alpha_i = 62.5 deg` 时，轮距较小
- 当 `alpha_i = 8 deg` 时，轮距较大

说明：

- 若后续实物零点、编码器正方向、物理角正方向与上述假设不一致，则应只在 `alpha_i` 的映射层修正，不应修改后续 Jacobian 主体推导
- 本计划默认四个模块的径向射线方向固定，仅轮心沿各自射线做伸缩，不考虑射线方向本身随机构改变
- 后续所有关于轮心位置、轮心速度、底盘速度观测、舵向目标和动力学分配的推导，都基于上述几何前提展开

目标场景包括：

- 前后不同高
- 四轮独立姿态
- 变形过程中保持舵向与轮速控制连续
- 高速旋转和关节变形叠加

核心原则：

- 先统一几何和运动学
- 再从统一模型推导观测器
- 再推导舵向和轮速前馈
- 最后再做动力学分配和约束优化

不要继续在 rigid swerve 的闭式公式上局部打补丁。

## 当前版本定位

当前代码可以视作：

- 对称变形
- 慢速变形
- 小范围姿态变化

下的工程近似模型。

它已经正确引入了：

- 每轮独立轮距 `R_i`
- 每轮独立径向速度 `dot(R_i)`

但仍缺少完整的：

- Jacobian 观测模型
- 舵向角速度前馈中的 `omega dot(R_i)` 和 `ddot(R_i)` 项
- 基于统一几何的动力学分配

## 坐标定义

底盘坐标系 `B`：

- `x` 向前
- `y` 向左
- `z` 向上

四个轮模块沿固定对角射线分布：

- 左前：`phi_1 = pi / 4`
- 左后：`phi_2 = 3 pi / 4`
- 右后：`phi_3 = -3 pi / 4`
- 右前：`phi_4 = -pi / 4`

定义每个轮的径向单位向量与切向单位向量：

```text
e_r,i = [cos(phi_i), sin(phi_i)]^T
e_t,i = [-sin(phi_i), cos(phi_i)]^T
```

其中：

- `e_r,i` 表示底盘中心指向该轮中心的方向
- `e_t,i` 表示绕底盘中心转动时该轮中心的瞬时切向方向

## 关节几何

设第 `i` 个关节的物理角为 `alpha_i`，语义为“连杆与地面的夹角”。

V2 已知机构范围：

- 上限：`62.5 deg`
- 下限：`8 deg`

轮心到底盘中心的距离定义为：

```text
R_i(alpha_i) = R_0 + L cos(alpha_i)
```

其中：

- `R_0` 为固定基座半径
- `L` 为连杆长度

于是轮心位置：

```text
p_i = R_i e_r,i
```

一阶导数：

```text
dot(p_i) = dot(R_i) e_r,i
dot(R_i) = -L sin(alpha_i) dot(alpha_i)
```

二阶导数：

```text
ddot(p_i) = ddot(R_i) e_r,i
ddot(R_i) = -L cos(alpha_i) dot(alpha_i)^2 - L sin(alpha_i) ddot(alpha_i)
```

注：当前代码只显式用了 `dot(R_i)`，若要推导完整前馈，`ddot(R_i)` 也需要进入模型。

## 轮心速度模型

设底盘中心平动速度为：

```text
v = [vx, vy]^T
omega = wz
```

二维平面内刚体上点 `p_i` 的绝对速度为：

```text
v_i = v + omega J p_i + dot(p_i)
```

其中：

```text
J = [0 -1
     1  0]
```

代入 `p_i = R_i e_r,i` 后得到：

```text
v_i = v + omega R_i e_t,i + dot(R_i) e_r,i
```

这是整套推导的核心公式。

含义拆解：

- `v`：底盘整体平动带来的轮心速度
- `omega R_i e_t,i`：底盘转动带来的轮心切向速度
- `dot(R_i) e_r,i`：关节伸缩带来的轮心径向速度

## 滚动约束与无侧滑约束

设第 `i` 个轮子的滚动方向单位向量为：

```text
s_i = [cos(zeta_i), sin(zeta_i)]^T
```

法向单位向量：

```text
n_i = [-sin(zeta_i), cos(zeta_i)]^T
```

无侧滑约束：

```text
n_i^T v_i = 0
```

轮速约束：

```text
s_i^T v_i = r omega_i
```

代入轮心速度公式：

```text
r omega_i = s_i^T v + omega R_i s_i^T e_t,i + dot(R_i) s_i^T e_r,i
```

再利用：

```text
s_i^T e_r,i = cos(zeta_i - phi_i)
s_i^T e_t,i = sin(zeta_i - phi_i)
```

得到标量形式：

```text
r omega_i = s_i^T v + omega R_i sin(zeta_i - phi_i) + dot(R_i) cos(zeta_i - phi_i)
```

## 底盘速度观测

将上式整理成关于 `vx, vy, wz` 的线性方程：

```text
cos(zeta_i) vx + sin(zeta_i) vy + R_i sin(zeta_i - phi_i) wz
= r omega_i - dot(R_i) cos(zeta_i - phi_i)
```

四个轮联立写为：

```text
A x = b
```

其中：

```text
x = [vx, vy, wz]^T
```

第 `i` 行定义为：

```text
A_i = [cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i)]
b_i = r omega_i - dot(R_i) cos(zeta_i - phi_i)
```

由于 4 个方程、3 个未知数，是超定系统，推荐直接用最小二乘：

```text
x_hat = (A^T A)^(-1) A^T b
```

更稳妥的形式是加权最小二乘：

```text
x_hat = (A^T W A)^(-1) A^T W b
```

权重 `W` 可用于表达：

- 某个轮打滑时可信度下降
- 某个轮接地较差
- 某个轮传感器异常

这一步应作为新的 `vx, vy, wz` 观测器核心，而不是继续使用 rigid swerve 的闭式合成公式。

## 目标舵向与目标轮速

给定目标底盘速度 `v_cmd = [vx, vy]^T`、目标角速度 `w_cmd`，以及期望关节状态对应的 `R_i, dot(R_i)`，第 `i` 个轮的目标轮心速度为：

```text
v_i^* = v_cmd + w_cmd R_i e_t,i + dot(R_i) e_r,i
```

目标舵向：

```text
zeta_i^* = atan2(v_i,y^*, v_i,x^*)
```

目标轮速：

```text
omega_i^* = ||v_i^*|| / r
```

该写法比 rigid swerve 的手工展开式更适合可变轮距底盘，因为所有几何变化都已经被统一吸收到 `v_i^*` 里。

## 舵向角速度推导

若二维向量：

```text
u = [u_x, u_y]^T
zeta = atan2(u_y, u_x)
```

则：

```text
dot(zeta) = (u_x dot(u_y) - u_y dot(u_x)) / ||u||^2
```

令：

```text
u_i = v_i = v + omega R_i e_t,i + dot(R_i) e_r,i
```

于是：

```text
dot(zeta_i) = (u_i,x dot(u_i,y) - u_i,y dot(u_i,x)) / ||u_i||^2
```

因此关键是继续求 `dot(u_i)`。

## 轮心速度导数

由：

```text
u_i = v + omega R_i e_t,i + dot(R_i) e_r,i
```

求导得：

```text
dot(u_i) = a + dot(omega) R_i e_t,i + omega dot(R_i) e_t,i + ddot(R_i) e_r,i
```

其中：

```text
a = [ax, ay]^T
dot(omega) = az
```

这是完整可变轮距模型下的舵向速度前馈基础。

和当前工程近似相比，新增的关键项是：

- `omega dot(R_i) e_t,i`
- `ddot(R_i) e_r,i`

这两项正是“边变形边旋转”时误差明显增大的来源。

## 轮速导数与轮电机前馈

目标轮速已定义为：

```text
omega_i^* = ||u_i^*|| / r
```

继续求导：

```text
dot(omega_i^*) = (u_i^{*T} dot(u_i^*)) / (r ||u_i^*||)
```

其中 `dot(u_i^*)` 使用上一节的完整公式。

如果要继续做轮电机前馈，可以把它建立在：

- `omega_i^*`
- `dot(omega_i^*)`

之上，而不要继续只从 rigid swerve 的力分配闭式公式出发。

## 平面动力学分配

若第 `i` 个轮沿其滚动方向 `s_i` 提供接地点驱动力 `f_i`，则整车平面 wrench 满足：

```text
sum_i f_i s_i = m a
sum_i f_i (p_i x s_i) = J az
```

其中二维叉乘标量为：

```text
p_i x s_i = R_i sin(zeta_i - phi_i)
```

将 4 个轮堆叠为矩阵：

```text
B f = tau
```

其中：

```text
tau = [m ax, m ay, J az]^T
```

第 `i` 列：

```text
B_i = [cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i)]^T
```

推荐采用最小范数分配：

```text
f = B^T (B B^T)^(-1) tau
```

轮电机力矩：

```text
tau_wheel,i = r f_i
```

注意：

- 这和速度观测矩阵有统一的几何结构
- 更适合后续加入约束优化

## 舵电机控制推荐形式

舵电机控制可以沿用现有二环结构，但目标量必须来自完整模型：

```text
zeta_i_ref = atan2(u_i,y^*, u_i,x^*)
dot(zeta_i)_ref = (u_x dot(u_y) - u_y dot(u_x)) / ||u||^2
tau_steer = PID_vel(dot(zeta)_ref + PID_ang(zeta_ref - zeta) - dot(zeta))
```

这样结构上仍与现有 notebook 一致，但数学来源已经统一。

## 约束优化的正确接入位置

功率约束、摩擦约束、打滑约束，不应再从 rigid swerve 的旧闭式参数直接替换 `R -> R_i`。

推荐顺序：

1. 先完成统一几何与观测模型
2. 先完成目标舵向、目标轮速、目标轮力分配
3. 再在 `f_i` 或 `tau_wheel,i` 层加入约束
4. 若要考虑关节动作代价，再把 joint actuator 功率、速度、加速度约束并入优化变量

否则很容易得到“数学形式像对了，但物理意义不完整”的约束器。

## 推荐实现步骤

### 第1步：统一几何层

实现每轮状态：

- `phi_i`
- `alpha_i`
- `dot(alpha_i)`
- `R_i`
- `dot(R_i)`
- 可选 `ddot(R_i)`

并集中放在一个 Geometry/ModuleState 结构里。

### 第2步：重写速度观测器

基于矩阵：

```text
A_i = [cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i)]
b_i = r omega_i - dot(R_i) cos(zeta_i - phi_i)
```

最小二乘求 `vx, vy, wz`。

### 第3步：重写目标舵向/轮速生成

直接由：

```text
u_i^* = v_cmd + w_cmd R_i e_t,i + dot(R_i) e_r,i
```

生成：

- `zeta_i_ref`
- `omega_i_ref`

### 第4步：补全舵向速度前馈

使用：

```text
dot(u_i) = a + az R_i e_t,i + omega dot(R_i) e_t,i + ddot(R_i) e_r,i
```

推 `dot(zeta_i)`。

### 第5步：重写轮力分配

使用：

```text
B_i = [cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i)]^T
```

从目标底盘加速度求每轮驱动力，再换算为轮力矩。

### 第6步：最后再接约束

将：

- 电机力矩限制
- 地面摩擦限制
- 功率限制
- 关节功率限制

放在统一分配器之后逐步加入。

## 推导时的检查点

推公式时可以一直检查以下退化情况：

1. 若四轮 `R_i` 相同且 `dot(R_i) = 0`
则应严格退化为普通舵轮公式。

2. 若 `wz = 0`
则每个轮的目标方向应仅由 `v + dot(R_i) e_r,i` 决定。

3. 若 `v = 0` 但 `wz != 0`
则轮心速度应为纯切向加径向叠加。

4. 若 `dot(R_i)` 很大
则舵向应明显偏向径向分量，而不能仍接近 rigid swerve 的切向解。

5. 若 `u_i = 0`
则 `atan2` 和 `dot(zeta_i)` 会奇异，需要单独定义降级策略。

## 本计划的使用方式

后续如果要正式实现，建议按下面顺序推进：

1. 先在 notebook 或文档里把本计划中的运动学公式写完整
2. 再把 `deformable_wheel_controller.cpp` 重构为几何层 + 观测层 + 分配层
3. 先确保无约束版本工作稳定
4. 最后再逐步恢复功率/摩擦/QCP 约束
