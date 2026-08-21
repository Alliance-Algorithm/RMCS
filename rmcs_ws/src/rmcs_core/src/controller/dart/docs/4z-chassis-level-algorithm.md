# 4z 底盘调平算法

本文档面向希望理解或改进 4z 底盘控制器的开源贡献者，重点说明控制目标、算法流程、速度分配方法和安全保护规则。具体实车调参值只作为参考，不作为算法本身的约束。

4z 底盘可以抽象为一个矩形四轴升降平台。四个轴独立驱动，平台姿态由安装在底盘上的 IMU 测量。

```text
front_left   front_right

back_left    back_right
```

完整调平任务由两步组成：

```text
ZERO_CALIBRATE -> LEVEL
```

`ZERO_CALIBRATE` 用底部限位建立每个轴的软件零点；`LEVEL` 在零点有效后，根据 IMU 的 pitch 和 roll 反馈调平平台。

## 硬件抽象

每个升降轴提供：

- 电机角度反馈；
- 电机速度反馈；
- 电机力矩反馈；
- 底部限位开关；
- 目标速度输出；
- 力矩限制输出。

底盘姿态输入包括：

- `pitch`；
- `roll`；
- 可选的 `pitch_rate`；
- 可选的 `roll_rate`。

控制器使用以下符号约定：

- 电机负速度表示该轴向下运动；
- 电机正速度表示该轴向上运动；
- pitch 为正表示前侧更高；
- roll 为正表示左侧更高。

零点保存在控制器内部，属于软件零点。标定流程不会改写电机硬件零点。

## 命令流程

4z 底盘控制器响应四类命令：

- `IDLE`：停止当前命令并清理输出；
- `ABORT`：停止当前命令、清理输出，并返回 `ABORTED`；
- `ZERO_CALIBRATE`：寻找四个轴的底部限位并提交软件零点；
- `LEVEL`：使用已提交零点和 IMU 姿态反馈执行调平。

`IDLE`、`ABORT` 或新命令抢占时，会清理当前阶段、临时零点和稳定计数器。已经成功提交的软件零点会保留，直到下一次成功标定替换它。

如果没有有效零点，`LEVEL` 会被拒绝并返回失败。

## 零点标定

零点标定为后续调平提供相对高度参考。

### 下行找底

进入 `ZERO_CALIBRATE` 后，四个轴同时向下运动：

```text
v[i] = -zero_down_speed
```

每个轴独立处理：

1. 当底部限位触发时，锁存当前电机角度到临时零点缓存；
2. 该轴以 0 速度保持一段时间；
3. 保持结束后，该轴停止输出速度；
4. 未触底的轴继续向下寻找底部。

只有四个轴都获得有效底部角度后，控制器才一次性提交新零点：

```text
zero_angle[i] = pending_zero_angle[i]
zero_valid = true
```

这种一次性提交策略可以避免标定中途失败或中止时，只替换了部分轴零点。

### 上行回退

零点提交后，四个轴向上回退一段距离，使平台离开底部限位：

```text
height[i] = angle[i] - zero_angle[i]
height[i] >= zero_rollback_angle
```

每个轴达到回退距离后停止。所有轴回退完成并保持一段时间后，零点标定成功。

后续调平使用相对高度：

```text
height[i] = angle[i] - zero_angle[i]
```

`height` 越小，表示该轴越接近底部零点。

## 姿态控制

调平目标不是要求 IMU 原始读数为 0，而是要求 IMU 读数接近配置中的目标偏置。偏置用于补偿 IMU 安装误差：

```text
pitch_error = pitch_offset - pitch
roll_error  = roll_offset  - roll
```

两个误差分别进入 pitch 和 roll PID：

```text
pitch_cmd =
    pitch_pid.update(pitch_error)
    - level_pitch_rate_kd * pitch_rate

roll_cmd =
    roll_pid.update(roll_error)
    - level_roll_rate_kd * roll_rate
```

`pitch_rate` 和 `roll_rate` 是可选阻尼项。没有角速度输入时，控制器将其视为 0。

这里的 `pitch_cmd` 和 `roll_cmd` 不是电机力矩，而是抽象的姿态修正量。它们会在下一步被分配为各个升降轴的速度目标。

## 速度分配

pitch 和 roll 的修正依赖四个轴的相对运动。几何符号定义如下：

| 轴 | pitch 系数 | roll 系数 |
| --- | ---: | ---: |
| `front_left` | `+1` | `+1` |
| `front_right` | `+1` | `-1` |
| `back_left` | `-1` | `+1` |
| `back_right` | `-1` | `-1` |

控制器需要求解：

```text
B * v = [pitch_cmd, roll_cmd]
```

其中 `v` 是各轴速度修正量，`B` 是由上表几何系数组成的矩阵。

这个方程通常不是唯一解：控制目标只有 pitch 和 roll 两个量，但可用电机轴有三个或四个。控制器选择最小范数解，也就是在所有能满足 pitch/roll 修正目标的速度组合里，选择总动作量最小的一组：

```text
minimize sqrt(v0^2 + v1^2 + ...)
```

这样可以避免引入不必要的整体升降，也能减少单个轴的额外运动。

四轴全部参与时，对称解等价于：

```text
front_left  = ( pitch_cmd + roll_cmd) / 4
front_right = ( pitch_cmd - roll_cmd) / 4
back_left   = (-pitch_cmd + roll_cmd) / 4
back_right  = (-pitch_cmd - roll_cmd) / 4
```

只修正 pitch 时，前后轴反向运动；只修正 roll 时，左右轴反向运动；两种误差同时存在时，修正量线性叠加。

在最终三轴调平阶段，参考轴固定不动。控制器会用剩余三个轴重新求三轴最小范数解，而不是先计算四轴结果再把固定轴速度裁掉。

## 调平阶段

### 阶段一：四轴姿态调整

`LEVEL` 开始后，四个轴都参与调平。控制器循环执行：

1. 读取 pitch 和 roll；
2. 计算 `pitch_error` 和 `roll_error`；
3. 通过两个 PID 得到 `pitch_cmd` 和 `roll_cmd`；
4. 将姿态修正量分配到四个轴；
5. 判断姿态和修正速度是否稳定。

稳定条件包括：

```text
abs(pitch - pitch_offset) <= level_pitch_tolerance
abs(roll  - roll_offset)  <= level_roll_tolerance
```

并且所有参与调平轴的修正速度都低于：

```text
level_correction_velocity_deadband
```

这些条件需要连续满足 `level_settle_ticks` 个控制周期。

稳定后，控制器选择相对高度最小的轴作为参考轴：

```text
reference_axis = arg min(angle[i] - zero_angle[i])
```

参考轴表示当前最接近底部零点的轴。

### 阶段二：保持姿态并整体下降

选出参考轴后，控制器在姿态修正速度上叠加共同下降速度：

```text
target_velocity[i] = correction[i] - level_descend_speed
```

这一步的目标是在下降过程中继续抑制 pitch 和 roll 误差。

下降过程中持续检查底部限位：

- 如果参考轴先触底，直接进入最终三轴调平；
- 如果非参考轴先触底，暂停其他轴，只让参考轴继续下降；
- 如果没有轴触底，继续四轴调平并整体下降。

### 阶段三：固定参考轴，三轴调平

参考轴触底后，该轴保持 0 速度，不再参与姿态分配。剩余三个轴根据当前 pitch 和 roll 重新求三轴最小范数解：

```text
B3 * v3 = [pitch_cmd, roll_cmd]
```

当姿态误差和修正速度再次连续满足稳定条件后，`LEVEL` 返回 `SUCCEEDED`。

## 输出和保护规则

4z 控制器输出每个轴的速度目标：

```text
/dart/chassis/<axis>_motor/control_velocity
```

速度目标由独立的速度 PID 转换成电机力矩：

```text
实际电机速度 -> 速度 PID -> control_torque
```

4z 控制器还会输出每个轴的力矩限制：

```text
/dart/chassis/<axis>_motor/control_torque_limit
```

速度发布前会统一经过以下保护。

### 速度限幅

所有有限速度先限制到：

```text
[-chassis_velocity_limit, chassis_velocity_limit]
```

### 底部限位保护

如果某个轴已经触发底部限位，并且目标速度仍会让它继续向下运动，该轴输出会被禁用：

```text
control_velocity = NaN
control_torque_limit = NaN
```

0 速度不受底部限位保护影响，因为触底后的阶段切换需要允许轴保持不动。

### 顶部软限位保护

零点有效后，控制器会根据相对高度执行顶部软限位：

```text
height[i] = angle[i] - zero_angle[i]
```

如果某轴已经达到或超过顶部软限位，并且目标速度仍会让它继续上升，该轴输出会被禁用。

### `NaN` 输出语义

`NaN` 表示该轴当前不应由 4z 控制器主动驱动。速度为 `NaN` 时，对应力矩限制也为 `NaN`。

`IDLE`、`SUCCEEDED`、`FAILED` 和 `ABORTED` 等终止状态都会将四轴输出清理为 `NaN`。

## 失败条件

以下情况会使控制器返回 `FAILED` 并清理输出：

- 零点标定时缺少任一轴角度或底部限位输入；
- `LEVEL` 时尚未完成有效零点标定；
- `LEVEL` 时缺少任一轴角度、底部限位、pitch 或 roll 输入；
- PID 或速度分配计算产生非有限值；
- 当前参与调平的几何矩阵退化，无法求解 pitch/roll 修正。

缺少 `pitch_rate` 或 `roll_rate` 不会导致失败；控制器会使用 0 作为默认角速度。

## 简化流程图

```text
ZERO_CALIBRATE
    |
    v
四轴下行寻找底部限位
    |
    v
全部触底并保持完成
    |
    v
提交四轴软件零点
    |
    v
四轴上行回退
    |
    v
LEVEL
    |
    v
四轴姿态调平
    |
    v
姿态稳定？
    | 否
    +---- 继续调平
    |
   是
    |
    v
选择相对高度最小轴为参考轴
    |
    v
保持调平并整体下降
    |
    +---- 非参考轴先触底
    |          |
    |          v
    |    参考轴单独下降
    |
    v
参考轴触底
    |
    v
固定参考轴，剩余三轴调平
    |
    v
SUCCEEDED
```

## 参数参考

主要参数可以分为几组：

- 标定速度和保持时间：
  `zero_down_speed`、`zero_rollback_speed`、`zero_rollback_angle`、`zero_hold_ticks`；
- 姿态目标和稳定判定：
  `pitch_offset`、`roll_offset`、`level_pitch_tolerance`、`level_roll_tolerance`、`level_settle_ticks`；
- 姿态控制器：
  `level_pitch_pid_*`、`level_roll_pid_*`、`level_pitch_rate_kd`、`level_roll_rate_kd`；
- 输出和保护：
  `level_descend_speed`、`level_correction_velocity_deadband`、`chassis_velocity_limit`、`chassis_torque_limit`、`upper_soft_limit_angle`。

当前机器人使用的具体数值位于 `rmcs_bringup/config/dart-launcher.yaml`。这些数值是特定平台的调参结果，不应视为算法接口的一部分。

## 实现参考

- 控制器：`rmcs_core/src/controller/dart/four_z_chassis_controller.cpp`
- 硬件桥接：`rmcs_core/src/hardware/4z-dart-launcher.cpp`
- 启动参数：`rmcs_bringup/config/dart-launcher.yaml`
- 任务序列：`rmcs_dart_guidance/include/rmcs_dart_guidance/task/dart_chassis_level_task.hpp`
