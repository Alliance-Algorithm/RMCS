# 4z 底盘

## 相关电机和设备

1. yaw 电机。
2. chassis 电机四个，布局为长方形，轴距长约 876 mm，宽约 486 mm；电机带动丝杠驱动，丝杠导程约 1 mm，不作为严格控制量换算依据。
3. limit switch 四个，置于底部；顶部使用电控软限位。

## 设计约定

1. 四个 chassis 电机方向相同：
    - 向下运动时，电机速度为负。
    - 向上运动时，电机速度为正。
2. IMU 姿态符号：
    - pitch 抬头为正，低头为负。
    - roll 左高为正，右高为负。
3. 四轴逻辑命名：
    - `front_left`
    - `front_right`
    - `back_left`
    - `back_right`
4. 现有代码和硬件 topic 中的 `front_back` 是历史误命名，实际应为 `front_right`。后续代码和配置应迁移到 `front_right`；如需要兼容旧参数，只在参数读取处做兼容，不继续扩散 `front_back`。
5. 控制参数默认使用电机角度和速度单位，即 rad / rad/s。丝杠导程只用于估算一个初始输出量，不作为必须使用的换算依据。
6. controller 内部保存虚拟零点，不调用硬件侧 `DjiMotor::calibrate_zero_point()`。

## 控制接口

### 命令

`ChassisCommand` 保留以下命令：

- `IDLE`：清当前命令阶段、计数器和输出；四轴速度/力矩限制输出为 `NaN`；不清已提交的零点。
- `ABORT`：清当前命令阶段、计数器和输出；四轴速度/力矩限制输出为 `NaN`；状态为 `ABORTED`；不清已提交的零点。
- `ZERO_CALIBRATE`：四轴底部限位零点标定。
- `LEVEL`：4z 底座调平。

### 输出

4z controller 输出四轴速度目标和力矩限制：

- `/dart/chassis/<axis>_motor/control_velocity`
- `/dart/chassis/<axis>_motor/control_torque_limit`

其中 `<axis>` 为 `front_left`、`front_right`、`back_left`、`back_right`。

四个电机最终需要 `control_torque`，由 YAML 中四个 `PidController` 将速度闭环转换得到：

- measurement：`/dart/chassis/<axis>_motor/velocity`
- setpoint：`/dart/chassis/<axis>_motor/control_velocity`
- control：`/dart/chassis/<axis>_motor/control_torque`
- output_abs_limit：`/dart/chassis/<axis>_motor/control_torque_limit`

### 输入

4z controller 读取：

- 四轴电机 `angle`、`velocity`、`torque`。
- 四个底部 limit switch。
- `/dart/chassis/imu/pitch`
- `/dart/chassis/imu/roll`
- `/dart/chassis/imu/pitch_rate`
- `/dart/chassis/imu/roll_rate`

pitch / roll 控制器内置在 4z controller 中，使用 `controller/pid/pid_calculator.hpp`。第一版可以将 rate 阻尼参数置 0，但接口和参数需要预留。

## 内部状态

controller 至少保存：

- `zero_angle[4]`：已提交的四轴虚拟零点。
- `zero_valid`：四轴零点是否有效。
- `pending_zero_angle[4]`：本次标定过程中暂存的零点。
- `zero_latched[4]`：本次标定中每轴是否已经锁存 pending zero。
- `axis_hold_ticks[4]`：每轴触底后的 0 速度保持计数。
- `stage`：当前命令阶段。
- `reference_axis`：`LEVEL` 中选出的参考轴。

新命令抢占当前命令时，重置当前命令阶段、计数器、暂存量和输出，但保留已经提交的 `zero_angle` 和 `zero_valid`。

## `ZERO_CALIBRATE`

进入 `ZERO_CALIBRATE` 时先检查四轴角度输入和四个底部限位输入是否 ready；缺失时输出全 `NaN`，打印 error log，并返回 `FAILED`。

### 阶段 1：下行找底部限位

1. 四轴以 `-zero_down_speed` 下行。
2. 如果某轴在命令开始时 bottom limit 已经为 true，立即将该轴当前角度锁存到 `pending_zero_angle`。
3. 如果某轴运动过程中首次触发 bottom limit：
    - 锁存该轴当前角度为 `pending_zero_angle`。
    - 该轴速度置 0，开始 `zero_hold_ticks` 计数。
4. 对已经触底锁存的轴：
    - `axis_hold_ticks < zero_hold_ticks` 时，速度保持 0。
    - 保持完成后，该轴速度置 `NaN`。
5. 未触底轴继续下行。

四轴全部锁存后，才一次性提交：

- `zero_angle = pending_zero_angle`
- `zero_valid = true`

如果在四轴全部锁存前收到 `ABORT`、新命令抢占或任务失败，丢弃 `pending_zero_angle`，不污染旧零点。

### 阶段 2：上行回退

1. 四轴以 `zero_rollback_speed` 上行。
2. 回退不做严格位置闭环，只按相对零点行程判断：
    - `angle[i] - zero_angle[i] >= zero_rollback_angle` 时，该轴认为回退完成。
3. 单轴回退完成后，该轴速度置 0。
4. 四轴都回退完成后进入阶段 3。

### 阶段 3：回退后保持并完成

1. 四轴速度置 0，保持 `zero_hold_ticks`。
2. 保持完成后四轴速度置 `NaN`。
3. 状态返回 `SUCCEEDED`。

## `LEVEL`

进入 `LEVEL` 时必须检查：

- `zero_valid == true`
- 四轴角度输入 ready
- 四个底部限位输入 ready
- pitch / roll 输入 ready

如果 `zero_valid == false`，不运动，输出全 `NaN`，打印 error log，返回 `FAILED`。其他关键输入缺失时同样返回 `FAILED`。

相对高度定义：

```text
height[i] = angle[i] - zero_angle[i]
```

`height` 越小，表示该轴越接近底部零点。

### 几何符号

四轴几何系数：

| axis | pitch 系数 | roll 系数 |
| ---- | ---------- | --------- |
| front_left | +1 | +1 |
| front_right | +1 | -1 |
| back_left | -1 | +1 |
| back_right | -1 | -1 |

姿态控制输出：

```text
pitch_error = pitch_offset - pitch
roll_error  = roll_offset  - roll
pitch_cmd = pitch_pid.update(pitch_error) - level_pitch_rate_kd * pitch_rate
roll_cmd  = roll_pid.update(roll_error)  - level_roll_rate_kd  * roll_rate
```

`pitch_offset` / `roll_offset` 单位为弧度，表示平台调平时 IMU 应读到的目标
pitch / roll 值，用于修正 IMU 与平台的安装误差。例如平台物理水平但 IMU pitch
读数为 `0.01`，则配置 `pitch_offset: 0.01`。

`pitch_cmd` 和 `roll_cmd` 表示需要分配到四轴上的速度修正量。

### 速度分配

四轴调平时，求解：

```text
B * v = [pitch_cmd, roll_cmd]
```

其中 `B` 为 pitch / roll 几何系数组成的矩阵，`v` 为四轴速度修正。使用最小范数解，避免引入不必要的整体升降速度。

三轴调平时，固定 `reference_axis`，只对剩余三轴构造 `B3`，求：

```text
B3 * v3 = [pitch_cmd, roll_cmd]
```

同样使用最小范数解。不要简单复用四轴分配后把固定轴裁掉。

### 阶段 1：原地调平并选择参考轴

1. 不叠加整体下降速度，只使用 pitch / roll 修正速度。
2. 当 pitch 和 roll 连续 `level_settle_ticks` 都在容差内，且四轴修正速度绝对值都低于 `level_correction_velocity_deadband` 时，认为姿态稳定。
3. 在姿态稳定时，选择 `height` 最小的轴作为 `reference_axis`，进入阶段 2。

### 阶段 2：保持调平并整体下降

1. 在调平速度修正的基础上，叠加整体慢速下降：

```text
v[i] = level_correction[i] - level_descend_speed
```

2. 任意轴触发 bottom limit 后：
    - 如果触底轴就是 `reference_axis`，进入阶段 3。
    - 如果触底轴不是 `reference_axis`，进入参考轴单独下降子阶段。

### 阶段 2b：参考轴单独下降

1. 其他轴输出 `NaN`。
2. `reference_axis` 继续以 `-level_descend_speed` 下行。
3. 当 `reference_axis` 触发 bottom limit 后，进入阶段 3。

### 阶段 3：固定参考轴，三轴调平

1. `reference_axis` 输出 0 速度保持，不参与调平分配。
2. 只允许另外三轴使用三轴最小范数解继续调平。
3. 当 pitch 和 roll 连续 `level_settle_ticks` 都在容差内，且允许运动轴的修正速度绝对值都低于 `level_correction_velocity_deadband` 时，状态返回 `SUCCEEDED`。

## 保护规则

保护规则在所有速度计算完成后、发布输出前统一执行，优先级最高。

1. 底部限位保护：
    - 某轴 bottom limit 为 true 且输出速度 `< 0` 时，该轴速度裁剪为 `NaN`。
    - 输出速度为 0 时不受底部限位保护影响，用于触底后的保持阶段。
2. 顶部软限位保护：
    - 仅在 `zero_valid == true` 时启用。
    - 某轴 `height >= upper_soft_limit_angle` 且输出速度 `> 0` 时，该轴速度裁剪为 `NaN`。
3. 速度限幅：
    - 保护前先将速度限制到 `[-chassis_velocity_limit, chassis_velocity_limit]`。
    - 限位保护仍然可以把限幅后的速度裁剪为 `NaN`。
4. 力矩限制：`
    - 有效运动或 0 速度保持时输出 `chassis_torque_limit`。
    - `NaN` 速度输出时，力矩限制也输出 `NaN`。
5. 软件层不对限位开关再做去抖，信任已有输入。

## 失败策略

不设计复杂错误码。以下情况直接输出全 `NaN`，打印 error log，并返回 `FAILED`：

- `ZERO_CALIBRATE` 缺少四轴角度或底部限位输入。
- `LEVEL` 时 `zero_valid == false`。
- `LEVEL` 缺少四轴角度、底部限位或 pitch / roll 输入。
- 计算结果出现非有限值，且无法通过裁剪恢复为安全输出。

`IDLE` 和 `ABORT` 不属于失败，不打印 error。

## 参数清单

速度参数使用正数幅值，代码内部决定方向：

- `zero_down_speed`
- `zero_rollback_speed`
- `level_descend_speed`

行程和时间：

- `zero_rollback_angle`
- `zero_hold_ticks`，默认 500
- `upper_soft_limit_angle`
- `level_settle_ticks`

调平参数：

- `level_pitch_tolerance`
- `level_roll_tolerance`
- `pitch_offset`
- `roll_offset`
- `level_correction_velocity_deadband`
- `level_pitch_pid_kp`
- `level_pitch_pid_ki`
- `level_pitch_pid_kd`
- `level_pitch_pid_integral_min`
- `level_pitch_pid_integral_max`
- `level_pitch_pid_output_min`
- `level_pitch_pid_output_max`
- `level_roll_pid_kp`
- `level_roll_pid_ki`
- `level_roll_pid_kd`
- `level_roll_pid_integral_min`
- `level_roll_pid_integral_max`
- `level_roll_pid_output_min`
- `level_roll_pid_output_max`
- `level_pitch_rate_kd`
- `level_roll_rate_kd`

输出限制：

- `chassis_velocity_limit`
- `chassis_torque_limit`

手动控制：

- `manual_chassis_velocity_sensitivity`

## 验收场景

1. 未标定时收到 `LEVEL`：四轴不动，打印 error，状态 `FAILED`。
2. `ZERO_CALIBRATE` 开始时已有轴触底：该轴立即锁存 pending zero。
3. `ZERO_CALIBRATE` 中部分轴触底：触底轴速度 0 保持，再变 `NaN`；未触底轴继续下行。
4. 四轴全部触底前 `ABORT`：旧零点不变。
5. 四轴全部触底后：零点整体提交，然后统一上行回退。
6. `IDLE` 或 `ABORT` 后，再次 `LEVEL` 不丢失已提交零点。
7. `LEVEL` 阶段 1：姿态稳定后选择 `height` 最小的轴作为参考轴。
8. `LEVEL` 阶段 2：非参考轴先触底时，只让参考轴继续下降直到触底。
9. `LEVEL` 阶段 3：参考轴固定，另外三轴用三轴最小范数解调平。
10. 任意阶段触发底部限位或顶部软限位保护时，该轴不会继续朝受限方向运动。
