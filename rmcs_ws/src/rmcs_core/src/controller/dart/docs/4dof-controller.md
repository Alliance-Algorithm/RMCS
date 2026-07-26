# 四自由度底座控制方案

四自由度底座包含 `pitch`、`roll`、`yaw`、`height`。其中 `yaw` 只输出控制速度，由外部通用 PID 控制器把 yaw 速度目标转换成 yaw 电机力矩；4Z 底盘负责 `pitch`、`roll`、`height`，采用 IMU 作为 `pitch/roll` 主反馈，四个 Z 轴相对编码器用于 `height`、同步约束和触底标定。

## 目标

- 当前阶段上层只发离散命令：触底标定、调零、下降、空闲。
- 当前阶段不提供任意 `target_roll/target_pitch/target_height` 的通用姿态目标接口。
- 空闲时四个 Z 轴输出 `NaN`，依赖丝杠自锁保持位置。
- 没有绝对编码器时支持混合零点：上电零点可用于低速调试，触底标定后才认为 `height` 绝对可信。
- 每个 Z 轴有独立底部限位开关。某轴触底后，禁止该轴继续向下输出是最高优先级；该轴仍允许向上输出。
- `roll/pitch` 是硬目标，`height` 是软目标；若调整姿态时触底导致目标 `height` 不可达，自动降级为只控制 `roll/pitch`。
- `front_back_motor` 是现有硬件接口名，控制方案中按逻辑 `front_right_motor` 使用，不改已有 topic。

控制优先级从高到低：

1. 底部限位触发轴禁止向下控制。
2. 已标定轴超过 `max_stroke` 后禁止继续向上控制。
3. `IDLE`/`ABORT` 释放输出。
4. `roll/pitch` 姿态目标。
5. `height` 软目标。
6. 四轴同步项和其它辅助控制。

任何命令、任何控制模式、任何 PID 或同步项输出，都不能绕过第一优先级。

## 硬件接口

相关电机：

- `/dart/chassis/front_left_motor`
- `/dart/chassis/front_back_motor`，逻辑上是 front-right
- `/dart/chassis/back_left_motor`
- `/dart/chassis/back_right_motor`
- `/dart/yaw/motor`

每个 DJI 电机已有接口：

- 输入状态：`angle`、`velocity`、`torque`、`encoder_angle`
- 输出控制：`control_torque`

需要补齐的硬件状态接口：

- `/dart/chassis/imu/pitch`
- `/dart/chassis/imu/roll`
- `/dart/chassis/imu/pitch_rate`
- `/dart/chassis/imu/roll_rate`
- `/dart/chassis/front_left_motor/bottom_limit_switch`
- `/dart/chassis/front_back_motor/bottom_limit_switch`
- `/dart/chassis/back_left_motor/bottom_limit_switch`
- `/dart/chassis/back_right_motor/bottom_limit_switch`

限位开关输入假设已经由上游完成防抖，控制器直接使用 bool 值。

需要的 yaw 控制接口：

- `4dof-controller` 输出 `/dart/yaw/motor/control_velocity`
- 外部 `rmcs_core::controller::pid::PidController` 读取 yaw motor velocity 和上述 setpoint，输出 `/dart/yaw/motor/control_torque`

## 坐标约定

底座坐标系：

- `x` 正方向指向前方。
- `y` 正方向指向左侧。
- `z` 正方向向上。
- `height` 为四个 Z 轴相对零点的平均上升高度，正方向向上。
- `pitch` 沿用现有 chassis IMU 约定：`pitch < 0` 表示前方更高。
- `roll` 沿用现有 chassis IMU 约定：`roll > 0` 表示左侧更高。

四个轴的位置参数从 yaml 读取，默认矩形布局：

```text
front_left:  x = +axis_x, y = +axis_y
front_back:  x = +axis_x, y = -axis_y
back_left:   x = -axis_x, y = +axis_y
back_right:  x = -axis_x, y = -axis_y
```

小角度下，四角高度和底座姿态满足：

```text
z_i = height - pitch * x_i + roll * y_i
```

这个符号关系可以检查：当前方更高时，前侧 `z_i` 更大，因此 `pitch` 为负；左侧更高时，左侧 `z_i` 更大，因此 `roll` 为正。

## 组件划分

### 4dof-controller

职责：

- 读取上层 `/dart/4dof/command`。
- 将 4DOF 命令翻译为 `/dart/chassis/4z/command`。
- 输出 yaw 控制速度 `/dart/yaw/motor/control_velocity`。
- 汇总 4Z status 到 `/dart/4dof/status`。

命令建议定义为 `FourDofCommand`：

```cpp
enum class FourDofCommand : uint8_t {
    IDLE = 0,
    ABORT,
    CALIBRATE_BOTTOM,
    LEVEL_ZERO,
    DOWN,
};
```

命令语义：

| 命令 | 4Z 命令 | yaw control_velocity | 4DOF status |
|----|----|----|----|
| `IDLE` | `IDLE` | `NaN` | `IDLE` |
| `ABORT` | `ABORT` | `NaN` | `ABORTED` |
| `CALIBRATE_BOTTOM` | `CALIBRATE_BOTTOM` | `0.0` | 跟随 4Z status |
| `LEVEL_ZERO` | `LEVEL_ZERO` | `0.0` | 跟随 4Z status |
| `DOWN` | `DOWN` | `0.0` | 跟随 4Z status |

后续视觉闭环接入时，只需要改变 yaw velocity setpoint 的来源，不需要让 4Z 控制器使用 yaw 或 IMU yaw。

### 4z-axis-chassis-status

职责：

- 读取四个 Z 轴电机角度和速度。
- 读取四个底部限位开关。
- 读取 IMU pitch/roll/pitch_rate/roll_rate。
- 输出当前 `pitch`、`roll`、`height`、`pitch_rate`、`roll_rate`、`height_calibrated`。

状态接口建议：

- `/dart/chassis/pitch`
- `/dart/chassis/roll`
- `/dart/chassis/height`
- `/dart/chassis/pitch_rate`
- `/dart/chassis/roll_rate`
- `/dart/chassis/height_calibrated`
- `/dart/chassis/motor_estimated_pitch`，调试用
- `/dart/chassis/motor_estimated_roll`，调试用

### 4z-axis-chassis-controller

职责：

- 读取 `/dart/chassis/4z/command`。
- 读取 4Z status 输出和限位开关。
- 解算四个 Z 轴 `control_torque`。
- 发布 `/dart/chassis/4z/status`。
- 发布控制降级诊断：`/dart/chassis/height_degraded`、`/dart/chassis/height_target_reachable`。

命令建议定义为 `FourZChassisCommand`，和 4DOF 命令保持同名映射：

```cpp
enum class FourZChassisCommand : uint8_t {
    IDLE = 0,
    ABORT,
    CALIBRATE_BOTTOM,
    LEVEL_ZERO,
    DOWN,
};
```

## 状态解算

### 零点

每个轴维护两个零点：

- `startup_zero_angle_i`：上电或组件启动时记录的电机角度。用于未触底标定前低速调试。
- `bottom_zero_angle_i`：该轴底部限位触发时记录的电机角度。用于绝对 `height`。

参考零点选择：

```text
if all bottom_zero_angle_i valid:
    reference_zero_i = bottom_zero_angle_i
    height_calibrated = true
else:
    reference_zero_i = startup_zero_angle_i
    height_calibrated = false
```

触底标定过程中，某个轴限位触发时立即锁存该轴 `bottom_zero_angle_i`。如果上电时或标定命令开始时某个轴已经压住限位，则认为该轴当前电机角度就是底部零点并立即锁存。只有四个轴都锁存后，`height_calibrated` 才为 true。

### 单轴高度

每个轴的高度由电机多圈角度换算：

```text
z_i = axis_height_per_motor_rad_i * axis_height_direction_i * (motor_angle_i - reference_zero_i)
v_i = axis_height_per_motor_rad_i * axis_height_direction_i * motor_velocity_i
```

参数说明：

- `axis_height_per_motor_rad_i`：电机输出轴每转 1 rad 对应的 Z 轴线位移，单位 m/rad。
- `axis_height_direction_i`：符号参数。取 `+1` 表示电机角度增大时轴向上，取 `-1` 表示电机角度增大时轴向下。

触底标定后，使用 `max_stroke` 限制每个轴从底部零点开始的最大上行电机角度：

```text
stroke_angle_i = axis_height_direction_i * (motor_angle_i - bottom_zero_angle_i)
stroke_angle_i = clamp(stroke_angle_i, 0, max_stroke)  # 诊断值
```

当 `stroke_angle_i >= max_stroke` 时，该轴禁止继续向上输出，但仍允许向下输出。`max_stroke` 是软上限，单位 rad，表示触底标定后的零点开始电机允许转动的最大角度值。

### height

```text
height = (z_fl + z_fr + z_bl + z_br) / 4
height_velocity = (v_fl + v_fr + v_bl + v_br) / 4
```

其中 `fr` 对应现有接口名 `front_back_motor`。

### pitch/roll

控制用 pitch/roll 直接来自 IMU：

```text
pitch = imu_pitch
roll = imu_roll
pitch_rate = imu_pitch_rate
roll_rate = imu_roll_rate
```

四轴编码器也可以估计一个仅用于诊断的姿态：

```text
motor_estimated_pitch = -((z_fl + z_fr) - (z_bl + z_br)) / (4 * axis_x)
motor_estimated_roll  =  ((z_fl + z_bl) - (z_fr + z_br)) / (4 * axis_y)
```

诊断姿态不作为主反馈，避免相对编码器零点误差影响调平闭环。

## 目标解算

当前阶段不提供任意 `target_height/target_pitch/target_roll` 外部目标接口。下面的目标姿态公式只作为 `LEVEL_ZERO` 内部调零和降级判断使用：

```text
target_z_i = target_height - target_pitch * x_i + target_roll * y_i
```

目标优先级：

```text
roll/pitch: hard target
height: soft target
```

`LEVEL_ZERO` 正常情况下同时控制当前锁定的 `target_height` 和 `target_pitch=0/target_roll=0`。如果在调零过程中触发底部限位，并且继续追踪 `target_height` 会要求某个触底轴继续向下，则当前命令进入 `HEIGHT_DEGRADED` 模式。降级后只保证 `target_pitch/target_roll`，`height` 不再作为动作完成判据。

降级状态是命令周期内锁存的：一旦进入 `HEIGHT_DEGRADED`，直到命令切换、`IDLE` 或 `ABORT` 才复位，避免在限位附近反复切换。

触发降级的判据：

```text
if command == LEVEL_ZERO
   && bottom_limit_switch_i
   && target_z_i < z_i + height_degrade_margin:
    height_degraded = true
```

如果输出门控已经屏蔽了任意轴的向下控制，也进入降级：

```text
if command == LEVEL_ZERO
   && bottom_limit_switch_i
   && limited_axis_effort_i < 0:
    height_degraded = true
```

如果某轴已经有可信底部零点，也可以用几何可达性提前判断：

```text
axis_floor_i = 0.0  # 以 bottom_zero 为参考时的底部高度
if target_z_i < axis_floor_i + height_degrade_margin:
    height_target_reachable = false
```

四个常用目标：

### IDLE

不生成目标高度。四个 Z 轴输出 `NaN`。

### CALIBRATE_BOTTOM

不使用目标高度。所有未触底轴以固定向下控制量运动：

```text
axis_effort_i = -calibrate_down_effort
```

某轴限位触发后，锁存底部零点。若该轴控制量仍为向下，则最高优先级限位门控会输出 `NaN`；若后续控制量为向上，则允许该轴向上输出。四轴全部触发后命令成功。

### LEVEL_ZERO

命令边沿锁定当前 `height` 作为软目标，姿态目标固定为零：

```text
target_height = current_height_on_command_edge
target_pitch = 0.0
target_roll = 0.0
```

正常情况下调零保持当前 `height`。如果姿态调整过程中触底导致该 `height` 不可达，则进入 `HEIGHT_DEGRADED`，后续只用 `pitch/roll` 判断完成。

### DOWN

命令边沿锁定当前 `pitch/roll`，整体向下运动：

```text
target_pitch = current_pitch_on_command_edge
target_roll = current_roll_on_command_edge
height effort = -down_effort
```

任意一个底部限位触发时，命令成功并释放全部 Z 轴。这个命令不会要求四轴全部触底，因此通常不一定完成完整高度标定。

## 控制解算

控制器内部先在 `height/pitch/roll` 三个模态上计算控制量，再映射到四个 Z 轴。控制模式分为两种：

- `NORMAL_POSE_CONTROL`：同时控制 `height/pitch/roll`。
- `HEIGHT_DEGRADED`：只控制 `pitch/roll`，禁用 height 闭环。

### 误差

```text
height_error = target_height - height
pitch_error = target_pitch - pitch
roll_error = target_roll - roll
```

`height_error` 只在 `NORMAL_POSE_CONTROL` 中参与控制和完成判据。进入 `HEIGHT_DEGRADED` 后，`height_error` 仅用于诊断。

### 模态控制量

`NORMAL_POSE_CONTROL` 使用三个闭环：

```text
u_height = PID_height(height_error) - kd_height_velocity * height_velocity
u_pitch  = PID_pitch(pitch_error)   - kd_pitch_rate * pitch_rate
u_roll   = PID_roll(roll_error)     - kd_roll_rate * roll_rate
```

`HEIGHT_DEGRADED` 禁用 height PID，只计算姿态控制量：

```text
u_pitch = PID_pitch(target_pitch - pitch) - kd_pitch_rate * pitch_rate
u_roll  = PID_roll(target_roll - roll)    - kd_roll_rate * roll_rate
```

`DOWN` 命令使用固定向下高度控制量，同时保持姿态：

```text
u_height = -down_effort
u_pitch  = PID_pitch(target_pitch - pitch) - kd_pitch_rate * pitch_rate
u_roll   = PID_roll(target_roll - roll)    - kd_roll_rate * roll_rate
```

`CALIBRATE_BOTTOM` 命令不做姿态闭环：

```text
u_height = -calibrate_down_effort
u_pitch = 0
u_roll = 0
```

### 模态到四轴

每个轴的正方向定义为“向上控制量”。`NORMAL_POSE_CONTROL` 下四轴控制量为：

```text
axis_effort_i = u_height - u_pitch * x_i + u_roll * y_i + u_sync_i
```

展开后：

```text
fl = u_height - u_pitch * (+axis_x) + u_roll * (+axis_y) + u_sync_fl
fr = u_height - u_pitch * (+axis_x) + u_roll * (-axis_y) + u_sync_fr
bl = u_height - u_pitch * (-axis_x) + u_roll * (+axis_y) + u_sync_bl
br = u_height - u_pitch * (-axis_x) + u_roll * (-axis_y) + u_sync_br
```

这个映射的直观含义：

- `u_height > 0`：四轴一起向上。
- `u_pitch > 0`：前轴向下、后轴向上，使 pitch 增大。
- `u_roll > 0`：左轴向上、右轴向下，使 roll 增大。

`HEIGHT_DEGRADED` 下不能继续追踪目标 `height`，但仍要尽量让触底轴不再被压下，同时让其它轴通过上升完成姿态调整。先计算纯姿态控制量：

```text
raw_i = -u_pitch * x_i + u_roll * y_i + u_sync_i
```

然后只针对已经触底的轴求一个公共向上偏置：

```text
u_height_bias = max(0, max_for_limited_axes(-raw_i + limit_hold_margin))
axis_effort_i = u_height_bias + raw_i
```

`u_height_bias` 对四个轴相同，只会改变整体高度，不会改变 `pitch/roll` 控制分量。这样通常能避免触底轴继续向下，其它轴仍可通过向上运动达到目标 `pitch/roll`。

如果当前没有任何轴处于限位触发状态，但已经通过几何判断发现目标 `height` 不可达，则也进入 `HEIGHT_DEGRADED`。此时先求满足目标姿态和底部约束的最低整体高度：

```text
base_i = -target_pitch * x_i + target_roll * y_i
min_height_for_target_attitude = max_i(axis_floor_i + height_degrade_margin - base_i)
effective_height_target = max(height, min_height_for_target_attitude)
height_bias_error = effective_height_target - height
```

再用只允许向上的 bias 控制器生成公共偏置：

```text
u_height_bias = max(0, PID_height_bias(height_bias_error))
axis_effort_i = u_height_bias + raw_i
```

`effective_height_target` 只用于使目标姿态对应的四轴高度不低于底部约束，不作为完成判据。

### 同步项

IMU 是姿态主反馈，但四个相对编码器仍需要限制轴间机械误差。先用当前 IMU 姿态和平均高度计算每个轴“应该处于的高度”：

```text
plane_z_i = height - pitch * x_i + roll * y_i
sync_error_i = plane_z_i - z_i
u_sync_i = PID_sync(sync_error_i) - kd_sync_velocity * (v_i - height_velocity)
```

同步项必须限幅，避免编码器零点误差或 IMU 噪声压过 pitch/roll 主闭环：

```text
if bottom_limit_switch_i && u_sync_i < 0:
    u_sync_i = 0

if height_calibrated && stroke_angle_i >= max_stroke && u_sync_i > 0:
    u_sync_i = 0

u_sync_i = clamp(u_sync_i, -sync_output_max, +sync_output_max)
```

也就是说，触底轴的同步项如果会导致该轴继续向下，先置 0；达到 `max_stroke` 的轴如果同步项会导致继续向上，也先置 0。最终输出仍必须经过限位和行程门控。

### 轴向控制量到电机力矩

`axis_effort_i` 是“轴向上为正”的抽象控制量。无保护时的电机力矩映射为：

```text
raw_control_torque_i = axis_torque_direction_i * clamp(axis_effort_i, -axis_effort_max_i, +axis_effort_max_i)
```

参数说明：

- `axis_torque_direction_i = +1`：正电机力矩使该轴向上。
- `axis_torque_direction_i = -1`：正电机力矩使该轴向下。

### 输出门控

底部限位保护是最高优先级输出门控，在所有命令和所有控制模式中都生效。`max_stroke` 是触底标定后的上行软限位。控制器必须先完成 PID、姿态分配、同步项和限幅，再执行输出门控；门控结果才是最终写入电机的 `control_torque`。

统一门控逻辑：

```text
limited_axis_effort_i = clamp(axis_effort_i, -axis_effort_max_i, +axis_effort_max_i)

if bottom_limit_switch_i && limited_axis_effort_i < 0:
    control_torque_i = NaN
else if height_calibrated && stroke_angle_i >= max_stroke && limited_axis_effort_i > 0:
    control_torque_i = NaN
else:
    control_torque_i = axis_torque_direction_i * limited_axis_effort_i
```

符号语义：

```text
axis_effort_i > 0  =>  该轴向上
axis_effort_i < 0  =>  该轴向下
```

因此，限位触发后的允许/禁止关系为：

```text
bottom_limit_switch_i == true && limited_axis_effort_i < 0  => control_torque_i = NaN
bottom_limit_switch_i == true && limited_axis_effort_i > 0  => allow output
bottom_limit_switch_i == true && limited_axis_effort_i = 0  => allow output 0.0
```

`max_stroke` 触发后的允许/禁止关系为：

```text
height_calibrated == true && stroke_angle_i >= max_stroke && limited_axis_effort_i > 0  => control_torque_i = NaN
height_calibrated == true && stroke_angle_i >= max_stroke && limited_axis_effort_i < 0  => allow output
height_calibrated == true && stroke_angle_i >= max_stroke && limited_axis_effort_i = 0  => allow output 0.0
```

在 `HEIGHT_DEGRADED` 中，`u_height_bias` 只是为了减少触底轴产生向下控制的需求；如果限幅、同步项或姿态控制仍让触底轴出现向下控制，最高优先级底部限位门控必须屏蔽该轴，输出 `NaN`。降级模式不能绕过输出门控。

## 命令状态机

所有命令采用电平保持语义：上层持续写同一个命令，控制器持续执行；命令变为 `IDLE` 后释放输出。

### IDLE

- 四轴 `control_torque = NaN`。
- reset 所有 PID。
- status = `IDLE`。

### ABORT

- 四轴 `control_torque = NaN`。
- reset 所有 PID。
- status = `ABORTED`。

### CALIBRATE_BOTTOM

- 命令边沿清空本次标定进度，但不清空已知底部零点，除非参数 `clear_bottom_zero_on_calibrate_start=true`。
- 未触底轴输出固定向下控制量。
- 触底轴锁存 `bottom_zero_angle_i`；如果该轴控制量仍为向下，最高优先级限位门控输出 `NaN`。
- 触底轴后续如果需要向上运动，限位门控允许向上控制量输出。
- 四轴都触底后 status = `SUCCEEDED`，四轴保持 `NaN`。
- 超过 `calibrate_timeout_ticks` 后 status = `FAILED`，四轴 `NaN`。

### LEVEL_ZERO

- 命令边沿锁定当前 height 作为软目标。
- 持续闭环到 `pitch=0/roll=0`。
- 正常模式下，当以下条件连续满足 `level_settle_ticks` 后 status = `SUCCEEDED`：

```text
abs(height_error) < level_height_tolerance
abs(height_velocity) < level_height_velocity_tolerance
abs(pitch_error) < level_pitch_tolerance
abs(roll_error) < level_roll_tolerance
abs(pitch_rate) < level_pitch_rate_tolerance
abs(roll_rate) < level_roll_rate_tolerance
```

- 降级模式下，`height` 不再作为动作完成判据，当以下条件连续满足 `level_settle_ticks` 后 status = `SUCCEEDED`：

```text
abs(pitch_error) < level_pitch_tolerance
abs(roll_error) < level_roll_tolerance
abs(pitch_rate) < level_pitch_rate_tolerance
abs(roll_rate) < level_roll_rate_tolerance
```

- 成功后四轴输出 `NaN`。
- 如果触发限位导致目标 height 不可达，控制器进入 `HEIGHT_DEGRADED`，只要 `pitch/roll` 收敛就成功。
- 如果进入降级后 `pitch/roll` 仍无法收敛，超过 `level_timeout_ticks` 后 status = `FAILED`。

### DOWN

- 命令边沿锁定当前 pitch/roll。
- 持续整体下行，同时保持命令边沿的 pitch/roll。
- 如果命令开始时任意一个底部限位已经为 true，立即 status = `SUCCEEDED`，四轴输出 `NaN`。
- 执行过程中任意一个底部限位变为 true 后 status = `SUCCEEDED`，四轴输出 `NaN`。
- 超过 `down_timeout_ticks` 后 status = `FAILED`，四轴输出 `NaN`。

## 参数建议

```yaml
four_z_chassis_status:
  ros__parameters:
    axis_x: 0.20
    axis_y: 0.15
    front_left_height_per_motor_rad: 0.001
    front_back_height_per_motor_rad: 0.001
    back_left_height_per_motor_rad: 0.001
    back_right_height_per_motor_rad: 0.001
    front_left_height_direction: 1.0
    front_back_height_direction: 1.0
    back_left_height_direction: 1.0
    back_right_height_direction: 1.0

four_z_chassis_controller:
  ros__parameters:
    calibrate_down_effort: 0.2
    down_effort: 0.2
    max_stroke: 20.0
    axis_effort_max: 1.0
    sync_output_max: 0.1
    height_degrade_margin: 0.001
    limit_hold_margin: 0.02
    level_height_tolerance: 0.005
    level_height_velocity_tolerance: 0.01
    level_pitch_tolerance: 0.01
    level_roll_tolerance: 0.01
    level_pitch_rate_tolerance: 0.02
    level_roll_rate_tolerance: 0.02
    level_settle_ticks: 250
    level_timeout_ticks: 5000
    calibrate_timeout_ticks: 5000
    down_timeout_ticks: 5000
    height_kp: 0.0
    height_ki: 0.0
    height_kd: 0.0
    height_bias_kp: 0.0
    height_bias_ki: 0.0
    height_bias_kd: 0.0
    pitch_kp: 0.0
    pitch_ki: 0.0
    pitch_kd: 0.0
    roll_kp: 0.0
    roll_ki: 0.0
    roll_kd: 0.0
    sync_kp: 0.0
    sync_ki: 0.0
    sync_kd: 0.0
```

实际参数需要根据丝杠导程、电机方向、机构尺寸和重力负载调试。第一版建议先将 `height_kp` 设为 0，仅开启 `pitch/roll` 小增益和较小 `sync_output_max`。

## 调试顺序

1. 只加载 status 组件，确认四轴上移时 `z_i` 增大，IMU pitch/roll 符号符合约定。
2. 单独验证四个底部限位开关 topic，确认每个开关对应正确轴。
3. 低力矩执行 `CALIBRATE_BOTTOM`，确认触底轴立即输出 `NaN`，四轴触底后 `height_calibrated=true`。
4. 在离地高度执行 `LEVEL_ZERO`，先只开 pitch PID，再开 roll PID，最后增加同步项。
5. 在 `LEVEL_ZERO` 过程中手动触发单轴底部限位，确认 `height_degraded=true`，且完成判据只检查 `pitch/roll`。
6. 执行 `DOWN`，确认命令开始时或执行过程中任意限位为 true 都会释放全部 Z 轴并成功。
7. 接入 `4dof-controller` 和 yaw velocity PID，确认 active 命令期间 yaw velocity setpoint 为 `0.0`，空闲时为 `NaN`。
