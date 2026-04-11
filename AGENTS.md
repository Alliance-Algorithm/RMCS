# RMCS AGENTS

这份文档给后续接手 RMCS 的 agent 用，目标是两件事：

1. 快速理解 RMCS 的运行时架构，而不是只看某一个包。
2. 快速定位四套兵种配置的入口、差异和调参位置。

如果你只记一件事，请记住：

- RMCS 的运行时不是靠 ROS topic 自动拼起来的，而是靠 `rmcs_executor` 在进程内把 `Component::InputInterface` 和 `OutputInterface` 按“字符串名字 + C++ 类型”配对。
- 兵种不是写死在代码里的，而是由 `rmcs_ws/src/rmcs_bringup/config/*.yaml` 的 `components` 列表决定。
- 大多数行为改动，真正的入口不是 launch，而是某个兵种 YAML。

## 1. 仓库地图

### 顶层

- `README.md`
  - 开发、构建、部署、`set-robot` / `launch-rmcs` 的说明。
- `rmcs_ws/src`
  - ROS 2 工作区的主体。
- `tools/pid_tuning`
  - PID 识别与调参脚本。

### 工作区主包

- `rmcs_bringup`
  - 启动入口与兵种 YAML。
- `rmcs_executor`
  - 组件执行器。负责插件加载、输入输出配对、依赖排序、1kHz 更新循环。
- `rmcs_core`
  - 机器人业务主体，含硬件抽象、控制器、裁判系统、广播器。
- `rmcs_msgs`
  - RMCS 自定义消息、枚举、轻量接口类型。
- `rmcs_description`
  - typed TF 描述，定义 `base_link / yaw_link / pitch_link / camera_link` 等结构。
- `fast_tf`
  - RMCS 自己的轻量 TF 计算/广播工具。
- `rmcs_utility`
  - CRC、环形缓冲区、计时器、结构化绑定等基础工具。
- `serial`
  - 串口库。
- `hikcamera`
  - 海康相机驱动，当前默认兵种配置里没有启用 auto aim 链路。

### `rmcs_core` 内部层次

- `src/hardware`
  - 整机硬件插件与底层设备封装。
- `src/controller`
  - 底盘、云台、射击、PID 控制器。
- `src/referee`
  - 裁判系统收发、UI 合成。
- `src/broadcaster`
  - TF 和数值广播器。
- `librmcs`
  - RMCS 下位机通讯/固件相关代码与构建脚本。运行时主工程链接的是 `librmcs-sdk`，但这里仍然是理解板间协议和固件结构的重要参考。

## 2. 运行时架构

### 2.1 启动链路

启动入口在 `rmcs_ws/src/rmcs_bringup/launch/rmcs.launch.py`。

- launch 读取参数 `robot`
- 决定加载哪个 `rmcs_ws/src/rmcs_bringup/config/<robot>.yaml`
- 只启动一个节点：`rmcs_executor`
- `rmcs_executor` 再按 YAML 的 `components` 列表加载所有插件组件

`auto.<robot>` 这种写法在 launch 里被识别了，但自动逻辑目前还没有展开，代码里是 `pass`。

### 2.2 组件系统

`rmcs_executor` 的核心约束：

- 每个插件都继承 `rmcs_executor::Component`
- 组件通过 `register_input("/name", ...)` 和 `register_output("/name", ...)` 声明接口
- 执行器会把同名、同类型的输入输出直接在进程内绑定
- 如果输出重名、类型不匹配、或者出现循环依赖，启动直接失败
- 更新顺序不是写死的，而是根据依赖关系做拓扑排序

这意味着：

- 改 topic 名不是“小改动”，它会影响整个组件图
- 新增兵种时，第一件事应该看 YAML 里的组件链是否闭环
- 组件的输入输出名，本质上就是 RMCS 的“内部总线协议”

### 2.3 执行模型

`rmcs_executor::Executor` 的执行模型非常重要：

- 自动注入一个 `PredefinedMsgProvider`
  - `/predefined/update_rate`
  - `/predefined/update_count`
  - `/predefined/timestamp`
- 从 YAML 读取 `update_rate`，当前四套兵种都是 `1000.0`
- 开一个独立线程做固定周期循环
- 每轮按依赖顺序调用所有组件的 `update()`
- ROS executor 仍然存在，但主要负责组件里那些继承了 `rclcpp::Node` 的订阅/发布/参数机制

因此 RMCS 是：

- ROS 2 宿主
- 进程内组件图
- 固定频率实时-ish 控制循环

三者叠在一起的系统。

### 2.4 partner component

很多硬件插件会在构造时创建 partner component，例如：

- `OmniInfantry` 会创建 `InfantryCommand`
- `MecanumHero` 会创建 `HeroCommand`
- `SteeringInfantry` / `SteeringHero` 都会创建自己的 command 组件
- `DualYawController` 会创建状态组件，合成总 yaw 角和角速度

这些 partner component 往往负责“把控制量真正打到板子上”，所以：

- 状态组件负责采集并注册输出
- command 组件负责消耗控制输出并发送 CAN / UART 指令

### 2.5 一个典型数据流

以兵种主链为例，一般是：

1. `hardware::*`
   - 输出遥控器、IMU、电机角度/速度/力矩、超级电容、裁判串口接口、TF。
2. `referee::Status`
   - 从 `/referee/serial` 解析比赛状态，输出热量、功率限制、弹量等规则信息。
3. `controller::*`
   - 根据 `/remote/*`、`/gimbal/*`、`/chassis/*`、`/referee/*` 计算新的控制量。
4. `hardware::*Command`
   - 读取 `/.../control_torque`、`/.../control_velocity`、`/.../control_angle`
   - 转成 CAN/UART 指令发往下位机。
5. `referee::Command` / `broadcaster::*`
   - 向裁判系统发 UI，或向 ROS 网络广播 TF / 数值。

## 3. 信号命名约定

RMCS 的内部接口大致按命名空间分层：

- `/remote/*`
  - DR16 遥控器与鼠标键盘。
- `/chassis/*`
  - 底盘姿态、速度、功率、超级电容、轮组和爬台机构。
- `/gimbal/*`
  - 云台角度、摩擦轮、供弹、推弹、发射状态。
- `/referee/*`
  - 裁判系统输入输出、UI 交互。
- `/tf`
  - 整车 typed TF 状态。
- `/predefined/*`
  - 执行器内建时钟与计数。

对 agent 来说，最关键的是：

- `DjiMotor` 默认暴露 `angle / velocity / torque / max_torque`
- `LkMotor` 默认暴露 `angle / velocity / torque / temperature / max_torque`
- `Dr16` 统一暴露 `/remote/joystick/*`、`/remote/switch/*`、`/remote/mouse*`、`/remote/keyboard`
- `Supercap` 统一暴露 `/chassis/power`、`/chassis/voltage`、`/chassis/supercap/*`

所以多数控制器其实是在消费这些设备封装自动产出的信号。

## 4. 核心模块职责

### 4.1 `rmcs_executor`

关键文件：

- `rmcs_ws/src/rmcs_executor/src/main.cpp`
- `rmcs_ws/src/rmcs_executor/src/executor.hpp`
- `rmcs_ws/src/rmcs_executor/include/rmcs_executor/component.hpp`

职责：

- 用 pluginlib 按字符串加载组件
- 配对输入输出
- 计算组件依赖
- 单线程定频执行

注意：

- `rmcs_executor/README.md` 明确要求：被激活的 interface 必须作为成员变量保存，否则可能出现越界或析构问题。

### 4.2 `rmcs_core::hardware`

这里不是“单电机驱动”，而是“整车硬件插件”。

现有主兵种硬件插件：

- `OmniInfantry`
  - 单板步兵，底盘是全向轮。
- `SteeringInfantry`
  - 双板步兵，底盘是舵轮。
- `MecanumHero`
  - 双板英雄，底盘是麦轮。
- `SteeringHero`
  - 三板英雄，底盘是舵轮，带双 yaw、推弹和爬台。

常见封装设备：

- `device::Dr16`
- `device::DjiMotor`
- `device::LkMotor`
- `device::Bmi088`
- `device::Supercap`
- `device::Benewake` / `device::Gy614` / `device::Hipnuc`

硬件层有两个共性：

1. 状态更新
   - 读取 CAN/UART/IMU
   - 更新电机、超级电容、TF、遥控器状态
2. 命令下发
   - 消费控制器输出
   - 组帧发到不同板子的 CAN/UART

### 4.3 `rmcs_core::controller`

#### 底盘控制

- `ChassisController`
  - 负责遥控器解释、模式切换、整车速度目标。
  - 当前枚举：`AUTO / SPIN / STEP_DOWN / LAUNCH_RAMP`
- `ChassisPowerController`
  - 负责功率预算、虚拟 buffer、超级电容充电功率和底盘功率上限。
- `OmniWheelController`
  - 全向底盘的运动学、功率约束和轮扭矩分配。
- `SteeringWheelController`
  - 舵轮底盘的速度/加速度控制、舵向和轮扭矩求解。
- `ChassisClimberController`
  - 仅 Steering Hero 使用。负责爬台状态机、履带/支撑联动。

#### 云台控制

- `SimpleGimbalController`
  - 步兵常用，输出 yaw/pitch 角误差。
- `HeroGimbalController`
  - 英雄用，支持 IMU / ENCODER 两套模式切换。
- `DualYawController`
  - 仅 Steering Hero 使用。把 top yaw 与 bottom yaw 串起来控制。
- `TwoAxisGimbalSolver`
  - 通过 TF 和 pitch 限位求控制方向。
- `PreciseTwoAxisGimbalSolver`
  - 英雄编码器模式下的精细 pitch 控制。

#### 射击控制

- `FrictionWheelController`
  - 摩擦轮启停、软启动、卡轮检测、弹丸发射检测。
- `HeatController`
  - 根据裁判系统冷却与热量限制，输出允许弹量。
- `BulletFeederController17mm`
  - 17mm 供弹轮状态机，支持卡弹反转。
- `BulletFeederController42mm`
  - 42mm 供弹状态机，供英雄麦轮版本使用。
- `PutterController`
  - Steering Hero 专用推弹机构控制器。

#### 通用 PID

- `PidController`
  - `setpoint - measurement`
- `ErrorPidController`
  - 直接把输入当误差

两者都通过 YAML 里的字符串参数决定输入输出名字，所以它们是“拼接控制链”的常用积木。

### 4.4 `rmcs_core::referee`

分两部分：

- `referee::Status`
  - 从 `/referee/serial` 解析比赛数据，输出功率、热量、弹量、robot id、比赛阶段等。
- `referee::Command`
  - 把交互 UI、地图、文本封包后通过 `/referee/serial` 发回裁判系统。

UI 链路：

- `referee::app::ui::Infantry`
- `referee::app::ui::Hero`
- `referee::command::interaction::Ui`
- `referee::command::Interaction`

UI 组件本身不直接发串口，它们只产出 `Field`，再由 `Interaction` 和 `Command` 汇总下发。

### 4.5 `broadcaster`

- `TfBroadcaster`
  - 读取 `/tf`，按 50ms 节流广播。
- `ValueBroadcaster`
  - 把选定的 `double` 型内部信号转成 ROS topic。
  - `forward_list` 是按参数动态配置的。

### 4.6 `rmcs_description + fast_tf`

RMCS 没有直接用传统的“全 ROS TF 树拼装方式”做内部控制，而是：

- 在进程内维护 typed TF
- 控制器直接消费 `/tf`
- 必要时再由 `TfBroadcaster` 发出去

这也是 `TwoAxisGimbalSolver` 能直接用 link 类型算控制方向的基础。

## 5. 兵种设置的真实入口

兵种的 source of truth 在这里：

- `rmcs_ws/src/rmcs_bringup/config/omni-infantry.yaml`
- `rmcs_ws/src/rmcs_bringup/config/steering-infantry.yaml`
- `rmcs_ws/src/rmcs_bringup/config/mecanum-hero.yaml`
- `rmcs_ws/src/rmcs_bringup/config/steering-hero.yaml`

一份 YAML 里真正决定兵种的是两块：

1. `rmcs_executor.ros__parameters.components`
   - 组件图本身
2. 每个组件名对应的 `ros__parameters`
   - 零点、板卡序列号、PID、动力学参数、射频、热量参数等

## 6. 四套兵种总表

| 配置名 | 硬件插件 | 板卡拓扑 | 底盘 | 云台 | 射击 | 额外功能 |
| --- | --- | --- | --- | --- | --- | --- |
| `omni-infantry` | `rmcs_core::hardware::OmniInfantry` | 单 `CBoard` | `ChassisController + ChassisPowerController + OmniWheelController` | `SimpleGimbalController + yaw/pitch PID` | 17mm 供弹、双摩擦轮 | 步兵 UI |
| `steering-infantry` | `rmcs_core::hardware::SteeringInfantry` | top board + bottom board | `ChassisController + ChassisPowerController + SteeringWheelController` | `SimpleGimbalController + yaw/pitch PID` | 17mm 供弹、双摩擦轮 | 舵轮零点校准 |
| `mecanum-hero` | `rmcs_core::hardware::MecanumHero` | top board + bottom board | `ChassisController + ChassisPowerController + OmniWheelController` | `HeroGimbalController + yaw/pitch PID` | 42mm 供弹、四摩擦轮 | 外置 IMU、测距/温度接口 |
| `steering-hero` | `rmcs_core::hardware::SteeringHero` | top board + bottom board one + bottom board two | `ChassisController + ChassisPowerController + SteeringWheelController + ChassisClimberController` | `HeroGimbalController + DualYawController + pitch PID` | 推弹器、四摩擦轮 | 双 yaw、爬台、ValueBroadcaster、Hero UI |

## 7. 各兵种设置要点

### 7.1 `omni-infantry`

特征：

- 单板结构，最简单。
- 底盘是四个全向轮。
- 云台 yaw 和 pitch 都是单轴。
- 射击链是 `FrictionWheelController + HeatController + BulletFeederController17mm`。

关键参数：

- `infantry_hardware`
  - `board_serial`
  - `yaw_motor_zero_point`
  - `pitch_motor_zero_point`
- `gimbal_controller`
  - `upper_limit / lower_limit`
- `friction_wheel_controller`
  - 两个摩擦轮目标转速 `660`
- `bullet_feeder_controller`
  - `bullets_per_feeder_turn=8`
  - `shot_frequency=20`
  - `safe_shot_frequency=10`
- `heat_controller`
  - `heat_per_shot=10000`
  - `reserved_heat=10000`

备注：

- 这套配置带 `referee::app::ui::Infantry`。
- yaw 走两级链路：`SimpleGimbalController -> ErrorPid(yaw angle) -> Pid(yaw velocity)`。

### 7.2 `steering-infantry`

特征：

- 上板管云台 pitch / 摩擦轮，下板管底盘、yaw、遥控器、裁判系统。
- 底盘是四舵轮。
- 当前配置里步兵 UI 被注释掉了。

关键参数：

- `steeringInfantry_hardware`
  - `board_serial_top_board`
  - `board_serial_bottom_board`
  - `yaw_motor_zero_point`
  - `pitch_motor_zero_point`
  - 四个舵向零点 `left_front_zero_point / left_back_zero_point / right_back_zero_point / right_front_zero_point`
- `bullet_feeder_controller`
  - `bullets_per_feeder_turn=10`
  - `shot_frequency=24`
- `steering_wheel_controller`
  - `mess`
  - `moment_of_inertia`
  - `vehicle_radius`
  - `wheel_radius`
  - `friction_coefficient`
  - `k1 / k2 / no_load_power`

备注：

- yaw 没有单独的 yaw velocity PID 组件，底层硬件下发里直接把 `control_velocity - imu.gy()` 送给 yaw 电机。
- 这是“舵轮步兵”的基准改装模板。

### 7.3 `mecanum-hero`

特征：

- top board 负责 pitch、四摩擦轮、scope/player viewer、外置传感器。
- bottom board 负责麦轮底盘、yaw、供弹、超级电容、遥控器、裁判系统。
- 射击是 42mm 供弹器，不是 17mm 供弹轮。

关键参数：

- `hero_hardware`
  - `board_serial_top_board`
  - `board_serial_bottom_board`
  - `yaw_motor_zero_point`
  - `pitch_motor_zero_point`
  - `external_imu_port`
- `friction_wheel_controller`
  - 四个摩擦轮目标转速：`450 / 450 / 535 / 535`
- `heat_controller`
  - `heat_per_shot=100000`
  - `reserved_heat=0`

组件特点：

- 云台控制用 `HeroGimbalController`
  - 支持 IMU / ENCODER 模式切换
- 供弹用 `BulletFeederController42mm`
- 底盘仍然是 `OmniWheelController`

备注：

- `referee_ui_hero` 在 YAML 里被注释掉了。
- top board 里有外置 IMU 线程，FPS 足够时会融合 Hipnuc IMU。

### 7.4 `steering-hero`

特征：

- 目前最复杂的一套。
- 三板结构：
  - top board：top yaw、pitch、四摩擦轮、bullet feeder、putter、光电/灰度传感器
  - bottom board one：前轮、前舵向、前后爬台、底盘 IMU
  - bottom board two：后轮、后舵向、bottom yaw、遥控器、超级电容、裁判串口
- 云台是 `HeroGimbalController + DualYawController`
- 射击是 `PutterController`
- 底盘带 `ChassisClimberController`

关键参数：

- `hero_hardware`
  - 三个板卡序列号
  - `bottom_yaw_motor_zero_point`
  - `top_yaw_motor_zero_point`
  - `pitch_motor_zero_point`
  - `bullet_feeder_motor_zero_point`
  - 四个舵向零点
- `dual_yaw_controller`
  - top/bottom yaw 的角度环和速度环 PID
- `climber_controller`
  - 前履带速度、后支撑速度、自动爬台阈值与同步系数
- `friction_wheel_controller`
  - 四个摩擦轮目标转速：`380 / 380 / 542 / 542`

组件特点：

- `DualYawController` 负责把 top yaw 和 bottom yaw 合成为总 yaw
- `PutterController` 负责推弹状态机、卡弹处理和射击条件输出
- `ValueBroadcaster` 开启，可把指定内部 `double` 量发布为 ROS topic

备注：

- `TfBroadcaster`、`PlayerViewer`、部分 auto aim 组件在 YAML 里是注释状态。
- 这是当前最容易“一改牵全身”的兵种，改动前先确认 top/bottom yaw、putter、climber 三条链是否同时闭环。

## 8. 对 agent 最有用的判断规则

### 8.1 改兵种优先改 YAML，不要先改 C++

多数“换车”“换兵种”“换参数”需求，应该先问自己：

- 只是换组件组合吗
- 只是换 PID / 零点 / 转速 / 动力学参数吗
- 只是启用/停用 UI、broadcaster、auto aim 吗

如果答案是是，优先改 `rmcs_bringup/config/*.yaml`。

### 8.2 改接口名时要把它当协议升级

因为 `rmcs_executor` 是按字符串配对：

- 改一个输出名，所有下游 `register_input` 都要跟着改
- 改一个类型，启动时会直接 type mismatch
- 产生重复输出名，启动直接报错

### 8.3 先分清“硬件能力”与“控制器拼法”

常见错误是把两者混在一起：

- 硬件插件决定“车上实际有什么”
  - 几块板
  - 哪些电机
  - 哪些传感器
  - 输出了哪些 `/gimbal/*` `/chassis/*` `/remote/*`
- YAML 组件图决定“这些能力被怎么用”
  - 哪个控制器消费哪个信号
  - 是否启用 UI / broadcaster / auto aim

### 8.4 舵轮平台一定要同时检查两类参数

改 `steering-*` 兵种时，至少同时核对：

- 舵向零点
- `mess / moment_of_inertia / vehicle_radius / wheel_radius`
- `k1 / k2 / no_load_power`

这些参数来自机械与电气现实，不能只调 PID。

### 8.5 英雄车要特别留意双 yaw 和射击链

改 `steering-hero` 时最容易漏的是：

- `HeroGimbalController`
- `DualYawController`
- `pitch_angle_pid_controller`
- `pitch_velocity_pid_controller`
- `PutterController`

这五者不是并列关系，而是串成一条完整控制链。

## 9. 推荐阅读顺序

第一次接手 RMCS，建议按下面顺序读：

1. `README.md`
2. `rmcs_ws/src/rmcs_bringup/launch/rmcs.launch.py`
3. 目标兵种的 YAML
4. `rmcs_executor/include/rmcs_executor/component.hpp`
5. `rmcs_executor/src/executor.hpp`
6. 对应兵种的 `hardware/*.cpp`
7. 该 YAML 里实际启用的控制器源文件
8. 如涉及裁判 UI，再读 `referee/*`

不要一开始就通读整个 `rmcs_core/src/controller`，先跟着某套兵种 YAML 走一遍，理解会快很多。
