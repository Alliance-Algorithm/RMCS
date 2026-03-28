# 基于 `sentry.yaml` 的 RMCS 哨兵自瞄调用链分析

## 1. 范围
本文分析的是 RMCS 在 `rmcs_bringup/config/sentry.yaml` 下实际启动的这条运行链，以及它直接依赖的关键实现：

- `rmcs_ws/src/rmcs_bringup/config/sentry.yaml`
- `rmcs_ws/src/rmcs_core/src/hardware/sentry.cpp`
- `rmcs_ws/src/sp_vision_25/src/sp_vision_bridge.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/sentry_gimbal_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/planner.hpp`
- `rmcs_ws/src/rmcs_core/src/controller/shooting/*.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/*.cpp`
- `rmcs_ws/src/sp_vision_25/configs/sentry.yaml`
- `rmcs_ws/src/sp_vision_25/tasks/auto_aim/{yolo,solver,tracker,target}.*`

目标不是解释所有算法细节，而是把这条链在“数据从哪里来、经过哪些组件、输出到哪里”上讲清楚，便于后续排查和二次开发。

## 2. 先说结论
`sentry.yaml` 对应的 RMCS 自瞄主链不是 `sp_vision_25/src/sentry.cpp` 那条独立程序链。独立程序链已完全废弃，无参考价值。

RMCS 实际跑的是一条“执行器 DAG + 视觉桥接线程”的链：

```text
predefined_msg_provider
  -> /predefined/timestamp

sentry_hardware
  -> /tf
  -> /gimbal/hard_sync_snapshot
  -> /remote/*
  -> /gimbal/{top_yaw,bottom_yaw,pitch}/*
  -> /gimbal/{left_friction,right_friction,bullet_feeder}/*
  -> /chassis/*/*
  -> /chassis/yaw/velocity_imu
  -> /referee/serial

/gimbal/hard_sync_snapshot
  -> sp_vision_bridge
     worker: Camera.read
          -> YOLO.detect
          -> Solver.set_R_gimbal2world
          -> Tracker.track
          -> /gimbal/auto_aim/target_snapshot

/gimbal/auto_aim/target_snapshot + /tf + 云台状态 + 底盘 IMU
  -> gimbal_controller
     -> /gimbal/{top_yaw,bottom_yaw,pitch}/control_torque
     -> /gimbal/auto_aim/fire_control
     -> /gimbal/yaw/{angle,control_angle_error,velocity}

/gimbal/auto_aim/fire_control
  + /gimbal/friction_ready
  + /gimbal/control_bullet_allowance/limited_by_heat
  -> bullet_feeder_controller
  -> /gimbal/bullet_feeder/control_velocity
  -> bullet_feeder_velocity_pid_controller
  -> /gimbal/bullet_feeder/control_torque

/gimbal/yaw/{angle,control_angle_error}
  -> chassis_controller
  -> chassis_power_controller
  -> steering_wheel_controller
  -> /chassis/*/{control_torque}

以上所有 /gimbal/*/control_torque 与 /chassis/*/control_torque
  -> sentry_hardware_command
  -> CAN/UART 下发到真实电机
```

## 3. 组件装配
`rmcs_bringup/config/sentry.yaml` 里实际启用的关键组件如下：

- `rmcs_core::hardware::Sentry -> sentry_hardware`
- `rmcs_core::referee::Status -> referee_status`
- `sp_vision_25::bridge::SpVisionBridge -> sp_vision_bridge`
- `rmcs_core::controller::gimbal::SentryGimbalController -> gimbal_controller`
- `rmcs_core::controller::shooting::FrictionWheelController -> friction_wheel_controller`
- `rmcs_core::controller::shooting::HeatController -> heat_controller`
- `rmcs_core::controller::shooting::BulletFeederController17mm -> bullet_feeder_controller`
- `rmcs_core::controller::pid::PidController -> left_friction_velocity_pid_controller`
- `rmcs_core::controller::pid::PidController -> right_friction_velocity_pid_controller`
- `rmcs_core::controller::pid::PidController -> bullet_feeder_velocity_pid_controller`
- `rmcs_core::controller::chassis::ChassisController -> chassis_controller`
- `rmcs_core::controller::chassis::ChassisPowerController -> chassis_power_controller`
- `rmcs_core::controller::chassis::SteeringWheelController -> steering_wheel_controller`

默认未启用的相关组件：

- `VirtualTargetSnapshotProvider`
- `AutoAimResponseRecorder`

这意味着当前哨兵链依赖的是真实视觉输入，不是虚拟目标，也不会默认记录自瞄响应 CSV。

## 4. 上游：时间源、硬件状态与硬同步
### 4.1 `predefined_msg_provider`
执行器内置组件提供 `/predefined/timestamp`。`sentry_hardware` 和 `gimbal_controller` 都直接消费它。

### 4.2 `sentry_hardware`
`Sentry` 是整条链的硬件入口。它做了三类事：

1. 输出当前机器人状态
- `/tf`
- `/gimbal/hard_sync_snapshot`
- `/gimbal/yaw/velocity_imu`
- `/gimbal/pitch/velocity_imu`
- `/chassis/yaw/velocity_imu`
- 各个云台、摩擦轮、拨弹、底盘舵轮/驱动轮的 angle/velocity/torque/max_torque
- `/remote/*`
- `/referee/serial`

2. 建立真实 TF
- 固定 `BottomYawLink -> TopYawLink`
- 固定 `PitchLink -> CameraLink`
- 每周期根据下位机反馈刷新 `BottomYawLink -> OdomImu`
- 每周期根据电机角度刷新 `BottomYawLink -> TopYawLink` 与 `TopYawLink -> PitchLink`

3. 创建隐藏伙伴组件 `sentry_hardware_command`
- 这个伙伴组件不写在 YAML 里
- 它负责消费所有 `/gimbal/*/control_torque` 和 `/chassis/*/control_torque`
- 最终把命令打包后发往 CAN/UART

### 4.3 硬同步快照是怎么来的
`TopBoard` 在 `before_updating()` 里开启 GPIO 下降沿监听。相机硬同步脉冲到来后会置位 `hard_sync_pending_`，随后 `Sentry::update_hard_sync_snapshot()` 发布一份 `HardSyncSnapshot`：

- `valid`
- `exposure_timestamp`
- 相机在 `OdomImu` 下的姿态四元数 `qw/qx/qy/qz`

它本质上是“这次曝光对应的世界姿态快照”，给视觉线程做时序对齐。

## 5. 视觉桥：`SpVisionBridge`
### 5.1 输入输出
`SpVisionBridge` 在 DAG 里的接口很简单：

- 输入：`/gimbal/hard_sync_snapshot`
- 输出：`/gimbal/auto_aim/target_snapshot`

### 5.2 真正的工作发生在后台线程
`before_updating()` 会启动一个 worker 线程。这个线程内部构造：

- `io::Camera`
- `auto_aim::YOLO`
- `auto_aim::Solver`
- `auto_aim::Tracker`

然后循环执行：

```text
Camera.read(frame, frame_timestamp)
-> 读取最新 hard_sync_snapshot
-> 用 snapshot.exposure_timestamp 覆盖帧时间
-> solver.set_R_gimbal2world(snapshot quaternion)
-> detector.detect(frame)
-> tracker.track(armors, frame_timestamp)
-> 取 targets.front()
-> 转成 rmcs_msgs::TargetSnapshot
-> 存到 latest_target_snapshot
```

执行器主线程中的 `update()` 并不重新做检测，只是：

- 保存最新硬同步快照
- 把 worker 算出的 `latest_target_snapshot` 发布到 DAG

因此：

- DAG 是 1000 Hz
- 视觉频率不是 1000 Hz，而是由相机和 worker 线程吞吐决定
- DAG 每次拿到的都是“最近一次视觉结果”

### 5.3 `sp_vision_25/configs/sentry.yaml` 在这里起什么作用
这个 YAML 是视觉桥内部的配置文件，不是 RMCS 外层组件 DAG。

它提供了：

- 敌方颜色 `enemy_color`
- YOLO 模型选择与模型路径
- 传统阈值参数
- tracker 参数
- aimer/planner 参数
- 相机内参与畸变参数
- `R_camera2gimbal` / `t_camera2gimbal`

其中 `SpVisionBridge` 会先把相对路径模型文件解析成运行时绝对路径，再交给视觉模块加载。

### 5.4 `TargetSnapshot` 里到底装的是什么
`SpVisionBridge` 直接把 `auto_aim::Target::ekf_x()` 拷进 `rmcs_msgs::TargetSnapshot::state`。

当前状态量语义是：

```text
state[0]  = x
state[1]  = vx
state[2]  = y
state[3]  = vy
state[4]  = z
state[5]  = vz
state[6]  = a   (目标中心相位角)
state[7]  = w   (目标角速度)
state[8]  = r   (主半径)
state[9]  = l   (长短半径差 / radius delta)
state[10] = h   (高低装甲板高度差)
```

同时还有：

- `valid`
- `converged`
- `armor_count`
- `armor_type`
- `armor_name`
- `timestamp`

### 5.5 视觉侧的目标形成过程
这条桥接链里，目标形成过程是：

```text
YOLO.detect
-> Tracker.track
   -> lost/detecting/tracking/temp_lost 状态机
   -> Solver.solve(armor) 做 PnP 与姿态解算
   -> Target EKF predict/update
   -> 生成 armor_xyza_list 与 ekf_x
-> TargetSnapshot
```

需要注意：

- Tracker 会先按敌我颜色过滤，再按优先级排序
- 这里没有接入 `Decider.decide(...)`
- 这里也没有接入多相机全向感知切换逻辑

## 6. 自瞄核心：`SentryGimbalController`
### 6.1 输入
`SentryGimbalController` 消费的关键输入包括：

- `/remote/joystick/left`
- `/remote/switch/{left,right}`
- `/remote/mouse`
- `/remote/mouse/velocity`
- `/predefined/timestamp`
- `/tf`
- `/gimbal/top_yaw/{angle,velocity}`
- `/gimbal/bottom_yaw/{angle,velocity}`
- `/gimbal/pitch/{angle,velocity}`
- `/chassis/yaw/velocity_imu`
- `/referee/shooter/initial_speed`（可选）
- `/gimbal/auto_aim/target_snapshot`（可选）

### 6.2 输出
它会输出：

- `/gimbal/top_yaw/control_torque`
- `/gimbal/bottom_yaw/control_torque`
- `/gimbal/pitch/control_torque`
- `/gimbal/auto_aim/fire_control`
- `/gimbal/yaw/control_angle_error`
- `/gimbal/yaw/angle`
- `/gimbal/yaw/velocity`

### 6.3 模式切换
控制器内部有三种模式：

- `kDisabled`
- `kManual`
- `kAutoAim`

规则大致是：

- 遥控器异常或双开关都在 `DOWN` 时进入 `Disabled`
- 鼠标右键按下，或右开关在 `UP` 时请求 `AutoAim`
- 否则进入 `Manual`

但即便请求了 `AutoAim`，如果当前 `target_snapshot` 无效或超时，也会退回 `Manual`。

### 6.4 自动瞄准阶段做了什么
`make_auto_aim_plan()` 的主要流程是：

```text
检查 target_snapshot.valid 和新鲜度
-> 复制 snapshot，构造只保留旋转的 rotation_snapshot
-> Planner.plan(rotation_snapshot, now, bullet_speed)
-> 根据目标角速度决定 high_speed_delay / low_speed_delay
-> 做一次弹道解算，得到飞行时间
-> 用 snapshot_age + planning_delay + fly_time 外推目标中心
-> 计算 bottom_yaw / top_yaw / pitch 的目标值
-> 计算 yaw/pitch 的速度与加速度前馈
-> 得到 AutoAimPlan{fire, bottom_yaw, top_yaw, pitch}
```

其中 `Planner` 的逻辑是：

1. 用 `TargetSnapshot` 重建 `TargetModel`
2. 根据 `w` 选择高/低速延迟
3. 先预测到延迟时刻
4. 选水平距离最近的装甲板作为当前瞄准装甲板
5. 根据弹速和弹道方程再预测到飞行时间之后
6. 用 TinyMPC 解 yaw/pitch 轨迹跟踪
7. 在 `kHalfHorizon + kShootOffset` 位置比较参考与预测误差
8. 若 `hypot(yaw_err, pitch_err) < fire_thresh`，则 `fire = true`

### 6.5 控制律结构
最终控制是“角度环 + 速度环 + 前馈”的结构：

- 角度误差先过 angle PID，得到速度参考
- 叠加速度前馈
- 再由 velocity PID 生成主反馈扭矩
- 最后叠加加速度前馈、黏性摩擦前馈、库仑摩擦前馈、pitch 重力前馈

特别注意：

- `bottom_yaw` 是按世界系 yaw 闭环，不直接用顶部 IMU yaw 速度
- `bottom_yaw` 速度会和 `/chassis/yaw/velocity_imu` 组合，重建底座世界角速度
- 输出给底盘跟随的是 `/gimbal/yaw/control_angle_error`

### 6.6 超时保护
如果满足任一条件，就不会继续走自动瞄准计划：

- `target_snapshot.valid == false`
- `now - snapshot.timestamp > result_timeout`
- `Planner.plan()` 返回 `control=false`
- 弹道无解
- 任意中间瞄准点无效

此时控制器会退回手动模式，并清掉 `fire_control`。

## 7. 射击链
`fire_control` 不会直接触发电机。RMCS 里的射击链是分层门控的。

### 7.1 摩擦轮
`FrictionWheelController` 消费遥控器输入和摩擦轮实际速度，输出：

- `/gimbal/left_friction/control_velocity`
- `/gimbal/right_friction/control_velocity`
- `/gimbal/friction_ready`
- `/gimbal/friction_jammed`
- `/gimbal/bullet_fired`

其中：

- `friction_ready` 表示摩擦轮已到工作速度
- `bullet_fired` 通过主摩擦轮的瞬时掉速估计

随后左右摩擦轮的速度 PID 会把 `control_velocity` 变成 `control_torque`。

### 7.2 热量限制
`HeatController` 输入：

- `/referee/shooter/cooling`
- `/referee/shooter/heat_limit`
- `/gimbal/bullet_fired`

输出：

- `/gimbal/control_bullet_allowance/limited_by_heat`

它本质上在本地维护一个估计热量，并把剩余热量折算成“还能打几发”。

### 7.3 拨弹
`BulletFeederController17mm` 同时消费：

- `/gimbal/friction_ready`
- `/gimbal/bullet_fired`
- `/gimbal/control_bullet_allowance/limited_by_heat`
- `/remote/*`
- `/gimbal/auto_aim/fire_control`
- `/gimbal/bullet_feeder/velocity`

它只有在下面这些条件同时成立时，才会让拨弹转起来：

- 摩擦轮 ready
- 热量额度大于 0
- 手动左键/拨杆触发，或“允许自动射击且 `fire_control=true`”

然后输出：

- `/gimbal/bullet_feeder/control_velocity`
- `/gimbal/shooter/mode`

再由 `bullet_feeder_velocity_pid_controller` 转成：

- `/gimbal/bullet_feeder/control_torque`

## 8. 底盘跟随链
自瞄不只控制云台，还会影响底盘。

### 8.1 `ChassisController`
它读取：

- `/gimbal/yaw/angle`
- `/gimbal/yaw/control_angle_error`
- `/remote/*`

输出：

- `/chassis/angle`
- `/chassis/control_angle`
- `/chassis/control_mode`
- `/chassis/control_velocity`

其中：

- 平移速度由遥控器和导航速度叠加
- 旋转速度在 `AUTO/SPIN/STEP_DOWN/LAUNCH_RAMP` 间切换
- `STEP_DOWN` / `LAUNCH_RAMP` 会利用 `gimbal_yaw_angle_error` 做底盘对正

### 8.2 `ChassisPowerController`
它读取裁判功率、超级电容状态和底盘当前模式，输出：

- `/chassis/supercap/charge_power_limit`
- `/chassis/control_power_limit`

### 8.3 `SteeringWheelController`
它再把：

- `/chassis/control_velocity`
- `/chassis/control_power_limit`
- 舵轮与驱动轮反馈

变成：

- 四个舵向电机 `control_torque`
- 四个驱动轮电机 `control_torque`

这些控制量最终和云台控制量一起汇总到 `sentry_hardware_command` 下发。

## 9. 裁判系统在自瞄链里的作用
`referee_status` 从 `/referee/serial` 解帧后，会向自瞄链提供至少这些关键信息：

- `/referee/shooter/cooling`
- `/referee/shooter/heat_limit`
- `/referee/shooter/bullet_allowance`
- `/referee/shooter/initial_speed`

当前用途分别是：

- 热量限制
- 热量限制
- UI 等周边使用
- 供 `gimbal_controller` 注册为可选输入

## 10. 当前实现里值得特别记住的点
### 10.1 `SpVisionBridge` 是异步桥，不是同步组件
执行器每周期只发布“最近一次视觉结果”，不会等待视觉线程算完。

### 10.2 `target_snapshot` 过期会直接退回手动
这决定了：

- 视觉卡顿会立即体现在自瞄掉线
- `result_timeout` 是非常关键的系统参数

### 10.3 `bullet_speed` 目前没有完整接进计划链
`gimbal_controller` 虽然注册了 `/referee/shooter/initial_speed`，但在
`make_auto_aim_plan()` 里，`Planner.plan()` 前那一段目前写死使用 `23.0` 作为弹速。

这意味着：

- 当前计划链并没有完全使用裁判实时弹速
- 若弹速漂移明显，理论上会影响预测与开火时机

### 10.4 `fire_control` 只是“允许开火”
真正是否拨弹，还要经过：

- 摩擦轮 ready
- 热量额度
- 遥控/自动射击触发条件

因此排查“明明锁住了但不发弹”时，不能只看 `fire_control`。

## 11. 适合调试时的追踪顺序
### 11.1 自瞄完全不工作
按这个顺序看：

```text
/gimbal/hard_sync_snapshot
-> /gimbal/auto_aim/target_snapshot
-> /gimbal/auto_aim/fire_control
-> /gimbal/{top_yaw,bottom_yaw,pitch}/control_torque
```

### 11.2 能锁目标但不射击
继续看：

```text
/gimbal/auto_aim/fire_control
-> /gimbal/friction_ready
-> /gimbal/bullet_fired
-> /gimbal/control_bullet_allowance/limited_by_heat
-> /gimbal/bullet_feeder/control_velocity
-> /gimbal/bullet_feeder/control_torque
```

### 11.3 云台正常但底盘不跟
看：

```text
/gimbal/yaw/angle
-> /gimbal/yaw/control_angle_error
-> /chassis/control_mode
-> /chassis/control_velocity
-> /chassis/control_power_limit
-> /chassis/*/{control_torque}
```

## 12. 一句话总结
当前 `sentry.yaml` 对应的 RMCS 哨兵自瞄链，本质上是：

“`Sentry` 硬件组件提供时序、姿态、遥控和电机接口；`SpVisionBridge` 在后台线程里做单相机检测跟踪并发布 `TargetSnapshot`；`SentryGimbalController` 基于目标状态、弹道和 MPC 生成三轴控制与开火许可；射击链和底盘链再分别完成门控、限热、限功率和实际执行。”
