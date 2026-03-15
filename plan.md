# Planner 接入 RMCS 设计草案

## 范围
- 仅考虑 `omni-infantry`
- 目标是一次性完成以下三件事：
  - 在 `sp_vision_bridge` 中接入 `Planner`，增加 `1kHz` planner 线程
  - 用共享状态交接 `Target/Plan`
  - 新增 `omni` 专用云台控制器，替代现有 `SimpleGimbalController + 云台 PID 控制器`

## 核心约束

### 1. `Planner` 不能直接驱动 RMCS joint
- `sp_vision` 中的 `plan.yaw/pitch` 是世界系方向参数，不等价于 RMCS 电机 joint 角。
- 真实控制必须继续经过 RMCS 的几何层，把目标方向映射到 `yaw/pitch` 电机误差。
- 因此不能把 `plan.yaw/pitch` 直接接到 `/gimbal/yaw/control_angle`、`/gimbal/pitch/control_angle`。

### 2. 必须保留方向语义输出
- `sp_vision_bridge` 需要继续输出 `/gimbal/auto_aim/control_direction`。
- planner 的导数信息应作为前馈附加输出，而不是替代方向输出。
- 当前实现采用 planner 原始状态/导数输出：
  - `/gimbal/auto_aim/plan_yaw`
  - `/gimbal/auto_aim/plan_pitch`
  - `/gimbal/auto_aim/plan_yaw_velocity`
  - `/gimbal/auto_aim/plan_yaw_acceleration`
  - `/gimbal/auto_aim/plan_pitch_velocity`
  - `/gimbal/auto_aim/plan_pitch_acceleration`
  - `/gimbal/auto_aim/fire_control`
  - `/gimbal/auto_aim/laser_distance`

### 3. `omni-infantry` 需要专用 controller
- 不直接改通用 `SimpleGimbalController`。
- 新增 `OmniInfantryPlannerGimbalController`，仅在 `omni-infantry.yaml` 中使用。
- 原因：
  - `SimpleGimbalController` 还被 `steering-infantry` 复用。
  - 本次 controller 需要内置 PID、前馈叠加、planner 超时处理，已偏离原组件职责。

### 4. `yaw/control_angle_error` 与 `pitch/control_angle_error` 不能删
- `ChassisController` 依赖 `/gimbal/yaw/control_angle_error` 做底盘跟随。
- 即使云台 PID 内置进新 controller，也必须继续发布：
  - `/gimbal/yaw/control_angle_error`
  - `/gimbal/pitch/control_angle_error`

### 5. `pitch` 硬件链必须一并改
- 当前 `OmniInfantry::command_update()` 中：
  - yaw 走 torque
  - pitch 走 velocity
- 如果目标是“两轴都由新 controller 直接闭环并叠加 planner 前馈”，则 `pitch` 不能继续硬编码只发 velocity。
- 当前实现把 `pitch` 改为直接发送 `generate_torque_command()`，与新 controller 的力矩输出对齐。

### 6. planner 线程与检测线程要解耦
- 检测线程只负责：
  - `Camera -> YOLO -> Solver -> Tracker -> latest_target`
- planner 线程只负责：
  - `latest_target + bullet_speed -> Planner.plan() -> latest_plan`
- `Planner` 计算耗时约 `200us`，`1kHz` 可行，但共享锁必须只用于短时拷贝，不能包住计算。

### 7. `bullet_speed` 与 IMU 对齐仍是精度风险点
- `Planner` 继续使用 bridge 中的 `bullet_speed_snapshot_`。
- bridge 当前使用“最新 IMU 姿态”而非严格按图像时间戳插值，这一点仍弱于 `standard_mpc`。
- 本次可以先保持现状，但文档中要明确这是后续精度优化点。

## 文件改动目录

### `rmcs_ws/src/sp_vision_25/`
- `src/sp_vision_bridge.cpp`
- 如需复用结构，可考虑新增：
  - `src/` 下 bridge 相关辅助头/实现
  - 或仅在现文件内增加共享状态与线程逻辑

### `rmcs_ws/src/rmcs_core/`
- 新增：
  - `src/controller/gimbal/omni_infantry_planner_gimbal_controller.cpp`
- 可能复用：
  - `src/controller/gimbal/two_axis_gimbal_solver.hpp`
  - `src/controller/pid/pid_calculator.hpp`
- 修改：
  - `plugins.xml`
  - `CMakeLists.txt`
  - `src/hardware/omni_infantry.cpp`

### `rmcs_ws/src/rmcs_bringup/`
- 修改：
  - `config/omni-infantry.yaml`

## 大致变更思路

### 一、改造 `sp_vision_bridge`
1. 保留现有相机检测 worker。
2. 在 bridge 内新增共享状态，例如：
   - `latest_target`
   - `latest_target_timestamp`
   - `latest_plan`
   - `latest_plan_timestamp`
   - `latest_laser_distance`
3. 新增 `planner_thread_`，循环频率 `1kHz`：
   - 读取一份 `latest_target` 拷贝
   - 读取 `bullet_speed_snapshot_`
   - 调用 `Planner::plan()`
   - 回写 `latest_plan`
4. `update()` 只做：
   - 更新 `bullet_speed_snapshot_`
   - 更新最新 IMU pose
   - 从共享状态读出最近 plan
   - 发布方向、前馈和开火结果
5. `control_direction` 由 `plan.yaw/pitch` 转换得到。

### 二、bridge 输出策略
1. 保留现有：
   - `/gimbal/auto_aim/control_direction`
   - `/gimbal/auto_aim/fire_control`
   - `/gimbal/auto_aim/laser_distance`
2. 新增 planner 原始输出：
   - `/gimbal/auto_aim/plan_yaw`
   - `/gimbal/auto_aim/plan_pitch`
   - `/gimbal/auto_aim/plan_yaw_velocity`
   - `/gimbal/auto_aim/plan_yaw_acceleration`
   - `/gimbal/auto_aim/plan_pitch_velocity`
   - `/gimbal/auto_aim/plan_pitch_acceleration`
3. 对 plan 增加超时保护：
   - 超时后方向清零
   - planner 原始导数量清零
   - `fire_control = false`

### 三、新增 `OmniInfantryPlannerGimbalController`
1. 输入：
   - `/tf`
   - `/gimbal/yaw/velocity_imu`
   - `/gimbal/pitch/velocity_imu`
   - `/remote/*`
   - `/gimbal/auto_aim/control_direction`
   - `/gimbal/auto_aim/plan_yaw`
   - `/gimbal/auto_aim/plan_pitch`
   - `/gimbal/auto_aim/plan_yaw_velocity`
   - `/gimbal/auto_aim/plan_yaw_acceleration`
   - `/gimbal/auto_aim/plan_pitch_velocity`
   - `/gimbal/auto_aim/plan_pitch_acceleration`
2. 输出：
   - `/gimbal/yaw/control_torque`
   - `/gimbal/pitch/control_torque`
   - `/gimbal/yaw/control_angle_error`
   - `/gimbal/pitch/control_angle_error`
3. 内部逻辑：
   - 继续使用 RMCS 几何层计算 angle error
   - 内置 angle PID 与 velocity PID
   - 由 planner 世界系 `yaw/pitch/vel/acc` 重建方向导数
   - 再通过 TF 里的真实 yaw/pitch joint 轴，把世界系导数映射成 joint-space 前馈
   - `velocity_ref = angle_pid(err) + velocity_ff`
   - `torque = velocity_pid(velocity_ref - imu_velocity) + acc_ff_gain * acc_ff`
4. 保留遥控兜底逻辑：
   - 未启用自瞄时仍允许手动控制
   - 未启用控制时输出 `NaN`/安全值，维持当前 RMCS 习惯

### 四、改造 `omni_infantry.cpp`
1. 保持 yaw 电机走 torque。
2. 将 pitch 电机从当前硬编码 `generate_velocity_command(...)` 改为 `generate_torque_command()`。
3. 使新 controller 输出的 `/gimbal/pitch/control_torque` 可以真正到达硬件。
4. 不改 chassis、referee、shooting 相关链路。

### 五、改造 `omni-infantry.yaml`
1. 替换 controller：
   - 删除 `rmcs_core::controller::gimbal::SimpleGimbalController`
   - 增加 `rmcs_core::controller::gimbal::OmniInfantryPlannerGimbalController`
2. 删除旧云台 PID 组件：
   - `yaw_angle_pid_controller`
   - `yaw_velocity_pid_controller`
   - `pitch_angle_pid_controller`
3. 新增新 controller 参数：
   - `upper_limit`
   - `lower_limit`
   - `yaw_angle_kp/ki/kd`
   - `yaw_velocity_kp/ki/kd`
   - `pitch_angle_kp/ki/kd`
   - `pitch_velocity_kp/ki/kd`
   - `yaw_acc_ff_gain`
   - `pitch_acc_ff_gain`
4. 保留射击、底盘、摩擦轮、热量限制等现有组件。

## 推荐实施顺序
1. 先改 `sp_vision_bridge`，确保能稳定产出 `plan + ff`。
2. 再加 `OmniInfantryPlannerGimbalController`，先只接方向和速度前馈。
3. 然后改 `omni_infantry.cpp`，打通 `pitch torque`。
4. 最后清理 `omni-infantry.yaml` 中旧 PID 组件并联调。

## 明确不在本次范围内
- 其它兵种适配
- buff 模式接入
- 对 `Planner` 做“真闭环 MPC”改造
- bridge 中 IMU 时间插值重构

## 一句话结论
本方案对 `omni-infantry` 可落地，但必须同时满足三点：
- `Planner` 只作为高频参考与前馈源，不直接绕过 RMCS 几何层
- 新增 `omni` 专用 controller，而不是硬改通用 `SimpleGimbalController`
- `omni_infantry.cpp` 里的 `pitch` 硬件发送链也要一起改
