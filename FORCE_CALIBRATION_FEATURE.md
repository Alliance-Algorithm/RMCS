# 力矩闭环功能说明

## 功能概述

为飞镖发射控制逻辑增加了力矩闭环功能，通过PID控制force_screw_motor（丝杆电机），使每次发射前的力值闭环到上次记录的力值，提高发射一致性。

## 实现原理

1. **力值记录**：在每次 `launch_prepare` 命令执行前，记录当前力传感器通道1的读数到 `last_fire_force_`
2. **力矩闭环**：在 `LaunchPreparationTask` 末尾（传送带上行复位后），如果启用且非第一发，执行力矩闭环
3. **PID控制**：
   - 输入：目标力值（上次记录的力）、当前力传感器读数（int类型，自动转换为double）
   - 输出：force_screw_motor 的速度命令（通过 `/force/control/velocity` 接口）
   - 控制逻辑：error = 目标力 - 当前力，PID输出映射到电机转速
   - 电机正转力增大，反转力减小
4. **完成条件**：力误差在容差范围内持续 N 帧，或超时

## 代码修改

### 新增文件

- `rmcs_ws/src/rmcs_dart_guidance/src/manager/action/force_screw_calibration_action.hpp`
  - 实现力矩闭环控制动作

### 修改文件

1. **dart_manager.cpp**
   - 添加力传感器输入：`/force_sensor/channel_1/weight` 和 `/force_sensor/channel_2/weight`（int类型）
   - 使用现有的丝杆电机速度输出：`/force/control/velocity`（已存在）
   - 添加成员变量：`last_fire_force_`（记录上次力值）
   - 在 `cancel_all()` 和 `on_task_failure()` 中，`force_control_velocity_` 已经被设置为0，确保丝杆电机停止
   - 在 `launch_prepare` 命令处理中记录力值（使用通道1）
   - 添加力矩闭环参数加载

2. **launch_preparation_task.hpp**
   - 添加 `force_screw_calibration_action.hpp` 头文件
   - 在构造函数中添加力矩闭环相关参数
   - 在步骤2（PID减速）后插入力矩闭环动作（步骤2.5）

3. **dart-guidance.yaml**
   - 添加力矩闭环配置参数

## 配置参数

在 `dart-guidance.yaml` 的 `dart_manager` 节点下添加：

```yaml
# 力矩闭环参数
enable_force_calibration: true       # 启用力矩闭环（第一发目标9025±30，后续用记录值）
force_tolerance: 5.0                 # N - 力值容差（第二发及以后）
force_settle_ticks: 50               # ticks - 力值稳定确认帧数
force_timeout_ticks: 2000            # ticks - 力矩闭环超时帧数
force_kp: 0.1                        # PID参数Kp（需调试）
force_ki: 0.0                        # PID参数Ki（需调试）
force_kd: 0.01                       # PID参数Kd（需调试）
force_max_velocity: 5.0              # rad/s - 丝杆电机最大速度限制
```

**注意**：
- 第一发使用固定目标力 9025，容差 ±30（硬编码）
- 第二发及以后使用上次记录的力值，容差使用配置的 `force_tolerance`

## 使用说明

1. **初次使用**：
   - 保持 `enable_force_calibration: false`
   - 观察力传感器读数是否正常
   - 确认丝杆电机控制正常

2. **调试PID参数**：
   - 设置 `enable_force_calibration: true`
   - 从小的Kp开始（如0.05），观察响应
   - 如果震荡，减小Kp或增加Kd
   - 如果有稳态误差，适当增加Ki

3. **安全考虑**：
   - 设置合理的 `force_max_velocity` 限制
   - 设置合理的 `force_timeout_ticks` 避免卡死
   - 监控力传感器读数范围
   - 确保丝杆电机有行程限位保护

## 执行流程

### 第一发（is_first_shot=true）
1. 传送带下行
2. PID减速
3. 扳机锁定
4. 传送带上行复位
5. **力矩闭环**（目标力 9025，容差 ±30）

### 第二发及以后（is_first_shot=false）
1. 传送带下行
2. PID减速
3. 扳机锁定 + 升降下行（并行）
4. 传送带上行复位
5. **力矩闭环**（闭环到上次记录的力值，使用配置的容差）

### 重置机制
- 遥控器双下时，重置 `fire_count_` 为 0，下次发射将作为第一发处理

## 紧急停止

在以下情况下，丝杆电机会自动停止：
- 调用 `cancel_all()`（双下紧急停止）
- 任务失败 `on_task_failure()`
- 力矩闭环动作退出 `on_exit()`

## 注意事项

1. **首次启用前必须调试**：PID参数需要根据实际硬件调整
2. **力传感器校准**：确保力传感器已正确校准
3. **电机方向**：确认电机正转时力增大，反转时力减小
4. **行程限位**：确保丝杆电机有软件或硬件限位保护
5. **超时处理**：如果力矩闭环超时返回FAILURE，任务会进入ERROR状态

## 调试建议

1. 先在手动模式下测试丝杆电机控制
2. 观察力传感器读数变化范围
3. 从保守的PID参数开始（小Kp，零Ki，小Kd）
4. 逐步调整参数，观察收敛速度和稳定性
5. 记录不同力值下的PID表现
