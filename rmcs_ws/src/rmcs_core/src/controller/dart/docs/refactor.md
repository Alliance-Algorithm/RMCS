# 飞镖控制代码重构

## rmcs-core 组件

1. **belt-controller** — 同步带；输入 `BeltCommand`，输出 `MechanismStatus` + 电机控制
2. **trigger-controller** — 扳机舵机 + 滑台；输入 `TriggerCommand` + setpoint，输出 status
3. **filling-controller** — 填装升降 + 限位舵机；输入 `FillingCommand`，输出 status
4. 底盘（暂不接入）

类型定义在 `rmcs_dart_guidance/include/rmcs_dart_guidance/msg/`（C++ enum）。  
`rmcs_core` 依赖 `rmcs_dart_guidance`。

## guidance 边界

上层只发命名命令、收 status 推进 Task；完成判定在 core。

## 具体逻辑

### belt-controller
只需要输出电机的控制速度，正为下行，负为上行

各个参数需要通过get_parameter从yaml获得

上层值保留如下几个指令：

- IDLE：控制速度为nan

- ABORT：控制速度为0

- DOWN_SLOW：控制速度为 belt_slow_down_velocity，力矩限制 belt_load_torque_limit，同时读取多圈角度。当运行的角度达到 slider_rail_length 或者堵转时，判定完成，完成后控制速度置0

- DOWN_FAST：控制速度为 belt_fast_down_velocity，力矩限制 belt_load_torque_limit,同时读取多圈角度。当运行的角度达到 slider_rail_length 或者堵转时，判定完成，完成后控制速度置0

- INIT：控制速度为 belt_up_soft_stage1_velocity，上行，力矩限制 belt_unload_torque_limit,堵转后判断完成，完成后控制速度置0

- UP_SOFT：一阶段控制速度为 belt_up_soft_stage1_velocity,当一阶段运行的角度达到 slider_rail_length * soft_stage1_persent ,进入二阶段；二阶段控制速度为 belt_up_soft_stage2_velocity，当一阶段运行的角度达到 slider_rail_length * soft_stage2_persent ,进入三阶段；三阶段控制速度为 belt_up_soft_stage3_velocity，当电机运行堵转后，判断完成,完成后控制速度置nan,一阶段力矩限制 belt_load_torque_limit，二三阶段力矩限制 belt_unload_torque_limit

- UP_SOFT_PART：控制速度为 belt_up_stage1_velocity ,力矩限制 belt_load_torque_limit， 当运行的角度达到 slider_rail_length * part_persent，判断完成，完成后控制速度置0

- DOWN_SLOW_PART：控制速度为 belt_slow_down_velocity ,力矩限制 belt_load_torque_limit，当运行的角度达到 slider_rail_length * part_persent 或者堵转，判断完成，完成后控制速度置0

- UP_HARD：控制速度为 belt_up_stage2_velocity，力矩限制 belt_load_torque_limit，当一阶段运行的角度达到 slider_rail_length 或者堵转后，判断完成，完成后控制速度置0

- BRAKE：控制速度为0，力矩限制 belt_load_torque_limit，这个由上层判断完成（大部分使用场景是在等待别的任务完成）

### filling-controller

上层只保留如下几个指令：

- IDLE：控制速度为nan

- ABORT：控制速度为0

- LIFT_UP：lift motor的控制速度为 lift_control_velociry，堵转后判断完成,完成后控制速度置0,力矩限制 lift_torque_limit

- LIFT_DOWN：lift motor的控制速度为 lift_control_velociry，堵转后判断完成，完成后控制速度置0,力矩限制 lift_torque_limit

- LIMIT_FREE：filling limit servo的控制角度为 free angle（uint16_t），收到命令100ms后完成,完成后保持原来的控制角度

- LIMIT_LOCK：filling limit servo的控制角度为 lock angle（uint16_t），收到命令100ms后完成,完成后保持原来的控制角度

- LIMIT_PULSE_FILL：filling limit servo的控制角度为 free angle（uint16_t），pulse_time后，控制角度为lock angle（uint16_t），切换控制角度后100ms判断完成,完成后保持原来的控制角度

### trigger-controller

上层只保留如下几个指令：

- IDLE：trigger motor的控制力矩为nan

- ABORT：trigger motor的控制力矩为nan（是的，这不是写错，就是nan）

- TRIGGER_FREE：扳机控制角度为 free_angle（double）

- TRIGGER_LOCK：扳机控制角度为 lock_angle（double）

- CARRIAGE_UP：滑台目标速度为 carriage_velocity，由 trigger-controller 内置速度 pid 输出控制力矩，上层判断完成,力矩限制 carriage_torque_limit

- CARRIAGE_DOWN：滑台目标速度为 carriage_velocity，由 trigger-controller 内置速度 pid 输出控制力矩，上层判断完成,力矩限制 carriage_torque_limit

- CARRIAGE_GOTO：运动到相对零点编码器值 set_point 的位置，内置一个位置 pid 用于将编码器误差转换成目标速度，再由内置速度 pid 输出控制力矩；运动到 250ms 内的误差都小于一个可接受的误差范围内，完成判断,力矩限制 carriage_torque_limit

- CARRIAGE_CALIBRATE：每次滑台下行分两阶段；一阶段直接输出恒定控制力矩 carriage_calibrate_torque_limit（下行为负方向），持续 carriage_calibrate_launch_ticks 后进入二阶段；二阶段直接输出恒定控制力矩 carriage_calibrate_control_torque（下行为负方向），堵转判定只看速度绝对值连续小于等于 carriage_stall_velocity_threshold 达到 carriage_stall_ticks 后记录当前编码器值；未完成三次堵转时上行回退，目标速度为 calibrate_rollback_velocity，由内置速度 pid 输出控制力矩，回退 calibrate_rollback 编码器值（不做精确闭环，只按累计路程判断），然后再次下行；三次堵转时的原编码器数值取平均值作为新的零点，第三次堵转后仍需上行回退 calibrate_rollback 编码器值，回退完成后才判断完成,回退阶段力矩限制 carriage_calibrate_torque_limit
