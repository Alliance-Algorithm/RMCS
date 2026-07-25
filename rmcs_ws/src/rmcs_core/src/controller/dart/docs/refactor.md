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

- DOWN_SLOW：控制速度为 belt_slow_down_velocity，同时读取多圈角度。当运行的角度达到 slider_rail_length 或者堵转时，判定完成，完成后控制速度置0

- DOWN_FAST：控制速度为 belt_fast_down_velocity，同时读取多圈角度。当运行的角度达到 slider_rail_length 或者堵转时，判定完成，完成后控制速度置0

- UP_SOFT：一阶段控制速度为 belt_up_soft_stage1_velocity，当一阶段运行的角度达到 slider_rail_length * soft_stage1_persent ,进入二阶段；二阶段控制速度为 belt_up_soft_stage2_velocity，当一阶段运行的角度达到 slider_rail_length * soft_stage2_persent ,进入三阶段；三阶段控制速度为 belt_up_soft_stage3_velocity，当电机运行堵转后，判断完成,完成后控制速度置nan

- UP_SOFT_PART：控制速度为 belt_up_stage1_velocity ,当运行的角度达到 slider_rail_length * part_persent，判断完成，完成后控制速度置0

- DOWN_SLOW_PART：控制速度为 belt_slow_down_velocity ,当运行的角度达到 slider_rail_length * part_persent 或者堵转，判断完成，完成后控制速度置0

- UP_HARD：控制速度为 belt_up_stage2_velocity，当一阶段运行的角度达到 slider_rail_length 或者堵转后，判断完成，完成后控制速度置nan

- BRAKE：控制速度为0，这个由上层判断完成（大部分使用场景是在等待别的任务完成）

### filling-controller

上层只保留如下几个指令：

- IDLE：控制速度为nan

- ABORT：控制速度为0

- LIFT_UP：lift motor的控制速度为 lift_control_velociry，堵转后判断完成,完成后控制速度置0

- LIFT_DOWN：lift motor的控制速度为 lift_control_velociry，堵转后判断完成，完成后控制速度置0

- LIMIT_FREE：filling limit servo的控制角度为 free angle（uint16_t），收到命令100ms后完成,完成后保持原来的控制角度

- LIMIT_LOCK：filling limit servo的控制角度为 lock angle（uint16_t），收到命令100ms后完成,完成后保持原来的控制角度

- LIMIT_PULSE_FILL：filling limit servo的控制角度为 free angle（uint16_t），pulse_time后，控制角度为lock angle（uint16_t），切换控制角度后100ms判断完成,完成后保持原来的控制角度
