# 硬件电机测试（hardware）运行手册

测试入口：`rmcs_bringup/config/hardware.yaml`。

## 1. CAN 口映射

- `can_index: 0` -> DM8009
- `can_index: 1` -> LK MG4010Ei10
- `can_index: 2` -> DJI GM6020
- `can_index: 3` -> DJI M3508

## 2. 启动

```bash
source /workspaces/RMCS/rmcs_ws/install/setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=hardware
```

## 3. 关键参数

- `can_index`：选择 CAN 口和电机类型
- `motor_id`：电机 ID
- `feedback_id`：DM8009 反馈 ID
- `encoder_zero_point`：LK / DJI 零点
- `angle_bias`：DM8009 角度偏置
- `motor_reversed`：方向反接
- `multi_turn_angle`：LK / DJI 多圈角度

## 4. 遥控行为

- 双下：失能
- 其余：使能
- LK 4010i10 保留位置 / 速度 / 扭矩模式
- DM8009 / GM6020 / M3508 使用扭矩测试模式

## 5. 状态服务

```bash
ros2 service call /rmcs/service/hardware_motor_test_status std_srvs/srv/Trigger "{}"
```
