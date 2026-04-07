# steering-infantry 电机 CAN ID 对照审计

## 范围
- 文件: `rmcs_ws/src/rmcs_core/src/hardware/steering-infantry.cpp`
- 目标: 收集所有电机的发送报文 ID 与回调报文 ID，反推出电机 ID，并检查是否有偏差。

## 反推规则
- `DjiMotor` 采用 4 路合帧发送（`device::CanPacket8`）。
- 对于 `can_id = 0x200` 的发送帧:
  - `q0 -> 0x201`
  - `q1 -> 0x202`
  - `q2 -> 0x203`
  - `q3 -> 0x204`
- 对于 `can_id = 0x1FF / 0x1FE` 的发送帧（本项目代码中的使用方式）:
  - `q0 -> 0x205`
  - `q1 -> 0x206`
  - `q2 -> 0x207`
  - `q3 -> 0x208`
- `LkMotor` 直接使用单独 `can_id` 发送与接收（非 4 路合帧电机映射）。

## TopBoard 电机映射
代码位置:
- 构造/电机定义: `121-137`
- 发送: `212-253`
- 回调: `256-279`

| 电机 | 发送总线/ID/槽位 | 回调 ID | 反推电机 ID | 结果 |
|---|---|---|---|---|
| `/gimbal/left_friction` | CAN1, `0x200`, `q2` | `0x203` | `3` | 一致 |
| `/gimbal/right_friction` | CAN1, `0x200`, `q3` | `0x204` | `4` | 一致 |
| `/gimbal/bullet_feeder` | CAN1, `0x1FF`, `q0` | `0x205` | `5` | 一致 |
| `/gimbal/yaw` (LK) | CAN2, `0x141` | `0x141` | `0x141` | 一致 |
| `/gimbal/pitch` (LK) | CAN2, `0x142` | `0x142` | `0x142` | 一致 |

## BottomBoard_one 电机映射
代码位置:
- 构造/电机定义: `308-328`
- 发送: `396-434`
- 回调: `437-466`

| 电机 | 发送总线/ID/槽位 | 回调 ID | 反推电机 ID | 结果 |
|---|---|---|---|---|
| `/chassis/right_front_wheel` (`chassis_wheel_motors_[1]`) | CAN1, `0x200`, `q0` | `0x201` | `1` | 一致 |
| `/chassis/left_front_wheel` (`chassis_wheel_motors_[0]`) | CAN1, `0x200`, `q1` | `0x202` | `2` | 一致 |
| `/chassis/left_front_steering` (`chassis_steering_motors_[0]`) | CAN1, `0x1FE`, `q1` | `0x206` | `6` | 一致 |
| `/chassis/right_front_steering` (`chassis_steering_motors_[1]`) | CAN1, `0x1FE`, `q3` | `0x208` | `8` | 一致 |
| `/chassis/climber/left_back_motor` (`chassis_back_climber_motor_[0]`) | CAN2, `0x200`, `q0` | `0x201` | `1` | 一致 |
| `/chassis/climber/right_back_motor` (`chassis_back_climber_motor_[1]`) | CAN2, `0x200`, `q1` | `0x202` | `2` | 一致 |
| `/chassis/climber/left_front_motor` (`chassis_front_climber_motor_[0]`) | CAN2, `0x200`, `q2` | `0x203` | `3` | 一致 |
| `/chassis/climber/right_front_motor` (`chassis_front_climber_motor_[1]`) | CAN2, `0x200`, `q3` | `0x204` | `4` | 一致 |

## BottomBoard_two 电机映射
代码位置:
- 构造/电机定义: `491-507`
- 发送: `563-591`
- 回调: `594-610`

| 电机 | 发送总线/ID/槽位 | 回调 ID | 反推电机 ID | 结果 |
|---|---|---|---|---|
| `/chassis/right_back_wheel` (`chassis_wheel_motors2_[1]`) | CAN1, `0x200`, `q2` | `0x203` | `3` | 一致 |
| `/chassis/left_back_wheel` (`chassis_wheel_motors2_[0]`) | CAN1, `0x200`, `q3` | `0x204` | `4` | 一致 |
| `/chassis/left_back_steering` (`chassis_steering_motors2_[0]`) | CAN1, `0x1FE`, `q0` | `0x205` | `5` | 一致 |
| `/chassis/right_back_steering` (`chassis_steering_motors2_[1]`) | CAN1, `0x1FE`, `q2` | `0x207` | `7` | 一致 |

## 总结
- 结论 1: 所有电机都能在代码中找到“发送 -> 回调”的闭环映射，未发现直接冲突或漏接收。
- 结论 2: 从代码反推，当前电机 ID 规划为:
  - TopBoard: 摩擦轮 `3/4`，拨弹 `5`，LK 云台 `0x141/0x142`
  - BottomBoard_one: 轮 `1/2`，前转向 `6/8`，攀爬 `1/2/3/4`（CAN2）
  - BottomBoard_two: 后轮 `3/4`，后转向 `5/7`

## 偏差/问题反馈
- 问题 1（注释已提示）: 文件头标注“电机ID没更改”，说明该映射可能为占位或沿用旧车配置，不能等同于当前实车真值。
- 问题 2（可维护性）: 多处左右轮顺序与 ID 顺序是“交叉映射”（例如右前轮是 ID1、左前轮是 ID2），逻辑上可行，但后续维护容易误改。
- 问题 3（非 ID 但会影响状态）: `TopBoard::update()` 中调用了 `gimbal_pitch_motor_.update_status()`，但未调用 `gimbal_yaw_motor_.update_status()`，可能导致 yaw 角度状态不更新。

## 建议
- 用实车实测确认最终 ID:
  1. 在各 `can*_receive_callback` 临时打印所有 `can_id`。
  2. 单电机动作，建立“物理电机 <-> can_id”表。
  3. 以实测表覆盖本文件映射，再同步更新发送槽位与回调分发。
