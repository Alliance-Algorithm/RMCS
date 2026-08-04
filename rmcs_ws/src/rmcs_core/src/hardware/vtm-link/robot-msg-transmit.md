# 机器人自定义通信协议

新增协议，该协议和custom-msg-transmit的通道一样，都是通过vt13图传链路串口发布。使用 `referee::Frame` 封装，`cmd_id = 0x0310`。

## 数据包格式

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 1B | message_type | 固定 `0x05` |
| 1..4 | 4B | pitch 编码器值 | 来源 `/gimbal/pitch/raw_angle`, `int32_t` |
| 5..8 | 4B | yaw 编码器值 | 来源 `/gimbal/top_yaw/raw_angle`, `int32_t` |
| 9..12 | 4B | 当前目标弹速 | 12 或 16, `int32_t`. 前级控制速度未 ready 时为 `0` |
| 13..16 | 4B | 前级摩擦轮控制速度 | 来源 `/gimbal/first_front_friction/control_velocity`, `int32_t`, 单位 rad/s |
| 17..20 | 4B | 后级摩擦轮控制速度 | 来源 `/gimbal/first_back_friction/control_velocity`, `int32_t`, 单位 rad/s |
| 21..24 | 4B | 摩擦轮 0 当前速度 | 来源 `/gimbal/first_front_friction/velocity`, `int32_t`, 单位 rad/s |
| 25..28 | 4B | 摩擦轮 1 当前速度 | 来源 `/gimbal/second_front_friction/velocity`, `int32_t`, 单位 rad/s |
| 29..32 | 4B | 摩擦轮 2 当前速度 | 来源 `/gimbal/third_front_friction/velocity`, `int32_t`, 单位 rad/s |
| 33..36 | 4B | 摩擦轮 3 当前速度 | 来源 `/gimbal/first_back_friction/velocity`, `int32_t`, 单位 rad/s |
| 37..40 | 4B | 摩擦轮 4 当前速度 | 来源 `/gimbal/second_back_friction/velocity`, `int32_t`, 单位 rad/s |
| 41..44 | 4B | 摩擦轮 5 当前速度 | 来源 `/gimbal/third_back_friction/velocity`, `int32_t`, 单位 rad/s |
| 45..299 | 255B | 填充 | `0x00` |

数据体总计 300 字节（1B msg_type + 11×4B 数据 + 255B 填充）。

所有数值字段为 `int32_t` 小端序。当对应 input 未 ready 时，字段填入 `INT32_MIN` (-2147483648)。

## 目标弹速推断

根据前级摩擦轮控制速度 (rad/s) 推断:

| 前级控制速度 | 目标弹速 |
|-------------|---------|
| < 420 | 12 |
| ≥ 420 | 16 |

若前级控制速度 input 未 ready，目标弹速为 `0`。

## 发送策略

- 无缓存队列，只发最新数据
- 发送间隔 500ms
- input 不全 ready 仍发送，对应字段填 `INT32_MIN`
