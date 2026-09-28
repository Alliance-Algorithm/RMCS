# DM 启动与失能诊断

关节使用 MIT 力矩模式。使能请求、反馈已使能和控制允许是三个不同状态；
四台状态均有效且稳定后才输出运动力矩。未激活时发送零力矩帧。

## 启动流程

- 清错期间保持零力矩；同总线的左右电机错开周期发送系统帧。
- 每台使能都要求 FC 之后的新鲜 `status=1` 反馈，旧缓存不能完成确认。
- 四台连续就绪 50 ms 后开放运动；启动只重试未确认的电机，单台间隔至少 100 ms。
- 启动总时限 2 s；超时或运行中失去反馈就绪后锁住输出，直到双下/取消使能。
- 双下持续发送零力矩并重复 FD；使能、重试和故障状态复位。
- 运行中不周期发送 FC，也不在失能后自动恢复运动。

板卡关闭 CAN 自动重发时，必须依赖反馈确认，不能把“发送过 FC”当作使能成功。

## 状态入口

```sh
ros2 service call /rmcs/service/robot_status std_srvs/srv/Trigger '{}'
```

检查 `Joint control`、四台 DM 的 `status/fault/fresh/age_ms`，并保留首次
`[joint_enable] first unavailable` 日志。

| 字段 | 含义 |
| --- | --- |
| `enabled` | 上层使能请求 |
| `control_active` | 四台已确认，允许控制输出 |
| `phase=startup_timeout` | 2 s 内未完成启动确认 |
| `phase=running_unavailable` | 曾经开放运动，随后失去就绪 |
| `ever_active` | 本次请求是否曾允许控制 |
| `pending_mask` | LH、LK、RH、RK 对应位 0–3；4 为右髋，8 为右膝 |
| `FC_attempts` | 四台各自的使能尝试次数 |
| `controller_reason` | 角度控制器拒绝运行的原因 |

反馈新鲜且 `status=0` 表示电机回报失能；角度未到目标而输出为零时，先检查门控和原因。
仅靠零指令无法判断故障来自目标角、控制器、使能或 CAN。

## 实现与验证

- `hardware/device/dm_joint_enable_sequence.hpp`：确认、重试、超时、复位。
- `hardware/device/dm_motor.hpp`：协议、反馈及接收时间。
- `hardware/wheel-leg-infantry-rl.cpp`：板卡收发、运动门控和状态服务。

`dm_joint_enable_sequence_test` 覆盖丢 FC、旧反馈、短暂使能、运行中失能、故障码和复位；
`dm_motor_test` 覆盖帧和反馈转换。使用 `BUILD_TESTING=ON` 构建后执行：

```sh
colcon test --merge-install --packages-select rmcs_core --ctest-args -R 'dm_motor_test|dm_joint_enable_sequence_test'
```

一次性 VEL 故障注入脚本及生成报告已移除，历史记录可在 Git 查看。
这些离线回归不替代实车 CAN 抓包或带载验证。
