# 大 yaw 完整辨识

运行配置为 `gimbal_controller_identification.yaml`。joint2 抬升关闭，不输出支撑力矩；小yaw通过滑模保持解锁时捕获的电机相对角度。先确认大yaw有转动空间，遥控双下后回中解锁，大yaw速度不超过10 rad/s时自动开始，不再等待joint2到位。

每遍按以下顺序运行，第二遍把整套力矩符号反转。每遍67.9秒，总135.8秒。

|阶段|力矩与时间|
|---|---|
|静置|0 Nm，1 s|
|脉冲|2/3.5/5/6 Nm；每级按`+ - - +`各一次，出力0.20 s、间隔0.70 s|
|正弦扫频|峰值4 Nm，0.30→1.20 Hz，16 s|
|恒力矩|+3 Nm保持2 s，0 Nm保持0.5 s，-3 Nm保持2 s|
|线性|`0→+3.5→0→-3.5→0`，每段4 s，连续两组|

大yaw辨识分支直接下发所定义波形，硬件夹紧到±6 Nm；大yaw鲁棒控制不参与本轮。完成后大yaw停止出力，小yaw继续保持；双下复位后可再做一轮。遥控/反馈失效或大yaw速度超10 rad/s将停止本轮；查看首个故障原因，不会在条件恢复后自行重跑。

小yaw使用位置外环生成回到捕获角度的目标速度（增益4 s⁻¹、最高0.35 rad/s），内环复用已辨识模型和通用滑模速度控制器，速度反馈增益20 s⁻¹、滑模增益12 rad/s²，力矩上限10 Nm、上升变化率75 Nm/s。0.35 rad/s是回位目标速度上限，受外力推动的实际速度仍可能超过它。保持模式不积累速度误差积分。相对角度以小yaw电机编码器为准，会随大yaw整体转动，不是空间姿态锁定。双下或反馈失效会清除保持目标。

Foxglove可查看：

- `/gimbal/test/state`：等待、运行、完成或故障。
- `/gimbal/test/big_yaw_identification_phase`：pulse/sine/constant/ramp等当前阶段。
- `/gimbal/test/big_yaw_identification_pass`：1或2。
- `/gimbal/test/identification_elapsed_s`：本轮经过时间。
- `/gimbal/test/big_yaw_identification_torque`、`/gimbal/test/big_yaw_torque_applied`：波形请求与软件下发值。
- `/chassis/big_yaw/angle`、`/chassis/big_yaw/velocity`、`/chassis/big_yaw/torque`：电机反馈。
- `/gimbal/test/big_yaw_identification_fault_reason`：首个故障原因，双下清除。
- `/gimbal/test/small_yaw_hold_target_angle`、`/gimbal/test/small_yaw_hold_position_error`：小yaw相对角度目标与误差。
- `/gimbal/test/small_yaw_robust_target_velocity`、`/gimbal/test/small_yaw_robust_velocity`、`/gimbal/test/small_yaw_robust_torque_applied`：小yaw滑模保持内环的目标速度、实际速度和下发力矩。

服务启动自动录制到 `/root/rmcs_recordings/big_yaw_identification`，`LATEST`指向最近目录。配置原文、标量消息、原子CAN样本及日志均保存；断开Foxglove不影响录包。`.script/record-gimbal status --root /root/rmcs_recordings/big_yaw_identification` 可检查录制。

`/gimbal/feedback/big_yaw_can_sample`保存同一CAN2/0x33反馈帧的原始角度和协议反馈力矩、主机CAN接收回调的monotonic时间，以及最近命令的发送时间和量化力矩。DM8009此协议没有独立电流字段，因此`current_amp`为NaN；反馈力矩也不能当作独立轴端力矩传感器。CAN接收时间为主机时间，不是电机硬件时间。原子消息里的命令有独立时间戳，分析时需要按时间对齐。
