`check-big-yaw-velocity-zero` 独立订阅大 yaw 现有遥测，核对协议速度与编码器位置变化的速度是否一致。它不发布电机命令、不写参数、不重启，也不会自动校准或改变协议解码。

保持机械装置确实静止，遥控双下后，在开发容器运行：

```sh
.script/check-big-yaw-velocity-zero
.script/check-big-yaw-velocity-zero --duration 5 --output-dir /tmp/big-yaw-zero-check
```

默认 SSH 别名 `remote`，远端加载 `/root/env_setup.bash`，先订阅热身 1 秒，再观察约 5 秒；`--host` 可选择其他 SSH 别名。可选的新 `--output-dir` 保存在本地，包含报告、完整接收样本 JSON 和 CSV。该目录必须尚不存在。

工具只读取 `/chassis/big_yaw/angle`、`/chassis/big_yaw/velocity`、大 yaw 在线、遥控在线及两开关、big yaw 软件施加力矩和鲁棒控制启用状态。每个 1 秒窗口要求连续双下、遥控和大 yaw 在线、控制未启用、软件施加力矩为零，且编码器位置跨度小于 0.0005 rad；各流间隔超过 50 ms 或缺少端点样本的窗口不参与统计。这些只是诊断筛选条件，不会成为控制限制。

协议均速按该流自己的接收时间积分；位置速度用解包角后的窗口端点位移除以时间。输出 `observed_protocol_bias_rad_s` 为“协议均速减位置速度”的中位数及 10/90 分位。无合格静止窗口时输出 `valid=false`、偏置为 `null`，退出码 2；连接或采样执行失败退出码 1。正常得到至少一个合格窗口退出码 0，仍应查看窗口数和分布。

时间来自远端 `time.monotonic_ns()` 接收回调，不是电机采样或原子 CAN 时间。观察到约 −0.006 rad/s 的差值，表示该条件下协议反馈偏负；它本身不能证明电机固件的量化约定、主机解码错误，也不是应当直接写进控制器的修正值。
