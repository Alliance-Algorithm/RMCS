`record-gimbal` 在小电脑启动独立的 ROS bag 订阅进程，持续保存现有 RMCS ValueBroadcaster 的原始消息。它不修改参数、不重启 executor，也不发布电机命令；录制频率取决于现有发布频率。

在开发容器、仓库目录运行：

```sh
.script/record-gimbal start
.script/record-gimbal status
.script/record-gimbal stop
```

默认 SSH 别名是 `remote`，可使用 `--host creeper@主机地址`；`--root /绝对路径` 可修改远端保存根目录。`--config /远端绝对路径.yaml` 选择录制话题和元数据使用的配置，路径位于小电脑上，启动前应与当前运行的配置保持一致。无需 `RMCS_PATH`。远端自动加载 `/opt/ros/jazzy/setup.bash` 及存在时的 `/rmcs_install/setup.bash`。

例如录制小 yaw 辨识，使用独立保存根目录：

```sh
.script/record-gimbal start --config /rmcs_install/share/rmcs_bringup/config/small_yaw_identification.yaml --root /root/rmcs_recordings/small_yaw_identification
.script/record-gimbal status --root /root/rmcs_recordings/small_yaw_identification
.script/record-gimbal stop --root /root/rmcs_recordings/small_yaw_identification
```

`status` 和 `stop` 按 `--root` 查找已有录制，无需重复指定 `--config`，也不会读取配置文件。不同配置请使用不同根目录；同一根目录已有活跃录制时，即使换了 `--config`，`start` 仍会返回已有录制，必须先停止它再开始新录制。

每次录制保存到远端 `~/rmcs_recordings/big_yaw/北京时间日期_时间_微秒/`，每 300 秒滚动一个 MCAP 文件。小 yaw 两种辨识配置会额外录制原子 CAN 诊断话题 `/gimbal/feedback/small_yaw_can_sample`；大 yaw 的 `gimbal_controller_identification.yaml` 会额外录制 `/gimbal/feedback/big_yaw_can_sample`。大 yaw 样本包含同帧原始角度、MIT 反馈力矩、主机 CAN 接收回调的单调时间戳，以及最后一次发送的量化命令和时间；DM 反馈没有独立电流字段，因此 `current_amp` 为 NaN。该时间戳不是电机硬件采样时间。`LATEST` 指向最近一次目录；重复 `start` 遇到仍活跃的录制会返回其状态，不再启动第二个进程。进程脱离 SSH 运行，断开终端后继续记录。

录制话题动态读取所选配置的 `value_broadcaster.ros__parameters.forward_list`；省略 `--config` 时仍使用远端 `/rmcs_install/share/rmcs_bringup/config/gimbal_controller_test.yaml`。目录包含所选配置原文、配置路径与 SHA256 和进程元信息 `recording.json`、可靠订阅深度 1000 的 `qos.yaml`、`recorder.log` 及 `bag/*.mcap`。录制缓存为 16 MiB；不会将 200 Hz 数据降为诊断日志的 1 Hz。

`stop` 核对 PID 启动时间、进程命令和本次 bag 路径后，仅向该录制进程发送 SIGINT，最多等候 10 秒完成写盘；不会强制杀进程。读取文件时优先使用已经滚动完成的 MCAP；当前写入文件可在 `stop` 完成后读取。普通标量消息不带电机采样时间戳，录包的接收时间不能证明每条都是新的硬件反馈。

小 yaw 两种辨识配置及大 yaw 的 `RMCS_ROBOT_TYPE=gimbal_controller_identification` 已接入同一服务自动录制路径；大 yaw 保存根目录为 `/root/rmcs_recordings/big_yaw_identification`。服务启动会调用录制器，服务停止会先正常结束录包再停止控制进程。部署更新后的服务脚本和录制器才会生效，其他机器人配置不会自动录制。
