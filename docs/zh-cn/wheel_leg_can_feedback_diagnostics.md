# 轮腿反馈与驱动诊断

当前实现为 `hardware/wheel_leg.cpp` 和 `hardware/util/wheel_leg_drive.hpp`。
本次保留原工作区未提交的 DM 左右交替调度和 WheelBipe 配置修改，
同时将硬件统一为 PC 侧 PD 后的 MIT 纯力矩执行。旧专用帧镜像、辨识 CSV、
原始帧记录器和预览入口已删除。

## 接收映射

| SDK 通道 | 设备 | ID / 接收帧 | 命令 |
| --- | --- | --- | --- |
| `kCan0` | 左、右 M3508 | ID 1/2，0x201/0x202 | 0x200 前两个槽 |
| `kCan1` | 左、右 DM 髋 | ID 1/2，反馈 ID 由 `dm_master_ids` 前两项指定 | ID 1/2 |
| `kCan2` | 左、右 DM 主动膝驱动轴 | ID 1/2，反馈 ID 由后两项指定 | ID 1/2 |

保留安装方向、编码器零点和轮侧 15.62 减速比。仅匹配 8 字节标准 CAN 数据帧。
反馈发布顺序统一为左髋、左膝驱动、右髋、右膝驱动、左轮、右轮。
`kCanN` 是 SDK 名称，实物端口需按板卡标记核对。

## 时效与发送

硬件统一拥有 CAN/IMU 接收时效：正式 YAML 为 40 ms；DR16 为 100 ms。
原控制器 40 ms 与硬件 50 ms 的重复门已收敛成硬件 40 ms，
因此等待驱动就绪阶段同样受 40 ms 门约束。
缺失任何必要反馈时不发送非零力矩；启动阶段等待就绪，运行中失效由主控制器锁存。

DM 正常按左右交替时隙发送，每台 500 Hz、每条总线每毫秒一帧。
清错/使能确认、必要重试及失能顺序保留。双下或使能请求撤销时立即发送四路 FD，
随后发送零增益零力矩 MIT，不等待时隙。两轮仍按 executor 周期发送。
硬件不接收位置目标、Kp/Kd 或 DM 内置位置 PD 请求。

## 可用诊断

`/rmcs/service/robot_status` 显示整体反馈、遥控时效、驱动就绪和逐侧 DM 状态/故障码。
每轴还提供 `feedback_steady_ns`、`feedback_sequence`；IMU 同样提供接收时间和序号。
ValueBroadcaster 保留当前运行阶段、故障、自起阶段及 DM 状态/故障码。
查询方式：

```sh
.script/remote-status
# 或在已运行的 ROS 图中只读调用
ros2 service call /rmcs/service/robot_status std_srvs/srv/Trigger '{}'
```

状态码应结合接收时龄阅读。主机收到反馈或提交发送请求，均不能单独证明实物链路和
控制映射已通过验收。此次构建及离线调度测试未连接硬件，也未证明先前反馈中断已经消失。
历史原始诊断记录已保留在重构前快照；旧 `feedback/*` 实验统计不再属于现行接口。
