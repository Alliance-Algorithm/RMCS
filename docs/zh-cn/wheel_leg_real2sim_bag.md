# 轮腿 real2sim：同步 bag 记录组件设计

## 目标与边界

用同一条预设关节轨迹分别驱动架空真机和 Isaac Sim，对拍四台 DM、两台 M3508 的指令与反馈。记录器只观察 RMCS 接口，不负责电机使能、设零、轨迹生成或 RL 推理；录制失败不能被解释为一次有效的辨识试验。

现有 `wheel-leg-infantry-rl.yaml` 中的 `ValueBroadcaster` 仅发布无采样时间戳的 50 Hz 标量，不适合辨识毫秒级滞后。`StaticTorqueTestController` 只在驻留点写一行 CSV，`SweptFrequencyController` 只写单轴扫频 CSV；二者可作为实验激励的参考，但不能替代整腿同步样本。当前 RL 的目标角、实际发送的六轴力矩和每轴反馈接收时间也尚未全部暴露为 RMCS 输出。

## 组件边界与调度

建议新增 `rmcs_core::controller::identification::WheelLegBagRecorder`（RMCS `Component` + `rclcpp::Node`），只在辨识专用 bringup 中启用；首期的激励组件另设为 `WheelLegTrajectoryTestController`，并与 `rmcs::rl::RlController` 互斥，避免多个组件同时发布 `/control_torque`。

```text
WheelLegInfantryRL::Board（反馈状态、逐轴接收时刻）
    → WheelLegTrajectoryTestController（有界目标轨迹、PC PD、力矩命令）
    → WheelLegInfantryRL::Command（CAN 发送队列、发送前最终力矩）
    → WheelLegBagRecorder::update（同一 executor tick 复制一条固定大小样本）
    → 有界 SPSC 环形缓冲 → 非实时 ROS 发布线程 → ros2 bag record
```

记录器通过 `InputInterface` 订阅以上组件的输出，不创建反向控制依赖。给硬件 `Command` partner 增加**实际入队的**力矩值、入队时刻和可观测的入队结果，使记录器明确位于它之后；状态、实验目标和最终发送命令共享 `/predefined/update_count`。RMCS 状态更新在命令发送之前，同一 tick 的记录应明确表示“该 tick 使用的最新反馈 + 对应的 CAN 发送请求”，不能把 CAN 入队时刻称为电机实际收到指令的时刻。

若以后需要记录 RL 闭环，应从 RL 组件明确输出**执行后的**模型坐标目标、模型坐标测量值和最终电机力矩；不能根据 ONNX 的 6D 原始动作反推目标。首期专用辨识 bringup 不要求校准门控全开，仍须有独立的零力矩启动、机械限位、反馈超时和独立急停。当前硬件要求六轴及 IMU 全部反馈新鲜才发送非零力矩，单轴台架若只接一台电机需另设计受控的测试硬件配置，不能直接绕过看门狗。

## 单一同步样本合同（建议）

为避免多个无时间戳标量 topic 的错配，新建纯接口包 `rmcs_identification_msgs`（`rosidl_default_generators`），定义固定数组 `WheelLegSample.msg`；避免修改当前只保留 `.msg` 文件、尚未接入 `rosidl_generate_interfaces` 的 `rmcs_msgs` 构建路径。topic：`/wheel_leg/identification/sample`。

| 字段组 | 必须记录的值 | 解释 |
| --- | --- | --- |
| 帧信息 | `header.stamp`、`run_id`、`sample_index`、`update_count`、`phase` | ROS 时间在采样 tick 取得；试验阶段包含 PREPARE/EXCITE/SETTLE/STOP |
| 时序 | `sample_steady_ns`、`send_enqueue_steady_ns`、`motor_rx_steady_ns[6]`、`imu_rx_steady_ns`、逐轴 `rx_count[6]` | 单调时钟纳秒，允许识别重复使用的反馈；接收时刻不是传感器的物理采样时刻 |
| 实际接收 | `motor_q_rad[6]`、`motor_dq_rad_s[6]`、`motor_reported_torque_nm[6]`、`dm_status[4]`、`dm_fault[4]`、`dm_temperature[4]`、`imu_orientation`、`imu_angular_velocity`、`feedback_fresh` | 记录**驱动换算后、尚未作关节映射**的电机量；DM 力矩是报文估计，不是外部测力计读数 |
| 命令 | `joint_target_q_rad[4]`、`joint_target_dq_rad_s[4]`、`wheel_target_dq_rad_s[2]`、`requested_motor_torque_nm[6]`、`queued_motor_torque_nm[6]`、`command_valid` | 四腿模型坐标顺序 LH/LK/RH/RK，电机数组顺序 LH/RH/LK/RK/LW/RW；区分控制器计算与硬件限幅/失联置零后的 CAN 入队力矩 |
| 关节派生量 | `model_q_rad[4]`、`model_dq_rad_s[4]`、`model_mapping_valid` | 仅在传动映射经过标定验证时有效；否则只比较原始电机量 |
| 完整性 | `queue_dropped_total`、`enqueue_error_total` | 超过预设丢样阈值的试验不得用于毫秒级拟合；其他告警由低频状态 topic 发布 |

数组顺序应写入消息注释并随包固化为测试；不能用模型 P 顺序替代电机数组顺序。某字段无真实数据来源时使用有效位，不应填零伪装观测。例如真机没有弹簧位移传感器就不能把仿真中的弹簧坐标记作实测。如果 Board API 只确认软件入队，`enqueue_error_total` 只代表入队错误，不能被解释为电机收到 CAN 帧的确认。

单独发布一次 `/wheel_leg/identification/metadata`（建议 `std_msgs/msg/String` JSON，transient-local 或录制开始后重复发布），内容包含 schema 版本、Git commit、模型及配置 SHA-256、四台 DM 的 CAN/Master ID 与 `P_MAX/V_MAX/T_MAX`、逐轴符号/零位/传动矩阵、软限位和 IMU 外参、控制频率、Kp/Kd、额定电压或采样电压、目标轨迹的名字/种子/幅值/频率、试验边界条件（架空固定方式）。`run_id` 同时出现在样本和 metadata 中。配置与标定值不得默默从当时的 YAML 推测。

## 实时与录制策略

1. 在 executor 的 1 kHz tick 内，只复制固定大小、已预分配的结构并推入 `rmcs_utility::RingBuffer<Sample>`。记录器是唯一生产者；独立线程是唯一消费者。不要在实时路径做 ROS 序列化、bag 写盘、文件 flush、动态扩容或常规日志输出。
2. 后台线程转换为 ROS 消息并发布；外部 `ros2 bag record` 负责持久化。环形缓冲满时增加丢样计数，**不得阻塞控制周期**；以 `sample_index` 和 `update_count` 检测缺口。以预留约 4 秒的缓冲为起点（1 kHz 时至少 4096 条），根据实测吞吐再调整。
3. 开始试验前先启动 bag 并确认订阅者、metadata 已进入 bag；停止后查 `ros2 bag info`、样本首尾、丢样计数和相邻 tick 的间隔分布。录制期间附加发布频率 ≤ 1 Hz 的状态/丢样计数 topic。不要以收到 topic 的时间代替样本采样时间。
4. 观测/动作延迟拟合需要板级采样时刻或回环测量。当前 CAN 回调能提供主机**接收时刻**，但并不知道驱动的真实采样时刻；若缺少硬件时钟同步，应把结果标记为端到端有效延迟，不拆成未经测量的具体环节。

## 真机与 Isaac Sim 的同轨迹对拍

先做固定车体、轮子离地的整腿关节轨迹，再单独验证接地接触。真机和 Isaac 必须使用同一版本轨迹生成器与参数、相同模型关节坐标/初态、相同 PC PD、命令限幅与更新周期。先拟合编码器/传动映射及弹簧、摩擦、惯量，再拟合执行器响应和端到端延迟；保留未参与拟合的轨迹检查重现性。仅用某个模型的 ONNX action 作为试验输入无法隔离电机与机构差异。

首批离线指标：各关节 `q_target/q_real/q_sim` 三线图、RMSE/峰值误差、阶跃上升与稳定时间、超调、双向扫频幅值/相位、邻接关节交叉响应、命令与反馈力矩对比，以及时间戳/丢样报告。没有外部测力传感器时，电机反馈力矩和拟合的“实际机械输出力矩”应分开标注。

## 实施顺序与验收

1. 先增加硬件反馈接收时刻、逐轴有效位和 `Command` 入队力矩输出；使用内存/伪硬件驱动验证 `status → controller → command → recorder` 的执行顺序。独立设计辨识 bringup，保持现有 RL bringup 默认标定门控不变。
2. 增加接口包与只读记录组件；用伪设备持续 1 kHz 采样 30 秒，录 bag 后离线校验字段顺序、时钟单调、消息总数、标量精度以及断流和队列满时的计数。
3. 完成带显式运动限幅与急停的双关节轨迹组件；先测试不使能电机时的零力矩与录制，再架空实机逐步增加幅值。若有效反馈、配置元数据或缓冲区条件不满足，当前 run 标记为不可用于辨识。
4. 给 Isaac 环境实现相同的预设轨迹输入与同结构样本导出，留出训练/验证两套轨迹；验证后再将实测物理参数和合理的随机化范围反馈到训练环境。
