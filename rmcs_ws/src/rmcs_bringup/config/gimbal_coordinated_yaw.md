# 单摇杆大小 yaw 联动

`gimbal_controller_test.yaml`用右摇杆X累加总目标角。小 yaw先响应目标，大 yaw在小轴偏离中位约5°后接管；小轴按大轴实测速反向补偿，随后回到中位附近。摇杆回中时保存总目标角，继续完成交接。

使能时捕获小 yaw当前角作为中位，将当前合成朝向定义为零。测试前将小轴放在机械中间、右摇杆回中，再按双下流程使能。joint2解除双下后抬升30°并保持。

## 参考分配

两轴正方向相同时，总角为大轴相对使能起点的增角，加小轴相对捕获中位的增角。`coordinated_small_yaw_direction`支持+1或-1，当前使用+1。

- 总目标按实际dt累加摇杆输入。0.35 rad防积累窗口只截断会继续增加积差的新输入，不会让实际运动拖走已保存目标。
- 大轴首次接管需小轴偏角达到`coordinated_follow_start_offset_rad`（5°）。启动后继续跟随到原0.01 rad中位死区，待偏角、大轴实测速及两轴速度参考共同停稳，持续至少响应预留时间后才等待下一次启动。这不是永久5°死区。
- 大轴需求为连续扣除中位死区后的小轴偏角乘跟随增益。回中时使用较低的`coordinated_centered_follow_gain`，再按反向补偿能力、全局速度和加速度限制。
- 小轴相对速度需求为摇杆角增速加总角反馈纠偏，再减大轴实测速。仅回中时对总角误差连续扣除0.003 rad死区；实际大轴运动的补偿独立保留。
- 小轴最终合成速度按剩余相对角度提前制动。±45°处外转参考为零，向内回转允许，同时考虑对侧边界的刹停距离。

| 当前配置 | 数值 |
|---|---:|
| 摇杆最大目标角增速 | 4.5 rad/s |
| 大 yaw全局速度 / 参考加速度上限 | 4.5 rad/s / 3 rad/s² |
| 小 yaw全局速度 / 参考加速度上限 | 8 rad/s / 6 rad/s² |
| 大 yaw开始接管偏角 | 5° |
| 跟随增益 / 回中跟随增益 | 6.4 / 3.2 s⁻¹ |
| 大 yaw停稳中位死区 | 0.01 rad，约0.57° |
| 总角反馈增益 / 回中误差死区 | 2 s⁻¹ / 0.003 rad |
| 小 yaw外转零速边界 / 故障边界 | ±45° / ±0.8 rad |
| 制动响应预留 | 0.32 s |
| 大轴反向补偿能力使用比例 | 0.85 |

本轮两轴全局速度和摇杆角增速上限均为上一轮的2倍，参考加速度也乘2。全局上限不是可执行速度保证；角度制动、执行响应和10 Nm力矩都会限制实速。两轴辨识模型及鲁棒内环保留，小轴力矩建立斜率60 Nm/s。

## 制动与反向补偿能力

距某方向边界剩余角度为d，参考减速度为a，更新周期为dt，响应预留为tau。该方向速度上限为：

```text
min(Vmax, sqrt((a*(dt+tau))² + 2*a*max(d,0)) - a*(dt+tau))
```

边界外该方向上限为0。`coordinated_small_speed_zero_offset_rad=π/4`，设0可关闭角度制动；`coordinated_small_braking_response_time_s=0.32`预留执行滞后。按1 ms周期和6 rad/s²减速度，中位向外上限约1.70 rad/s，许可角域内最高约2.84 rad/s，不会实际达到名义8 rad/s。

`coordinated_counterrotation_speed_fraction=0.85`将大轴需求限制为小轴当下反方向包络能力的85%，给总角纠偏留余量；对正负方向分别计算，不缩放用户目标。限制位于大轴加速度斜坡之前，保留减速平滑性。比例设0可保留旧分配。只增加大轴速度而忽略反补能力，会使满速回中产生大幅总角过冲。

普通小轴参考按配置加速度变化。若外力或执行滞后已导致无法平滑刹停，优先将速度参考投影回包络，惯性加速度前馈仍限±6 rad/s²，与硬件校验同步。±45°约束参考外转速度，实际关节可能因惯性略微越过，不能视为机械硬限位。

`coordinated_follow_start_offset_rad=0`关闭迟启动；`coordinated_centered_follow_gain=0`回中时沿用主增益。默认值保留旧行为，当前配置显式启用新功能。

## 许可与测量

双下、遥控离线或任一 yaw反馈失效时两轴撤力。位置跳变、控制更新间隔超过20 ms或小轴超出捕获中位±0.8 rad会清目标并失去协调许可，需重新双下、回中后解锁。故障边界约45.84°，没有随速度设置放宽。joint2保留独立控制和保护。

大轴硬件角连续性参数`position_tracking_max_velocity_rad_s`与速度上限同步为4.5 rad/s，其他配置默认仍为1；50 ms采样窗口、0.03 rad量化余量和故障重解锁保留。协调器按距上次角变化的累计控制时间检查两轴阶梯反馈，最多50 ms，避免把1 kHz控制周期当成CAN采样间隔。非联动模式摇杆参数仍限2 rad/s。

仅协调补偿前馈将绝对值不超过0.02 rad/s的大轴实测速置零，抑制静止量化噪声；原始角度和内环测速保留。ArmBoard IMU主要随大轴运动，不能直接作为总指向反馈。本模式没有底盘移动后的惯性空间锁定。

## 验证与录制

测试覆盖迟启动、正反向接管、回中继续跟随、停稳后重新等待启动、合成方向±1、故障清零、角度展开、异步CAN阶梯及速度和加速度合同。简化速度执行仿真检查持续满杆后的交接与总角过冲，不能替代摩擦、10 Nm饱和、弹性或实机延迟测试，也不能证明实际转速翻倍。

Foxglove保留既有两轴目标/实际速度、力矩、许可及以下话题，共103项：

| 话题 | 含义 |
|---|---|
| `/gimbal/test/coordinated_yaw_active` | 协调许可 |
| `/gimbal/test/yaw_target_angle` | 累计总目标角 rad |
| `/gimbal/test/yaw_actual_angle` | 编码器合成总角 rad |
| `/gimbal/test/yaw_angle_error` | 总目标角减实际总角 rad |
| `/gimbal/test/small_yaw_center_angle` | 捕获中位 rad |
| `/gimbal/test/small_yaw_center_offset` | 相对中位偏角 rad |
| `/gimbal/test/small_yaw_coordinated_target_velocity` | 小轴速度参考 rad/s |
| `/gimbal/test/small_yaw_coordinated_target_acceleration` | 小轴惯性加速度前馈 rad/s² |
| `/gimbal/test/yaw_reference_limited` | 新增目标量或参考速度受限，非故障标志 |

录制工具为`.script/record-gimbal`，选择`gimbal_controller_test.yaml`，保存根目录`/root/rmcs_recordings/dual_yaw_control`。服务模板自动录制三种辨识配置，双轴控制录制需显式启动。
