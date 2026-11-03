云台测试的 Foxglove 布局：`gimbal_controller_test_foxglove.json`。左侧主图“大 yaw：目标角速度与实际角速度”同时显示黄色目标 `/gimbal/test/big_yaw_target_velocity.data` 和蓝色实际 `/chassis/big_yaw/velocity.data`，单位 rad/s，图例显示当前数值。当前测试 YAML 已启用大 yaw 滑模速度控制，实际输出仍由双下确认、摇杆回中和硬件联锁决定；Foxglove 只负责查看数据。

更新布局后，在 Foxglove 的 **Layouts → Import layout** 重新导入上述 JSON。布局在客户端保存，更新仓库或小电脑上的 JSON 不会自动修改已打开的布局。

远端 Bridge 3.4.1 使用 `foxglove.sdk.v1` 协议，请使用支持该协议的新版 Foxglove。

1. 在 Windows 上打开 Foxglove，选择 **Open connection → Foxglove WebSocket**，地址填写 `ws://localhost:8765`。先确保下述 SSH 隧道及 VS Code 端口转发已建立。
2. 在 **Layouts** 菜单中选择 **Import layout**，导入旁边的 JSON 文件。
3. 左上角独立显示 `/arm/joint_1/motor/angle.data` 的小 yaw / joint1 角度反馈，单位 rad，图例显示当前数值。旁边的角度图同时显示俯仰电机、joint1–joint6 和 IMU 角度；左侧其余面板显示角速度、原始编码器曲线，右侧显示遥控器、反馈在线状态和双下保护。曲线与状态时间线默认显示最近 20 秒。

若只在现有布局中加一条反馈曲线，添加 Plot 面板，将字段设为 `/arm/joint_1/motor/angle.data`。若角度为 NaN，检查 `/gimbal/feedback/small_yaw_online.data`；false 表示电机反馈已超时，此时曲线会空白。

测试和辨识配置都会随 RMCS 自动启动 Foxglove Bridge，Bridge 异常退出后由 launch 自动重启，小电脑重启后随 RMCS 服务恢复。在本地开发容器运行以下隧道；Windows 上的 Foxglove 需要在 VS Code 的 **Ports** 中将容器端口 `8765` 转发到本机。

```sh
ssh -p 2022 -N -L 127.0.0.1:8765:127.0.0.1:8765 -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 remote
```

数据由 RMCS 组件接口传递：硬件和控制器通过 `register_output` / `register_input` 配对，YAML 中的 `ValueBroadcaster` 以 200 Hz 统一转发给 Foxglove Bridge。无需在控制器中单独创建 ROS 发布器。

机械臂反馈使用 `0x9C` 状态查询。启用 `joint2_pid_enabled` 时，joint2 还接收 PID 的 `0xA1` 电流命令；joint3–joint6 仅查询反馈。总线映射为 joint1–joint3：CAN1 / `0x141`–`0x143`；joint4–joint6：CAN2 / `0x144`、`0x145`、`0x141`。CAN1 与 CAN2 的 `0x141` 分别是 joint1 和 joint6。角度按 LK 电机编码器、方向、减速比和配置零点换算为 rad；joint2 使用 42:1 减速比，joint2–joint6 零点沿用对应机械臂配置。俯仰电机仍为 CAN1 / `0x148`，仅查询反馈。小 yaw 在启用阻尼时还接收 `0xA1` 电流命令。

两份 YAML 均启用小 yaw 相对角度保持：双下确认后释放开关，右摇杆回中时捕获当时的 `/arm/joint_1/motor/angle` 作为目标；偏离后回正，停下后仍按角度误差维持力矩。控制为受限 PD，初始 `Kp=20 Nm/rad`、`Kd=2.5 Nm·s/rad`，上限 `2.5 Nm`，建立力矩变化率 `75 Nm/s`，减小幅值、撤力和反号不保留旧力矩；保持模式以 `500 Hz` 更新电流命令。D 项使用约 `10 ms` 的编码器速度，并发布到原阻尼速度话题；只在首个短窗尚未建立时使用原 `40 ms` 估速作为后备。位置硬死区 `0.0002 rad`。没有积分，持续外力下可能留下小偏差；参数尚需实机验证。双下、遥控超时、反馈失效或控制间隔超过 `20 ms` 时立即撤力并清目标；反馈或调度故障后需要重新双下确认，重新解锁捕获当前位置，避免拉回上次目标。大 yaw 辨识完成后保持仍继续。设置 `small_yaw_position_hold_enabled: false` 回到原反向制动，设置 `small_yaw_damping_enabled: false` 关闭全部小 yaw 力矩。旧 `small_yaw_damping_torque*` 话题在保持模式表示保持控制指令，回正时力矩可能与速度同向。独立力矩图范围 ±3 Nm，新增电机反馈电流折算力矩供对比；该值使用电机型号的名义力矩常数，并非力矩传感器测量。改变小 yaw 控制会改变对大 yaw 的反作用力，旧辨识参数需要按新配置重新验证。

| 显示内容 | 字段路径 | 单位或含义 |
| --- | --- | --- |
| 俯仰电机编码器 | `/arm/image_pitch/motor/raw_angle.data` | 原始计数 0–65535；-1 表示无有效反馈 |
| 俯仰电机角度、角速度 | `/arm/image_pitch/motor/angle.data`、`/arm/image_pitch/motor/velocity.data` | rad、rad/s；包含现有零点和方向换算 |
| 小 yaw 编码器 | `/arm/joint_1/motor/raw_angle.data` | joint1 原始计数 0–65535；-1 表示无有效反馈 |
| 小 yaw 角度、角速度 | `/arm/joint_1/motor/angle.data`、`/arm/joint_1/motor/velocity.data` | rad、rad/s；包含零点和反向换算 |
| 小 yaw 阻尼许可、出力状态 | `/gimbal/test/small_yaw_damping_permitted.data`、`/gimbal/test/small_yaw_damping_active.data` | true 表示许可、正在下发非零控制力矩 |
| 小 yaw 阻尼使用的编码器估速 | `/gimbal/test/small_yaw_damping_velocity.data` | rad/s；当前控制使用的编码器估速，保留原电机速度反馈供对比 |
| 小 yaw 控制请求、下发值 | `/gimbal/test/small_yaw_damping_torque.data`、`/gimbal/test/small_yaw_damping_torque_applied.data` | Nm；下发值为电流编码还原的软件命令；保持模式也可主动回正 |
| 小 yaw 保持目标与误差 | `/gimbal/test/small_yaw_hold_target_angle.data`、`/gimbal/test/small_yaw_hold_position_error.data` | rad；误差为目标减实际角度；无目标时目标为 NaN、误差归零 |
| 小 yaw 阻断原因 | `/gimbal/test/small_yaw_block_reason.data` | `none` 允许；其余值说明保护、反馈或等待重新双下等原因 |
| 小 yaw 保持目标有效 | `/gimbal/test/small_yaw_hold_reference_valid.data` | true 表示已捕获本次解锁目标 |
| 小 yaw 电机反馈电流、折算力矩 | `/arm/joint_1/motor/current.data`、`/arm/joint_1/motor/torque.data` | A、Nm；来自电机返回电流，采用同一反向约定 |
| 机械臂 joint2–joint6 角度 | `/arm/joint_2/motor/angle.data` 至 `/arm/joint_6/motor/angle.data` | rad；joint2–joint3 来自 CAN1，joint4–joint6 来自 CAN2；各自在线状态见 `/gimbal/feedback/joint_N_online.data` |
| joint2 关节编码器 | `/arm/joint_2/encoder/angle.data`、`/arm/joint_2/encoder/velocity.data`、`/arm/joint_2/encoder/raw_angle.data` | 独立 EncoderBoard（`D4-9C35-EDF1-2B20-7AE6-4ED0-8308`）CAN1 / `0x200` 的 KTH7823 反馈，PID 使用此关节角；在线状态见 `/gimbal/feedback/joint_2_encoder_online.data`；超时角度为 NaN、原始值为 -1 |
| joint2 抬升 PID | `/gimbal/test/joint2_pid_target_angle.data`、`/gimbal/test/joint2_pid_position_error.data`、`/gimbal/test/joint2_pid_torque.data`、`/gimbal/test/joint2_pid_torque_applied.data` | 双下确认、遥控回中且反馈有效后，从解锁位置抬升约 30°；双下或反馈失效立即发零力矩；力矩图显示请求值、量化下发命令和电机反馈电流折算值 |
| joint2 电机反馈电流、力矩估算 | `/arm/joint_2/motor/current.data`、`/arm/joint_2/motor/torque.data` | A、Nm；由 CAN1 / `0x142` 电机反馈电流按 MF7015V210T 标称力矩常数和 42:1 减速比折算，不是独立力矩传感器测量 |
| CAN1 电机 ID 扫描 | `/gimbal/feedback/can1_motor_id_scan.data` | 最近 1 秒内以标准 LK 状态应答的 CAN1 ID；低频 `0x9C` 只读查询用于核实小 yaw 是否使用了不同 ID |
| IMU 姿态 | `/roll_imu_angle.data`、`/pitch_imu_angle.data`、`/yaw_imu_angle.data` | rad |
| IMU 角速度 | `/roll_imu_velocity.data`、`/pitch_imu_velocity.data`、`/yaw_imu_velocity.data` | rad/s |
| 左右开关 | `/remote/switch/left.data`、`/remote/switch/right.data` | 0=未知/失联，1=上，2=下，3=中 |
| 遥控有效 | `/remote/online.data` | true 表示左右开关数据均有效 |
| 俯仰、小 yaw、IMU 在线 | `/gimbal/feedback/motor_online.data`、`/gimbal/feedback/small_yaw_online.data`、`/gimbal/feedback/imu_online.data` | true 表示对应反馈未超时 |
| 反馈就绪 | `/gimbal/test/feedback_ready.data` | 大 yaw 在线、角度有效且角度和速度为有限值 |
| 双下保护 | `/gimbal/test/double_down_protection.data` | true 表示双下保护中；仍继续采集反馈 |
| 双下确认 | `/gimbal/test/double_down_confirmed.data` | true 表示本次遥控连接后已执行过双下确认 |
| 测试许可 | `/gimbal/test/enabled.data` | 大 yaw 控制器的测试许可；硬件还会独立检查保护条件 |
| 测试状态 | `/gimbal/test/state.data` | 包括双下保护、等待摇杆回中、自动辨识与辨识完成 |
| 大 yaw 角度、速度、反馈力矩 | `/chassis/big_yaw/angle.data`、`/chassis/big_yaw/velocity.data`、`/chassis/big_yaw/torque.data` | rad、rad/s、Nm |
| 大 yaw 硬件阻断原因 | `/gimbal/test/big_yaw_block_reason.data` | `none` 表示无阻断；其余值说明硬件联锁、反馈或电机使能情况 |
| 大 yaw 跟踪故障、原点有效 | `/gimbal/test/big_yaw_position_tracking_fault.data`、`/gimbal/test/big_yaw_angle_origin_valid.data` | 跟踪故障时停止；双下、连续有效反馈且低速时清故障并重建原点，解除双下后才能出力 |
| 大 yaw 力矩命令与实际下发值 | `/gimbal/test/big_yaw_control_torque.data`、`/gimbal/test/big_yaw_torque_applied.data` | Nm；下发值是软件指令，电机反馈力矩见上一行 |
| Robust 速度目标与实际速度 | `/gimbal/test/big_yaw_target_velocity.data`、`/chassis/big_yaw/velocity.data` | rad/s；摇杆 X 目标限幅为 0.35 rad/s，起步限制目标加速度；回中或保护时目标立即清零，反拨从零建立反向目标 |
| Robust 力矩请求、影子值与下发值 | `/gimbal/test/robust_control_torque.data`、`/gimbal/test/robust_shadow_torque.data`、`/gimbal/test/big_yaw_torque_applied.data` | Nm；硬件侧 Robust 力矩限幅 3.2 Nm；停车或制动时立即撤去旧推进力，制动力和新方向力矩按 5 Nm/s 建立 |

右摇杆 X 回到 ±0.05 死区后，目标速度和目标加速度立即为零，算法停用摩擦、偏置和前馈补偿，只按实际速度产生反向制动力。硬件端独立约束回中时下发力矩与实际速度异号或为零；静止时输出零。反拨时先制动，接近静止后允许新方向起步补偿。停车耗时仍取决于实际惯量、摩擦和反馈，不能仅凭目标清零判断机构已经静止。

DM8009 的速度反馈为 12 位，静止附近仍可能报告 ±0.0061 或 ±0.0183 rad/s。`standstill_velocity_threshold: 0.02` 使非零速度目标下的算法将这段范围视为静止，避免测速符号导致一侧起步补偿被误判为反转制动。超过该范围的反转仍先制动；摇杆回中时仍按原始速度制动。

双下时应看到保护状态为 true、测试许可为 false，而有效反馈继续更新。遥控失联或开关无效后需要重新双下确认。传感器超时会将对应曲线置为 NaN（编码器为 -1），应结合在线标志判断反馈是否有效。joint2 力矩图里，PID 请求值和编码后下发值是软件侧数值，电机反馈曲线是根据 CAN 返回电流估算的轴端力矩；结合 `/gimbal/test/joint2_pid_permitted.data`、`/gimbal/test/joint2_pid_active.data` 和 `/gimbal/test/joint2_pid_block_reason.data` 判断 PID 是否获准运行。YAML 转发云台、joint1–joint6 反馈和状态；JSON 布局显示关节角与 joint2 力矩曲线，大 yaw 字段可按上表加入 Plot。

机械臂板的俯仰仅查询反馈，小 yaw 可执行上述相对角度保持或反向制动。硬件命令端会重复检查双下确认、摇杆回中、遥控超时和反馈有效性；反馈失效、跟踪故障或任一操作联锁不满足时保持关闭并发送零力矩。大 yaw 的展开角和相对角仅用于遥测，不再作为角度锁止条件。查看曲线不等于保存数据。持续录制与状态检查使用仓库的 [record-gimbal](../../../../.script/record-gimbal.md)，录制进程独立于控制器和 SSH 连接。

布局采用 Foxglove 官方示例使用的 `configById` / `layout` 导出格式。Plot 的数值图例及 State Transitions 的中文状态映射按[官方布局类型定义](https://github.com/foxglove/foxglove-sdk/blob/29b72bd2771befa0dd4613c4671d211fc2785cb3/python/foxglove-sdk/python/foxglove/layouts/__init__.py)核对；旧版 Studio 若未显示中文状态标签，仍可按面板中的原始值和上表读取。格式示例见[官方布局 JSON](https://github.com/foxglove/foxglove-sdk/blob/29b72bd2771befa0dd4613c4671d211fc2785cb3/python/foxglove-sdk-examples/so101-visualization/foxglove/lerobot_layout.json)。

大 yaw 位置跟踪丢失后，即使反馈恢复也保留故障锁存；需要双下、连续位置反馈和 `|velocity| <= 0.1 rad/s` 才能重建原点。双下期间继续发送关闭命令，解除双下并满足其它联锁后恢复许可。`big_yaw_block_reason` 中 `position_tracking_fault` 表示跟踪故障，`angle_origin_unavailable` 表示尚未建立原点，`motor_enabling` / `motor_fault_clearing` 表示电机使能/清错阶段，`test_disabled` 表示控制器未请求运行（包括辨识结束），`none` 表示已允许正常力矩路径。


`joint2_lift_test.yaml` 用于单独验证 joint2 抬升：关闭大 yaw 自动辨识与速度控制，关闭小 yaw 出力，保留反馈与 Foxglove。手动抬升验证表明 joint2 编码器角度减小，故 `joint2_lift_direction: -1.0`，解锁后目标为当时角度减 30°，目标受关节限位夹紧，双下等联锁继续生效。按操作要求，力矩上限为 24 Nm，Kp 为 48 Nm/rad，增力变化率为 48 Nm/s，内部位置参考变化速度为 1 rad/s（并非实际关节速度限幅）。静止且尚未抬动时，软件输出约 0.5 秒达到上限；24 Nm 是闭环输出上限，运动后输出随误差和速度变化。发送端硬限幅同步为 24 Nm。joint2 力矩图显示范围为 ±26 Nm，现有 Foxglove 布局需要重新导入 JSON 才会应用此范围。

24 Nm 试验已确认能自行起升，但初版先停在约 71.8°，距目标约 52.5° 仍差 19.3°；后半段接近目标的运动经操作员确认有手动辅助。Ki=4、积分容量16 Nm的后续记录显示分段抬升，最终积分达到上限时仍差约3.3°。Ki=8、积分容量20 Nm、Kd=3 的两段记录中，许可持续有效，均出现约6°超调及随后数度回落；未发现到位清积分。该过程符合静摩擦松脱后积存力矩继续推动、随后积分缓慢卸载的表现，不能用另一次双下后的大幅下落解释。

当前本地独立抬升配置保留 Ki=8 Nm/(rad·s)、积分状态上限2.5 rad·s、24 Nm 总上限，将 Kd 从3改为10 Nm·s/rad，并新增 `joint2_pid_torque_release_nm_s: 240.0`：同号减力按240 Nm/s，反向请求先减至0、下一次更新才以48 Nm/s建立反向力矩。未配置或设为0时保留原对称斜率行为；显式减力斜率必须不小于增力斜率。正常到位保留积分支撑，双下、失去许可或反馈异常仍立即清积分和输出。合成静动摩擦模型覆盖真实20 ms编码器估速、额外0/20 ms延迟和5 ms命令保持，最终静差约0.09–0.53°；这些参数不是辨识得到的机械臂模型，不能据此承诺实机平顺或精确到位。此版本尚待部署和无手扶实机验证。
