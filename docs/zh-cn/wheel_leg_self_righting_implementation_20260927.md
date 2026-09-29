# 24 V 倒地自起的 RMCS 接入状态

当前为 **RMCS 代码实现和离线验证**；尚未拿到实车四轴 L1→模型零位/方向、DM 反馈端连续角语义、全行程闭链实测表、壳体几何/气簧力曲线及 IMU/支撑探测误差。RL 配置的 `recovery_enabled`、`recovery_profile_ready`、`calibration_ready`、`soft_limits_ready`、`imu_alignment_ready` 均保持 `false`。不将仿真数据填作实车标定，也不重新给已在 **105°** 姿态内部设零的四台 DM 发送设零命令。

## 已接入的控制链

- `rmcs_rl/src/recovery_controller.{hpp,cpp}` 是不访问 CAN、ROS/ONNX 的 200 Hz 纯状态机：正立等待着地、收腿、放支点、侧摆、定向回转、蹬伸、捕获、准备、200 ms 模型侧力矩混合、有界失败；支持一次 PREPARE 倒扣重选路、同侧两主动轴共享圈数、异常观测/8 s 超时退出。24 V 速度搜索的较快候选为 `orbit/side=5`、`rollover=5.25`、`capture=4 rad/s`，**只作待标定参考**。
- `rmcs_rl/src/recovery_observer.{hpp,cpp}` 以**实际标定**的两侧闭链单调 LUT、髋轴/轮心位置、壳体轮廓点、气簧力曲线和现有编码器、BMI088 比力/姿态，推条件轮着地高度、真实内膝角/余量、轮高差和气簧补偿。双轮反向小力矩探测、静置和驻留后才提出候选支撑；高度不是独立测距，支撑仍须在实体地面/质量/摩擦工况下验证。LUT 越界或缺失立即无效，不能由 IMU 单独宣布支撑。
- `rmcs_rl/src/rl_controller.*` 的 `PREPARE` 保留六轴力矩唯一拥有权：恢复脚本和 50 Hz ONNX 影子推理分别维护目标，恢复期间影子结果不更新上一动作；接管时按 200 ms 权重混合**模型侧力矩**，混合后检查机械余量、24 V 的条件 DM 扭矩—速度包络和**实测允许的累计高于 20 Nm 时长**，再经 `Jᵀ` 与设备最大力矩输出。输出含 `/wheel_leg/rl/recovery/phase`、`failure`、`support_confirmed` 和 `geometry_valid`。恢复完成后的 RL 继续使用最新闭链表限制膝内角，失姿/反馈异常退出。命令轮速仍为**轮轴 rad/s**；`wheel_model_scale` 默认为 1，不在策略侧再次乘减速比。
- `WheelLeg` 用既有 CAN 帧时间/序号接口；另发布 BMI088 比力（m/s²）和接收时间、`dm_control_ready`。RL 的 `/wheel_leg/enable_request` 与双拨杆 MIDDLE、新反馈及设备状态共同决定驱动使能；清错/使能帧阶段保持力矩零、恢复参考和时钟不推进。RL 图 `require_enable_request: true`，已设零电机的 `allow_set_zero: false`。M3508 继续在**设备配置**中按参考分支 `.set_reduction_ratio(15.8)` 一次换算反馈/电流；不动 ONNX 轮输出含义。
- 自起请求只在当前是 `AUTO` 模式、启动时速度／转向摇杆已回中时进入；已知 `STEP_DOWN`、`LAUNCH_RAMP` 等非 AUTO 模式直接锁存 `kUnsafeRequest` 并撤销使能。这只是拒绝**已知**非平地意图，没有前探传感器，不能证明脚下实际平整。恢复进行时即使操作员移动摇杆，`BLEND` 的策略观测仍是零速／0.305 m；进入 RL 后还需**至少 1 s 且连续 1 s** 条件站高、两轮几何、倾角/角速、壳体离地与关节低速稳定证据，才允许非零运动／跳跃请求；证据中断则重新计时。输出 `/wheel_leg/rl/recovery/motion_hold` 标明当前是否仍在屏蔽运动指令；未启用恢复的普通 RL 不受保持期影响。此处的 `body_clear` 来自已校准壳体几何与条件轮接地假设，并非直接测得的接触力。

### 双下失能 → 双中使能的实际命令链

遥控器从任意状态进入**双下**，底盘组件清除 `armed`、递增复位计数、发布 `control_state=1`，RL 清六轴力矩并撤销 `/wheel_leg/enable_request`。`WheelLeg` 独立依据**新鲜 DR16 状态**把四台 DM 的本次系统帧置为 `0xFD` 失能，连续重发 100 周期并继续周期重发；两台 M3508 下发零电流，不存在 DM 式使能指令。即使先前没有使能请求，双下边沿也触发新一轮 `0xFD`。诊断 `tx_kind=2`、`last_system_cmd=0xFD` 可查实际已排队帧。

必须**先双下，再两侧都到中位**（允许一侧先到中位，但中间态仍保持失能），才能发布 `control_state=3` 请求恢复。双中也不会绕过标定门禁：RL 检查模型/IMU/软限位/恢复配置、AUTO 模式和回中摇杆后才发布 `enable_request=true`。在 DR16/六轴/IMU 反馈仍新鲜的前提下，`WheelLeg` 对四台 DM **直接发第一帧 `0xFB` 清错，下一发送周期发 `0xFC` 使能**；不再要求双中前先收齐四台 `status=0`。首次 `0xFC` 后另发**一帧零增益、零力矩 MIT**，清除前一会话可能遗留的电机内位置目标，然后继续重发 `0xFC`；这帧不会宣布 `dm_control_ready`，也不携带脚本力矩。若反馈仍报故障，则以 10 Hz 重试 `0xFB` 而**不向故障轴发 `0xFC`**；已清错但仍未报使能的轴以 10 Hz 重试 `0xFC`。仅当两侧 DM 均回报 `status=1`、无故障且已实际发送首帧正式 MIT 时 `dm_control_ready=true`；此前四台 DM 的 PC 侧目标力矩为零、两台轮电机仍发零电流、恢复状态机不推进。任一侧退出中位／反馈超时会重新发 `0xFD`，自起失败会锁存故障，须再次双下复位。**不会发送 `0xFE` 设零**（四轴已在 105° 姿态设零）。该软件时序的实车 CAN 到达/状态回复仍待台架回执。

## 使能前必须提供的数据

### 机体坐标：原 CAD 的 90°不能在运行时重复做

用户确认**实体底盘及 BMI088 安装均为车体 +X 向前的右手系**。原始 CAD URDF 是**−Y 向前、+X 向左、+Z 向上**；冻结的 `model/纯底盘_v5_232mm/urdf/robot.urdf` 已在导出时对根部 visual/collision/inertial 和髋关节位置左乘 `Rz(+90°)`。因此 `−Y_CAD → +X_policy`、`+X_CAD → +Y_policy`；冻结 manifest 的 `control_frame` 是 `Xforward_Yleft_Zup`，与实车控制系**名义一致**。该旋转已进入资产，既不再乘到 RMCS 机身速度命令／IMU 上，也不把六维关节动作像三维向量那样旋转。

`/wheel_leg/imu/*` 来自 `Bmi088Ekf` 的 body 输出；默认 `body_to_sensor=I`。当实测传感器 +X 前、+Y 左、+Z 上及其 quaternion/gyro 符号一致时，`imu_to_base=I` 才是这个**冻结策略坐标系**的候选值。`imu_to_base` 只补偿真实 IMU 安装外参，不补偿源 CAD→已冻结策略资产的导出旋转。核验应包括水平静置 `g_B≈(0,0,-1)`、车头抬起/压下时 `g_B.x` 的正负与模型一致、侧倾时 `g_B.y` 的正负与模型一致、正 yaw 为 `+Z`，以及实际向前滚动是否对应策略的 `command/forward>0`；在角速度和轮转方向标定前，仍保持 `imu_alignment_ready=false`、`calibration_ready=false` 和恢复门禁关闭。腿／轮动作反映的是模型关节与轮轴目标，实际符号通过各轴 `leg_motor_to_model`、`wheel_model_scale` 和设备反馈方向实测，不套 `Rz(+90°)`。

在 `wheel-leg-infantry-rl.yaml` 中显式提供已核验的 `recovery_dm_feedback_position_max` 四值、`recovery_above_rated_budget_s`（实测允许高于额定 20 Nm 的累计时长；耗尽即裁剪到额定值，不作为精确热模型），以及 `recovery_orbit_speed`、`recovery_side_speed`、`recovery_rollover_speed`、`recovery_capture_speed` 和按策略 P 顺序的 **8 组**四主动轴模型参考：`recovery_fold_p4`、`recovery_thrust_p4`、`recovery_side_extended_p4`、`recovery_stand_p4`（0.305 m、Kp120）、`recovery_upright_p4`（0.305 m、Kp80）、`recovery_support_extended_p4`（0.40 m、Kp120）、`recovery_upright_support_extended_p4`（0.40 m、Kp80）、`recovery_capture_extended_p4`（0.35 m、Kp120）。中倾角的普通准备目标为已认证 ONNX 的 actor nominal，不把伸腿支撑姿态当接管准备目标。四轴位置必须由**实际电机反馈 API 端**与模型坐标标定，不能仅由内膝 **105° 设零**与电机内部读数 0 推定。恢复代码不把 MIT `P_MAX` 猜作反馈回绕周期；若运动时角度跳变/触及反馈量化端点，则锁定故障。

两侧各填 `recovery_{left,right}_delta_rad`、`_inner_knee_deg`、`_slider_m`、`_wheel_at_hip_zero_m`（每个关节点 3 个 XYZ 值）、`_hip_origin_m`、`_hip_axis`、`_spring_compression_at_zero_m`；另填 `recovery_shell_points_body_m`（壳体最危险接地点 XYZ 扁平数组，至少 4 点）、`recovery_spring_stroke_m`、`recovery_spring_force_n` 四项多项式。LUT 必须单调覆盖工作域并校核膝 40–110° 和气簧行程；缺失或超界会拒绝动作。`recovery_profile_ready` 只有实测曲线、轴方向/反馈端、误差/时间标定验收后才设 `true`。现有默认 `false` 配置下不会启动自起。

## 本地验证和能力边界

恢复后解锁遥控运动的判定采用 **RL 接管后至少 1 s 的实际经过时间**及 **连续 1 s 的条件站立证据**双门禁；它只把摇杆命令留在零，不是支撑探测的替代品，也不能消除翻起瞬间的壳体冲击。代码仍缺少把 **RMCS C++ 控制器**接进同一 Isaac Sim 机械资产的逐案闭环验证。

在开发容器内使用**仓库脚本**而非临时 Docker 构建命令：

```bash
.script/build-rmcs --packages-select rmcs_msgs rmcs_core rmcs_rl --parallel-workers 2
source /opt/ros/jazzy/setup.bash
colcon test --merge-install --packages-select rmcs_rl rmcs_core
colcon test-result --verbose
```

纯 C++ 恢复单测覆盖未知支撑拒绝接管、200 ms 混合、整圈同侧配对、无接触倒扣有界退出、条件闭链/双向轮探测、空中等待，以及平地 AUTO／摇杆回中请求和 1 s 策略运动保持边界；它们不是 Isaac Sim 里**用 C++ 控制器替换 Python 后的闭环回归**。24 V Python 脚本速度档完整 50 案见相邻训练仓库 `docs/V5_MOTOR_ENVELOPE_RECOVERY_TIMING_20260927.md`；旧的合成电机 110° 零点下，较快 5/5/5.25/4 档前／后倒扣进入纯 RL 的中位数约 4.74／4.81 s，但仍有额定转速超界。用户已纠正内部设零机械姿态为 **105°**；据此新增的 Isaac Sim 14 姿态×5 次（额外覆盖四种前后劈叉镜像姿态）记录在 `reports/v5_24v_split_zero105_full14x5_20260927/`：原 10 类 **45/50**、前后劈叉 **15/20**，其中“后倒＋右腿前伸”只有 **1/5** 末尾严格通过。**这说明当前脚本不能可靠处理劈叉倒地，更不能把快视频或旧零点下的时间当实机耗时。**后续台阶录像中高低轮、壳体搭台阶和台阶顶趴地未通过严格回归；不能把 AUTO／中位摇杆门禁当作可靠台阶识别。下阶段需先按实测表生成配置，再做同工况逐 200 Hz C++/Python 对照、真实倒地构型回归及驱动时序验收；当前恢复门禁继续关闭。
