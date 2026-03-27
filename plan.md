# sp_vision_25 移植到 deformable-infantry 计划

## 目标

在 `deformable-infantry` 分支上完成 `sp_vision_25` 子模块接入与稳定运行。

迁移原则：

- 以 `sp_vision_25-omni` 的能力和结构作为功能参考
- 以 `dev/sp-vision` 分支中的落地方式作为 RMCS 侧接入参考
- 优先复用当前 `deformable-infantry` 分支里已经存在的 `sp_vision_25` 接口约定
- 先打通链路，再做参数、外参、控制器增益和时序调优

---

## 现状判断

当前仓库并不是“从零开始接入 sp_vision_25”，而是已经做了相当一部分移植，核心结论如下：

1. `deformable-infantry` 分支已经有 `sp_vision_25` 桥接组件接入
2. `deformable-infantry` 分支已经有 deformable 专用云台控制器
3. `deformable-infantry` 的 bringup YAML 已经包含视觉桥、云台控制、射击控制等链路
4. 当前更像是“确认迁移基线并补齐剩余差异”，而不是“第一次集成”

因此，这个任务的重点应从“接入 `sp_vision_25`”调整为：

- 确认采用哪套 `sp_vision_25` 输出契约
- 对齐 `deformable-infantry` 的 RMCS 接口
- 完成 deformable 专用参数和时序调优
- 做启动、接口和闭环验证

---

## 关键结论

### 推荐基线

推荐继续使用当前 `sp_vision_25/main` 这条桥接方式，而不是切到 `sp_vision_25/dev/omni` 风格。

原因：

- 当前 RMCS `deformable-infantry` 已经围绕 planner 输出做了对接
- `dev/sp-vision` 参考实现也是 planner 输出风格
- deformable 专用云台控制器已经消费了这套输出
- 射击控制已经消费 `/gimbal/auto_aim/fire_control`

### 当前兼容的视觉输出契约

现有 `sp_vision_25` 桥大致输出：

- `/gimbal/auto_aim/control_direction`
- `/gimbal/auto_aim/fire_control`
- `/gimbal/auto_aim/laser_distance`
- `/gimbal/auto_aim/plan_yaw`
- `/gimbal/auto_aim/plan_pitch`
- `/gimbal/auto_aim/plan_yaw_velocity`
- `/gimbal/auto_aim/plan_pitch_velocity`
- `/gimbal/auto_aim/plan_yaw_acceleration`
- `/gimbal/auto_aim/plan_pitch_acceleration`

这套接口已经能被 deformable 侧消费。

### 不推荐直接切换到 dev/omni 契约

`sp_vision_25/dev/omni` 更偏向输出 `target_snapshot`，而不是直接输出 planner 结果。
如果走这条路，需要 RMCS 侧新增：

- 新消息类型
- 新控制器消费逻辑
- 新 planner / fire decision 迁移
- 新的类型与 YAML 接口对齐

这不是最短路径，应作为后续重构方向，而不是本次移植的首选路径。

---

## 迁移总策略

分四个阶段推进：

1. 锁定接口契约
2. 校验并整理 RMCS 链路
3. 补齐 deformable 专用配置与调参
4. 做启动验证、数据流验证和闭环验证

---

## 阶段一：锁定接口契约

### 目标

明确本次移植采用的 `sp_vision_25` 输出模型，避免后面反复返工。

### 决策

采用当前 planner-output 模型，不切换到 `target_snapshot` 模型。

### 需要确认的点

- `sp_vision_25` 是否继续输出 planner 结果而不是 target snapshot
- `deformable_infantry_gimbal_controller` 是否完整消费这些 planner 输出
- `bullet_feeder_controller_17mm` 是否继续消费 `fire_control`

### 重点检查文件

- `rmcs_ws/src/sp_vision_25/src/sp_vision_bridge.cpp`
- `rmcs_ws/src/sp_vision_25/plugins.xml`
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/deformable_infantry_gimbal_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/shooting/bullet_feeder_controller_17mm.cpp`

### 完成标准

- 明确所有视觉输出接口名与 C++ 类型
- 明确所有下游消费者及其依赖关系
- 不再考虑切换到另一套输出契约

### 第一阶段检查结果

当前阶段已经完成接口契约核对，结论如下：

1. 当前 `sp_vision_25` 子模块 HEAD 为 `b84f3a4`，与本地 `main` 一致；本地 `dev/omni` 为 `285fa0c`。
2. 当前接入到 RMCS 的桥接实现是 `rmcs_ws/src/sp_vision_25/src/sp_vision_bridge.cpp` 中的 planner-output 模型，而不是 `target_snapshot` 模型。
3. 当前 `SpVisionBridge` 已确认输出以下接口：
   - `/gimbal/auto_aim/control_direction`：`Eigen::Vector3d`
   - `/gimbal/auto_aim/fire_control`：`bool`
   - `/gimbal/auto_aim/laser_distance`：`double`
   - `/gimbal/auto_aim/plan_yaw`：`double`
   - `/gimbal/auto_aim/plan_pitch`：`double`
   - `/gimbal/auto_aim/plan_yaw_velocity`：`double`
   - `/gimbal/auto_aim/plan_yaw_acceleration`：`double`
   - `/gimbal/auto_aim/plan_pitch_velocity`：`double`
   - `/gimbal/auto_aim/plan_pitch_acceleration`：`double`
4. 当前 `SpVisionBridge` 已确认输入以下接口：
   - `/gimbal/hard_sync_snapshot`：`rmcs_msgs::HardSyncSnapshot`
   - `/predefined/timestamp`：`std::chrono::steady_clock::time_point`
   - `/referee/shooter/initial_speed`：`float`，可选，未接入时回退到 `bullet_speed_fallback`
5. `DeformableInfantryGimbalController` 已完整消费 `control_direction + plan_*` 全套 planner 输出，并在 `before_updating()` 中为全部可选输入绑定安全默认值。
6. `BulletFeederController17mm` 已消费 `/gimbal/auto_aim/fire_control`，且在视觉未接入时默认绑定 `false`，不会误触发开火。
7. `deformable-infantry.yaml` 当前已经引用：
   - `sp_vision_25::bridge::SpVisionBridge -> sp_vision_bridge`
   - `rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller`
   - `rmcs_core::controller::shooting::BulletFeederController17mm -> bullet_feeder_controller`
8. `dev/omni` 分支里的 `SpVisionBridge` 依赖 `rmcs_msgs/target_snapshot.hpp`，而当前 RMCS 工作树中不存在该头文件，因此如果直接切换到该契约，至少需要补一轮 `rmcs_msgs + controller + bringup` 级别的改造。

阶段一结论：

- 当前分支应锁定 planner-output 契约
- 不应在本轮移植中切换到 `target_snapshot` 契约
- 下一阶段应进入 DAG 接线与输入输出链路完整性核对

---

## 阶段二：校验 RMCS 链路完整性

### 目标

确保 `deformable-infantry` 的执行图完整，`sp_vision_25` 可以在 DAG 中成功接线。

### 重点检查链路

#### 视觉输入侧

需要确认以下输入是否都存在：

- `/gimbal/hard_sync_snapshot`
- `/predefined/timestamp`
- `/referee/shooter/initial_speed`（可选）
- 必要时与 `/tf` 的时序关系

#### 视觉输出侧

需要确认以下输出是否都有消费者：

- `/gimbal/auto_aim/control_direction`
- `/gimbal/auto_aim/fire_control`
- `/gimbal/auto_aim/plan_yaw`
- `/gimbal/auto_aim/plan_pitch`
- `/gimbal/auto_aim/plan_yaw_velocity`
- `/gimbal/auto_aim/plan_pitch_velocity`
- `/gimbal/auto_aim/plan_yaw_acceleration`
- `/gimbal/auto_aim/plan_pitch_acceleration`

### 重点检查文件

#### bringup
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml`
- `rmcs_ws/src/rmcs_bringup/config/omni-infantry.yaml`

#### hardware / referee
- `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry.cpp`
- `rmcs_ws/src/rmcs_core/src/referee/status.cpp`

#### controller
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/deformable_infantry_gimbal_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/shooting/bullet_feeder_controller_17mm.cpp`

### 关键检查项

- 接口名是否完全一致
- C++ 类型是否完全一致
- 是否存在必需输入未接上
- 是否存在可选输入未设置默认值
- 是否存在老接口名残留
- 是否存在 YAML 中组件实例名或 plugin 名不一致

### 完成标准

- `deformable-infantry.yaml` 可以形成完整 DAG
- 所有关键视觉接口都能成功接线
- 启动阶段不会因为接口缺失、重名、类型不匹配而失败

### 第二阶段检查结果

当前阶段已经完成静态 DAG 与关键接口链路核对，结论如下：

1. `deformable-infantry.yaml` 中视觉主链已正确声明：
   - `sp_vision_25::bridge::SpVisionBridge -> sp_vision_bridge`
   - `rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller`
   - `rmcs_core::controller::shooting::BulletFeederController17mm -> bullet_feeder_controller`
2. `sp_vision_bridge` 的必需输入链路已确认完整：
   - `/gimbal/hard_sync_snapshot` 由 `rmcs_core::hardware::DeformableInfantry` 提供
   - `/predefined/timestamp` 由 executor 内置 `PredefinedMsgProvider` 提供
   - `/referee/shooter/initial_speed` 由 `rmcs_core::referee::Status` 提供，且在 bridge 中是可选输入
3. `deformable gimbal` 的视觉消费链路已确认完整：
   - `/gimbal/auto_aim/control_direction`
   - `/gimbal/auto_aim/plan_yaw`
   - `/gimbal/auto_aim/plan_pitch`
   - `/gimbal/auto_aim/plan_yaw_velocity`
   - `/gimbal/auto_aim/plan_yaw_acceleration`
   - `/gimbal/auto_aim/plan_pitch_velocity`
   - `/gimbal/auto_aim/plan_pitch_acceleration`
4. `bullet_feeder` 的视觉消费链路已确认完整：
   - `/gimbal/auto_aim/fire_control` 由 `BulletFeederController17mm` 消费
   - `/gimbal/friction_ready` 与 `/gimbal/bullet_fired` 由 `FrictionWheelController` 提供
   - `/gimbal/control_bullet_allowance/limited_by_heat` 由 `HeatController` 提供
5. deformable 硬件对控制量的消费链路已确认完整：
   - `/gimbal/yaw/control_torque` 由 yaw 电机命令输入消费
   - `/gimbal/pitch/control_velocity` 由 pitch 电机命令输入消费
   - `/gimbal/bullet_feeder/control_velocity` 由 `PidController` 转换为 `/gimbal/bullet_feeder/control_torque` 后再由硬件消费
6. deformable 控制器依赖的非视觉输入链也已确认存在：
   - `/remote/joystick/left`、`/remote/switch/*`、`/remote/mouse*`、`/remote/keyboard`、`/remote/rotary_knob` 由 `device::Dr16` 提供
   - `/gimbal/yaw/velocity`、`/gimbal/pitch/velocity`、`/gimbal/bullet_feeder/velocity` 由电机状态输出提供
   - `/gimbal/yaw/velocity_imu`、`/gimbal/pitch/velocity_imu` 由 deformable 顶板 BMI088 输出提供
7. 当前静态检查中未发现视觉相关接口名冲突、缺失必需输入或明显类型不匹配。

当前阶段发现的非阻塞问题：

- `deformable-infantry.yaml` 中 `sp_vision_bridge` 仍保留了 `virtual_*`、`fire_control_enabled`、`output_csv_path`、`csv_decimation` 等更像 `SpVisionResponseTestBridge` 的参数；当前 `SpVisionBridge` 实现并不会使用这些参数。
- `/gimbal/auto_aim/laser_distance` 在 deformable 链路中当前没有活跃消费者，但这不会阻塞 DAG。
- 容器内可以使用build-rmcs 脚本进行构建

阶段二结论：

- 从静态接线角度看，`deformable-infantry` 已具备完整视觉 DAG
- 当前阻塞点不在接口配线，而在后续的配置正确性、时序质量与实机调参
- 下一阶段应转向 deformable 专用视觉配置、外参与控制参数核对

---

## 阶段三：补齐 deformable 专用配置与调参

### 目标

在链路已经打通的基础上，让 deformable 机器人上的视觉与控制行为真正可用。

### 重点不是代码接线，而是参数与物理模型一致性

### 主要风险点

#### 1. 相机内外参与安装位姿不匹配

当前 `sp_vision_25` 使用的配置可能仍接近 omni 的相机设置。
需要重点核对：

- `camera_matrix`
- `distort_coeffs`
- `R_camera2gimbal`
- `t_camera2gimbal`
- `R_gimbal2imubody`
- 相机型号与曝光参数
- 配置文件路径是否正确指向 deformable 用的 YAML

#### 2. deformable 云台动力学与 omni 不同

当前 deformable 控制器的控制方式与硬件结构与 omni 不完全相同，尤其要注意：

- yaw 侧是否按 planner 扭矩前馈工作
- pitch 侧是否是速度控制而非完全对称的扭矩控制
- planner 的速度/加速度前馈是否与当前控制器设计匹配

#### 3. bullet speed 来源可能掩盖问题

`sp_vision_25` 可能在拿不到裁判系统弹速时回退到 fallback。
这会导致“能运行但不准确”。

需要确认：

- `/referee/shooter/initial_speed` 是否稳定存在
- fallback 只作为兜底，不应长期依赖

#### 4. hard sync 时间对齐问题

视觉规划依赖：

- 图像时间戳
- 下位机姿态
- `hard_sync_snapshot`

如果时间不同步，会出现：

- 目标方向漂移
- planner 导数异常
- 命中率下降
- 自动开火时机不稳定

### 重点检查文件

- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml`
- `rmcs_ws/src/sp_vision_25/configs/deformable-infantry.yaml`
- `rmcs_ws/src/sp_vision_25/configs/standard4.yaml`
- `rmcs_ws/src/sp_vision_25/configs/standard3.yaml`
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/deformable_infantry_gimbal_controller.cpp`

### 完成标准

- deformable 使用专属视觉配置
- 外参、弹速、曝光、控制器参数与实机一致
- planner 前馈不会导致明显振荡或滞后
- 自动瞄准和开火逻辑达到可调试状态

### 第三阶段执行结果

当前阶段已经完成一轮“可安全落地”的配置整理，结果如下：

1. 已新增 `rmcs_ws/src/sp_vision_25/configs/deformable-infantry.yaml`，以当前 `standard4.yaml` 的有效参数为基线，单独承载 deformable 的视觉配置。
2. 已将 `rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml` 的 `sp_vision_bridge.config_file` 切换到 `configs/deformable-infantry.yaml`，避免后续 deformable 与其他机器人共享同一份通用配置文件。
3. 已删除 `deformable-infantry.yaml` 中 `virtual_*`、`fire_control_enabled`、`output_csv_path`、`csv_decimation` 等仅对 `SpVisionResponseTestBridge` 有意义、但对当前 `SpVisionBridge` 无效的参数，减少误导。
4. 已显式补出 `manual_joystick_sensitivity` 与 `manual_mouse_sensitivity`，使 deformable 当前手动操控灵敏度不再依赖控制器内默认值。
5. 已删除 `gimbal_controller` 中未被 `DeformableInfantryGimbalController` 实际读取的 `pitch_velocity_*` 与 `pitch_acc_ff_gain` 参数，并保留注释说明“deformable pitch 当前仍由硬件层做速度控制”。
6. 已确认 `standard3.yaml` 与原 `standard4.yaml` 的主要差异集中在三类：
   - 推理设备：`GPU` vs `CPU`
   - 视觉零偏：`yaw_offset` / `pitch_offset`
   - 机械外参：`t_camera2gimbal`
   这说明当前 deformable 使用的 `standard4` 基线本身并不是简单沿用 omni 的配置。

当前阶段仍未完成、且必须依赖实机/容器环境的工作：

- 校验 `deformable-infantry.yaml` 中 `bullet_speed_fallback: 28.0` 与裁判系统真实弹速是否一致
- 校验 `deformable-infantry.yaml` 中 yaw/pitch 控制参数是否适配当前硬件极限与安装位姿
- 校验 `deformable-infantry.yaml` 中上下限 `upper_limit` / `lower_limit` 是否覆盖完整可用俯仰范围
- 在 devcontainer / ROS 环境中补做 build 与 launch 验证；当前宿主机缺少 `colcon` 与 ROS 安装，只完成了 YAML 语法级校验

阶段三结论：

- deformable 现在已经拥有独立的视觉配置入口，后续外参和规划参数可以单独演进
- 当前最值得继续推进的是实机参数核对，而不是继续做接口层改造
- 下一阶段应转入编译、启动、数据流与闭环联调验证

---

## 阶段四：验证与联调

### 目标

验证移植结果不仅能编译和启动，而且在运行时具备正确的数据流与控制行为。

### 验证分层

#### 1. 启动验证

确认：

- `robot:=deformable-infantry` 能正常启动
- executor 不报接口配线错误
- `sp_vision_bridge` 成功创建
- deformable gimbal controller 成功创建

#### 2. 数据流验证

重点检查以下链路：

- `/gimbal/hard_sync_snapshot` -> `sp_vision_bridge`
- `/referee/shooter/initial_speed` -> `sp_vision_bridge`
- `sp_vision_bridge` -> `/gimbal/auto_aim/*`
- `/gimbal/auto_aim/*` -> `deformable_infantry_gimbal_controller`
- `/gimbal/auto_aim/fire_control` -> `bullet_feeder_controller_17mm`

#### 3. 失效保护验证
确认 vision 数据过期时：

- `control_direction` 清零
- planner 导数清零
- `fire_control = false`

#### 4. 闭环行为验证

确认：

- 手动模式下行为不被破坏
- 开启自瞄后云台能正确跟随 planner
- 仅在满足射击条件时才开火
- friction ready / heat allowance / fire control 三者逻辑正常

#### 5. 实机调参验证

确认：

- yaw 跟踪稳定
- pitch 跟踪稳定
- 打击近距离 / 中距离 / 远距离时没有明显系统性偏差
- 连续目标切换时不会出现严重跳变

### 完成标准

- 系统可稳定启动
- 自动瞄准链路完整
- 自动开火链路完整
- 主要问题收敛到参数调优层，而不是接口层或架构层

---

## 文件级实施清单

### RMCS bringup
- `rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml`
- `rmcs_ws/src/rmcs_bringup/config/omni-infantry.yaml`

用途：

- 对照 omni 的视觉接入方式
- 校验 deformable DAG 是否正确引用视觉桥与 deformable gimbal controller
- 核对视觉配置文件路径与组件参数

### RMCS core
- `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry.cpp`
- `rmcs_ws/src/rmcs_core/src/referee/status.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/gimbal/deformable_infantry_gimbal_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/shooting/bullet_feeder_controller_17mm.cpp`

用途：

- 确认视觉输入生产者是否正确
- 确认视觉输出消费者是否正确
- 确认 deformable 控制器对 planner 输出的消费逻辑是否完整

### sp_vision_25
- `rmcs_ws/src/sp_vision_25/src/sp_vision_bridge.cpp`
- `rmcs_ws/src/sp_vision_25/plugins.xml`
- `rmcs_ws/src/sp_vision_25/configs/deformable-infantry.yaml`
- `rmcs_ws/src/sp_vision_25/configs/standard4.yaml`
- `rmcs_ws/src/sp_vision_25/configs/standard3.yaml`

用途：

- 确认 bridge 的输入输出接口
- 确认 deformable 使用的视觉配置是否专用且合理
- 明确当前集成采用的是 planner-output 契约

---

## 本次移植的推荐执行顺序

1. 先确认 `sp_vision_25` 输出契约，锁定采用 planner-output 模型
2. 对照 `dev/sp-vision` 与当前 `deformable-infantry`，确认 bringup 链路不回退
3. 校验 `deformable-infantry.yaml` 中所有视觉相关组件与接口
4. 校验 hardware / referee / gimbal / shooter 的接口完整性
5. 整理 deformable 专用视觉配置文件
6. 调整云台控制器与 planner 前馈相关参数
7. 做启动验证
8. 做数据流验证
9. 做失效保护验证
10. 做实机联调与参数收敛

---

## 风险清单

### 高风险
- 误切到 `target_snapshot` 契约，导致 RMCS 侧消费者全部失配
- 相机外参与 deformable 实际安装不一致
- hard sync 时序不对，造成自瞄不稳定
- pitch 控制模式与 planner 假设不一致

### 中风险
- 弹速长期走 fallback，导致预测偏差
- YAML 参数名沿用 omni 旧值，出现“能跑但不准”
- fire_control 接线正确但阈值过严/过松，导致不开火或乱开火

### 低风险
- UI / debug 信息不一致
- `laser_distance` 暂无消费者
- 局部日志或配置命名不统一

---

## 最终交付标准

满足以下条件视为本轮移植完成：

1. `deformable-infantry` 启动链路完整
2. `sp_vision_25` 能稳定接收姿态、时间戳和弹速
3. `sp_vision_25` 能稳定输出 planner 与 fire_control
4. deformable 专用 gimbal controller 能正确消费 planner 输出
5. bullet feeder 能正确消费 fire_control
6. vision 超时保护生效
7. 实机上自动瞄准与自动开火达到可调试、可继续优化的状态
