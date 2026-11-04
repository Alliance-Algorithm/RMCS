# 轮腿实车状态三维镜像遥测

`rmcs_core::broadcaster::WheelLegTelemetry` 将轮腿硬件反馈和状态估计参考结果发布为 ROS 2 消息，供本机 Isaac Sim 镜像按实测姿态显示。组件只有输入接口与 ROS 发布器。

## 配置与构建

以下配置已经加入组件，默认 100 Hz：

- `wheel-leg-infantry-traditional.yaml`
- `wheel-leg-infantry-prone-v4.yaml`
- `wheel-leg-infantry-wheelbipe.yaml`
- `wheel-leg-infantry-wheelbipe-recovery.yaml`

其他轮腿配置需要同时具备 `WheelLeg`、`WheelLegChassisController` 与 `WheelLegStateEstimator` 提供的接口，然后加入：

```yaml
rmcs_executor:
  ros__parameters:
    components:
      # 原有组件列表中追加
      - rmcs_core::broadcaster::WheelLegTelemetry -> wheel_leg_telemetry

wheel_leg_telemetry:
  ros__parameters:
    enabled: true
    update_rate: 100.0
```

`enabled: false` 时组件不注册输入或 ROS 发布器。`update_rate` 支持 `(0,1000]` Hz，仍受执行器更新频率限制。原有 `value_broadcaster.forward_list` 继续负责标量显示，无需为关节与 IMU 添加标量转发项。

在原构建环境运行：

```bash
build-rmcs --packages-select rmcs_core rmcs_bringup
source rmcs_ws/install/setup.bash
```

部署后按原流程启动所选 RMCS 配置。例如现有传统配置入口为：

```bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-infantry-traditional
```

在相同 ROS 2 环境、相同 domain 中启动 [Foxglove bridge](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)：

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

默认 WebSocket 地址为 `ws://实车IP:8765`。若使用自定义话题白名单，应包含下面五个话题。本机镜像仅订阅它们，不发控制命令。

## 消息约定

全部消息使用 `chassis_body` frame、rad / rad/s / m，QoS 为 best effort、volatile、深度 5。发布器以单调时钟节流，同一个执行器周期的五条消息具有完全相同的 ROS `header.stamp`。

| 话题，前缀 `/wheel_leg/telemetry/` | 消息 | 内容 |
|---|---|---|
| `joint_states` | `sensor_msgs/msg/JointState` | 四个腿部主动轴、两个轮子的硬件层输出轴角与角速度 |
| `imu_body` | `sensor_msgs/msg/Imu` | 安装补偿后的机体到惯性世界姿态四元数、机体角速度 |
| `rl_projected_gravity` | `geometry_msgs/msg/Vector3Stamped` | 状态估计器的机体系单位重力向量，尚未做策略观测缩放 |
| `rl_angular_velocity` | `geometry_msgs/msg/Vector3Stamped` | 机体角速度，尚未做策略观测缩放 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 有效性、反馈年龄、标定身份、模型轴角、被动膝内角和腿长 |

`joint_states.name` 固定为 `left_hip_joint, left_knee_joint, right_hip_joint, right_knee_joint, left_wheel, right_wheel`。接收端仍必须按名称寻址，因为训练模型的主动关节顺序不同。

硬件层已处理左右电机方向、轮子减速比 15.62 以及 IMU 安装变换；接收端不能重复处理。四个腿轴到模型坐标的公式为 `-q_api + kMotorOffsets[i]`，轮轴为原值。`left_knee_joint` / `right_knee_joint` 是主动传动轴，需闭链解算才能得到被动膝内角。

`imu_body` 的 ROS 四元数字段为 `x,y,z,w`，内部 Eigen 为 `w,x,y,z`。线加速度没有发布，`linear_acceleration_covariance[0] = -1`。共同发布时间表示执行器快照；CAN 与 IMU 的采集仍异步，因此 diagnostics 还包含各自的单调时钟反馈年龄。

diagnostics 的 status 名称为 `wheel_leg/telemetry`，关键值包括：

- `schema=wheel_leg_telemetry_v1`、`coordinate=rmcs_hardware_output_shaft_v1`。
- `calibration_sha256`、`calibration_model_spec_sha256`，来自生产标定头文件。
- `feedback_fresh`、`state_valid`、`reset_count`。
- `imu_age_s`、`motor_age_s/<轴名>`；缺失或异常未来时间为 -1。
- `model_position_rad/<轴名>`、四个腿轴的 `model_offset_rad/<轴名>`。
- `left/passive_knee_rad`、`right/passive_knee_rad`、左右 `leg_length_m`。

## 本机镜像

相邻的 `isaac_wheeled_rl_train` 项目提供：

```bash
scripts/view_wheel_leg_live.sh --url ws://实车IP:8765 --record session.jsonl
scripts/view_wheel_leg_live.sh --replay session.jsonl
```

映射、独立几何重建、GUI 依赖与核对流程见该项目的 `docs/WHEEL_LEG_LIVE_MIRROR.md`。协议接收与记录实现复用本仓库 `rmcs_ws/src/rmcs_core/tool/wheel_leg_telemetry.py`，无需在 GUI 主机安装 ROS。

镜像固定世界位置，只更新实测朝向与机构姿态。没有云台反馈时保持标注的名义云台零位。断线、缺帧、过期反馈或标定不一致会冻结最后有效姿态并标红；软件对齐检查不代表实车几何测量或动力学验证已经完成。
