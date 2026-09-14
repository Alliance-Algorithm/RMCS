# 基于RMCS的电机转动项目

本项目基于 [RMCS](https://github.com/Alliance-Algorithm/RMCS)（RoboMaster Control System, ROS2）搭建，用于驱动单个 **M3508 电机**，实现**速度（转速）控制**与**位置（角度）控制**两种转动方式。包含两个独立可运行的子项目：

| 子项目 | 组件类 | 配置文件 | 控制方式 |
| --- | --- | --- | --- |
| 速度控制 | `rmcs_core::hardware::MotorDrive` | `motor_drive.yaml` | 速度环 PID + 遥控器 |
| 位置控制 | `rmcs_core::hardware::MotorPosition` | `motor_position.yaml` | 角度环 PID + 速度环 PID（双闭环） |
| 龙门架控制 | `rmcs_core::hardware::Gantry` | `gantry.yaml` | 虚拟主轴双闭环 + 交叉耦合同步控制 |

各子项目共用同一个 RMCS 工作区，硬件与上位机相同，仅控制组件与参数配置不同。

## 项目结构

```
rmcs_ws/
└── src/
    ├── rmcs_bringup/config/
    │   ├── motor_drive.yaml        # 速度控制项目配置
    │   ├── motor_position.yaml     # 位置控制项目配置
    │   └── gantry.yaml             # 龙门架项目配置
    └── rmcs_core/src/hardware/
        ├── motor_drive.cpp         # 电机速度控制组件（遥控器/定速/方波）
        ├── motor_position.cpp      # 电机位置控制组件（目标角度订阅/方波/角度序列）
        └── gantry.cpp              # 龙门架同步升降组件（虚拟主轴双闭环 + 交叉耦合同步）
```

## 硬件与依赖

- 单个 **M3508** 电机，接入 RMCS 下位机（`rmcs_slave`）的 CAN1。
- 速度控制项目可选 **DR16 遥控器**（DBus 接入）。
- 上位机与下位机通过 USB（`board_serial`）连接。

## 构建方式

两个子项目位于同一个 `rmcs_ws` 工作区内，**共用一套构建流程**：先构建整个工作区，再分别运行各个项目的配置。

### 进入开发环境

前提：已准备好 RMCS 开发容器（Docker / Dev Container）。终端内执行：

```bash
build-rmcs
```

该命令会在 `rmcs_ws` 下对工作区进行 `colcon` 构建（采用 `--symlink-install --merge-install`），构建完成后即可运行任意一个子项目。

若需指定架构交叉编译，可运行：

```bash
# linux/amd64 的 latest-full 变体，编译到 arm64
build-rmcs-cross --target-arch arm64
```

### 项目一：电机速度控制（MotorDrive）

**控制逻辑**（见 `motor_drive.cpp`）：

- 遥控器解锁（DR16 有效且右拨杆中挡）后，左摇杆 Y 轴映射为目标转速，经**速度环 PID** 输出力矩。
- 未解锁时，可选择输出固定转速 `fixed_velocity` 或**方形波** `square_wave`（用于调 PID）。

**构建/运行**：

```bash
# 设置机器人类型为速度控制项目
set-robot motor_drive

# 启动
launch-rmcs
```

### 项目二：电机位置控制（MotorPosition）

**控制逻辑**（见 `motor_position.cpp`）：

- 通过订阅 `/motor/target_angle` 或由 `square_wave` / `sequence_angles` 生成目标角度，**沿优弧（反向绕大圈）转动**。
- 采用**角度环 PID + 速度环 PID 双闭环**：角度环输出目标速度，速度环输出力矩。
- 误差限幅 ±2π，未收到目标时不使能。

**构建/运行**：

```bash
# 设置机器人类型为位置控制项目
set-robot motor_position

# 启动
launch-rmcs
```

### 项目三：龙门架同步升降（Gantry）

**机械与反馈**：左右各一根丝杆，分别由一台 M3508 驱动，横梁刚性连接两侧螺母。
每台电机自带编码器，经丝杆导程换算为左右高度 `h_left`、`h_right`；可选左右限位开关用于回零。

**控制逻辑**（见 `gantry.cpp`）：

- **虚拟主轴**：以两侧平均高度 `h = (h_left + h_right) / 2` 作为主轴位置，做高度环 + 速度环双闭环。
- **交叉耦合同步控制**：用 `e_sync = h_left - h_right` 经 PD 生成差动力矩 `tau_sync`，
  以 `tau_left = tau_common - tau_sync`、`tau_right = tau_common + tau_sync` 主动消除左右错位。
- **目标速率限幅**（梯形规划）+ 速度前馈，保证平缓升降。
- **负载前馈/观测**：静止时用速度残差慢积分估计左右保持力矩，补偿左右负载不一致。
- **安全**：`|e_sync|` 超限、回零超时、电机过温时进入故障并输出零力矩。
- 无外部传感器时，以多圈编码器 + 丝杆导程估计姿态；有外部传感器时可接入限位开关回零。

**构建/运行**：

```bash
# 设置机器人类型为龙门架项目
set-robot gantry

# 启动
launch-rmcs
```

目标高度可通过话题 `/gantry/target_height_cmd`（`std_msgs/Float64`）下发，
或遥控器解锁（右拨杆中挡）后用左摇杆 Y 轴连续调节；`/gantry/home`（`std_srvs/Trigger`）触发回零。

## 调试

运行时可使用 `foxglove` 或 `rmcs-cli` 查看 `/motor/*` 相关话题：
- `/motor/velocity`：电机实时转速
- `/motor/angle_single_turn`：单圈规整角度
- `/motor/control_velocity` / `/motor/control_angle`：当前控制量
- `/motor/control_torque`：PID 输出力矩
- `/motor/enabled`：是否使能

龙门架项目可查看 `/gantry/*` 相关话题：

- `/gantry/height` / `/gantry/height_left` / `/gantry/height_right`：主轴高度与左右高度
- `/gantry/target_height`：规划后的目标高度
- `/gantry/sync_error` / `/gantry/tilt_angle`：左右高度差与横梁倾角
- `/gantry/left/control_torque` / `/gantry/right/control_torque`：左右输出力矩
- `/gantry/enabled` / `/gantry/homed` / `/gantry/fault`：使能、回零、故障状态

