# 基于RMCS的电机转动项目

本项目基于 [RMCS](https://github.com/Alliance-Algorithm/RMCS)（RoboMaster Control System, ROS2）搭建，用于驱动单个 **M3508 电机**，实现**速度（转速）控制**与**位置（角度）控制**两种转动方式。包含两个独立可运行的子项目：

| 子项目 | 组件类 | 配置文件 | 控制方式 |
| --- | --- | --- | --- |
| 速度控制 | `rmcs_core::hardware::MotorDrive` | `motor_drive.yaml` | 速度环 PID + 遥控器 |
| 位置控制 | `rmcs_core::hardware::MotorPosition` | `motor_position.yaml` | 角度环 PID + 速度环 PID（双闭环） |

两个子项目共用同一个 RMCS 工作区，硬件与上位机相同，仅控制组件与参数配置不同。

## 项目结构

```
rmcs_ws/
└── src/
    ├── rmcs_bringup/config/
    │   ├── motor_drive.yaml        # 速度控制项目配置
    │   └── motor_position.yaml     # 位置控制项目配置
    └── rmcs_core/src/hardware/
        ├── motor_drive.cpp         # 电机速度控制组件（遥控器/定速/方波）
        └── motor_position.cpp      # 电机位置控制组件（目标角度订阅/方波/角度序列）
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

## 调试

运行时可使用 `foxglove` 或 `rmcs-cli` 查看 `/motor/*` 相关话题：

- `/motor/velocity`：电机实时转速
- `/motor/angle_single_turn`：单圈规整角度
- `/motor/control_velocity` / `/motor/control_angle`：当前控制量
- `/motor/control_torque`：PID 输出力矩
- `/motor/enabled`：是否使能

