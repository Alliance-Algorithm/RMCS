# 代码库指南

## 项目概览
RMCS（RoboMaster Control System）是基于 ROS 2 Jazzy 的机器人控制系统。开发在 Docker devcontainer 中进行（镜像 `qzhhhi/rmcs-develop`），部署到 MiniPC 上的 runtime 容器。

## 目录结构

```text
.
├── .clang-format / .clang-tidy / .clangd   # 代码风格与静态分析
├── .devcontainer/                           # VS Code devcontainer 配置
├── .script/                                 # 开发脚本（构建、部署、远程调试）
├── docs/zh-cn/                              # 中文文档（Docker、WSL2、nvim 等）
├── plan.md                                  # Planner 接入 RMCS 设计草案
├── Dockerfile / docker-compose.yml          # 容器镜像定义
└── rmcs_ws/src/                             # ROS 2 工作区
    ├── rmcs_executor    # 组件 DAG 执行器框架
    ├── rmcs_core        # 硬件抽象、控制器、裁判系统（28 个 pluginlib 组件）
    ├── rmcs_bringup     # 启动文件与机器人配置 YAML
    ├── rmcs_description # URDF 与 mesh
    ├── rmcs_msgs        # 自定义 ROS 2 消息
    ├── rmcs_utility     # 共享工具类
    ├── sp_vision_25     # [子模块] 视觉系统：自瞄、打符、全向感知、MPC 规划
    ├── auto_aim_2       # [子模块] 旧版自瞄（逐步替换中）
    ├── fast_tf          # [子模块] 高性能 TF2 广播
    ├── hikcamera        # [子模块] HikRobot 相机驱动
    └── serial           # [子模块] ROS 2 串口驱动
```

子模块需要 `git clone --recurse-submodules` 或 `git submodule update --init --recursive`。

## 构建、测试与开发命令

### 构建
- `build-rmcs`：在 `rmcs_ws` 中执行 `colcon build --symlink-install --merge-install`
- `build-rmcs --packages-select sp_vision_25 rmcs_core`：增量构建指定包
- `clean-rmcs`：清理 `rmcs_ws/{build,install,log}`

### 运行
- `set-robot (robot name)`：写入 `RMCS_ROBOT_TYPE` 到 `~/env_setup.{bash,zsh}`
- `launch-rmcs`：`ros2 launch rmcs_bringup rmcs.launch.py robot:=$RMCS_ROBOT_TYPE`
- `ros2 launch rmcs_description display.launch.py`：仅查看 URDF/TF，不启动控制链

### 远程部署
- `set-remote <host>`：配置 MiniPC SSH 连接
- `sync-remote`：监视并持续同步构建产物到部署容器
- `wait-sync`：阻塞直到同步完成
- `ssh-remote`：SSH 进入部署容器
- `attach-remote`：查看 RMCS 实时输出（等价于 `ssh-remote service rmcs attach`）
- `attach-remote -r`：重启后查看实时输出
- 典型流程：`build-rmcs && wait-sync && attach-remote -r`

### 测试
- `colcon test --packages-select rmcs_core rmcs_bringup`：包级检查
- 功能验证以实际启动和硬件联调为主，无集中式单元测试框架

### 其他
- `foxglove`：启动 Foxglove 可视化
- `play-autoaim`：回放录制的自瞄数据

## 组件系统工作原理

### 核心机制
系统不是"手写 main"，而是 **YAML 驱动的组件图**。`rmcs_executor` 是单线程 ROS 2 节点，通过 pluginlib 加载组件，自动按接口名连边并做拓扑排序。

关键概念：
- **接口不是 ROS 2 topic**：`/gimbal/...`、`/chassis/...` 等是进程内强类型接口名，执行器把输入直接绑定到输出的内存地址
- **接口名 + C++ 类型必须同时匹配**：缺失必需输入、重复输出名、类型不匹配或环依赖都会在启动阶段报错
- **Partner component**：组件可在内部创建隐藏伙伴组件（如 `OmniInfantry` 创建 `infantry_hardware_command`），DAG 不只来自 YAML
- **生命周期钩子**：`before_updating()` 设置默认值和初始化，`update()` 是主循环逻辑

### 启动流程
```text
set-robot <type> → launch-rmcs → rmcs.launch.py → rmcs_executor (respawn=true)
  1. 创建内置 predefined_msg_provider（提供 update_rate / update_count / timestamp）
  2. 读取 YAML components: 列表，pluginlib 装载各组件
  3. 按 register_input / register_output 同名接口自动连边
  4. 计算拓扑顺序；配线失败则启动中止
  5. 以 update_rate（默认 1000Hz）循环执行整条链路
```

### 可用的机器人配置
- `omni-infantry.yaml`：四轮全向步兵
- `steering-infantry.yaml`：舵轮步兵
- `mecanum-hero.yaml`：麦克纳姆轮英雄

## 组件 DAG (以最简单的 omni-infantry 为例)

```text
predefined_msg_provider
  → sp_vision_bridge

infantry_hardware
  → referee_status
  → gimbal_controller
  → friction_wheel_controller
  → bullet_feeder_controller
  → chassis_controller
  → referee_ui
  → referee_command
  → infantry_hardware_command          ← partner component，统一下发控制量

referee_status
  → sp_vision_bridge
  → heat_controller
  → chassis_power_controller
  → referee_ui

sp_vision_bridge
  → gimbal_controller
  → bullet_feeder_controller

gimbal_controller
  → chassis_controller
  → infantry_hardware_command

friction_wheel_controller
  → heat_controller
  → bullet_feeder_controller
  → left_friction_velocity_pid_controller
  → right_friction_velocity_pid_controller

heat_controller
  → bullet_feeder_controller

bullet_feeder_controller
  → bullet_feeder_velocity_pid_controller

chassis_controller
  → chassis_power_controller
  → omni_wheel_controller
  → referee_ui_infantry

chassis_power_controller
  → omni_wheel_controller
  → infantry_hardware_command
  → referee_ui_infantry

left/right_friction_velocity_pid_controller
  → infantry_hardware_command

bullet_feeder_velocity_pid_controller
  → infantry_hardware_command

omni_wheel_controller
  → infantry_hardware_command

referee_ui_infantry → referee_ui → referee_interaction → referee_command
```

## 关键接口分层

### 系统时序
`predefined_msg_provider` 输出 `/predefined/update_rate`、`/predefined/update_count`、`/predefined/timestamp`。

### 硬件状态层
`OmniInfantry` 及其设备输出：
- `/remote/*`（遥控器）、`/tf`（坐标变换）
- `/gimbal/*/{angle,velocity,torque,max_torque}`
- `/chassis/*/{velocity,power,voltage}`、`/referee/serial`

Partner component `infantry_hardware_command` 消费：
- `/gimbal/*/control_torque`、`/chassis/*/control_torque`、`/chassis/supercap/charge_power_limit`

### 视觉规划层
`SpVisionBridge` 内部运行两个独立线程：
- **检测线程**：Camera → YOLO → Solver → Tracker → `latest_target`
- **Planner 线程（1kHz）**：`latest_target` + `bullet_speed` → `Planner.plan()` → `latest_plan`

输入：`/tf`、`/predefined/timestamp`、可选 `/referee/shooter/initial_speed`

输出：
- `/gimbal/auto_aim/control_direction`（方向语义）
- `/gimbal/auto_aim/fire_control`（开火决策）
- `/gimbal/auto_aim/plan_{yaw,pitch}`（世界系规划角度）
- `/gimbal/auto_aim/plan_{yaw,pitch}_velocity`
- `/gimbal/auto_aim/plan_{yaw,pitch}_acceleration`

超时保护：plan 过期时方向清零、导数清零、`fire_control = false`。

### 云台控制层
`OmniInfantryPlannerGimbalController`（omni-infantry 专用，不改通用 `SimpleGimbalController`）：
- 输入遥控量、`/tf`、IMU 角速度、全部视觉规划结果
- 内部：RMCS 几何层算 angle error → angle PID + velocity PID + planner 前馈
- 输出 `/gimbal/{yaw,pitch}/control_torque`
- 同时发布 `/gimbal/{yaw,pitch}/control_angle_error` 供底盘跟随

### 射击层
- `FrictionWheelController`：`/gimbal/*_friction/control_velocity`、`/gimbal/friction_ready`、`/gimbal/bullet_fired`
- `HeatController`：`/gimbal/control_bullet_allowance/limited_by_heat`
- `BulletFeederController17mm`：结合遥控、热量和 `fire_control` → `/gimbal/bullet_feeder/control_velocity`、`/gimbal/shooter/mode`

### 底盘层
- `ChassisController`：遥控 + 云台误差 → `/chassis/control_{mode,velocity,angle}`
- `ChassisPowerController`：裁判功率 + 超级电容 → `/chassis/control_power_limit`
- `OmniWheelController`：→ 四轮 `/chassis/*_wheel/control_torque`

### 裁判/UI 层
`Status` 解帧 → `Ui`/`Infantry` 生成 UI → `Interaction` 汇总 → `Command` 编码写回 `/referee/serial`

## 调试指南

### 启动失败排查
1. `rmcs_bringup/config/<robot>.yaml` 是否列出了正确组件和参数
2. 组件声明的输入是否都能在别处找到同名输出
3. 是否有组件在 `before_updating()` 中为可选输入绑定了默认值，掩盖了上游缺失

### 运行时数据流追踪
按"外部输入 → 中间接口 → 最终控制量"追踪：
- **自瞄异常**：`/tf` → `/gimbal/auto_aim/*` → `/gimbal/{yaw,pitch}/control_torque`
- **底盘异常**：`/chassis/control_velocity` → `/chassis/control_power_limit` → `/chassis/*_wheel/control_torque`
- **射击异常**：`/gimbal/auto_aim/fire_control` → `/gimbal/control_bullet_allowance/*` → `/gimbal/bullet_feeder/control_velocity`

## 编码风格与命名规范

根目录 `.clang-format` 配置：
- 基于 LLVM 风格，C++20 标准
- 4 空格缩进，100 列宽
- `PointerAlignment: Left`（即 `int* p`）
- `AlignAfterOpenBracket: AlwaysBreak`

命名约定：
- 类型：`UpperCamelCase`
- 命名空间：按目录展开，如 `rmcs_core::controller::gimbal`
- YAML 参数、组件实例名、接口名：`snake_case` 或斜杠路径，如 `/gimbal/auto_aim/plan_yaw`
- 机器人配置文件：短横线命名，如 `omni-infantry.yaml`
- 机器相关串口、零点、板号只放在 `rmcs_bringup/config/*.yaml`，不要硬编码进共享逻辑
