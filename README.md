# RMCS

## 本地安装

默认本机已经安装并配置好 ROS 2 环境，并且当前 shell 中已经有 `ROS_DISTRO` 环境变量。除此之外，RMCS 还依赖以下软件包：

```bash
sudo apt update && sudo apt install -y \
  ros-$ROS_DISTRO-moveit \
  ros-$ROS_DISTRO-moveit-visual-tools \
  ros-$ROS_DISTRO-moveit-task-constructor \
  ros-$ROS_DISTRO-moveit-servo \
  ros-$ROS_DISTRO-moveit-resources \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers
```

## 控制器模块与接口

`rmcs_ws/src/rmcs_core/src/controller` 目前主要分为 `arm/` 和 `pid/` 两部分。对机械臂而言，`rmcs_ws/src/rmcs_bringup/config/engineer.yaml` 默认启用的组件链路为：

1. `rmcs_core::controller::arm::ArmConfig -> arm_config`
2. `rmcs_core::controller::arm::ArmSolver -> arm_solver`
3. `rmcs_core::controller::arm::ArmController -> arm_controller`
4. `rmcs_core::controller::arm::SubArmController -> sub_arm_controller`

需要先说明一点：源码里的 `register_input(...)` / `register_output(...)` 注册的是 RMCS 执行器内部数据通道，不是 ROS topic。当前这一目录下真正显式使用 ROS 接口的，主要是 `ArmConfig` 的 `/robot_description`、`/joint_states`，以及 `ArmController` 内部使用的 MoveIt `MoveGroupInterface("alliance_arm")`。

### 通信方式

`register_input(...)` / `register_output(...)` 的本质，是在同一个 RMCS 执行器进程内搭一套“按名字配对、按类型检查、按指针直接读写”的组件通信机制。

- `register_output(name, output, init_value)`：
  - 在当前组件里创建一个具名输出槽位，并在槽位内部构造数据对象。
  - 这个输出槽位拥有数据本体，后续由当前组件在 `update()` 里写入，例如 `*output = value;`。
- `register_input(name, input)`：
  - 在当前组件里声明“我要读名为 `name` 的某个输出”。
  - 注册时只记录名字、类型和一个待绑定指针；真正的数据地址要等执行器做统一配对时再填进去。
- 执行器配对阶段：
  - 按“同名 + 同类型”把输入通道和输出通道绑定起来。
  - 绑定完成后，`InputInterface<T>` 内部会直接指向对应 `OutputInterface<T>` 持有的数据地址。
- 运行阶段：
  - 生产者组件写 `*output_`。
  - 消费者组件读 `*input_`。
  - 中间没有 ROS 序列化、没有 topic 发布订阅，也没有额外拷贝，属于单进程内的直接数据流。

这套机制适合 RMCS 这种高频控制链路。像 `arm_config -> arm_controller -> arm_solver -> hardware` 之间的大部分状态量和控制量，都是这样在组件内部直接流动的。

一个最小例子如下：

```cpp
class DemoProducer final : public rmcs_executor::Component {
public:
    DemoProducer() {
        register_output("/demo/value", value_, 0.0);
    }

    void update() override {
        *value_ = 42.0;
    }

private:
    OutputInterface<double> value_;
};

class DemoConsumer final : public rmcs_executor::Component {
public:
    DemoConsumer() {
        register_input("/demo/value", value_);
        register_output("/demo/twice_value", twice_value_, 0.0);
    }

    void update() override {
        *twice_value_ = 2.0 * *value_;
    }

private:
    InputInterface<double> value_;
    OutputInterface<double> twice_value_;
};
```

这个例子的实际效果是：

1. `DemoProducer` 向 `/demo/value` 写出 `42.0`。
2. 执行器把 `DemoConsumer` 的 `value_` 绑定到这个输出槽位。
3. `DemoConsumer` 在自己的 `update()` 里直接读到 `42.0`。
4. 然后它再把结果 `84.0` 写到 `/demo/twice_value`，供下一个组件继续使用。

如果名字写错，或者输入输出类型不一致，例如一边是 `double`、另一边是 `Eigen::Vector3d`，执行器就无法正确配对。这也是 RMCS 里接口路径和类型都要写得很严格的原因。

### `arm/` 目录

#### `arm_config.cpp`

- 类型：插件组件，负责把 URDF 和电机反馈整理成机械臂控制链需要的统一状态量。
- ROS 接口：
  - 订阅 `/robot_description`，只在首次收到时解析 URDF。
  - 发布 `/joint_states`，供 RViz / MoveIt 使用。
- 输入通道：
  - `/main/arm/joint_{1..6}/motor/angle`
  - `/main/arm/joint_{1..6}/motor/velocity`
  - `/main/arm/joint_{1..6}/motor/torque`
  - `/main/arm/joint_2/encoder/angle`
- 输出通道：
  - `main_urdf_loaded`
  - `/main/arm/link_{1..6}/{mass,com,inertia,length}`
  - `/main/arm/joint_{1..6}/{lower_limit,upper_limit,velocity_limit,torque_limit,theta,velocity,torque,position,friction}`
- 备注：
  - `joint_2` 的关节角以编码器角度为准。
  - URDF 成功加载后会置位 `main_urdf_loaded`，后续不再重复解析。

#### `arm_solver.cpp`

- 类型：插件组件，负责主臂的力矩求解。
- 参数接口：
  - `controller_list` 用于组合控制项，当前代码支持 `pid`、`gravity`、`friction`、`zero_torque`。
  - `engineer.yaml` 当前配置为 `controller_list: [pid,gravity]`。
- 输入通道：
  - `/main/arm/joint_{1..6}/{theta,target_theta,lower_limit,upper_limit,velocity,friction}`
  - `/main/arm/link_{1..6}/{mass,com,length}`
  - `/main/arm/joint_4/position`
  - `/main/arm/enable_flag`
  - `main_urdf_loaded`
- 输出通道：
  - `/main/arm/joint_{1..6}/motor/control_torque`
- 备注：
  - `main_urdf_loaded` 置位前不会输出有效控制。
  - 当 `/main/arm/enable_flag` 为 `false` 时，会输出 `zero_torque` 分支产生的 `NaN`，由下游硬件侧解释为不驱动。

#### `arm_controller.cpp`

- 类型：插件组件，负责主臂使能和目标角生成，同时初始化 MoveIt 规划线程。
- 输入通道：
  - `/remote/joystick/right`
  - `/remote/joystick/left`
  - `/remote/switch/right`
  - `/remote/switch/left`
  - `/remote/mouse/velocity`
  - `/remote/mouse`
  - `/remote/keyboard`
  - `/remote/rotary_knob_switch`
  - `/main/arm/joint_{1..6}/theta`
  - `/sub/arm/joint_{1..6}/motor/angle`
- 输出通道：
  - `/main/arm/enable_flag`
  - `/main/arm/joint_{1..6}/target_theta`
- MoveIt 接口：
  - 使用 `MoveGroupInterface("alliance_arm")`。
- 备注：
  - 当前 `update()` 中真正生效的逻辑是安全使能和目标角转发：初始化后只有当左右拨杆都曾同时置于 `DOWN`，系统才允许进入正常控制。
  - 正常使能时，主臂 `target_theta` 直接跟随 `/sub/arm/joint_{1..6}/motor/angle`。
  - 文件里已经预留了 MoveIt 规划、DT7 摇杆位姿增量等函数，但它们目前没有在 `update()` 主流程中调用，因此这些遥控接口现阶段主要属于预留接口。

#### `sub_arm_controller.cpp`

- 类型：插件组件，负责副臂 / 示教臂的本地力矩控制。
- 参数接口：
  - `controller_list` 同样支持 `pid`、`gravity`、`friction`、`zero_torque`。
  - `engineer.yaml` 当前配置为 `controller_list: [friction,gravity]`。
- 输入通道：
  - `/sub/arm/joint_{1..6}/motor/angle`
  - `/sub/arm/joint_{1..6}/motor/velocity`
- 输出通道：
  - `/sub/arm/joint_{1..6}/motor/control_torque`
  - `/arm/joint_123/dm_enable_command`
- 备注：
  - 上电后的前 1000 个控制周期处于启动保持阶段，其中前 50 个周期会拉高 `/arm/joint_123/dm_enable_command`。
  - 当前工程配置没有启用这里的 `pid` 项；代码里的 `calculate_pid()` 依赖 `joint_target_theta`，但该输入目前并未在构造函数中注册，所以如果后续要启用 `pid`，需要先补齐 `/sub/arm/joint_{1..6}/target_theta` 一类接口。

#### `Kinematic.hpp`

- 类型：工具类头文件，不是插件组件。
- 作用：
  - `positive_kinematic()` 根据 6 个齐次变换矩阵计算末端位姿。
  - `get_x/y/z/roll/pitch/yaw()` 读取正解结果。
  - `arm_inverse_kinematic(std::array<double, 6>)` 提供一套静态逆解。
- 接口：
  - 实例化后会向组件注册 `/main/arm/Joint{1..6}/T` 六个输入矩阵。
- 备注：
  - 命名空间是 `rmcs_core::hardware::device`，虽然文件放在 `controller/arm/` 下，但更接近运动学工具而不是控制插件。

#### `trajectory.hpp`

- 类型：轨迹插值工具头文件，不是插件组件。
- 作用：
  - `Trajectory<LINE>`：笛卡尔直线插值。
  - `Trajectory<BEZIER>`：笛卡尔三次 Bezier 插值。
  - `Trajectory<JOINT>`：关节空间三次多项式插值。
- 主要接口：
  - `set_start_point(...)`
  - `set_end_point(...)`
  - `set_control_points(...)`
  - `set_total_step(...)`
  - `trajectory()`
  - `reset()`
  - `get_complete()`

#### `drag_teach.hpp`

- 类型：拖动示教辅助类，不是插件组件。
- 作用：
  - 将 6 维关节数据按二进制格式追加保存到文件，或按相同格式回放读取。
- 主要接口：
  - `Drag(filename)`
  - `read_data_from_file()`
  - `write_data_to_file(const std::array<double, 6>&)`
  - `reset_file_pointer()`
  - `get_data()`

### `pid/` 目录

#### `pid_controller.cpp`

- 类型：插件组件，通用 PID 控制器。
- 参数接口：
  - `measurement`
  - `setpoint`
  - `feedforward`（可选，默认 0）
  - `control`
  - `kp` / `ki` / `kd`
  - `integral_min` / `integral_max`
  - `integral_split_min` / `integral_split_max`
  - `output_min` / `output_max`
- 接口规则：
  - `measurement`、`setpoint`、`feedforward` 通过 `SmartInput` 解析，既可以写成常数，也可以写成某个 RMCS 内部通道名。
  - 如果参数值是以 `-` 开头的通道名，例如 `-/foo/bar`，则会自动按相反数读取该通道。
- 输出行为：
  - 输出通道名由 `control` 参数指定，控制量为 `feedforward + PID(setpoint - measurement)`。

#### `error_pid_controller.cpp`

- 类型：插件组件，直接对“误差量”做 PID。
- 参数接口：
  - `measurement`
  - `control`
  - `kp` / `ki` / `kd`
  - `integral_min` / `integral_max`
  - `output_min` / `output_max`
- 输入输出：
  - 从 `measurement` 参数指定的通道直接读取误差。
  - 向 `control` 参数指定的通道输出 PID 结果。
- 备注：
  - 它不负责 `setpoint - measurement` 这一步，适合上游已经提前算好误差的场景。

#### `pid_calculator.hpp`

- 类型：基础 PID 算法工具头文件。
- 主要接口：
  - `PidCalculator(kp, ki, kd)`
  - `update(err)`
  - `reset()`
- 特性：
  - 支持积分限幅、输出限幅、积分分离。
  - 输入误差非有限值时直接返回 `NaN`。

#### `matrix_pid_calculator.hpp`

- 类型：向量 / 矩阵 PID 工具头文件。
- 主要接口：
  - `MatrixPidCalculator<n, use_matrix_gain>(kp, ki, kd)`
  - `update(Vector err)`
  - `reset()`
- 特性：
  - 支持标量增益，也支持 `n x n` 增益矩阵。
  - 会自动过滤 `NaN` 项并做向量限幅。

#### `smart_input.hpp`

- 类型：参数适配工具头文件。
- 作用：
  - 在插件参数和 RMCS 内部通道之间做一层薄封装。
  - 支持“参数是常数”与“参数是通道名”两种模式。
  - 支持通道取反，用于少写一层独立的取负组件。

## Development

### Pre-requirements:

- x86-64 架构
- 任意 Linux 发行版，或 WSL2（仓库当前未提供单独的 WSL2 开发文档）
- [VSCode](https://code.visualstudio.com/)，安装 [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- 安装 Docker；如所在网络环境需要代理，请按本机环境自行配置

### Step 1：获取镜像

下载开发镜像：
```bash
docker pull qzhhhi/rmcs-develop:latest
```

也可自行使用仓库根目录下的 `Dockerfile` 构建镜像。

### Step 2：克隆并打开仓库

克隆仓库，注意需要使用 `recurse-submodules` 以克隆子模块：

```bash
git clone --recurse-submodules https://github.com/YuuuuuQingChi/RoboticArm-Moveit2.git
```

在 VSCode 中打开仓库：

```bash
code ./RMCS
```

按 `Ctrl+Shift+P`，在弹出的菜单中选择 `Dev Containers: Reopen in Container`。

VSCode 将拉起一个 `Docker` 容器，容器中已配置好完整开发环境，之后所有工作将在容器内进行。

如果 `Dev Containers` 在启动时卡住很长一段时间，优先检查 Docker 是否正常工作、镜像是否已拉取完成，以及本机代理配置是否正确。

### Step 3：配置 VSCode

在 VSCode 中新建终端，输入：

```bash
cp .vscode/settings.default.json .vscode/settings.json
```

这会应用我们推荐的 VSCode 配置文件，你也可以按需自行修改配置文件。

在拓展列表中，可以看到我们推荐使用的拓展正在安装，你也可以按需自行删减拓展。

### Step 4：构建

在 VSCode 终端中输入：

```bash
build-rmcs
```

将会运行 `.script/build-rmcs` 脚本，在路径 `rmcs_ws` 下开始构建代码。

构建完毕后，基于 `clangd` 的 `C++` 代码提示将可用。此时可以正常编写代码。

Note: 用于开发的所有脚本均位于 `.script` 中，参见 开发脚本手册(TODO)。

### Step 5 (Optional)：运行

编写代码并编译完成后，可以使用：

```bash
launch-rmcs
```

在本机上运行代码。在首次运行代码前，需要调用 `set-robot` 脚本设置机器人类型。

#### 确认设备接入

可以使用 `lsusb` 确定 [下位机](https://github.com/Alliance-Algorithm/rmcs_slave) 是否已接入，若已接入，则 `lsusb` 输出类似：

```
Bus 001 Device 004: ID a11c:75f3 Alliance RoboMaster Team. RMCS Slave v2.1.2
```

在 WSL2 下，需要使用 `usbipd` 将 USB 设备转接进 WSL。

#### 确认权限正确

在主机（不要在 `docker` 容器）的终端中输入：

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", MODE="0666"' | sudo tee /etc/udev/rules.d/95-rmcs-slave.rules &&
sudo udevadm control --reload-rules &&
sudo udevadm trigger
```

以允许非 root 用户读写 RMCS 下位机，此指令只需执行一次。

## Deployment

### Pre-requirements:

- x86-64 架构
- 任意 Linux 发行版
- 安装 Docker

### Step 1：获取镜像

下载部署镜像：

```bash
docker pull qzhhhi/rmcs-develop:latest
```

如果不方便在 MiniPC 上配置代理，可以在开发机上下载镜像后，使用

```bash
docker save qzhhhi/rmcs-runtime:latest > rmcs-runtime.tar
```

然后使用任意方式（如 scp）将 `rmcs-runtime.tar` 传送到 MiniPC 上，并在其上执行：

```bash
docker load -i rmcs-runtime.tar
```

即获取部署镜像。

### Step 2：启动容器

在 MiniPC 终端中输入：

```bash
docker run -d --restart=always --privileged --network=host -v /dev:/dev qzhhhi/rmcs-runtime:latest
```

即可启动部署镜像，此后镜像将保持开机自启。

### Step 3：远程连接

在开发容器终端中输入：

```bash
set-remote <remote-host>
```

其中，`remote-host` 可以为 MiniPC 的：

1. IPv4 / IPv6 地址 (e.g., 169.254.233.233)

2. IPv4 / IPv6 Link-local 地址 (e.g., fe80::c6d0:e3ff:fed7:ed12%eth0)

3. mDNS 主机名 (e.g., my-sentry.local)

参见 网络配置指南(TODO)。

接下来在开发容器终端中继续输入：

```bash
ssh-remote
```

即可在开发容器中，ssh 连接到远程的部署容器。

**RMCS 的所有代码更新和调试，都基于从开发容器向部署容器的 ssh 连接。**

部署容器会监听 TCP:2022 端口作为 ssh-server 端口，请注意保证端口空闲。

参见 容器设计思想(TODO)。

> Tip: GUI 可以从部署容器中穿出，尝试在 ssh-remote 中打开 rviz2。

### Step 4：同步构建产物

新启动的部署容器内是没有代码的，需要由开发容器上传。

在开发容器中构建完成后，可以执行指令：

```bash
sync-remote
```

这将拉起一个同步进程，自动将开发容器中的构建产物同步到部署容器。

同步进程除非主动使用 `Ctrl+C` 结束，否则不会退出，其会监视所有文件变更，并实时同步到部署容器。

> Tip: 由于 `build-rmcs` 采用 `symlink-install` 方式构建，因此对于配置文件和 .py 文件，直接修改其源文件，无需编译即可触发同步。

### Step 5：重启服务

RMCS 在部署容器中以服务方式启动 (`/etc/init.d/rmcs`)。

确认构建产物同步完毕后（以出现 `Nothing to do` 为标志），进入 `ssh-remote`，输入：

```bash
set-robot <robot-name>
```

设置启动的机器人类型（例如 set-robot sentry）。

接下来继续输入：

```bash
service rmcs restart
```

如果一切正常，其将会输出：

```bash
Successfully stopped RMCS daemon.
Successfully started RMCS daemon.
```

这证明 RMCS 已成功在部署容器中启动，并会随以后的每次容器启动而启动。

接下来可以使用：

```bash
service rmcs attach
```

查看 RMCS 的实时输出。

> 需要注意的是，`service rmcs attach` 本质上是连接到了一个 `GNU screen` 会话，因此任何按键都会被忠实地转发至 RMCS。例如，当键入 `Ctrl+C` 时，RMCS 会接到 `SIGINT`，从而停止运行。
> 
> 如果希望退出对实时输出的查看，可以键入 `Ctrl+A` ，然后按 `D`。
> 
> 如果希望向上翻页，可以键入 `Ctrl+A` ，然后按 `Esc`。
> 
> 更多快捷键组合参见 [Screen Quick Reference](https://gist.github.com/andrimanna/e5379fe6db3af0ecdb1e49e8cfb74d24)。

### Step 6：糖

指令 `ssh-remote` 后可以添加参数，参数内容即为建立链接后立即执行的指令。

例如：

```bash
ssh-remote service rmcs restart
```

可以在开发容器端快速重启部署容器中的 RMCS。

又如：

```bash
ssh-remote service rmcs attach
```

可以在开发容器端快速查看部署容器中 RMCS 的实时输出。

实际上，可以使用指令：

```bash
attach-remote
```

作为前者的替代（其实现就是前者）。

进一步的，还可以使用：

```bash
attach-remote -r
```

作为 `ssh-remote "service rmcs restart && service rmcs attach"` 的替代。

其在重启 RMCS 守护进程后，自动连接显示实时输出。

更进一步的，指令间还可以组合，例如：

```bash
build-rmcs && wait-sync && attach-remote -r
```

可以触发 RMCS 构建，`wait-sync` 等待文件同步完成，接下来重启 RMCS 守护进程后，显示实时输出。
