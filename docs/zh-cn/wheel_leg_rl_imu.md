# Wheel-leg IMU 安装外参与 RL 观测

## 配置

IMU 安装外参统一放在车辆 YAML 的 `wheel_leg.ros__parameters` 下，两份轮腿配置均提供：

- `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml`
- `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-unified-rl.yaml`

```yaml
wheel_leg:
  ros__parameters:
    imu_to_base: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    imu_position: [0.0, 0.0, 0.0]
    imu_angular_acceleration_filter_tau_s: 0.01
```

`imu_to_base` 是按行展开的 3×3 旋转矩阵 `R_base_imu`，把 BMI088 测量轴转换为训练使用的 `base_link` 轴。该 base 采用 X 前、Y 左、Z 上。源 CAD 导出 URDF 时已经施加的几何旋转不能在此重复施加。当前安装轴向按已有确认保留单位阵。

`imu_position` 是 **从训练/URDF `base_link` 原点指向 BMI088 加速度计测量中心** 的 `[x, y, z]`，在 **base_link 轴** 下表达，单位 **米**。这里的目标原点不是自动推定的整车质心，也不是任意板卡边角。若测量的是板卡基准点，须先计入芯片相对该点的偏移。未来给出的坐标必须先换算到训练使用的 base_link 轴。

当前 `[0.0, 0.0, 0.0]` 是待实测填写的占位值，关闭杆臂补偿，保留原来的即时加速度发布行为。非零值自动启用补偿。安装参数在启动时读取，修改 YAML 后重启 RMCS；不支持运行中热更新。

`imu_angular_acceleration_filter_tau_s` 是陀螺差分得到角加速度后的一级低通时间常数，单位秒；默认 0.01，0 表示不滤波。差分使用板卡时间，不使用主机回调到达间隔。旋转须正交且行列式为 +1；位置须为三个有限数；时间常数须为有限非负数。无效参数在创建硬件时拒绝，早于打开板卡连接。

控制器仍保留 `imu_alignment_ready` 作为部署就绪门限，外参只在硬件侧配置和应用一次。由旧配置迁移时，把控制器下面的 `imu_to_base` 移到 `wheel_leg` 下面，并增加位置与滤波参数。

## 采样与杆臂补偿

硬件先旋转原始加速度计、陀螺读数，再将加速度计比力换算到 base_link 原点：

```text
omega_base = R_base_imu * omega_imu
f_base     = R_base_imu * f_imu
             - alpha_base × r_base_imu
             - omega_base × (omega_base × r_base_imu)
```

这里 `r_base_imu = imu_position`。后两项分别是切向与向心加速度，适用于 IMU 与底盘刚性连接的情况。加速度计输出是比力，静止水平时约为 `[0,0,+9.80665] m/s²`。补偿保留 base 原点自身的平移加速度；不会直接将动态加速度当作重力。

启用补偿后：

1. 加速度样本等待前后两个有效陀螺样本；在加速度时间插值角速度，再应用角加速度低通结果。
2. 补偿后的比力同时送入姿态 EKF 和 `/wheel_leg/imu/acceleration`，不只修改遥测。该接口使用 base_link 原点和 base 轴。
3. 保留加速度计自己的板卡时间戳。没有新补偿结果时，不重复发布旧样本来刷新序号或接收时间。
4. 首次启动、采样间隔超过现有 1 ms 连续性门限、陀螺饱和后，必须重新建立连续的陀螺历史。不跨中断推算角加速度，也不发布未经补偿的非零偏移样本。
5. 新进入 DR16 双下时，命令线程请求复位，接收线程清除差分、滤波与待处理加速度历史；保留安装外参和连续物理姿态，避免跨线程直接修改 EKF。

角加速度滤波与离散采样会带来估计误差；静态测试不能检验运动中的杆臂误差。非零实测值仍须通过离线回放/仿真与实车传感器记录评估，软件测试不代表整车行为验收。

## RL 观测

`/wheel_leg/imu/quaternion` 是 base→world 姿态，`/wheel_leg/imu/angular_velocity` 是 base 轴角速度。控制器直接使用这些已对齐的输入，不再重复安装旋转：

```text
omega_RL = omega_base
g_RL     = inverse(normalize(q_world_base)) * [0,0,-1]
```

普通与 unified 两条策略链路都使用角速度和重力投影，没有把原始线加速度或 xyz 安装偏移直接加入 actor。观测维数、顺序、缩放和 ONNX 模型不变；35 维帧的第 4–6 项为 `0.5 * omega_base`，第 7–9 项为 `g_base`（从 0 计数）。位置补偿通过改善 EKF 的加速度输入间接作用于重力观测。

## 验证

只使用仓库构建入口：

```bash
build-rmcs --packages-select rmcs_core rmcs_bringup
cd rmcs_ws
colcon test --merge-install --packages-select rmcs_core --ctest-args -R 'wheel_leg_(imu|feedback|.*policy_integration|unified_observation)'
colcon test-result --test-result-base build/rmcs_core --verbose
```

`wheel_leg_imu` 覆盖：两份真实 YAML 的配置归属与外参解析、非法参数拒绝、零偏移兼容、旋转安装、切向/向心补偿、保留自身平移加速度、角加速度噪声滤波、补偿进入真实 EKF、板卡时间对齐、乱序/中断/饱和与复位后的重新建史。测试不创建硬件连接。

本次容器验证：`rmcs_core`、`rmcs_bringup` 构建通过；新增 11 项 IMU 测试通过，`rmcs_core` 当前全部 6 个测试目标（23 项 GTest）通过。未启动实车或下发运动命令，实测 xyz 尚未填写。

## 实现位置

- `rmcs_core/src/hardware/device/imu_lever_arm.hpp`：刚体测量点补偿、板卡时间插值与角加速度滤波。
- `rmcs_core/src/hardware/device/bmi088_ekf.hpp`：在 EKF 前应用外参及补偿，并发布原点比力。
- `rmcs_core/src/hardware/wheel_leg_imu.hpp`：YAML 参数加载与运行时修改拒绝。
- `rmcs_core/src/hardware/wheel-leg.cpp`：硬件配置、补偿样本发布、双下复位的接收线程交接。
- `rmcs_core/src/controller/chassis/wheel_leg/{configuration.cpp,observation.cpp,wheel_leg_controller.hpp}`：移除控制器中的重复安装旋转。
- `rmcs_core/test/wheel_leg_imu.cpp`、`rmcs_core/CMakeLists.txt`：IMU 回归与测试注册。
- 仓库 `.gitignore`：放行新增的 IMU 回归测试源码。
- `rmcs_bringup/config/wheel-leg-infantry{-rl,-unified-rl}.yaml`：两套车辆的安装配置。

以上包路径均位于 `rmcs_ws/src`。现有插件类名、组件列表与接口连接保留，未新增插件。
