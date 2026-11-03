# v202 传统控制上车配置与 IMU 同步

分支为 `dev/wheel_leg_tradition`。新增完整真机配置 [wheel-leg-traditional-v202.yaml](../../rmcs_ws/src/rmcs_bringup/config/wheel-leg-traditional-v202.yaml)，用于操作员在场的“自起 → 零指令站立”测试。开发与验证时的临时工作树为 `/home/noir/Documents/workspace/.worktrees/rmcs-wheelleg-tradition-deploy`；发布后可在任意该分支的检出目录构建，该绝对路径仅用于追溯测试。本轮未部署到车上。

该图使用 `WheelLeg` 硬件、生产自起组件、共享传统控制组件及诊断广播四个组件。传统控制组件是六个电机命令的唯一写入者：自起期间转发恢复输出，接稳后调用此前 Isaac 测试使用的 `TraditionalControlCore`。不加载旧 MuJoCo/V5 示例里的估计器、轨迹控制器和驱动桥，也没有 RL 或云台主动控制。

## IMU 与编码器映射

同步来源是当前主工作区 `/home/noir/Documents/workspace/RMCS` 的 `dev/wheel-leg-dual-deploy` 分支（HEAD `7add701a`，包括当时工作区内 IMU 修改）。`wheel_leg.imu_to_base = diag(-1, 1, -1)`，即原始加速度和角速度统一变为 `(-x, y, -z)`，在硬件 EKF 前应用。四元数、重力、角速度和加速度因此都采用底盘 `X前/Y左/Z上` 坐标系。传统控制和自起组件直接使用该结果，不再做第二次安装旋转。

`imu_position: [0, 0, 0]` 表示安装位置尚未实测，当前关闭杆臂补偿。代码支持测得位置后补偿切向及向心加速度，但本轮没有凭空填入安装尺寸。IMU 安装参数只允许启动时读取；双 DOWN 清除运动补偿历史，保留姿态与安装配置。

编码器复用当前 RMCS 的 v202/calibration110 配置，没有执行电机置零。顺序为左髋、左膝、右髋、右膝、左轮、右轮，驱动 API 已处理的反向不重复处理：

```text
q_CAD = hardware_to_cad_sign * (q_API - hardware_zero_rad)
sign = [-1, -1, -1, -1, +1, +1]
zero = [1.6, 3.0282761689183353, -1.6, -3.0283102010168, 0, 0]
```

自起组件采用等价的加性偏置表达。左右腿各自保留成对的独立整圈；传统接管从实测腿长开始升高，不强行回到零圈。物理轮前进方向和各腿实测角度仍需这次上车检查，`hardware_mapping_verified` 因此保持 `false`。新文件用显式 `hardware_commissioning_enabled: true` 开放这次监督测试，标定来源 ID 为 `rmcs-dual-deploy-7add701a-v202-calibration110-reused-20261104`；默认研究片段仍关闭真机功能。

板号沿用当前 RMCS 的 `AF-CBB3-7053-32C5-F3B5-B460-A786-DA9A-1FF1`。仅这份新配置设置 `dangerously_skip_version_checks: true`，与当前主分支自 `524595dd` 起使用的板卡固件兼容选项一致；硬件代码的默认值仍为 `false`。本轮未移植主工作区其他 CAN 调度或超时修改。

## 构建和启动

以下是本机现有开发容器内的可复现命令。显式指定 `RMCS_PATH`，避免构建脚本默认编译 `/workspaces/RMCS` 的另一个分支。初始化仓库锁定的 FastTF 子模块并清理 CMake 缓存，避免复用其他工作区的依赖路径。本轮已执行构建与离线测试，**没有执行下面的真机启动命令**。

```bash
# 在 ROS Jazzy 开发容器内，进入 dev/wheel_leg_tradition 的检出目录后执行
export RMCS_PATH="$(git rev-parse --show-toplevel)"
git -C "$RMCS_PATH" submodule update --init -- rmcs_ws/src/fast_tf
source /opt/ros/jazzy/setup.bash
bash "$RMCS_PATH/.script/build-rmcs" --packages-up-to rmcs_core rmcs_bringup --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
source "$RMCS_PATH/rmcs_ws/install/local_setup.bash"
ros2 launch rmcs_bringup rmcs.launch.py robot:=wheel-leg-traditional-v202
```

在车载环境运行时，应使用该分支构建并安装的 `rmcs_core`、`rmcs_executor` 和 `rmcs_bringup`，并加载对应安装空间；只复制 YAML 到旧二进制安装空间无法加载新增插件。启动已有自动 respawn 机制，进程重启后仍必须重新完成下述遥控解锁时序。

## 操作顺序和输出

1. 遥控两侧均 DOWN，连续保持至少 1 s，清除参考、积分、故障与完成状态，输出为零。
2. 两侧均 MIDDLE，发起本次自起并请求 DM 就绪；此阶段仍为零恢复力矩。
3. 左 MIDDLE、右 UP，允许腿轨迹和轮平衡。DM 未就绪时继续保持零输出；就绪等待上限为 1 s。
4. 连续满足接稳门 0.5 s 后，自起释放，传统控制从实测短腿缓升到 `0.334 m`，保持前进速度 `0 m/s`、偏航速度 `0 rad/s`。`allow_motion_commands: false`，摇杆与旋钮不改变该测试指令。
5. 双 DOWN 立即撤力并复位；再次尝试需重新驻留与解锁。遥控失效、反馈过期或锁存故障同样撤力，不自动恢复动作。

接稳门仍检查倾角、角速度、四腿轴速、两轮速度和关节误差。末段落轮继续采用最后 20° 轨道弧内缩膝到 48° 的原轨迹，再恢复到约 0.245 m 的捕获姿态。48° 是大小腿内夹角，数值越小腿越短，不是电机编码器角度。

保护包含：反馈最老时间不超过 20 ms、实测腿轴超速 3.14 rad/s、腿电机 40 N·m、单轮 4.5 N·m、机械相对角范围、DM 就绪和遥控新鲜度。传统组件还检查实际控制间隔不超过 10 ms 及每次调度时间恰好推进 1 ms。软件命令限速不能阻止接触冲击造成的实际超速。

诊断通过 `ValueBroadcaster` 以 50 Hz 转发到 ROS：

| 接口 | 含义 |
| --- | --- |
| `/wheelleg/recovery/{active,ready,phase,fault,route,route_waypoint}` | 自起占用、完成、阶段、故障及路线 |
| `/wheelleg/traditional/{failure,armed,fault_latched}` | 传统控制状态与锁存故障 |
| `/wheelleg/traditional/{period_ms,feedback_age_ms,pitch,roll}` | 实际周期、反馈时延与姿态 |
| `/wheelleg/reference/leg_length` | 当前平滑腿长参考 |
| `/wheelleg/traditional/cad_position/<axis>` | 映射后的 CAD 编码器角 |
| `/wheel_leg/<axis>/{angle,velocity,control_torque}` | API 反馈与命令 |

常见传统错误码：`12` 为过期/无效反馈，`13` 为站立倾角越界，`15` 为启动姿态不匹配，`16` 为机械相对角越界，`17` 为控制周期异常，`18` 为 DM 未就绪/掉线，`19` 为自起故障或恢复输出无效。`19` 时同时查看自起 `fault`；自起 `3` 表示实测腿轴超速。

当前无裁判串口和赛场功率约束，150 W 仅是未标定的本地损耗模型预算。不要将该值当成电源实测或赛场功率合格证明。

## 验证证据与当前边界

本轮初始化锁定的 FastTF 子模块、清理 CMake 缓存，通过仓库 `.script/build-rmcs` 构建至 `rmcs_core` 和 `rmcs_bringup` 的全部 7 个包；FastTF、executor、description、msgs、utility 的解析路径均已指向本工作树的安装空间。最终 `colcon test --merge-install` 报告 9 个 CTest 目标全部通过（含 IMU 的 11 个用例，总计 20 个测试结果）。[构建、测试、来源与离线等价回执](validation/traditional_commissioning_20261104/README.md)保存在分支内。

新的插件集成测试加载实际生产 YAML、自起组件、传统组件和广播组件，以内存传感器替代硬件，不构造板卡、不打开 CAN。覆盖无轮许可零输出、DM 等待、接稳交接、MIT 伺服退出、独立整圈、零指令保持、过期反馈锁存、双 DOWN 重试、腿轴超速和遥控失效。

恢复内核/ABI/几何，以及传统内核/ABI/气簧表的 SHA256 与 2026-11-03 原生 Isaac 回执一致；新 YAML 的 28 个参数、7 行姿态表和 2065 行气簧力表逐项完全相同。将此前两个正向工况的各 20000 条站立传感器包分别输入旧、新构建库，全部 16 个 ABI 输出差值均为零，无控制器故障。此项是离线等价检查，不是新增物理测试。

此前原生 Isaac 已通过“30° 前倾倒地 → 自起”和“30° 前倾站姿失能摔倒 → 自起”，交接后均连续保持 20 s 零指令站立，净 XY 位移约 4.33 cm、4.02 cm。**180° 倒扣仍在冲击时触发超速故障 3，侧躺也未取得当前组合的完整动态资格。** 原生测试关闭连杆间自碰撞、云台固定，且本轮没有实车/CAN 输出；不能把构建、内存接线或离线回放通过当成实车已通过。完整原生复测报告在 `/home/noir/Documents/workspace/wheelleg_controller_tuning/docs/SELF_RIGHTING_RETEST_20261103.md`。
