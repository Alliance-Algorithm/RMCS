# Wheel-leg 实车 IMU 与 RL 观测

## 当前约定：训练 IMU 使用 Body

用户已澄清：**RL 训练的 IMU 观测就是车身 Body xyz**，与实车板载 IMU 安装轴一致。URDF/CAD 坐标的朝向不用于推导 IMU 观测的换轴。此前采用的 `[y,-x,z]` 90° 转换已移除。

| 观测轴 | 实车 Body 轴 |
| --- | --- |
| RL IMU x | Body x（车头） |
| RL IMU y | Body y（左侧） |
| RL IMU z | Body z（上方） |

`WheelLegRlImu` 保留 RL 观测接口，负责检查输入、直传 Body 角速度和计算 Body 重力投影；它没有滤波、累计角度、校零或模式状态。

## 数据链路

| 环节 | 使用的坐标与处理 |
| --- | --- |
| `WheelLegInfantryRL` → `Bmi088Ekf` | 安装矩阵为单位阵；加速度和角速度按板卡 xyz 换算单位 |
| `/wheel_leg/imu/quaternion` | EKF 的 `q_WB`，表示 Body→world |
| `/wheel_leg/imu/angular_velocity` | Body 角速度，rad/s |
| `WheelLegChassisController` | 使用 Body 四元数，以 `q_WB * [1,0,0]` 的水平投影求车头航向 |
| `/wheel_leg/rl/imu/angular_velocity` | 直接复制 Body 角速度 |
| `/wheel_leg/rl/imu/projected_gravity` | 世界单位重力在 Body 系中的分量 |

```text
omega_RL = omega_Body
g_RL     = inverse(normalize(q_WB)) * [0,0,-1]
```

这里的逆旋转用于把世界重力投影到 Body；没有附加的 ±90° 安装旋转。不能用 `q_WB * [0,0,-1]` 替代它。

35 维策略观测中的第 4–6 项（从 0 计数）为 `0.5 * omega_Body`，第 7–9 项为 `g_Body`。例如 Body 角速度 `[1,2,3]` 在 RL 接口仍是 `[1,2,3]`，策略观测为 `[0.5,1,1.5]`。策略不使用线加速度或完整四元数作为观测。

两个 YAML 都加载 `rmcs_core::controller::chassis::WheelLegRlImu -> wheel_leg_rl_imu`，并通过 `take=vec3` 读取 RL 角速度与重力接口：

- `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml`
- `rmcs_ws/src/rmcs_rl/config/executor.yaml`

通用 `RlBridge` 的 `transform=projected_gravity` 按输入四元数直接乘重力；本车使用上面的向量接口，绕开该分支。无效输入会令两路 RL 输出为 NaN，避免沿用旧的有效观测。`/chassis/reset_count` 继续由桥清除观测历史及动作状态。

## 遥测与姿态查看器

`/wheel_leg/telemetry/imu_body` 保留 Body 四元数和角速度；`rl_projected_gravity`、`rl_angular_velocity` 也表达 Body 分量。话题名和 `frame_id=rl_base` 保留，但 `chassis_body → rl_base` 的 TF 为**单位变换**。

MuJoCo 画出的 **RL IMU** 三轴与 Body 三轴同向。图中 origin 的位置分开是为了看清标签。已归一化的 V5 模型根姿态使用 `q_WB`；模型与网格的几何导出变换不应附加到 IMU 观测。查看器独立比较 `gyro_RL == gyro_Body` 和 `g_RL == inverse(q_WB)*down`，旧版 90° 换轴数据会显示为误差。

## 本地验证

`wheel_leg_rl_imu_test` 使用生产 EKF、BMI088 采样封装、RL 观测组件及 chassis 控制器，覆盖：

- 三轴及混合角速度在 chassis/RL 两路保持一致。
- 水平、横滚、俯仰、组合倾斜、侧立与倒置的 Body 重力投影。
- 世界 yaw、四元数整体符号和归一化不改变应有的重力投影。
- 无效输入及恢复、量化板卡采样、双下后 Body 车头参考航向。
- 12 个已知姿态与 Isaac `ArticulationData` 的实际输出对照。

```sh
colcon build --merge-install --packages-select rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_core --ctest-args -R wheel_leg_rl_imu_test
```

Isaac 对照脚本将观察坐标直接设为给定 Body 姿态，读取 PhysX 的实际根位姿、`projected_gravity_b` 和 `root_ang_vel_b`。CSV 提供 C++ 测试的数值参考；JSON 记录安装版本、源码指纹、坐标约定和误差。

| 已知 Body 姿态 | 预期 RL 重力 |
| --- | --- |
| 水平 | `[0,0,-1]` |
| 绕 +x 横滚 +30° | `[0,-0.5,-0.866025]` |
| 绕 +y 俯仰 +30° | `[0.5,0,-0.866025]` |

重新生成参考的命令：

```sh
OMNI_KIT_ACCEPT_EULA=YES /home/noir/miniconda3/envs/isaaclab/bin/python rmcs_ws/src/rmcs_core/tool/validate_wheel_leg_rl_imu.py --usd /home/noir/Documents/workspace/example/wheeled-legged_RL/source/agent_world/agent_world/assets/usd_files/Wheel_leg_V1/Wheel_leg_V1.usd --output docs/zh-cn/wheel_leg_rl_imu_isaac_validation.json --reference-csv rmcs_ws/src/rmcs_core/test/data/wheel_leg_rl_imu_isaac.csv --device cpu
```

此前带 90° 安装矩阵的对照只验证了那一假设下的软件一致性；不能证明训练观测必须换轴。当前参考按用户确认的 Body 观测约定重新生成，未修改训练代码或 USD。仿真与软件测试不等于实车姿态对照。

本次验证：`rmcs_core`、`rmcs_bringup` 完整构建通过；C++ IMU/chassis 测试 7 项、查看器测试 7 项均通过；两份 YAML 的 ONNX 契约与有限值推理通过。12 个 Isaac 姿态与 C++ 输出对照的重力最大分量误差为 `4.76837e-7`，角速度最大分量误差为 `1.14441e-6 rad/s`。

## 部署

重新编译并重启实车 RMCS，以更新观测组件和遥测 TF；同步更新本机查看器。观测接口路径、维数和缩放保持一致，ONNX 图及权重无需修改：

- `layout_hash = 0xec4b460a8c38acc7`
- `model_id = 0xd4612f6cd48a6e9c`

两份 YAML 可分别用 `rmcs_ws/src/rmcs_rl/tool/check_policy_contract.py` 校验。布局哈希只描述接口布局，不能验证接口数值的坐标语义，因此部署时必须包含修正后的 `rmcs_core`。
