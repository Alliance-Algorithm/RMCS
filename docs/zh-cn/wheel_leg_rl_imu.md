# Wheel-leg 实车 IMU 与 RL 坐标

板载 IMU 的 xyz 与实车车身 xyz 一致。训练用的 base/IMU 坐标与车身的关系是：

| RL 分量 | 车身分量 |
| --- | --- |
| x | y |
| y | -x |
| z | z |

`WheelLegRlImu` 在 RMCS 内部接口上转换传给 RL 的数据。它没有滤波、累计角度、校零状态或模式状态；每拍只使用本拍输入计算输出。原车身 IMU 接口继续供 RMCS 控制器读取。

## 数据链路

- `/wheel_leg/imu/angular_velocity`：车身系角速度，rad/s。
- `/wheel_leg/imu/quaternion`：EKF 的 `q_WB`，表示车身向量到世界系的旋转。
- `/wheel_leg/rl/imu/angular_velocity`：`[omega_body.y, -omega_body.x, omega_body.z]`，rad/s。
- `/wheel_leg/rl/imu/projected_gravity`：世界单位重力在 RL 坐标中的分量。

重力计算为：

```text
g_body = inverse(q_WB) * [0, 0, -1]
g_RL   = [g_body.y, -g_body.x, g_body.z]
```

使用归一化四元数的共轭实现逆旋转。不能直接把 EKF 的 `q_WB` 作为世界到车身旋转使用，也不能只交换角速度分量而让重力投影继续使用车身坐标。

35 维策略观测中的第 4–6 项（从 0 计数）为 `0.5 * omega_RL`，第 7–9 项为 `g_RL`。例如车身角速度 `[1,2,3]` 对应 RL 接口 `[2,-1,3]`、策略观测 `[1,-0.5,1.5]`；车身水平时重力仍为 `[0,0,-1]`。

此策略不使用线加速度或完整四元数作为观测。转轴关系只作用于上述 IMU 观测，遥控命令的语义仍由底盘命令组件定义。

## 部署

两个配置文件都加载 `rmcs_core::controller::chassis::WheelLegRlImu -> wheel_leg_rl_imu`，并让 `rl_bridge` 读取上述 RL 接口：

- `rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml`
- `rmcs_ws/src/rmcs_rl/config/executor.yaml`

转换组件没有 ROS 参数。`/chassis/reset_count` 继续由现有 RL 桥清除观测历史和动作状态；坐标转换本身没有需要复位的历史。

接口路径改变后，`wheel_leg_v1.onnx` 的布局元数据已同步更新：

- `layout_hash = 0xec4b460a8c38acc7`
- `model_id = 0xd4612f6cd48a6e9c`

ONNX 的计算图和权重保持原样。部署时须一起更新组件、YAML 和 ONNX 文件，重新编译并重启 RMCS，避免新旧布局契约混用。

## 离线验证

`wheel_leg_rl_imu_test` 使用生产转换组件和生产 EKF，覆盖角速度三轴正方向、保持原始控制数据、水平/倾斜/侧立/倒置重力、世界航向不变性、四元数符号和归一化、无效输入及恢复。测试不连接板卡。

构建及测试：

```sh
colcon build --merge-install --packages-select rmcs_core rmcs_rl rmcs_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --merge-install --packages-select rmcs_core --ctest-args -R wheel_leg_rl_imu_test
```

模型契约使用 `rmcs_ws/src/rmcs_rl/tool/check_policy_contract.py` 分别对照两份 YAML 检查，并验证图与权重未发生变化。上述检查验证软件坐标链路，不代表已在实车上验证 RL 闭环效果。

本次验证结果：三个包构建完成；`wheel_leg_rl_imu_test` 的四项测试通过；两份配置的契约检查和 ONNX 有限值推理通过。ONNX 计算图（含权重）的序列化 SHA-256 为 `87a22cd557f2cf59dbdfae9b829f9affc589f900edcea7e4977f455a5b525d5a`，修改前后一致，仅布局描述和布局哈希两项元数据变化。
