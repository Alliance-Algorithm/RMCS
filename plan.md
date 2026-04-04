# 计划：DeformableChassisController 支持每轮独立关节角度

## 背景

当前 `DeformableChassisController` 使用**单个平均关节角度**（`processed_encoder_angle_`，由 `DeformableChassis` 将4个关节角取平均后输出），导致：
- 4个轮共享一个 `vehicle_radius_`（轮距 R）
- 4个轮的 `v_mech` 只有方向不同，幅值完全相同

实际上每个关节电机独立运动（特别是在前高后低、下台阶、上坡等模式下），V2 硬件已通过 LK MG5010Ei36 电机提供每轮独立的角度和角速度反馈。本次重构让底盘控制器读取每轮独立的关节角，获得正确的轮距 R_i 和速度补偿 v_mech_i。

### v_mech 方向分析

当前 v_mech 的**方向是正确的**：每个轮的机械速度沿径向（从底盘中心指向轮心方向 phi_i）。问题在于**幅值**：4个轮共享同一个 `v = -rod_length * sin(alpha_avg) * alpha_dot_avg`。改为每轮独立后，每轮有自己的 `v_i = -rod_length * sin(alpha_i) * alpha_dot_i`。

## 修改文件

仅修改一个文件：
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_wheel_controller.cpp`

## 实现步骤

### 第1步：添加每轮关节输入

移除单个 `processed_encoder_angle_` 输入，替换为8个输入（4个角度 + 4个角速度）：

```cpp
// 移除：
InputInterface<double> processed_encoder_angle_;

// 添加：
InputInterface<double> left_front_joint_angle_;
InputInterface<double> left_back_joint_angle_;
InputInterface<double> right_back_joint_angle_;
InputInterface<double> right_front_joint_angle_;

InputInterface<double> left_front_joint_velocity_;
InputInterface<double> left_back_joint_velocity_;
InputInterface<double> right_back_joint_velocity_;
InputInterface<double> right_front_joint_velocity_;
```

在构造函数中注册，读取硬件接口：
- `/chassis/left_front_joint/angle`、`/chassis/left_front_joint/velocity`
- `/chassis/left_back_joint/angle`、`/chassis/left_back_joint/velocity`
- `/chassis/right_back_joint/angle`、`/chassis/right_back_joint/velocity`
- `/chassis/right_front_joint/angle`、`/chassis/right_front_joint/velocity`

### 第2步：用每轮 JointStates 替换单个 EncoderState

```cpp
struct JointStates {
    Eigen::Vector4d alpha_rad;      // 每轮关节角度 [rad]
    Eigen::Vector4d alpha_dot_rad;  // 每轮关节角速度 [rad/s]
    Eigen::Vector4d radius;         // 每轮 R_i = chassis_radius + rod_length * cos(alpha_i)
    bool valid = false;
};
```

移除 `update_processed_encoder_state_()` 和单个 `AlphaBetaAngleFilter`。可选择使用4个独立的滤波器，或直接使用电机输出值（LK电机内置编码器精度足够）。

### 第3步：每轮独立轮距 R_i

将 `double vehicle_radius_` 替换为 `Eigen::Vector4d vehicle_radii_`：

```cpp
vehicle_radii_[i] = chassis_radius_ + rod_length_ * cos(alpha_i);
```

### 第4步：每轮独立 v_mech

更新 `calculate_mech_wheel_velocity()`，计算每轮独立的径向速度幅值：

```cpp
Eigen::Matrix<double, 4, 2> calculate_mech_wheel_velocity(const JointStates& joints) const {
    Eigen::Matrix<double, 4, 2> v_mech = Eigen::Matrix<double, 4, 2>::Zero();
    if (!joints.valid) return v_mech;

    // 每轮径向速度：v_i = -rod_length * sin(alpha_i) * alpha_dot_i
    Eigen::Vector4d v = -rod_length_ * joints.alpha_rad.array().sin()
                                      * joints.alpha_dot_rad.array();
    v = v.cwiseMax(-0.1).cwiseMin(0.1);  // 逐元素限幅

    // 方向为径向 (cos(phi_i), sin(phi_i))，这是正确的
    v_mech.col(0) = v.array() * cos_varphi_.array();
    v_mech.col(1) = v.array() * sin_varphi_.array();
    return v_mech;
}
```

### 第5步：底盘速度观测（每轮 R_i 加权）

更新 `calculate_chassis_velocity()`。

对于 phi = {0, pi/2, pi, 3pi/2} 的布局：

- **dx0 和 dy0 不受 R_i 影响**（因为对应轮的 sin(phi) 或 cos(phi) 为0）：
  - `dx0 = r/4 * sum(omega_eff_i * cos(zeta_i))`（不变）
  - `dy0 = r/4 * sum(omega_eff_i * sin(zeta_i))`（不变）

- **dtheta0 使用每轮 1/R_i 加权**：

  ```cpp
  velocity.z() = -(one_quarter_r) *
      (-wheel_eff[0] * sin_angle[0] / vehicle_radii_[0]
       + wheel_eff[1] * cos_angle[1] / vehicle_radii_[1]
       + wheel_eff[2] * sin_angle[2] / vehicle_radii_[2]
       - wheel_eff[3] * cos_angle[3] / vehicle_radii_[3]);
  ```

  推导：对每个轮 i 可独立估计 dtheta0，取加权平均：
  - 轮1 (phi=0): `dtheta0 = (u_1y - dy0) / R_1`
  - 轮2 (phi=pi/2): `dtheta0 = (dx0 - u_2x) / R_2`
  - 轮3 (phi=pi): `dtheta0 = (dy0 - u_3y) / R_3`
  - 轮4 (phi=3pi/2): `dtheta0 = (u_4x - dx0) / R_4`

### 第6步：期望轮速（每轮 R_i）

更新 `calculate_chassis_status_expected()`：

```cpp
// 每轮使用各自的 R_i
chassis_status.wheel_velocity_x =
    (vx - vehicle_radii_.array() * vz * sin_varphi_.array()).matrix() + v_mech.col(0);
chassis_status.wheel_velocity_y =
    (vy + vehicle_radii_.array() * vz * cos_varphi_.array()).matrix() + v_mech.col(1);
```

### 第7步：轮电机力矩（每轮 R_i）

更新 `calculate_wheel_control_torques()`：

```cpp
Eigen::Vector4d wheel_torques =
    wheel_radius_
    * (ax * mess_ * steering_status.cos_angle.array()
       + ay * mess_ * steering_status.sin_angle.array()
       + az * moment_of_inertia_
             * (cos_varphi_.array() * steering_status.sin_angle.array()
                - sin_varphi_.array() * steering_status.cos_angle.array())
             / vehicle_radii_.array())  // <-- 每轮 R_i
    / 4.0;
```

### 第8步：舵电机控制（每轮 R_i）

更新 `calculate_steering_control_torques()`：

```cpp
// 舵向控制速度公式中使用每轮 R_i
steering_control_velocities =
    vx * ay - vy * ax - vz * (vx * vx + vy * vy)
    + vehicle_radii_.array() * (az * vx - vz * (ax + vz * vy)) * cos_varphi_.array()
    + vehicle_radii_.array() * (az * vy - vz * (ay - vz * vx)) * sin_varphi_.array();

// 奇异点处理也用每轮 R_i
auto x = ax - vehicle_radii_[i] * (az * sin_varphi_[i] + 0 * cos_varphi_[i]);
auto y = ay + vehicle_radii_[i] * (az * cos_varphi_[i] - 0 * sin_varphi_[i]);
```

### 第9步：QCP 功率约束参数（每轮 R_i）

更新 `calculate_ellipse_parameters()`，将所有 `vehicle_radius_` 替换为每轮 `vehicle_radii_`：

```cpp
// b 系数：每轮 1/R_i
b = (k1_ * mess_ * moment_of_inertia_ * wheel_radius_ * wheel_radius_ / 8.0)
  * angular_acceleration_direction
  * (cos_alpha_minus_gamma.array() * sin_alpha_minus_varphi.array()
     / vehicle_radii_.array()).sum();

// c 系数：每轮 1/R_i^2
c = (k1_ * moment_of_inertia_ * moment_of_inertia_ * wheel_radius_ * wheel_radius_ / 16.0)
  * (sin_alpha_minus_varphi.array().square() / vehicle_radii_.array().square()).sum();

// e 系数：每轮 1/R_i
e = (moment_of_inertia_ * wheel_radius_ / 4.0)
  * angular_acceleration_direction
  * (double_k1_torque_base_plus_wheel_velocities.array()
     * sin_alpha_minus_varphi.array() / vehicle_radii_.array()).sum();
```

### 第10步：菱形打滑约束（使用平均 R）

```cpp
// 菱形约束使用平均 R 作为保守近似
const double avg_radius = vehicle_radii_.mean();
const double rhombus_top = rhombus_right * mess_ * avg_radius / moment_of_inertia_;
```

### 第11步：清理旧代码

- 移除 `AlphaBetaAngleFilter processed_encoder_ab_filter_`
- 移除 `EncoderState` 结构体和 `update_processed_encoder_state_()`
- 移除或保留 `encoder_alpha_`、`encoder_alpha_dot_`、`radius_` 输出接口（可保留 `radius_` 输出平均值用于调试）

## 变量替换汇总

| 旧（单个值） | 新（每轮独立） |
|---|---|
| `double vehicle_radius_` | `Eigen::Vector4d vehicle_radii_` |
| `processed_encoder_angle_` 输入 | 4x 关节角度 + 4x 关节角速度 输入 |
| `EncoderState` | `JointStates`（Vector4d 成员） |
| `v = -rod_length * sin(α) * α̇`（标量） | `v_i = -rod_length * sin(α_i) * α̇_i`（Vector4d） |
| `/ vehicle_radius_` | `/ vehicle_radii_.array()` 或 `/ vehicle_radii_[i]` |

## 验证方式

1. `build-rmcs --packages-select rmcs_core` —— 编译通过
2. 检查 `deformable-infantry-v2.yaml` 中接口名是否匹配
3. 当4个关节角相同时，行为应与当前单编码器版本完全一致（回归测试）
4. 非对称关节运动时（如前高后低），底盘不应漂移或出现虚假速度
