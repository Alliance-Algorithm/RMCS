# Deformable Infantry Omni 主动悬挂控制流图

本文档描述当前 `deformable-infantry-omni` 配置下，底盘轮系控制与主动悬挂控制的实际数据流和控制流。

## 总体控制流

```text
                                           1 kHz 主循环
                                                │
                                                ▼
                               DeformableInfantryOmni::update()
                                                │
                     ┌──────────────────────────┴──────────────────────────┐
                     │                                                     │
                     ▼                                                     ▼
        BottomBoard 读取底盘 IMU / 关节反馈 / 轮速 / 电机状态         TopBoard 读取云台 IMU
                     │
                     ├─ 输出 /chassis/imu/pitch
                     ├─ 输出 /chassis/imu/roll
                     ├─ 输出 /chassis/imu/pitch_rate
                     ├─ 输出 /chassis/imu/roll_rate
                     ├─ 输出 /chassis/*_joint/physical_angle
                     ├─ 输出 /chassis/*_joint/physical_velocity
                     └─ 输出 /chassis/radius
                                                │
                                                ▼
                               DeformableChassis::update()
                                                │
                     ┌──────────────────────────┴──────────────────────────┐
                     │                                                     │
                     ▼                                                     ▼
             update_velocity_control()                           update_lift_angle_error()
                     │                                                     │
                     │                                                     ├─ 读取 4 条腿当前角度/角速度
                     │                                                     ├─ 生成 4 条腿目标展开角
                     │                                                     ├─ update_active_suspension()
                     │                                                     │
                     │                                                     │   if 反馈无效 或 腿未展开:
                     │                                                     │       suspension_mode = false
                     │                                                     │       suspension_torque = NaN
                     │                                                     │
                     │                                                     │   else:
                     │                                                     │       alpha_avg = mean(alpha_i)
                     │                                                     │       Fz     = mg/4 + Kz*(alpha_nom - alpha_avg)
                     │                                                     │       Fpitch = Kp*pitch + Dp*pitch_rate
                     │                                                     │              + m*ax_ff*h/(4*a^2)
                     │                                                     │       Froll  = Kr*roll + Dr*roll_rate
                     │                                                     │              + m*ay_ff*h/(4*b^2)
                     │                                                     │
                     │                                                     │       对每条腿:
                     │                                                     │       Ni = Fz + x_i*Fpitch + y_i*Froll
                     │                                                     │          + D_leg*(-alpha_dot_i)/max(sin(alpha_i), 0.1)
                     │                                                     │       Ni = max(Ni, 0)
                     │                                                     │       tau_i = Ni * L * sin(alpha_i)
                     │                                                     │       限幅到 torque_limit
                     │                                                     │
                     │                                                     ├─ update_joint_target_trajectory()
                     │                                                     └─ 输出:
                     │                                                        /chassis/*_joint/target_physical_angle
                     │                                                        /chassis/*_joint/target_physical_velocity
                     │                                                        /chassis/*_joint/suspension_mode
                     │                                                        /chassis/*_joint/suspension_torque
                     │
                     ├─ 平移速度 = 遥控/键盘输入 × 云台 yaw 旋转
                     ├─ 角速度 = AUTO / SPIN / STEP_DOWN / LAUNCH_RAMP
                     ├─ 控制加速度估计 ax_ff, ay_ff = diff(平移速度指令)
                     └─ 输出 /chassis/control_velocity
                                                │
                     ┌──────────────────────────┴──────────────────────────┐
                     │                                                     │
                     ▼                                                     ▼
                  4 个 DeformableJointController                 DeformableOmniWheelController
                     │                                                     │
                     │                                                     ├─ 读取 /chassis/control_velocity
                     │                                                     ├─ 读取轮速、/chassis/radius、功率限制
                     │                                                     ├─ 速度 PID -> 底盘期望力矩 [tau_r, tau_theta]
                     │                                                     ├─ QCP 约束:
                     │                                                     │   功率二次约束
                     │                                                     │   + 摩擦约束
                     │                                                     │   + 若 com_height > 0:
                     │                                                     │       用 gamma_i 构造 8 个半平面
                     │                                                     │   + 若半平面退化:
                     │                                                     │       回退旧菱形约束
                     │                                                     └─ 输出 /chassis/*_wheel/control_torque
                     │
                     ├─ 普通模式:
                     │   角度 PID + 速度 PID
                     │
                     └─ suspension_mode = true 时:
                         角度 PID + 速度 PID + suspension_torque 前馈
                         ↓
                         输出 /chassis/*_joint/control_torque
                                                │
                                                ▼
                                  BottomBoard::command_update()
                                                │
                     ┌──────────────────────────┴──────────────────────────┐
                     │                                                     │
                     ▼                                                     ▼
                           4 个悬挂关节电机扭矩下发                         4 个轮子电机扭矩下发
```

## 主动悬挂子流程

```text
底盘 IMU(pitch, roll, pitch_rate, roll_rate)
        + 4 条腿 physical_angle / physical_velocity
        + 平移控制加速度估计 ax_ff / ay_ff
        │
        ▼
DeformableChassis::update_active_suspension()
        │
        ├─ 检查四条腿是否都已进入展开工况
        ├─ 计算平均腿角 alpha_avg
        ├─ 计算高度模态 Fz
        ├─ 计算俯仰模态 Fpitch
        ├─ 计算横滚模态 Froll
        ├─ 对 4 条腿分配法向力 Ni
        ├─ 法向力转关节力矩 tau_i
        └─ 输出 /chassis/*_joint/suspension_torque
                │
                ▼
DeformableJointController
        │
        ├─ suspension_mode = false:
        │   普通角度/速度闭环
        │
        └─ suspension_mode = true:
            普通闭环 + suspension_torque 扭矩前馈
                │
                ▼
        /chassis/*_joint/control_torque
```

## 控制职责拆分

- `DeformableChassis`
  - 生成底盘速度指令
  - 生成 4 条腿展开目标
  - 计算主动悬挂模态力和悬挂力矩前馈

- `DeformableJointController`
  - 将关节目标角、目标速度、悬挂前馈力矩组合成最终关节控制力矩

- `DeformableOmniWheelController`
  - 将底盘速度误差转换为底盘等效控制力矩
  - 通过 QCP 同时满足功率约束和摩擦约束
  - 输出 4 个轮子的控制力矩

- `BottomBoard`
  - 提供底盘 IMU 与关节反馈
  - 接收轮子与悬挂关节的最终控制力矩并下发电机

## 关键源码入口

- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_joint_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_omni_wheel_controller.cpp`
- `rmcs_ws/src/rmcs_core/src/controller/chassis/qcp_solver.hpp`
- `rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-omni.cpp`
