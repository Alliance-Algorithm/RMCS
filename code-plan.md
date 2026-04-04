
  # 改进 DeformableWheelController：每轮独立关节状态 + 正确的非对称半径运动学

  ## Summary

  - 当前草案需要修正两点：不要让 rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_wheel_controller.cpp 直接读取原始 joint 电机角度；也不要在 R_i 非对称后继续沿用现在的 vx/vy 平均观测公式。
  - 硬件相关的角度标定和方向统一继续放在 rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp；wheel controller 只消费“几何意义正确”的每轮 joint 状态。
  - 本次实现以当前分支的 V2 运行链路为主，但接口设计保持可迁移到 V1/V2，不把 legacy/V2 差异塞进 wheel controller。

  ## Key Changes

  - 在 rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_chassis.cpp 新增 8 个输出接口：/chassis/<wheel>_joint/geometry_angle 和 /chassis/<wheel>_joint/geometry_velocity，<wheel> 为 left_front、
    left_back、right_back、right_front，单位分别固定为 rad 和 rad/s。
  - 几何接口由 DeformableChassis 用现有 joint_angle_deg() 统一生成，再转成 rad，并通过 4 个 AlphaBetaAngleFilter 估计平滑角度和角速度；/chassis/processed_encoder/angle 保留为四轮几何角平均值的兼容/调试
    输出，不再作为主控制输入。
  - 在 rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_wheel_controller.cpp 移除单一 /chassis/processed_encoder/angle 输入和 EncoderState，改为 JointStates { alpha_rad, alpha_dot_rad, radius,
    valid }，由四轮 geometry_angle/geometry_velocity 填充。
  - 将 vehicle_radius_ 改为 Eigen::Vector4d vehicle_radii_，按 R_i = chassis_radius_ + rod_length_ * cos(alpha_i) 逐轮计算。
  - 将机械补偿速度改为逐轮 v_mech_i = -rod_length_ * sin(alpha_i) * alpha_dot_i，逐元素限幅后再投影到径向方向。
  - 重写 calculate_chassis_velocity()：先计算每轮接地点速度 u_i = wheel_radius_ * (omega_i - omega_mech_i) * [cos(zeta_i), sin(zeta_i)]，再用 8 个方程 u_ix = vx - wz R_i sin(varphi_i)、u_iy = vy + wz
    R_i cos(varphi_i) 做 8x3 最小二乘求解 [vx, vy, wz]，彻底消除对向轮半径不等时 yaw 项泄漏到 vx/vy 的问题。
  - calculate_chassis_status_expected()、calculate_wheel_control_torques()、calculate_steering_control_torques()、calculate_ellipse_parameters() 中所有依赖 vehicle_radius_ 的位置统一改成逐轮 R_i，不再保
    留标量半径分支。
  - 菱形打滑约束继续保留现有近似模型，但 rhombus_top 改为使用 mean(vehicle_radii_)，本轮不扩展为更大的约束模型重写。
  - 删除 controller 内部单个 processed_encoder_ab_filter_ 和 1kHz 的 RCLCPP_INFO 半径日志；/chassis/encoder/alpha、/chassis/encoder/alpha_dot、/chassis/radius 保留为四轮平均调试输出，避免现有可视化面板
    立刻失效。
  - 任意一轮 geometry_angle/geometry_velocity 当周期非有限时，wheel controller 当周期回退到名义对称模型：vehicle_radii_ = chassis_radius_ + rod_length_、v_mech = 0、调试输出给出名义值，控制输出必须保持
    有限且不复用陈旧非对称半径。

  ## Test Plan

  - build-rmcs --packages-select rmcs_core 编译通过，且不需要修改 component 列表或 YAML 连线。
  - 四轮几何角和几何角速度完全相同时，新实现的底盘观测和力矩分配与当前平均角版本在浮点容差内一致。
  - 非对称静态工况下，前后或左右轮几何角不同但几何角速度为 0，纯自旋指令不应被观测成虚假的 vx 或 vy。
  - 非对称动态工况下，四轮几何角速度不同，v_mech_i 应逐轮变化，轮速补偿和舵向控制输出保持有限且方向一致。
  - 将任意一轮几何输入置为 NaN 时，controller 应回退到名义对称模型，而不是继续使用旧的非对称半径。

  ## Assumptions

  - 当前机构的变形只改变轮心到底盘中心的径向距离 R_i，不改变轮位方向 varphi = {0, pi/2, pi, 3pi/2} 和轮序。
  - 原始 joint 反馈到几何角的映射关系仍由 DeformableChassis 负责，DeformableWheelController 不重复实现 V1/V2 标定逻辑。
  - 改完后不会再有其他组件消费 /chassis/processed_encoder/angle 作为主闭环输入；它只作为兼容/调试输出保留。


