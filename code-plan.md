实施 Checklist
阶段 0：先统一语义
1. 明确 wheel-demo.cpp 的几何反馈量只认：
   - /chassis/*_joint/physical_angle
   - /chassis/*_joint/physical_velocity
2. 明确新增目标量接口的语义是：
   - target_physical_*
   - 不是 motor angle
   - 不是 encoder angle
   - 不是中间控制变量
3. 确认 /chassis/control_velocity 当前上游坐标系语义：
   - 是否仍沿用旧控制器的 -pi/4 约定
   - 还是已经是 base_link 物理坐标
阶段 1：给 DeformableChassis 增加目标几何输出
1. 在 DeformableChassis 中新增四个输出成员：
   - left_front_joint_target_physical_angle_
   - left_back_joint_target_physical_angle_
   - right_back_joint_target_physical_angle_
   - right_front_joint_target_physical_angle_
2. 增加 register_output(...)：
   - /chassis/left_front_joint/target_physical_angle
   - /chassis/left_back_joint/target_physical_angle
   - /chassis/right_back_joint/target_physical_angle
   - /chassis/right_front_joint/target_physical_angle
3. 梳理 lf_current_target_angle_ / lb_current_target_angle_ / rb_current_target_angle_ / rf_current_target_angle_ 的单位和语义：
   - 确认是否已经是物理角
   - 如果不是，先补一层映射再输出
4. 在 update_lift_target_toggle() / update_lift_angle_error() 所在链路中，保证每帧都更新这些 target output
5. 验证 target angle 在以下模式下是否正确：
   - 普通升降
   - front_high_rear_low
   - front_low_rear_high
   - uphill
阶段 2：在 wheel-demo.cpp 增加目标几何输入
1. 新增四个输入成员：
   - left_front_joint_target_physical_angle_
   - left_back_joint_target_physical_angle_
   - right_back_joint_target_physical_angle_
   - right_front_joint_target_physical_angle_
2. 增加对应 register_input(...)：
   - /chassis/left_front_joint/target_physical_angle
   - /chassis/left_back_joint/target_physical_angle
   - /chassis/right_back_joint/target_physical_angle
   - /chassis/right_front_joint/target_physical_angle
3. 保留当前 feedback 输入不变：
   - /chassis/*_joint/physical_angle
   - /chassis/*_joint/physical_velocity
阶段 3：重构 joint 状态结构
1. 将当前 JointStates 拆成两套：
   - JointFeedbackStates
   - JointTargetStates
2. JointFeedbackStates 包含：
   - alpha
   - alpha_dot
   - alpha_ddot
   - radius
   - radius_dot
   - radius_ddot
   - valid
3. JointTargetStates 包含：
   - alpha
   - alpha_dot
   - alpha_ddot
   - radius
   - radius_dot
   - radius_ddot
   - has_velocity
   - has_acceleration
   - valid
4. 抽一个共享几何 helper：
   - 输入 alpha / alpha_dot / alpha_ddot
   - 输出 radius / radius_dot / radius_ddot
阶段 4：实现 feedback 状态更新
1. 保留现有 update_joint_states_() 思路，但改名为：
   - update_joint_feedback_states_()
2. 继续从 feedback 接口读取：
   - physical_angle
   - physical_velocity
3. 继续在这里计算：
   - R
   - dot(R)
   - ddot(R)
4. ddot(R) 第一版可继续用反馈速度差分
5. reset 时清理 feedback 差分缓存
阶段 5：实现 target 状态更新
1. 新增：
   - update_joint_target_states_()
2. 第一版只读取：
   - target_physical_angle
3. 如果还没有 target_velocity：
   - 用相邻两帧 target_angle 差分估计 target_alpha_dot
4. 第一版建议：
   - target_alpha_ddot = 0
   - target_radius_ddot = 0
5. 由 target 几何计算：
   - R*
   - dot(R)*
   - ddot(R)*
6. reset 时清理 target 差分缓存：
   - last_joint_target_angle_
   - last_joint_target_velocity_
阶段 6：函数级切换到 feedback / target
1. 保持使用 feedback：
   - update_joint_feedback_states_()
   - calculate_chassis_velocity(...)
   - debug 输出
2. 改成使用 target：
   - calculate_chassis_status_expected(...)
   - calculate_steering_control_torques(...)
3. 间接受 target 影响：
   - calculate_wheel_pid_torques(...)
   - 因为它依赖 chassis_status_expected
4. 第一阶段可暂时不动或只部分切换：
   - calculate_wheel_control_torques(...)
   - calculate_ellipse_parameters(...)
   - constrain_chassis_control_acceleration(...)
阶段 7：修正 calculate_chassis_status_expected(...)
1. 输入从 joint_feedback 改为 joint_target
2. 每轮目标轮心速度改为：
   - v_i^* = v^* + wz^* R_i^* e_t,i + dot(R_i)^* e_r,i
3. 生成：
   - wheel_velocity_x
   - wheel_velocity_y
4. 检查这里是否还需要受 chassis_velocity_expected_ 的能量裁剪影响
5. 验证：
   - 目标角变化时，轮心速度能否提前响应，而不是等反馈变化后才响应
阶段 8：修正 calculate_steering_control_torques(...)
1. 输入改成 joint_target
2. 目标角 zeta_i^* 使用 target wheel velocity
3. 目标舵速 dot(zeta_i)^* 使用：
   - R_i^*
   - dot(R_i)^*
   - 第一版可不使用 ddot(R_i)^* 或直接置 0
4. 保留现有二环结构：
   - angle PID
   - velocity PID
5. 验证：
   - 关节快速动作时，舵向是否不再慢半拍
阶段 9：检查 control_velocity 坐标映射
1. 对照旧 deformable_wheel_controller 的：
   - Rotation2Dd(-pi/4)
2. 明确当前新模型是否仍需该旋转
3. 用最简单工况验证：
   - 纯前进
   - 纯左移
   - 原地旋转
4. 若出现整体 45 deg 偏移，再决定是否保留坐标变换
阶段 10：编译与最小运行验证
1. 编译 rmcs_core
2. 确认 plugin 正常注册
3. 在 yaml 中仅替换组件类型，确保无需额外链路
4. 检查 DAG 是否缺接口
5. 先看目标接口是否已连通：
   - target_physical_angle 是否可达 wheel-demo.cpp
阶段 11：第一轮行为验证
1. 对称升降姿态：
   - 新旧控制器行为是否接近
2. 前高后低：
   - 目标舵向是否明显比旧控制器更一致
3. 变形过程中小平移：
   - 是否减少“轮/舵慢半拍”
4. 旋转叠加变形：
   - 是否仍稳定
5. 记录异常：
   - 舵抖
   - 轮速抖
   - 控制方向偏转
   - 功率异常
阶段 12：第二阶段补 target_velocity
1. 在 DeformableChassis 新增输出：
   - /chassis/*_joint/target_physical_velocity
2. 在 wheel-demo.cpp 新增输入并读取
3. 不再用差分 target_angle 估计 dot(alpha)^*
4. 更新：
   - target.radius_dot
5. 再验证舵速前馈是否更平滑
阶段 13：第三阶段补 target_acceleration
1. 在 DeformableChassis 新增输出：
   - /chassis/*_joint/target_physical_acceleration
2. wheel-demo.cpp 新增输入
3. 完整启用：
   - target.radius_ddot
4. 让 calculate_steering_control_torques(...) 使用完整：
   - ddot(R_i)^*
5. 再决定是否把轮速前馈也升级到 dot(omega_i)^*
阶段 14：约束层完整化
1. 先确认主控制链稳定
2. 再重推：
   - calculate_ellipse_parameters(...)
   - constrain_chassis_control_acceleration(...)
3. 逐步从“旧约束近似”迁移到：
   - 基于独立轮 target 几何的约束
4. 后续再考虑：
   - 轮功率
   - 关节功率
   - 摩擦约束
   - 法向载荷模型
建议执行顺序
1. 先做 target_physical_angle
2. 再把 calculate_chassis_status_expected(...) 和 calculate_steering_control_torques(...) 切到 target
3. 编译、跑通、验证
4. 再补 target_physical_velocity
5. 最后补 target_physical_acceleration 和完整约束
