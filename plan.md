 # Merge dev/sp-vision Into deformable-infantry With Deformable-Specific Retention

  ## 摘要

  - 以 deformable-infantry 为集成基线，合并 dev/sp-vision@4d95fc8，冲突默认取 dev/sp-vision。
  - 唯一明确的例外是 deformable 专属行为：保留 DeformableInfantry 硬件与底盘特性、保留 DjiMotor 的 encoder_angle 输出语义与使用链路、保留 deformable 的控制链入口名与机器人参数。
  - 云台直接沿用 OmniInfantryPlannerGimbalController 作为最终实现
  - 以 dev/sp-vision 的接口命名为准，引入 rmcs_ws/src/sp_vision_25、/gimbal/hard_sync_snapshot、/gimbal/auto_aim/* 规划接口和 vendored librmcs SDK。

  ## 实现变更

  - 基础合并策略：
      - 在干净 worktree / 新集成分支上执行，不在当前脏工作区直接操作。
      - 以 merge/deformable-infantry@cfb2026 作为已验证冲突解法参考，再补齐 dev/sp-vision 从 452f9f3 到 4d95fc8 的剩余增量。
      - 对冲突文件默认采用 dev/sp-vision 内容，再手工恢复 deformable 例外项。
  - 构建与仓库结构：
      - 更新 .gitmodules，移除 rmcs_ws/src/rmcs_core/librmcs 子模块项，新增 rmcs_ws/src/sp_vision_25。
      - 保留 dev/sp-vision 的 rmcs_ws/src/rmcs_core/CMakeLists.txt 方向，使用根目录 vendored SDK librmcs-sdk-src-3.0.1-0.dev.4.gbaf538b。
      - 合入 rmcs_msgs::HardSyncSnapshot，并保持 sp_vision_25 的 plugin 注册可用。
      - docker-compose.yml、.env 仍按目标分支开发环境语义保留 ${CONTAINER_USER}、${CONTAINER_HOME} 和当前 nvim 挂载方式，不切到 dev/sp-vision 的整目录 .config 挂载方案。
  - 硬件与底层接口：
      - 以 dev/sp-vision 版本的 rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry.cpp 为基线合入：
          - /predefined/timestamp 输入
          - /gimbal/hard_sync_snapshot 输出
          - RmcsBoard/CBoard 迁移后的串口与 CAN 路径
          - BMI088 新坐标映射
          - yaw 使用 generate_torque_command()
          - pitch 使用 generate_velocity_command()
          - rmcs_utility::RingBuffer 替换旧实现
      - 保留 deformable 专属硬件特性：
          - serial_filter_rmcs_board、serial_filter_top_board
          - 四个关节/转向电机零点和关节偏置参数
          - deformable chassis、SMC、observer 的现有接口与参数族
          - 现有关节校准依赖的 /chassis/*/encoder_angle 语义
      - rmcs_ws/src/rmcs_core/src/hardware/device/dji_motor.hpp 采用 dev/sp-vision 版本为基线，但必须保留并验证：
          - DjiMotorFeedback.encoder_angle
          - register_output(name_prefix + "/encoder_angle", ...)
          - encoder_angle_ = feedback.encoder_angle * 360.0 / 65535.0
          - encoder_angle() getter
          - deformable 相关代码对 /encoder_angle 的读取不改名、不改单位
      - 同时保留 dev/sp-vision 加入的 can_data.size() == 8 防护和 last_raw_angle() 接口。
  - 云台与 sp_vision_25 接入：
      - 不在 deformable 配置中继续使用 OmniInfantryPlannerGimbalController。
      - 修改 rmcs_ws/src/rmcs_core/src/controller/gimbal/deformable_infantry_gimbal_controller.cpp，使其成为 deformable 专用 planner 控制器：
          - 新增输入 /tf
          - 新增输入 /gimbal/yaw/velocity、/gimbal/pitch/velocity
          - 新增输入 /gimbal/yaw/velocity_imu、/gimbal/pitch/velocity_imu
          - 新增输入 /gimbal/auto_aim/control_direction
          - 新增输入 /gimbal/auto_aim/plan_yaw
          - 新增输入 /gimbal/auto_aim/plan_pitch
          - 新增输入 /gimbal/auto_aim/plan_yaw_velocity
          - 新增输入 /gimbal/auto_aim/plan_yaw_acceleration
          - 新增输入 /gimbal/auto_aim/plan_pitch_velocity
          - 新增输入 /gimbal/auto_aim/plan_pitch_acceleration
          - 输出 /gimbal/yaw/control_torque
          - 输出 /gimbal/pitch/control_velocity
          - 保留 /gimbal/yaw/control_angle_error
          - 保留 /gimbal/pitch/control_angle_error
      - 控制算法直接复用 dev/sp-vision planner 控制器思路：
          - 使用 TwoAxisGimbalSolver
          - 使用 yaw angle PID + yaw velocity PID + yaw velocity/acceleration 前馈
          - pitch 维持 angle PID 输出 velocity 的模式，不切到 torque
          - planner 不可用或目标超时时输出清零 / NaN 行为与 dev/sp-vision 一致
      - deformable 专属行为保留在该控制器里：
          - 手动模式的开关判定与遥控逻辑保留 deformable 语义
          - 若当前灵敏度与 omni 版本不同，则将手动灵敏度参数化并默认使用 deformable 现值
          - 继续保留 deformable 的上下限约束与几何求解路径
      - 在 rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml 中：
          - 新增 sp_vision_25::bridge::SpVisionBridge -> sp_vision_bridge
          - 保留 rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller
          - 删除 yaw_angle_pid_controller 与 pitch_angle_pid_controller 组件实例
          - 将 gimbal_controller 参数块改成 dev/sp-vision planner 风格参数集合：yaw_angle_*、yaw_velocity_*、yaw_vel_ff_gain、yaw_acc_ff_gain、pitch_angle_*、pitch_velocity_*、pitch_acc_ff_gain
          - 新增 sp_vision_bridge 参数块，接口名与 dev/sp-vision 一致
      - sp_vision_25 接口契约按当前实现固定为：
          - 输入 /gimbal/hard_sync_snapshot
          - 输入 /predefined/timestamp
          - 可选输入 /referee/shooter/initial_speed
          - 输出 /gimbal/auto_aim/control_direction
          - 输出 /gimbal/auto_aim/fire_control
          - 输出 /gimbal/auto_aim/laser_distance
          - 输出 /gimbal/auto_aim/plan_yaw
          - 输出 /gimbal/auto_aim/plan_pitch
          - 输出 /gimbal/auto_aim/plan_yaw_velocity
          - 输出 /gimbal/auto_aim/plan_pitch_acceleration
      - rmcs_ws/src/rmcs_core/plugins.xml 以 dev/sp-vision 版本为准，保留 DeformableInfantryGimbalController 注册项。
      - rmcs_ws/src/rmcs_bringup/config/deformable-infantry.yaml 以 dev/sp-vision 为基线，但恢复/保留 deformable 专属控制器类名和特性。
      - 数值参数默认沿用 dev/sp-vision 已接受值：pitch_motor_zero_point=15838、摩擦轮 800、shot_frequency=10、bullet_feeder_velocity_pid.kp=1.3，除非与 deformable 现有硬件特性直接冲突。
      - 保留 DeformableChassis、DeformableChassisController、SMC/observer 链，不将底盘控制切到 omni 栈。

  ## 测试计划

  - 执行 git submodule sync --recursive 与 git submodule update --init --recursive，确认 sp_vision_25 可用，且不再依赖 rmcs_core/librmcs 子模块。
  - 构建 rmcs_msgs、rmcs_core、rmcs_bringup、sp_vision_25，确认 vendored SDK 链接正常。
  - 启动 robot:=deformable-infantry，验收：
      - pluginlib 能加载 sp_vision_25::bridge::SpVisionBridge
      - DeformableInfantryGimbalController 成功加载
      - DAG 无缺失接口
      - /gimbal/hard_sync_snapshot 已由硬件层提供
      - /gimbal/auto_aim/* 全部由 sp_vision_bridge 提供并被 deformable gimbal controller 消费
      - bullet_feeder_controller 成功消费 /gimbal/auto_aim/fire_control
  - 启动 robot:=omni-infantry 做回归 smoke test，确认本次 merge 没破坏 dev/sp-vision 原生机器人链路。
  - 如有硬件条件，验证 deformable：
      - 手动云台控制仍可用
      - 视觉接管时 yaw 扭矩与 pitch 速度输出正常
      - 关节校准仍能读取 /chassis/*/encoder_angle
      - SMC 与底盘控制行为不退化

  ## 假设与默认选择

  - dev/sp-vision 头部提交固定为 4d95fc8。
  - 现有 merge/deformable-infantry@cfb2026 只作为冲突参考，不作为最终结果；最终结果必须再补上 4d95fc8 的剩余差异。
  - “保留 deformable infantry 的部分”解释为：保留 deformable 硬件拓扑、底盘控制、校准链路、控制器类名和专属行为；不保留旧的 gimbal PID 组件拆分方式。
  - “接口格式和名称与 dev/sp-vision 一致”解释为：所有 sp_vision 相关接口、hard_sync_snapshot、云台低层控制口和配置键名与 dev/sp-vision 保持一致；允许控制器实现仍然是 deformable 专用版本。