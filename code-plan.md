# Code Plan
## 目标
统一 `deformable-infantry-v2` 中 joint 相关控制链的角度语义：
- 原始角：
  - `/chassis/*_joint/angle`
  - 表示电机内部零点角
- 物理角：
  - `/chassis/*_joint/physical_angle`
  - 表示真实机构物理角
- 物理角速度：
  - `/chassis/*_joint/physical_velocity`
并让以下模块统一使用物理角语义：
- `DeformableInfantryV2`
- `DeformableChassisController` (`deformable_wheel_controller.cpp`)
- `ChassisTestV2Controller`
- `AdrcController` 的 joint measurement 配置
## 已确认映射关系
- 电机原始角增大时，真实物理角减小，并朝 `8 deg` 方向变化
- 电机零点对应真实物理角 `62.5 deg`
因此映射公式为：
```cpp
physical_angle = 62.5deg - motor_angle
physical_velocity = -motor_velocity
单位统一为：
- physical_angle: rad
- physical_velocity: rad/s
已实施方向
1. V2 hardware 发布 physical 语义
在 rmcs_ws/src/rmcs_core/src/hardware/deformable-infantry-v2.cpp 中：
- 新增输出：
  - /chassis/left_front_joint/physical_angle
  - /chassis/left_back_joint/physical_angle
  - /chassis/right_back_joint/physical_angle
  - /chassis/right_front_joint/physical_angle
  - /chassis/left_front_joint/physical_velocity
  - /chassis/left_back_joint/physical_velocity
  - /chassis/right_back_joint/physical_velocity
  - /chassis/right_front_joint/physical_velocity
- 在 joint motor 状态更新后，将原始 angle/velocity 映射为 physical 角和角速度并输出
3. deformable wheel controller 使用 physical 角
在 rmcs_ws/src/rmcs_core/src/controller/chassis/deformable_wheel_controller.cpp 中：
- joint 输入改为：
  - /chassis/*_joint/physical_angle
  - /chassis/*_joint/physical_velocity
- 轮距计算：
  - R_i = chassis_radius + rod_length * cos(physical_angle_i)
- 机构附加速度：
  - v_mech = -rod_length * sin(physical_angle_i) * physical_velocity_i
- 所有依赖 JointStates 的动力学和力矩计算自动继承 physical 语义
- 增加节流日志打印：
  - 四轮 physical 角
  - 四轮轮距
4. chassis test 使用 physical 角
在 rmcs_ws/src/rmcs_core/src/controller/chassis/chassis_test.cpp 中：
- 新增四路 physical angle 输入
- 正常模式下：
  - 继续按 min_angle/max_angle 发布 joint target_angle
- 双 DOWN 时：
  - /chassis/control_velocity = NaN
  - 四轮 target_angle = 当前四轮 physical_angle
- 这样不修改 ADRC，也能让误差接近 0，使输出力矩接近 0
5. ADRC measurement 切换到 physical angle
在 rmcs_ws/src/rmcs_bringup/config/deformable-infantry-v2.yaml 中：
- 4 个 joint ADRC 的 measurement 改为：
  - /chassis/*_joint/physical_angle
- setpoint 继续使用：
  - /chassis/*_joint/target_angle
这样 measurement 和 setpoint 使用同一套物理角语义。
6. 主配置切换到 ChassisTestV2Controller + ADRC
在 deformable-infantry-v2.yaml 中：
- 使用：
  - rmcs_core::controller::chassis::ChassisTestV2Controller -> chassis_test_controller
- 启用：
  - 4 个 AdrcController
  - ChassisPowerController
  - DeformableChassisController
- 更新 value_broadcaster 观测项：
  - 去掉旧的 control_angle_error
  - 增加 physical_angle/physical_velocity/target_angle
7. pluginlib 注册
在 rmcs_ws/src/rmcs_core/plugins.xml 中：
- 新增：
  - rmcs_core::controller::chassis::ChassisTestV2Controller
否则 executor 无法加载新控制器。
当前需要重点验证
1. 启动验证
确认以下组件能成功加载：
- DeformableInfantryV2
- ChassisTestV2Controller
- AdrcController x4
- ChassisPowerController
- DeformableChassisController
2. topic 验证
确认以下话题存在并有合理数值：
- /chassis/left_front_joint/physical_angle
- /chassis/left_back_joint/physical_angle
- /chassis/right_back_joint/physical_angle
- /chassis/right_front_joint/physical_angle
- /chassis/left_front_joint/physical_velocity
- /chassis/left_front_joint/target_angle
3. 映射方向验证
检查：
- 电机原始角增大时，physical_angle 是否减小
- 伸长到最大时，physical angle 是否接近 8 deg
- 零位时，physical angle 是否接近 62.5 deg
4. 双 DOWN 验证
在 left_switch == DOWN && right_switch == DOWN 时：
- /chassis/control_velocity 应为 NaN
- 四轮 target_angle 应锁定到当前四轮 physical_angle
- control_torque 应明显减小并趋近于 0
5. 底盘动力学验证
检查：
- 四轮 physical angle 相同时，四轮轮距应一致
- 非对称姿态时，四轮轮距应按预期分化
- deformable_wheel_controller 日志中的 physical angle / radius 是否符合机械直觉
后续可选优化
1. 发布更多调试量
可选新增：
- 四轮 vehicle_radius
- 四轮 v_mech
- physical angle 的 deg 版 broadcaster
2. 统一 DeformableChassis 旧逻辑
若后续还继续使用 DeformableChassis，需要将其 V2 分支也统一切换到 physical angle 语义，避免老逻辑继续读取原始电机角。
风险点
1. plugins.xml 未同步部署时，运行时仍会报 ChassisTestV2Controller 不存在
2. 配置未同步时，runtime 可能仍加载旧版 deformable-infantry-v2.yaml
3. physical_angle 接口如果未正确发布，会导致 ADRC 或 wheel controller 再次缺边
4. 如果 supercap 板卡实机没有回传，状态会保持默认，但不应再造成 DAG 启动失败
