 sp-vision 在 RMCS 中的接口接入分析                                                                              
                                                        
 1. 总体架构

 sp_vision_25 作为子模块接入 RMCS，通过 pluginlib 插件机制注册为 rmcs_executor::Component，与 RMCS
 其他组件共享同一进程内的强类型接口（InputInterface<T> / OutputInterface<T>），不走 ROS 2 topic。

 plugins.xml 注册了两个组件：
 ├── sp_vision_25::bridge::SpVisionBridge         ← 真实视觉桥接（相机+YOLO+规划）
 └── sp_vision_25::bridge::SpVisionResponseTestBridge  ← 虚拟目标测试桥接

 ---
 2. SpVisionBridge 接口清单

 2.1 输入接口（从 RMCS 硬件层读取）

 ┌─────────────────────────┬───────────────────────────────┬─────┬────────────────────┬────────────────────┐
 │         接口名          │           C++ 类型            │ 必  │       提供方       │        含义        │
 │                         │                               │ 需  │                    │                    │
 ├─────────────────────────┼───────────────────────────────┼─────┼────────────────────┼────────────────────┤
 │ /gimbal/hard_sync_snaps │                               │     │ 硬件层（如         │ 硬件同步快照：IMU  │
 │ hot                     │ rmcs_msgs::HardSyncSnapshot   │ 是  │ OmniInfantry）     │ 四元数(qw/qx/qy/qz │
 │                         │                               │     │                    │ ) + 曝光时间戳     │
 ├─────────────────────────┼───────────────────────────────┼─────┼────────────────────┼────────────────────┤
 │ /predefined/timestamp   │ std::chrono::steady_clock::ti │ 是  │ predefined_msg_pro │ 系统当前时间戳     │
 │                         │ me_point                      │     │ vider              │                    │
 ├─────────────────────────┼───────────────────────────────┼─────┼────────────────────┼────────────────────┤
 │                         │                               │     │                    │ 裁判系统弹速，缺失 │
 │ /referee/shooter/initia │ float                         │ 否  │ referee::Status    │ 时使用 bullet_spee │
 │ l_speed                 │                               │     │                    │ d_fallback         │
 │                         │                               │     │                    │ 参数（默认 23.0）  │
 └─────────────────────────┴───────────────────────────────┴─────┴────────────────────┴────────────────────┘

 2.2 输出接口（写入供下游控制器消费）

 ┌─────────────────────────────────────┬───────────────┬────────┬──────────────────────────────────────────┐
 │               接口名                │   C++ 类型    │ 默认值 │                  消费方                  │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │                                     │ Eigen::Vector │ (0,0,0 │ 云台控制器（DeformableInfantryGimbalCont │
 │ /gimbal/auto_aim/control_direction  │ 3d            │ )      │ roller /                                 │
 │                                     │               │        │ OmniInfantryPlannerGimbalController）    │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/fire_control       │ bool          │ false  │ BulletFeederController17mm（发弹决策）   │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/laser_distance     │ double        │ 0.0    │ UI 等                                    │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_yaw           │ double        │ 0.0    │ 云台控制器（前馈计算）                   │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_pitch         │ double        │ 0.0    │ 同上                                     │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_yaw_velocity  │ double        │ 0.0    │ 同上                                     │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_yaw_accelerat │ double        │ 0.0    │ 同上                                     │
 │ ion                                 │               │        │                                          │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_pitch_velocit │ double        │ 0.0    │ 同上                                     │
 │ y                                   │               │        │                                          │
 ├─────────────────────────────────────┼───────────────┼────────┼──────────────────────────────────────────┤
 │ /gimbal/auto_aim/plan_pitch_acceler │ double        │ 0.0    │ 同上                                     │
 │ ation                               │               │        │                                          │
 └─────────────────────────────────────┴───────────────┴────────┴──────────────────────────────────────────┘

 ---
 3. 内部线程模型

 SpVisionBridge 同时继承 rmcs_executor::Component 和 rclcpp::Node，在 before_updating() 中启动两个后台线程：

 ┌─────────────────────────────────────────────────────┐
 │  executor 主循环 (1000Hz)                            │
 │  update():                                          │
 │    1. 把 bullet_speed 写入原子变量                     │
 │    2. 从硬件层拷贝最新 HardSyncSnapshot 到互斥锁保护区  │
 │    3. publish_latest_result(): 检查超时，写输出接口     │
 └─────────────────────────────────────────────────────┘
         ▲ store_result()                ▲ store_latest_hard_sync_snapshot()
         │                               │
 ┌───────┴──────────┐   ┌───────────────┴──────────────┐
 │  Planner 线程     │   │  Worker 线程（检测线程）        │
 │  (1kHz 循环)      │   │  (相机帧率驱动)               │
 │                   │   │                              │
 │  1. load_latest_  │   │  1. Camera.read(frame)       │
 │     target()      │   │  2. 拿最新 HardSyncSnapshot    │
 │  2. Planner.plan  │   │  3. 设置 Solver 姿态          │
 │     (target,      │   │  4. YOLO.detect(frame)       │
 │      bullet_speed)│   │  5. Tracker.track(armors, t) │
 │  3. store_result()│◄──│  6. store_latest_target()     │
 │                   │   │                              │
 └───────────────────┘   └──────────────────────────────┘

 关键同步机制：
 - hard_sync_snapshot_mutex_：主循环 ↔ Worker 线程之间同步 IMU 快照
 - target_mutex_：Worker 线程 → Planner 线程传递最新 target
 - result_mutex_：Planner 线程 → 主循环传递规划结果
 - bullet_speed_snapshot_：std::atomic<double>，主循环 → Planner 线程传递弹速

 ---
 4. 超时保护

 publish_latest_result() 中检查 result_timeout_（默认
 0.2s）。如果距上次检测时间戳超过此值，所有输出接口清零（control_direction = (0,0,0), fire_control = false,
 所有导数 = 0）。这保证了视觉丢目标后控制器不会继续执行旧指令。

 ---
 5. 在 YAML 中的配置方式

 以 deformable-infantry.yaml 为例：

 # 组件列表中注册
 components:
   - sp_vision_25::bridge::SpVisionBridge -> sp_vision_bridge

 # 参数配置
 sp_vision_bridge:
   ros__parameters:
     config_file: configs/standard4.yaml    # sp_vision_25 内部配置文件路径
     bullet_speed_fallback: 28.0            # 裁判弹速缺失时的回退值
     result_timeout: 0.2                    # 结果超时秒数
     debug: false                           # 是否开启调试窗口

 ---
 6. DAG 中的位置（数据流）

 predefined_msg_provider ──┐
                           ├──→ sp_vision_bridge ──→ gimbal_controller ──→ hardware_command
 infantry_hardware ────────┘                    └──→ bullet_feeder_controller ──→ hardware_command
        │
        └──→ referee_status ──→ sp_vision_bridge (提供 bullet_speed)

 上游：predefined_msg_provider（时间戳）、硬件层（HardSyncSnapshot、弹速）
 下游：gimbal_controller（读取 control_direction + plan_yaw/pitch/vel/acc
 做前馈控制）、bullet_feeder_controller（读取 fire_control 决定是否发弹）

 ---
 7. 下游消费方式

 云台控制器（以 DeformableInfantryGimbalController 为例）

 // 所有视觉输入标记为 optional（第三参数 false）
 register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);
 register_input("/gimbal/auto_aim/plan_yaw", plan_yaw_, false);
 // ...

 // before_updating() 中为缺失的输入绑定默认值
 if (!auto_aim_control_direction_.ready())
     auto_aim_control_direction_.make_and_bind_directly(Eigen::Vector3d::Zero());

 // update() 中判断是否启用自瞄
 bool planner_active = auto_aim_requested() && !auto_aim_control_direction_->isZero();

 // 启用时：用 control_direction 设定云台目标方向
 //        用 plan_yaw/pitch/vel/acc 计算前馈力矩
 // 未启用时：走手动遥控

 发弹控制器

 register_input("/gimbal/auto_aim/fire_control", fire_control_, false);
 // 当自瞄启用 && 旋钮值 >= 0.7 && fire_control == true 时触发发弹

 ---
 8. 与 sp_vision_25 独立运行模式的区别

 sp_vision_25 仓库支持两种控制后端：
 - CBoard + SocketCAN（standard, mt_standard 等入口）：视觉直接通过 CAN 发送 yaw/pitch 给下位机
 - Gimbal + 串口（standard_mpc 等入口）：视觉通过串口协议发送完整规划结果

 在 RMCS 中接入时使用的是第三种方式：通过 SpVisionBridge 组件把视觉结果写入 executor 的共享内存接口，由 RMCS
 自己的云台控制器和发弹控制器消费。视觉不直接控制硬件，只提供方向向量 + MPC 规划轨迹作为控制器的输入。
