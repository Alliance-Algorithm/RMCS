# 仓库指南

## 项目定位
`sp_vision_25` 是一个以 C++17 编写的机器人视觉工程，核心能力分为三条线：
- `auto_aim`：装甲板检测、PnP 解算、EKF 跟踪、瞄准与开火决策。
- `auto_buff`：能量机关检测、姿态解算、目标预测与瞄准。
- `omniperception`：哨兵的多相机全向感知与目标切换决策。

这个仓库不是通用 SDK，而是“可直接上机器人”的应用工程。大多数 `src/*.cpp` 都是完整运行入口，区别主要在于控制后端：
- `CBoard + SocketCAN` 路线：`standard`、`mt_standard`、`uav`、`sentry*`、`auto_buff_debug`、`mt_auto_aim_debug`。
- `Gimbal + 串口` 路线：`standard_mpc`、`auto_aim_debug_mpc`、`auto_buff_debug_mpc`、`planner_test`、`fire_test` 等。

## 目录结构与真实职责
- `src/`：运行时入口程序。不要把它当成“示例目录”；这里的文件决定了真实启动链。
- `tasks/auto_aim/`：装甲板视觉主链，包含传统检测、YOLO、姿态解算、跟踪、瞄准、发射器和 MPC 规划器。
- `tasks/auto_buff/`：打符链路，包含 YOLO11 检测、几何解算、目标预测和瞄准。
- `tasks/omniperception/`：哨兵用的目标优先级、丢目标后的全向决策、多 USB 相机并行推理。
- `io/`：所有硬件和通信封装。包括工业相机、USB 相机、CBoard、云台串口、DM_IMU、ROS2 桥接、厂商 SDK。
- `tools/`：公共基础设施，主要是数学、EKF、RANSAC、绘图、日志、CRC、YAML 读取、轨迹和线程容器。
- `tests/`：独立可执行的验证程序，不走统一 `ctest`。
- `configs/`：机器人配置。配置是“扁平 YAML”，不是按模块拆分的层级 schema。
- `assets/`：模型和离线 demo。当前仓库内实际存在 `yolov5.xml/bin`、`yolov8.xml/bin`、`yolo11.xml/bin`、`yolo11_buff_int8.xml/bin`、`tiny_resnet.onnx`、`assets/demo/demo.avi/.txt`。
- `calibration/`：采图、相机标定、手眼标定、视频拆帧工具。
- `autostart.sh`：部署脚本入口，但它使用了机器相关绝对路径 `~/Desktop/sp_vision_25/`，并引用了当前仓库中不存在的 `watchdog.sh`。

## 构建图与依赖关系
核心 CMake 目标关系如下：

```text
tools                      # OBJECT library
io                         # STATIC library
auto_aim   -> io + OpenVINO + tinympcstatic
auto_buff  -> auto_aim + OpenVINO + Ceres
omniperception -> OpenVINO
src/tests/calibration -> 以上库的组合
```

关键外部依赖：
- 必需：`OpenCV`、`fmt`、`Eigen3`、`spdlog`、`yaml-cpp`、`nlohmann_json`、`OpenVINO`
- `auto_buff` 额外依赖：`Ceres`
- ROS2 条件依赖：`ament_cmake`、`rclcpp`、`std_msgs`、`rosidl_typesupport_cpp`、`sp_msgs`
- 工业相机 SDK：`MvCameraControl`、`MVSDK`
- 其他系统依赖：`usb-1.0`、仓库内自带的 `io/serial`

重要事实：
- 顶层和子模块都把 `OpenVINO_DIR` 写死为 `/opt/intel/openvino_2024.6.0/runtime/cmake`。迁移环境时优先处理这里。
- `io/CMakeLists.txt` 只支持 `x86_64` 和 `aarch64` 两种架构，并分别链接 `hikrobot/lib/amd64|arm64`、`mindvision/lib/amd64|arm64`。
- 没有统一 `ctest`；所有测试都以独立可执行程序运行。
- 没有 ROS2 环境时，`io` 会跳过 `ros2/*.cpp`，顶层也不会编译 `sentry*`、`publish_test`、`subscribe_test`、`topic_loop_test`。

常用构建命令：
- `cmake -B build`
- `cmake --build build -j$(nproc)`
- 或 `make -C build -j$(nproc)`

## 主要运行入口与启动链
### 1. `standard`
单线程基础自瞄链：

```text
Camera.read
-> CBoard.imu_at
-> YOLO.detect
-> Solver.set_R_gimbal2world
-> Tracker.track
-> Aimer.aim
-> CBoard.send
```

特点：
- 使用 `io::CBoard`，走 CAN。
- 入口会读取并打印模式切换，但当前实现没有按模式分支，主循环始终执行自瞄链。
- 适合做最短路径的自瞄问题定位，不适合作为多模式总入口模板。

### 2. `mt_standard`
多线程标准步兵链：

```text
检测线程: Camera.read -> MultiThreadDetector.push
主线程:
  mode == auto_aim:
    MultiThreadDetector.debug_pop
    -> CBoard.imu_at
    -> Solver -> Tracker
    -> CommandGener.push
命令线程:
  Aimer.aim -> Shooter.shoot -> CBoard.send
mode == small_buff/big_buff:
  Camera.read -> CBoard.imu_at
  -> Buff_Detector -> Buff Solver -> Buff Target -> Buff Aimer -> CBoard.send
```

特点：
- 这是目前最完整的 `CBoard` 多模式入口。
- `CommandGener` 单独线程约 500Hz 发送命令，且会把目标水平距离回填到 `io::Command::horizon_distance`。

### 3. `standard_mpc`
基于串口云台和 MPC 规划器的标准链：

```text
主线程:
  Camera.read -> Gimbal.q/state
  -> YOLO.detect -> Tracker.track
  -> target_queue.push
规划线程:
  Planner.plan(target, bullet_speed)
  -> Gimbal.send(VisionToGimbal)
buff 模式:
  Buff_Detector -> Buff Solver -> Buff Target -> Buff Aimer.mpc_aim -> Gimbal.send
```

特点：
- 控制后端切换为 `io::Gimbal`，不再走 `CBoard`。
- `mode()`、`state()`、`bullet_count` 都来自下位机串口协议。
- 这是后续改轨迹控制、控制器、发弹时序时最应该看的入口。

### 4. `uav`
无人机入口，仍走 `CBoard`：
- `auto_aim` / `outpost` 模式：`Detector`（传统方法）`-> Tracker -> Aimer -> Shooter -> CBoard.send`
- `small_buff` / `big_buff` 模式：`Buff_Detector -> Buff Solver -> Buff Target -> Buff Aimer -> CBoard.send`

注意：
- `uav.cpp` 当前默认用的是传统 `Detector`，不是 `YOLO`。

### 5. `sentry` / `sentry_bp` / `sentry_debug`
哨兵入口，只有在 ROS2 依赖齐全时才会编译。

公共主链：

```text
Camera.read -> CBoard.imu_at
-> YOLO.detect -> Decider 过滤/优先级
-> Tracker.track
-> 正常时 Aimer.aim
-> 丢目标时 Decider.decide(...)
-> Shooter.shoot
-> CBoard.send
-> ROS2 publish(target_info)
```

差异：
- `sentry`：丢目标时会联合 `usbcam1(video0)`、`usbcam2(video2)` 和 `back_camera(configs/camera.yaml)` 做全向决策。
- `sentry_bp`：丢目标时只使用 `back_camera`。
- `sentry_debug`：与 `sentry` 类似，但保留了更多可视化与调试输出。

### 6. `sentry_multithread`
哨兵多相机并行版本：

```text
4 路 USBCamera(video0/2/4/6)
-> Perceptron 并行推理
-> Decider.sort
主相机 YOLO.detect
-> Tracker.track(detection_queue, armors, t)
-> switching / lost / tracked 三态决策
-> CBoard.send
-> ROS2 publish(target_info)
```

这是哨兵二开时最重要的全向感知入口。

## 运行时输入输出与协议
### 工业相机 `io::Camera`
- 输入配置键：
  - 通用：`camera_name`、`exposure_ms`、`vid_pid`
  - `mindvision`：还需要 `gamma`
  - `hikrobot`：还需要 `gain`
- 输出：`cv::Mat img` + `std::chrono::steady_clock::time_point timestamp`
- 仅支持工业相机，USB 相机不是走这条类。

### USB 相机 `io::USBCamera`
- 打开方式：固定 `/dev/video*`，例如 `video0`、`video2`
- 输入配置键：`image_width`、`image_height`、`usb_frame_rate`、`usb_exposure`、`usb_gamma`、`usb_gain`
- 输出：`cv::Mat` + 时间戳
- 内部有采图线程和守护线程，掉线会尝试重连

### CBoard / CAN `io::CBoard`
- 输入配置键：`quaternion_canid`、`bullet_speed_canid`、`send_canid`、`can_interface`
- 下位机输入到视觉：
  - 四元数帧：`wxyz`，按 `1e4` 缩放
  - 状态帧：`bullet_speed` 按 `1e2` 缩放，`mode` 在 `data[2]`，`shoot_mode` 在 `data[3]`，`ft_angle` 按 `1e4` 缩放
- 视觉输出到下位机：
  - `io::Command {control, shoot, yaw, pitch, horizon_distance}`
  - `yaw/pitch/horizon_distance` 都按 `1e4` 打包进 CAN 帧
- 模式枚举：
  - `idle`
  - `auto_aim`
  - `small_buff`
  - `big_buff`
  - `outpost`

### 串口云台 `io::Gimbal`
- 输入配置键：`com_port`
- 下位机输入结构 `GimbalToVision`：
  - 帧头 `"SP"`
  - `mode`
  - `q[4]`，顺序为 `wxyz`
  - `yaw`、`yaw_vel`、`pitch`、`pitch_vel`
  - `bullet_speed`
  - `bullet_count`
- 视觉输出结构 `VisionToGimbal`：
  - 帧头 `"SP"`
  - `mode`：`0` 不控制，`1` 控制不开火，`2` 控制并开火
  - `yaw/yaw_vel/yaw_acc/pitch/pitch_vel/pitch_acc`
  - 末尾 `crc16`

### ROS2
- 发布：
  - topic: `auto_aim_target_pos`
  - type: `std_msgs/msg/String`
  - 内容格式：`x,y,z,id`
- 订阅：
  - `enemy_status`，类型 `sp_msgs::msg::EnemyStatusMsg`
  - `autoaim_target`，类型 `sp_msgs::msg::AutoaimTargetMsg`

### 离线数据与标定数据
- `auto_aim_test`、`auto_buff_test` 默认吃一对文件：`xxx.avi + xxx.txt`
- `txt` 每帧一行，格式是：`t w x y z`
- `calibration/capture.cpp` 会把图像存为 `jpg`，把姿态存为 `txt`，四元数顺序同样是 `wxyz`

## 配置系统与关键键位
配置是扁平 YAML；代码大量直接 `yaml["key"]` 或 `tools::read<T>()`，缺键会直接 `exit(1)`。加新参数时必须同步更新所有会走到该模块的配置文件。

当前代码真实读取的高频键位如下：
- `auto_aim`
  - 模型：`yolo_name`、`yolov5_model_path`、`yolov8_model_path`、`yolo11_model_path`、`classify_model`、`device`
  - 检测：`threshold`、`min_confidence`、`use_traditional`、`use_roi`、`roi.{x,y,width,height}`
  - 跟踪：`enemy_color`、`min_detect_count`、`max_temp_lost_count`、`outpost_max_temp_lost_count`
  - 瞄准/开火：`yaw_offset`、`pitch_offset`、`comming_angle`、`leaving_angle`、`decision_speed`、`high_speed_delay_time`、`low_speed_delay_time`、`left_yaw_offset`、`right_yaw_offset`、`first_tolerance`、`second_tolerance`、`judge_distance`、`auto_fire`
  - 解算：`R_gimbal2imubody`、`R_camera2gimbal`、`t_camera2gimbal`、`camera_matrix`、`distort_coeffs`
  - MPC：`fire_thresh`、`max_yaw_acc`、`Q_yaw`、`R_yaw`、`max_pitch_acc`、`Q_pitch`、`R_pitch`
- `auto_buff`
  - `model`
  - `fire_gap_time`
  - `predict_time`
  - 以及与姿态解算共用的相机标定参数
- `omniperception`
  - `image_width`、`image_height`
  - `fov_h`、`fov_v`
  - `new_fov_h`、`new_fov_v`
  - `enemy_color`
  - `mode`

需要特别注意的配置漂移：
- 旧配置文件里还能看到 `detect:`、`aim_time`、`wait_time`、`command_fire_gap` 这类字段，但当前 `auto_buff` 代码实际读取的是 `model`、`fire_gap_time`、`predict_time`。
- `auto_buff` 的 `YOLO11_BUFF` 目前在代码里固定 `compile_model(..., "CPU")`，不会使用通用的 `device` 键。
- `tests/cboard_test.cpp` 默认写的是 `configs/standard.yaml`，`tests/handeye_test.cpp` 默认写的是 `configs/handeye.yaml`，这两个文件当前仓库里都不存在。运行这些程序时请显式传入现有配置，例如 `configs/standard3.yaml` 或 `configs/calibration.yaml`。

## 常用运行命令
- `./build/standard configs/standard3.yaml`
- `./build/mt_standard configs/standard3.yaml`
- `./build/standard_mpc configs/standard3.yaml`
- `./build/uav configs/uav.yaml`
- `./build/sentry configs/sentry.yaml`
- `./build/sentry_multithread configs/sentry.yaml`
- `./build/auto_aim_test assets/demo/demo -c configs/demo.yaml`
- `./build/auto_buff_test assets/demo/demo -c configs/sentry.yaml`
- `./build/planner_test configs/standard3.yaml`
- `./build/planner_test_offline configs/standard3.yaml`
- `./build/camera_detect_test configs/standard3.yaml --tradition=false`
- `./build/usbcamera_detect_test configs/sentry.yaml -n video0 -d`

## 测试与验证策略
本仓库没有集中式测试框架，推荐按修改范围选择最短验证路径：
- 算法离线回归：
  - `auto_aim_test`
  - `auto_buff_test`
  - `planner_test_offline`
- 工业相机 / USB 相机 / CAN / 串口连通性：
  - `camera_test`
  - `usbcamera_test`
  - `multi_usbcamera_test`
  - `cboard_test`
  - `gimbal_test`
  - `dm_test`
- 端到端硬件链：
  - `standard`
  - `mt_standard`
  - `standard_mpc`
  - `sentry` / `sentry_multithread`
- 标定与坐标系：
  - `capture`
  - `calibrate_camera`
  - `calibrate_handeye`
  - `calibrate_robotworld_handeye`
  - `handeye_test`

经验规则：
- 改 `tasks/auto_aim/*` 的共享逻辑时，至少跑一个离线回归和一个真实入口。
- 改 `io/*` 时，优先跑最贴近硬件的单项测试，再跑对应入口。
- 改配置解析时，不要只改一个 YAML；要检查所有实际使用到的入口配置。

## 二次开发建议
- 在改代码前先确认目标入口属于 `CBoard` 还是 `Gimbal` 路线，这决定了控制协议、状态来源和调试方法。
- 如果要改“标准步兵主链”，优先在 `mt_standard` 或 `standard_mpc` 上动手，`standard` 过于简化。
- 如果要改“哨兵丢目标后的搜索策略”，入口先看 `tasks/omniperception/decider.*`，多相机并行链再看 `perceptron.*` 和 `sentry_multithread.cpp`。
- 加新配置时保持“代码读取键位”和“所有目标机器人 YAML”一致；仓库没有统一 schema 校验器。
- 不要把机器相关绝对路径写回代码。当前 `autostart.sh` 已经是机器特化脚本，应视为部署侧文件而不是通用开发模板。

## 代码风格与提交约束
- 使用仓库根目录 `.clang-format`，基于 Google 风格，100 列宽，`PointerAlignment: Middle`，自定义换行大括号。
- 文件名、目标名使用小写加下划线。
- 类名使用 PascalCase，命名空间保持小写。
- 标定、阈值、偏置、CAN ID、串口名、模型路径都应优先落在 YAML，不要硬编码进源码。
