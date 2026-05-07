# `standard_mpc` 控制链展开分析

## 1. 范围
本文只分析 `src/standard_mpc.cpp` 这条运行链，以及它直接依赖的关键组件：
- `io/gimbal/gimbal.*`
- `tasks/auto_aim/planner/planner.*`
- `tasks/auto_aim/tracker.*`
- `tasks/auto_aim/target.*`
- `tasks/auto_buff/buff_aimer.*`
- `tools/thread_safe_queue.hpp`

目标不是解释所有算法细节，而是把这条链在“启动、采样、状态估计、控制生成、下发、模式切换”上的真实行为讲清楚，便于后续二次开发。

## 2. 这条链到底是什么
`standard_mpc` 是标准步兵/地面平台的一条“工业相机 + 串口云台”运行链，和 `standard`/`mt_standard` 的核心区别有两点：
- 控制后端不是 `CBoard + CAN`，而是 `Gimbal + serial`
- 自瞄分支不是直接输出 `yaw/pitch`，而是输出一组 `yaw/pitch + vel + acc + fire` 的前馈控制量

这条链并不只有一种控制模式，而是三种：
- `AUTO_AIM`：主线程负责感知，规划线程负责发送控制
- `SMALL_BUFF` / `BIG_BUFF`：主线程既感知也直接发送控制
- `IDLE`：主线程持续发送全零控制

## 3. 线程拓扑
`standard_mpc` 至少包含 4 条线程：

```text
主线程
  读取 mode
  读工业相机
  对齐云台姿态
  执行 auto_aim 或 buff 感知链
  在 buff/idle 模式下直接发送命令

规划线程 plan_thread
  仅在 AUTO_AIM 模式工作
  读取 target_queue 中的最新目标
  调用 Planner 生成控制量
  以约 100Hz 发送给 Gimbal

Gimbal::read_thread
  后台持续读串口
  更新 mode / state / bullet_speed
  推送四元数队列

Recorder::saving_thread
  第一次 record 后启动
  异步把图像和姿态写到 records/*.avi + *.txt
```

额外说明：
- 工业相机驱动内部可能还有厂商 SDK 自己的线程，但不在本文件分析范围内。
- `tools::Plotter plotter;` 在 `standard_mpc` 中实际没有被使用。

## 4. 启动阶段
`src/standard_mpc.cpp` 的启动顺序是：

1. 解析命令行，要求显式提供 `config_path`
2. 创建 `Exiter / Plotter / Recorder`
3. 创建 `io::Gimbal`
4. 创建 `io::Camera`
5. 创建 `YOLO / Solver / Tracker / Planner`
6. 创建一个容量为 1 的 `target_queue`，并先压入一个 `std::nullopt`
7. 创建 buff 检测、解算、目标、瞄准对象
8. 启动 `plan_thread`
9. 进入主循环

这里有两个启动行为要特别记住：

### 4.1 `io::Gimbal` 构造会阻塞，直到收到第一帧姿态
`Gimbal` 构造函数会打开串口、启动 `read_thread`，然后执行一次 `queue_.pop()` 等第一帧四元数到达。也就是说：
- 如果串口没开起来，程序直接退出
- 如果串口开了但下位机不发姿态，程序会卡在启动阶段

### 4.2 `YOLO(config_path, true)` 会打开调试显示
`standard_mpc` 里 `YOLO` 的 `debug=true`。在 `yolov5/yolov8/yolo11` 实现里，调试模式会画检测结果并 `cv::imshow("detection", ...)`。

这意味着：
- 默认不是纯 headless 程序
- 无显示环境下运行时要谨慎

## 5. 主线程做了什么
主循环结构可以概括成：

```text
mode = gimbal.mode()
camera.read(img, t)
q = gimbal.q(t)
gs = gimbal.state()
recorder.record(img, q, t)
solver.set_R_gimbal2world(q)

if AUTO_AIM:
  yolo.detect -> tracker.track -> push latest target
else if SMALL_BUFF/BIG_BUFF:
  buff_detect -> buff_solve -> buff_target -> buff_aimer.mpc_aim -> gimbal.send
else:
  gimbal.send(all zero)
```

拆开看，有 5 个关键点。

### 5.1 模式来源不是图像线程，而是串口后台线程
`mode = gimbal.mode()` 读到的是 `Gimbal::read_thread` 在收到串口帧后更新的模式值。模式本身来自下位机。

但要注意：
- `plan_thread` 用的 `mode` 不是直接读 `gimbal.mode()`，而是读主线程更新的原子变量 `mode`
- 所以模式切换能否立刻影响 `plan_thread`，还取决于主线程下一次迭代何时执行到 `mode = gimbal.mode()`

### 5.2 图像时间和姿态时间做了对齐
主线程先 `camera.read(img, t)`，再调用 `gimbal.q(t)`。

`gimbal.q(t)` 的做法不是“拿最新姿态”，而是：
- 从四元数队列中取两帧包围 `t` 的姿态
- 对 `wxyz` 四元数做 `slerp`
- 返回和图像时间戳对齐的姿态

因此，`standard_mpc` 在姿态对齐这一步比 `latest state` 更精确。

### 5.3 `gimbal.state()` 和 `gimbal.q(t)` 不是同一时刻的量
`gimbal.state()` 读的是后台线程最近一帧串口状态：
- `yaw`
- `yaw_vel`
- `pitch`
- `pitch_vel`
- `bullet_speed`
- `bullet_count`

这份状态没有按图像时间戳回放，也没有插值。

所以主线程里同时拿到的：
- `q = gimbal.q(t)`：对齐到图像采样时刻
- `gs = gimbal.state()`：最近串口时刻

这两份数据在时间上并不严格一致。

在 `AUTO_AIM` 分支里影响较小，因为 `gs` 只在规划线程里用到 `bullet_speed`。

在 buff 分支里影响更大，因为 `buff_aimer.mpc_aim()` 会直接使用 `gs.yaw / gs.pitch`。

### 5.4 `Recorder` 默认是开启写盘的
`standard_mpc` 没有把 `recorder.record(...)` 注释掉，而是每帧都在执行。

默认行为是：
- 按 30 fps 采样
- 在 `records/` 下生成一对 `avi + txt`
- `txt` 内容格式是 `t w x y z`

这条链不是零 I/O 干扰路径。如果要看纯控制时序，通常需要先关掉 `Recorder`。

### 5.5 `target_queue` 是“单槽邮箱”，不是任务队列
`target_queue` 的类型是：

```cpp
tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
```

语义是：
- 容量只有 1
- 队列满时自动弹掉旧值，再放入新值
- `front()` 只读不弹

所以它的真实作用不是积压多帧目标，而是“让规划线程随时拿到最新一份目标快照”。

这点很关键：
- 规划线程永远只看最新目标
- 不会因为感知慢而堆积过时目标
- 也不会消费掉邮箱里的值；如果主线程没有新结果，规划线程会持续对同一份目标快照重复规划

## 6. `AUTO_AIM` 分支展开
### 6.1 感知链
主线程在 `AUTO_AIM` 模式下只做 3 件事：

```text
YOLO.detect(img)
-> Tracker.track(armors, t)
-> target_queue.push(target or nullopt)
```

也就是说：
- 主线程不直接发控制
- 主线程只负责把“当前目标估计”写入邮箱

### 6.2 `Tracker` 的输出不是原始装甲板，而是单个 `Target`
`Tracker` 做的事情包括：
- 过滤敌我颜色
- 按优先级排序装甲板
- 在 `lost / detecting / tracking / temp_lost` 状态机之间切换
- 用 `Solver` 先把装甲板解算到世界系，再更新目标 EKF

输出的 `Target` 内部状态向量是：

```text
x = [cx, vx, cy, vy, cz, vz, a, w, r, l, h]
```

含义：
- `cx, cy, cz`：旋转中心
- `vx, vy, vz`：平动速度
- `a`：当前参考装甲板角度
- `w`：角速度
- `r`：基础半径
- `l`：长短轴半径差
- `h`：高低装甲板高度差

也就是说，送进规划线程的不是“当前瞄点”，而是一个可继续预测的目标动力学模型。

### 6.3 规划线程怎么消费目标
规划线程循环逻辑是：

```text
if mode == AUTO_AIM:
  target = target_queue.front()
  gs = gimbal.state()
  plan = planner.plan(target, gs.bullet_speed)
  gimbal.send(plan)
  sleep 10ms
else:
  sleep 200ms
```

这里的几个行为很重要。

#### 行为 A：只要模式是 AUTO_AIM，它就会一直发命令
因为 `target_queue` 在启动时已经塞了一个 `nullopt`，所以进入 `AUTO_AIM` 后：
- 队列永远非空
- 即使没有识别到目标，`planner.plan(std::nullopt, ...)` 也会返回 `control=false`
- 然后规划线程仍然会以约 100Hz 发“关闭控制”的命令

#### 行为 B：规划线程拿到的是目标快照，不是共享对象
`front()` 返回的是 `Target` 的一份拷贝。

这意味着：
- 规划线程不会和主线程共享同一个 `Target` 实例
- 不存在多线程同时修改同一个 EKF 的问题
- 但也意味着规划线程内的预测不会回写到主线程

### 6.4 `Planner` 实际做了什么
`Planner::plan(std::optional<Target>, bullet_speed)` 分成两层。

#### 第一层：补偿软件延迟
如果目标存在，它先看角速度 `|w|`：
- 高于 `decision_speed`：使用 `high_speed_delay_time`
- 否则：使用 `low_speed_delay_time`

然后把目标先预测到：

```text
future = now + delay_time
```

#### 第二层：补偿弹丸飞行时间
接着 `Planner::plan(Target, bullet_speed)` 会：

1. 检查弹速，非法时回退到 `22 m/s`
2. 在 `target.armor_xyza_list()` 里选水平距离最近的装甲板
3. 用 `Trajectory(bullet_speed, d, z)` 解一次弹道
4. 再把目标预测 `fly_time`

也就是说，目标最终被推到了：

```text
图像时刻/目标时刻
-> 规划时刻的软延迟补偿
-> 子弹飞行时间补偿
```

### 6.5 `Planner` 的“时域中心”是什么
`Planner` 的常量：

```text
DT = 0.01
HALF_HORIZON = 50
HORIZON = 100
```

`get_trajectory()` 会围绕“命中时刻”构造一个大约 `[-0.50s, +0.49s]` 的参考轨迹窗口。

然后：
- `plan.target_yaw / target_pitch` 取中点 `HALF_HORIZON`
- `plan.yaw / pitch / vel / acc` 也取中点

所以这条控制链不是输出“立刻指向哪里”，而是在输出“围绕命中时刻中点”的参考控制量。

### 6.6 `Planner` 里的“MPC”到底闭环到哪里
当前 `Planner` 使用 `tinympc` 解的是一个 2 状态二阶系统：
- yaw: `[angle, vel]`
- pitch: `[angle, vel]`

但它的初值 `x0` 不是来自当前 `gimbal.state()`，而是直接取参考轨迹首点：

```text
x0_yaw   = [traj(0,0), traj(1,0)]
x0_pitch = [traj(2,0), traj(3,0)]
```

这意味着：
- 当前 `AUTO_AIM` 分支里的 `Planner` 只用到了 `gimbal.state().bullet_speed`
- 它没有把当前云台实测 `yaw/pitch/yaw_vel/pitch_vel` 作为优化初始条件
- 因此这套“MPC”更像参考轨迹生成器 + 前馈，而不是紧贴实测状态的闭环 MPC

这是二次开发时最值得注意的一点。

### 6.7 开火判定
`plan.fire` 的判断不是看当前中点误差，而是看 `HALF_HORIZON + 2` 处，也就是大约 `20ms` 之后：

```text
hypot(yaw_error, pitch_error) < fire_thresh
```

这本质上是“略向前看的命中误差门限”。

## 7. `BUFF` 分支展开
`standard_mpc` 的 buff 分支虽然也输出 `auto_aim::Plan`，但它不是 `Planner` 那套 tinyMPC。

主线程在 `SMALL_BUFF` / `BIG_BUFF` 下执行的是：

```text
buff_solver.set_R_gimbal2world(q)
-> buff_detector.detect(img)
-> buff_solver.solve(power_runes)
-> buff_target.get_target(power_runes, t)
-> copy target
-> buff_aimer.mpc_aim(target_copy, t, gs, true)
-> gimbal.send(buff_plan)
```

关键区别如下。

### 7.1 buff 模式没有独立规划线程
buff 模式下控制发送权回到主线程：
- 主线程直接 `gimbal.send`
- `plan_thread` 只是因为 `mode != AUTO_AIM` 而进入 `sleep_for(200ms)`

### 7.2 buff 分支的“mpc_aim”不是 `Planner`
`buff_aimer.mpc_aim()` 的控制量来源是：
- 先按 `detect_now_gap + predict_time_` 预测目标
- 再用两次弹道解算近似收敛飞行时间
- 再用有限差分估计 `yaw_vel / pitch_vel / yaw_acc / pitch_acc`

所以 buff 分支当前更接近：

```text
预测瞄点 + 数值微分前馈 + 开火节流
```

而不是 `Planner` 那套基于 `tinympc` 的二阶优化器。

### 7.3 buff 分支显式使用了当前云台状态
`buff_aimer.mpc_aim()` 在算加速度时会直接用 `gs.yaw / gs.pitch`：

```text
plan.yaw_acc   <- f(yaw, gs.yaw, last_yaw_mpc)
plan.pitch_acc <- f(pitch, gs.pitch, last_pitch_mpc)
```

所以和 `AUTO_AIM` 不同，buff 分支确实把当前云台状态拉进了控制计算。

### 7.4 buff 分支有“切扇叶”抑制逻辑
如果新旧 yaw/pitch 跳变太大：
- `switch_fanblade_ = true`
- `plan.control` 可能被拉低
- `plan.fire = false`
- 直到 `fire_gap_time_` 满足才允许再次发射

这是一种稳定性保护逻辑，不是通用 MPC 行为。

## 8. 模式切换与控制权切换
`standard_mpc` 的控制权分配是：

| 模式 | 谁发串口命令 | 控制来源 |
| --- | --- | --- |
| `AUTO_AIM` | `plan_thread` | `Planner` |
| `SMALL_BUFF` / `BIG_BUFF` | 主线程 | `buff_aimer.mpc_aim()` |
| `IDLE` | 主线程 | 全零命令 |

这个设计有 3 个直接后果。

### 8.1 切进 `AUTO_AIM` 可能有额外延迟
`plan_thread` 在非 `AUTO_AIM` 状态下一次睡 `200ms`。

因此从 buff/idle 切到 auto_aim 时，首帧 auto_aim 控制下发延迟大致可能包含：
- 主线程下一次采样到新 `mode` 的等待
- 再加上 `plan_thread` 最多 `200ms` 的睡眠尾巴

这条链的首个 auto_aim 控制不是即时切换的。

### 8.2 切出 `AUTO_AIM` 时可能还有一个尾命令
`plan_thread` 在 `AUTO_AIM` 下每 `10ms` 循环一次。

因此从 auto_aim 切出时，理论上还可能有一个约 `10ms` 内的尾命令窗口。

### 8.3 两个线程都会调用 `gimbal.send()`，但 `Gimbal::send()` 没有加锁
`Gimbal::send()` 内部会改共享成员 `tx_data_`，然后直接 `serial_.write(...)`，没有互斥保护。

这意味着在模式切换窗口附近：
- `plan_thread` 可能还在发 auto_aim 命令
- 主线程已经开始发 buff 或 idle 命令

当前代码没有显式保证这两类写串口行为互斥。

如果后续要增强稳定性，这里是必须优先处理的点。

## 9. 输入、输出、配置入口
### 9.1 输入
- 图像：`io::Camera.read(img, t)`
- 云台姿态：`gimbal.q(t)`，按图像时刻插值
- 云台状态：`gimbal.state()`
- 模式：`gimbal.mode()`

### 9.2 输出
输出统一走 `io::Gimbal::send(...)`，实际串口结构是：

```text
head = "SP"
mode = 0/1/2
yaw, yaw_vel, yaw_acc
pitch, pitch_vel, pitch_acc
crc16
```

模式编码含义：
- `0`：不控制
- `1`：控制但不开火
- `2`：控制并开火

### 9.3 `standard_mpc` 直接依赖的配置键
- 云台：
  - `com_port`
- 自瞄检测：
  - `yolo_name`
  - `yolov5_model_path` / `yolov8_model_path` / `yolo11_model_path`
  - `device`
  - `threshold`
  - `min_confidence`
  - `use_roi`
  - `roi`
- 解算：
  - `R_gimbal2imubody`
  - `R_camera2gimbal`
  - `t_camera2gimbal`
  - `camera_matrix`
  - `distort_coeffs`
- 跟踪：
  - `enemy_color`
  - `min_detect_count`
  - `max_temp_lost_count`
  - `outpost_max_temp_lost_count`
- 规划：
  - `yaw_offset`
  - `pitch_offset`
  - `decision_speed`
  - `high_speed_delay_time`
  - `low_speed_delay_time`
  - `fire_thresh`
  - `max_yaw_acc`
  - `Q_yaw`
  - `R_yaw`
  - `max_pitch_acc`
  - `Q_pitch`
  - `R_pitch`
- buff：
  - `model`
  - `fire_gap_time`
  - `predict_time`

## 10. 这条链当前的工程特征
### 10.1 优点
- 图像时间和姿态时间做了插值对齐
- 感知和 auto_aim 控制解耦，规划线程可以稳定 100Hz 输出
- `target_queue` 只保留最新目标，不会堆积延迟
- buff 与 auto_aim 共用一套云台串口协议，便于统一下位机接口

### 10.2 当前限制
- `AUTO_AIM` 的 `Planner` 没把当前云台姿态/速度作为优化初值输入
- buff 分支和 auto_aim 分支的控制器不是同一套方法
- 模式切换是轮询式，不是事件驱动式
- 串口发送权在模式切换附近没有统一仲裁
- 默认打开 `YOLO` 调试显示和 `Recorder` 录制，运行负担比“纯控制模式”更重

### 10.3 当前代码 caveat
- `Planner::plan(Target, bullet_speed)` 在检查 `Trajectory.unsolvable` 之前就使用了 `bullet_traj.fly_time`。如果后续要继续强化这条链，建议优先修这里。

## 11. 二开时建议从哪里下手
### 11.1 如果要做“真正闭环的 MPC”
优先改 `tasks/auto_aim/planner/planner.cpp`：
- 把 `gimbal.state()` 的 `yaw/pitch/yaw_vel/pitch_vel` 接进 `Planner`
- 用实测状态而不是 `traj(:,0)` 初始化优化器

### 11.2 如果要做更平滑的模式切换
优先改 `src/standard_mpc.cpp`：
- 去掉 `plan_thread` 的粗粒度 `sleep_for(200ms)`
- 用条件变量或 mailbox 唤醒
- 明确只允许一个线程拥有 `gimbal.send()` 的写权限

### 11.3 如果要做 headless 部署
至少先处理两点：
- 把 `YOLO(config_path, true)` 改为非调试模式
- 关闭 `recorder.record(...)`

### 11.4 如果要调“打不中但轨迹像是对的”
排查顺序建议是：
1. `gimbal.q(t)` 与图像是否对齐
2. `Solver` 标定参数是否正确
3. `Tracker` 是否稳定输出合理 `w`
4. `Planner` 的 `delay_time + fly_time` 组合是否过大或过小
5. 下位机是否真的按 `yaw_vel / yaw_acc / pitch_vel / pitch_acc` 执行

## 12. 一句话总结
`standard_mpc` 不是“相机一帧 -> 立刻打一帧命令”的简单链路，而是：

```text
主线程做时空对齐和目标建模
-> 单槽邮箱传递最新目标快照
-> 规划线程按软件延迟 + 飞行时间前瞻生成控制
-> 云台串口后台持续回灌模式和状态
```

后续二次开发时，最重要的判断标准是先分清：
- 你是在改“目标预测”
- 还是在改“参考轨迹生成”
- 还是在改“模式切换时的命令仲裁”

这三者在 `standard_mpc` 中已经被拆到了不同对象和线程里。
