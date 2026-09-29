# 轮腿控制代码整理与合同边界

2026-09-29。本次整理覆盖最终 PD 辨识、倒地自起动作序列、`rmcs_rl` 组件，以及 `rmcs_core` 的轮腿硬件和底盘组件。首轮先完成行为保持的结构重构，后续单独修复计时、配置校验和 PLANT 参考行为；两类改动的验证范围分别记录。验证在本地开发容器和离线测试完成，未启动电机、同步机器人或修改训练任务。

## 生产代码职责

沿用 RMCS `Component` 的输入/输出注册、依赖调度和 `update()`，ROS 参数与日志使用现有 `NodeMixin`。接口对象继续作为组件成员持有，不新增控制线程或另一套执行器。项目已为 C++23，沿用项目 `.clang-format`。

| 文件 | 职责 |
|---|---|
| `rmcs_rl/src/configuration.cpp` | 注册接口、加载并校验标定/恢复参数、构造观测器和 ONNX；定长参数通过 `std::array<N>` 表达维度 |
| `rmcs_rl/src/rl_controller.cpp` | INIT/IDLE/PREPARE/RL 生命周期、失能和故障锁存、200 Hz/50 Hz 调度；集中推理结果发布和故障退出 |
| `rmcs_rl/src/controller_recovery.cpp` | 普通 PREPARE 目标推进、恢复反馈采集、一次恢复动作推进、轮支撑探测及交接 |
| `rmcs_rl/src/recovery_controller.{hpp,cpp}` | 无 ROS/CAN/ONNX 的动作序列；参考生成与限速、阶段跃迁、阶段力矩分别实现 |
| `rmcs_rl/src/recovery_observer.{hpp,cpp}` | 标定闭链 LUT、条件接地高度、气簧补偿和支撑证据；保留几何算法，接入恢复控制的实际采样间隔 |
| `rmcs_rl/src/control_interval.hpp` | 恢复控制间隔采样与时钟异常锁存；复位后重新建立时间基准 |
| `rmcs_rl/src/observation.cpp` | 电机到模型反馈、命令参考、当前模型的 35D 观测 |
| `rmcs_rl/src/action.cpp` | 动作解码、PC PD、模型侧混合、机械保护、包络/峰值预算、`Jᵀ` 与设备限幅 |
| `rmcs_rl/src/policy.hpp`、`onnx_policy.cpp` | 当前部署合同常量、观测布局及预分配 ONNX 输入/输出；保留 `std::expected` 错误传播 |
| `rmcs_core/src/hardware/wheel-leg.cpp` | 硬件接口注册、参数和电机配置、反馈发布、使能请求评估、整批 CAN 命令排队及状态提交 |
| `rmcs_core/src/hardware/wheel_leg_control.hpp` | DM 使能/失能/故障清理调度、反馈门禁和关节发送顺序 |
| `rmcs_core/src/hardware/device/dm_motor.hpp` | DM 坐标转换、限幅和 MIT 帧编解码；纯力矩与位置 PD 共用编码步骤 |
| `rmcs_core/src/controller/chassis/wheel_leg_chassis_controller.cpp` | 遥控启动/复位、模式边沿、底盘命令和参数边界校验 |
| `rmcs_core/src/identification/wheel_leg_identification_axes.hpp` | 辨识控制器和记录器共用的六轴注册顺序 |
| `rmcs_core/src/identification/wheel_leg_identification_phase.hpp` | 辨识内部强类型阶段；端口与录包保持既有整数值 |

`rl_controller.cpp` 聚焦生命周期与调度；配置和恢复适配有独立实现文件，仍属于同一个组件。没有把节点配置、推理或硬件访问塞进纯动作状态机。

### rmcs_core 硬件与底盘整理

`WheelLeg`、底盘及辨识组件沿用 `NodeMixin` 参数声明方式。硬件构造流程拆成接口注册、控制配置和电机配置；反馈更新拆成电机、IMU、驱动就绪和遥控发布；命令更新按请求快照、轮命令、关节调度、关节命令和状态提交组织。一次更新仍使用同一个发送 builder，没有新增线程或改变批次边界。

保持 CAN0 轮电机、CAN1 髋、CAN2 膝及既有端口映射。正常关节发送顺序仍为 `[LH,RH,LK,RK]`；单侧辨识先排队非活动侧命令；失能批次先发各轴 FD，再追加 neutral MIT。FB/FC 重试、first-MIT 后才允许轮力矩的门禁、反馈时钟和计数器内存序均保留。`DmMotor` 只合并重复的 MIT 编码过程，没有更改量化、方向、限幅或帧布局。

底盘将原单次 `do-while` 拆成启动保持、复位和正常控制分支，模式切换独立实现；保留双下→双中握手、遥控异常复位及按键边沿。新增有限数和参数范围校验，兼容现有固定高度 `0.305 m`、高度范围/步长为 `0` 的配置。

辨识控制器内部使用 `IdentificationPhase`，通过 `std::to_underlying` 保持 `FAILED=-1 / IDLE=0 / PREPARING=1 / RUNNING=2 / COMPLETE=3` 的端口和录包合同。删除无生产引用的 `WheelLegRlConsumer` 源码与插件注册，保留历史 artifacts 中的旧配置，避免改写旧实验依据。

## 扫频只保留最终 PD 生产路径

当前腿部入口是 `start-pd-loaded` / `record-pd-loaded`，配置为
`wheel-leg-infantry-pair-pd-loaded-identification.yaml`。控制器只接受
`multiband_chirp`、`rl_pd`、50 Hz 目标与 1000 Hz PC 闭环；当前 profile 为右侧、160/2.5、模型侧 ±40 Nm。

```text
tau = Kp * (q_target - q_feedback) - Kd * dq_feedback
```

每 20 ticks 更新并保持目标，每 tick 以最新可用反馈重新计算力矩。轨迹导数只用于记录，没有速度前馈、积分或重力前馈。硬件仍使用模式 A 纯力矩；MIT 编码范围和实验输出上限保持分别处理。

当前共模/差模坐标为 `c=(qH+qK)/2`、`d=qK-qH`，输出 `qH=c-d/2`、`qK=c+d/2`。同侧两轴始终闭环，差模由两轴反向分担。保留初态加载、多构型整圈网格、多频段共模/差模/联合激励、有限上升时间阶跃、独立多正弦验证。指定测试初态为 161 段/671.42 s；加载段数由初态决定，不是所有初态固定相同时长。

轮速辨识仍由 `start-wheels` 和独立轮 profile 管理，50 Hz 目标/1 kHz 速度环，当前增益 0.6、±4.5 Nm，包含左右分别、双轮与真实滑行段。它不等于双侧四腿轴联合辨识；腿部生产控制器仍只允许单侧。

用户指出的训练文档
[V6_PAIR_REAL2SIM_TRAJECTORY_20260928.md](../../../robot_rl/isaac_wheeled_rl_train/docs/V6_PAIR_REAL2SIM_TRAJECTORY_20260928.md)
描述早期 87.2 s 小幅候选，后续追加了 200 Hz 串级 P/PI、20 Nm 的说明；该文件和其早期预览不是当前生产版本。旧 `u/v` 方案在相对运动时保持髋参考，而当前 `c/d` 方案由两轴分担，不能把两者当作同一参考轨迹。

串级 PI、bounded probe、hold/LUT、rotation-chirp 已由前一轮移出生产路径，保留在历史归档供旧 bag 回放；本轮不恢复这些分支。旧记录必须使用该 run 的参数、源码和库哈希，不能按当前 PD 重新解释旧力矩。历史沿革见[辨识清理记录](wheel_leg_identification_cleanup_20260928.md)。

## 倒地自起动作序列

`RecoveryController::step()` 保留原有次序：校验与总超时 → WAIT_GROUND/倒扣重选路 → **旧阶段推进参考** → 阶段跃迁 → **新阶段计算力矩** → 混合权重。不能先跃迁再推进目标，也不能把上一阶段的气簧补偿带进新阶段。

| 阶段 | 参考与行为 | 主要转段依据 |
|---|---|---|
| WAIT_GROUND | 零力矩等待，不推进姿态参考 | 接地候选、比力与低速证据持续 120 ms，按高度进入收腿或准备 |
| FOLD | 两侧收至 `fold`，共享同侧轴对的圈数 | 参考误差、实际跟踪与至少 300 ms，按路由转支撑/侧摆/定向回转 |
| PLANT | 仅 `abs(g_y)<0.5` 时在收腿参考上加主值俯仰支撑偏置，`sign(0)=0` | 参考接近、接触候选持续 60 ms；超过 2.5 s 无接触失败 |
| SIDE_SWING | 按轮高差选择两侧动作，定向半圈摆动后返回 | 侧倾变化触发重新收腿/选路，有次数和时长界限 |
| ORBIT | 同侧两轴保持相对形状、定向连续回转 | 姿态/条件高度/接触触发准备或蹬伸；转满一圈失败 |
| THRUST | `thrust` 加支点回转与机身俯仰补偿 | 接触、条件高度、倾角和角速度满足后进入捕获 |
| CAPTURE | 捕获参考向支撑姿态过渡，含俯仰角速度反馈 | 稳定支撑证据持续 100 ms 后混合 |
| PREPARE | 依据路由选正立/支撑/站立目标并做世界方向对齐 | 同一稳定支撑门槛；一次有界倒扣重选路 |
| BLEND | 继续脚本参考，外层将脚本和策略的模型侧力矩混合 | 默认 200 ms 完成；失姿退出 |
| COMPLETE / FAILED | 动作状态机返回零脚本力矩 | 外层分别接管 RL 或锁存失能 |

一般参考使用 `paired_delta()`，两条同轴主动输出共享圈数；ORBIT、THRUST、SIDE_SWING 使用有方向的连续差值。正立路径用四轴统一比例限速，其余路径维持逐轴限速，不改成另一种插值曲线。

当前候选速度保留 orbit/side/rollover/capture = 5/5/5.25/4 rad/s。训练侧的 CAPTURE 4.5 档、提前无门槛交接和台阶恢复属于研究分支，未并入 RMCS。依据见
[平地动作序列研究](../../../robot_rl/isaac_wheeled_rl_train/docs/V5_FLAT_SELF_RIGHTING_TRAJECTORY_20260927.md)和
[趴地/台阶试验](../../../robot_rl/isaac_wheeled_rl_train/docs/V5_SELF_RIGHTING_PRONE_STEP_20260927.md)。

外层保留六轴力矩唯一拥有权、PREPARE 影子推理不写上一动作、BLEND 模型侧力矩混合，以及混合后的机械约束、条件扭矩包络、峰值时间预算、映射与设备限幅。恢复后运动保持期的门槛和所有标定门禁保留，稳定时长改用实际时间累计。

### 结构重构后的行为修复

- 恢复观察器、动作状态机与恢复后的站稳计时共用每次恢复控制更新的实际 `dt`，不再混用固定 5 ms、实际时钟和 tick 计数。初次采样使用 5 ms；重复/倒退或超过 20 ms 的控制间隔锁存为异常，必须复位后重新建立基准。峰值力矩预算与输出前 deadline 检查保留独立的实际输出时间，覆盖控制计算或推理耗时。
- 等待驱动时也记录最近一次 PD 调度，以 `std::optional` 区分“尚未调度”和真实 tick 0，避免在 1 kHz 执行器中重复推进观察器；等待期间加速度观测缺失会清理旧支撑证据。
- `RecoveryConfig` 的速度和时长标量同时检查有限数与严格正值，参考向量检查全部有限，拒绝 NaN、±Inf 和非法非正参数。
- 轮辨识先处理新鲜遥控的双下复位，再检查时钟异常；异常时双下能够清理锁存并重新建立时间基准。再次运行仍需新的双下停留握手，复位当拍保持零输出及停止样本。
- PLANT 偏置与训练 sensor-only 实现对齐：使用主值 pitch，仅在 `abs(g_y)<0.5` 时施加偏置，正零和负零均按 `sign(0)=0` 处理。此项有意改变原行为，覆盖零俯仰、侧倾边界及跨 ±π 场景。

这些修复由各自的边界回归验证，不能引用首轮 C++ 重构前后逐拍一致结果证明其等价。

### 与训练侧动作实现的差异

逐段对照的是训练
[`inspect_v5_activation.py`](../../../robot_rl/isaac_wheeled_rl_train/scripts/inspect_v5_activation.py)
的 sensor-only 动作与
[`recovery_observer.py`](../../../robot_rl/isaac_wheeled_rl_train/src/wheeled_tasks/chassis/recovery_observer.py)。RMCS 是部署改写，**没有通过完整 Python↔C++ 逐拍等价验收**。PLANT 俯仰偏置已按上述边界对齐；以下其余差异仍需单独处理：

| 项目 | 训练脚本 | 当前 RMCS |
|---|---|---|
| 世界方向对齐 | PREPARE/BLEND <65°、CAPTURE <85°；独立 alignment 候选；PLANT 路线用 stand 基姿 | 三阶段均 <85°，使用 contact 候选；PLANT 路线用 nominal/plant 基姿 |
| 轮动作 | 包含 PLANT 平衡及 THRUST 世界轮角速度制动 | PLANT/THRUST 轮力矩为零，仅 PREPARE/CAPTURE/BLEND 平衡；观察器没有同样的世界轮速/疑似壳体接触输出 |
| 气簧补偿 | 按路由选择；LUT 先求梯度再插值 | 所有 FOLD 及 ORBIT/SIDE_SWING；LUT 当前段割线斜率 |
| 转段与超时 | 部分路线跟踪误差包角；PLANT `>=2.5s`；8s预算不含BLEND，部分当拍转段继续推进 | 所有FOLD均查连续跟踪差；PLANT `>2.5s`；8s包含BLEND，WAIT_GROUND当拍转段后返回 |
| 接管与探测 | 路由相关目标误差/支撑条件；逐轮探测与短暂丢证据容忍 | 统一姿态/轴速/壳体门禁；探测采用两轮最大加速度，几何无效立即清证据 |

Python 阶段编号 `WAIT_GROUND=10/CAPTURE=11` 与 RMCS 的 `CAPTURE=10/WAIT_GROUND=11` 相反。新增 `recovery_phase_name()` / `recovery_phase_from_name()`，以稳定名称映射日志和跨语言对照，未知名称返回空值；保留已发布的 RMCS 枚举整数，不直接比较两边编号。名称往返及 WAIT_GROUND/CAPTURE 对照有独立测试。

下一步动作迁移应先冻结资产、观测器、动作参数和阶段名称映射，建立逐阶段数值输入/输出对照，再使用同一资产闭环检验姿态通过率、捕获阶段冲击、超速、正立误翻和站稳后二次接触。当前 C++ 重构不能代替这些验证。旧 V5 的成功案例不作为新 V6 stop105 的自起验收。

## 当前 ONNX 与 V6 的版本边界

本仓库打包 ONNX 与训练 `models/v5_flat_12486/policy.onnx` 的 SHA-256 相同：
`ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d`。
`DeployedPolicyContract` 提炼的是这个旧 V5 部署基线，不是自动加载训练合同的版本选择器，也未增加模型哈希强制校验。

| 合同项 | 当前 RMCS/打包 V5 | 当前 V6 stop105 训练 |
|---|---|---|
| 策略/PC PD | 50/200 Hz | 50/1000 Hz |
| 腿 Kp/Kd | 60/2 | 160/2.5 |
| 轮速度增益 | 0.2，随后设备限幅 | 0.6，软件 ±4.5 Nm |
| P4 标称符号 | `[+.42,-.1374,-.42,+.1374]` | `[-.42,+.1374,+.42,-.1374]` |
| 35D 上下文 | normal/jump | manual35，还包含 STEP/LOWER 等请求 |
| 自起机械范围 | 旧 V5 的 40–110° LUT、108°软件保护 | 新资产被动膝止挡 105° |

相同 35D/6D 形状不能证明兼容。V6 接入需一起绑定模型身份、资产/nominal、manual35 编解码、PD 时钟/增益/轮限幅以及恢复几何，不能只替换 ONNX 或只改增益。105°电机内部设零与105°机械止挡是两件事；不重复设零、不重复添加 CAD 坐标旋转或 M3508 减速比。

## 验证

使用现有 `.script/build-rmcs --packages-select rmcs_core rmcs_rl --parallel-workers 2` 构建。首轮结构重构时，7 个 C++ 测试目标共 **93 项 gtest** 全部通过：DM/硬件门控27、腿规划9、腿组件23、轮规划4、轮组件4、恢复17、RL组件9。当时 `colcon test-result` 显示100项，其中包含7项CTest包装，不将包装重复当作行为用例。该数量是首轮记录，不是当前最终总数。

首轮新增两项恢复转段回归，检查 FOLD→PLANT 当拍参考/补偿，以及 BLEND 起始权重/COMPLETE 零力矩。当时以结构重构前后源码对照240组确定性反馈场景，共114,420步；phase、failure、reference、torque、blend输出完全一致。**该结果仅适用于首轮结构重构，不能延伸到后续 PLANT、实际计时等有意行为修复。** 它是 C++ 行为保持检查，不是 Python↔C++ 完整等价、PhysX 或实车自起成功率。

后续新增硬件发送顺序、底盘组件、时钟异常与双下复位、恢复配置有限数、PLANT 参考及阶段名称映射等回归。最终两包构建成功，9 个测试目标共 **128 项 gtest 全部通过**：DM/硬件门控28、底盘5、腿规划9、腿组件26、轮规划4、轮组件10、控制间隔5、恢复27、RL组件14。`colcon test-result --verbose` 显示137项，包含9项CTest包装；0错误、0失败、0跳过。

首轮新增9项实际 `RlController` 组件测试，绑定真实注册接口、加载打包ONNX和仅测试生效的合成标定，验证等待驱动时零输出/参考冻结、50Hz观测和动作历史、200Hz反馈重算/拍间保持、双下/显式重置、反馈/驱动故障锁存、非法请求及非有限观测退出。后续增加5项恢复开启的组件测试：tick 0等待、tick 1–4保持零输出、tick 5开始恢复；电机和加速度反馈各自缺失/过期时失能锁存，并在显式复位后重新进入恢复。测试的合成LUT经生产参数解析，不写入真实标定。

控制间隔测试使用可控时间点验证抖动、重复/倒退/停顿锁存、复位、真实站稳累计，以及推理采样后超时和实际输出间隔；不通过休眠模拟电机。完整恢复接管和实车CAN时序仍需单独闭环验收。

本轮重新运行 Python 验证：主机辨识套件83通过/6跳过；加载ROS和工作空间环境后，开发容器配置、握手、录包和同步脚本子集83通过/5跳过。容器补跑了主机缺ROS时跳过的MCAP往返测试；剩余5项是测试原有的非PD参数组合跳过。两组套件存在重叠，不合并计数。主机缺 `colorama`、容器缺 `scipy`，因此分别使用已有依赖运行相应套件，没有安装依赖。修改文件通过项目格式检查和 `git diff --check`。
