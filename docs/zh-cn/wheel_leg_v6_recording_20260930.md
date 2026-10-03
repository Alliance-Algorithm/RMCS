# V6 real2sim 新录包曲线

日期：2026-09-30。软件已实现，离线预览和编译验证见下文；尚未部署或驱动实机。
原始需求保存在 [录制协议草案](wheel_leg_v6_recording_protocol_20260930.md)。本次完成其单侧阶段：左、右各五类 run，包含架空跳跃。
开发分支为 `dev/wheel_leg_rl`；`dev/rmcs_rl` 仅用于读取标定来源，没有切换或合并参考分支。

## 标定与坐标

复用参考分支 `dev/rmcs_rl` 的电机零位与方向，冻结在 commit
`8149afda1de0f0e79dac07a6444fc3741759d5c2`。来源：

- [wheel-leg-infantry-rl.yaml](https://github.com/Alliance-Algorithm/RMCS/blob/8149afda1de0f0e79dac07a6444fc3741759d5c2/rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml)：四个电机在大腿朝上、内膝约 105° 处清零，旧模型偏置 `[-1.6,-2.93,1.6,2.93]`。
- [wheel_leg_joint_pair_geometry.hpp](https://github.com/Alliance-Algorithm/RMCS/blob/8149afda1de0f0e79dac07a6444fc3741759d5c2/rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg_joint_pair_geometry.hpp)：30–120°、每 5° 的 CAD 闭链诊断表。

硬件 API 已做电机反向，旧模型到 V6 再统一乘 −1，因此本次实际使用
`q_v6 = -q_api + [1.6,2.93,-1.6,-2.93]`，速度、力矩也只取反一次。不增加减速比。
P4 数组顺序始终为 `[LH,LK,RH,RK]`；DM 硬件参数顺序仍为 `[LH,RH,LK,RK]`。

绑定文件为 `.script/identification/calibration/v6_pair_from_dev_rmcs_rl.json`，包含来源 commit、源文件 SHA256、V6 manifest/model_spec SHA256、两侧映射与完整 LUT。
V6 stop105 manifest SHA256 为 `875fa71e89d4d669af0a5471b2334f878d302e06f1b354fc245b95cb5d6f29ba`。

`theta=0` 定义为 V6 底盘坐标中大腿向下的矢状面投影；正 theta 朝 +X 倾斜。
左侧 `qH=-pi/2-theta`，右侧 `qH=+pi/2+theta`。
`g_left(beta)=qK-qH` 取参考分支的 opening 表，右侧取其负值。
单调三次 Hermite 插值提供 g、g′、g″，不外推；一阶连续，节点处二阶导数可以不同。
所有平滑段将 theta/beta 导数映射到两个主动轴，再以 1 ms 网格审查速度和加速度。

原表明确是左右平均的 CAD 诊断值。本次继承已有电机标定，beta 和 theta 遥测标为 **FK 估计**。
`hardware_multipoint_g0_verified=false` 仅表示没有把该数值表冒充新一轮独立内膝角多点实测；不要求重新清电机零位。
30–120° 是诊断表定义域，**首批自动目标仍限定 65–100°**，不代表机械行程准入。
实际 105° 零位可作为入口初态，随后用 8 s 平滑移入内部域；106–107° 自然出现的反馈也可以记录，不能据此改变标称止挡。
更深静载箱与跳跃深度、102° 边界箱，需要用明确的 `--admitted-beta-min-deg`/`--admitted-beta-max-deg` 生成新 profile；默认不加入。

## 默认曲线

左右采用相同物理计划、分别映射至自身主动轴，每次只使能受试侧。对侧 DM 保持禁用，轮子零电流。
PD 固定 `160*(q_held-q)-2.5*dq`，每 tick 更新，整体限幅 ±40 Nm；额定 20 Nm、MIT 编码 54 Nm 的含义不变。
目标每 20 tick 更新一次（名义 50 Hz），dq_ref 仅作轨迹诊断。没有 I、重力/速度前馈或电机内部 PD。

|run|内容|段数|名义时长|用途|
|---|---|---:|---:|---|
|L01 / R01|65/75/85/95/100° × 四朝向 × 两到达方向；两朝向慢速屈伸；75/95° 的 0→360→0、45° 驻留|231|798.86 s / 13.31 min|静载与迟滞|
|L02 / R02|三个构型、共同/固定髋/固定共模/联合共 12 个 30 s 动态段；20 s 高频小幅；共模和差模正反大小阶跃|104|600 s / 10 min|动态拟合|
|L03 / R03|70/90°、两到达方向、多正弦与大小阶跃，独立新 run|81|294 s / 4.90 min|整包留出|
|LJ01 / RJ01|15 组不同的朝向/压缩深度/蹬伸时间组合，每组 3 周期；含 0/±15/±30/180°，周期完成后间隔 3/1.5/1 s|397|411.32 s / 6.86 min|架空瞬态拟合|
|LJ02 / RJ02|改变朝向顺序、压缩中心和持续时间，6 组合 × 3 周期|163|212.84 s / 3.55 min|跳跃整包留出|

每包都有开头与末尾同构型的 30 s 基准。时长包含入口、转场、最短到位驻留，实际等待可能延长，超时跳过则缩短。
静载点的到位 hold 要求末尾连续 2 s beta 进入 ±2° 箱且两轴速度不超过 0.08 rad/s；其余到位 hold 默认稳定 0.5 s。
100° 边界的外侧到达候选被准入域裁剪，不把它算作已经测到完整双向迟滞，manifest 保存具体 from/to。

动态段中心修正仅在 designated arrival hold、50 Hz 参考更新时发生，每次最多 0.01 rad、累计最多 0.20 rad、间隔至少 0.2 s。
还要给后续整个 group 的 beta 激励留出准入余量。进入动态段后冻结；不能为了到位越过命令域。
10 s 仍未到位则记 uncovered 并跳过该 group，下一转场从上一条已下发参考做逐轴限速/限加速度五次插值，记录 admission=3。
静态到位和跳跃蓄力的成功/失败都通过 latched arrival segment ID 留证，避免瞬间转场丢事件。

跳跃相位为压缩、蓄力、蹬伸、收腿、迎地展腿、架空缓冲形状、回稳、完整周期后的间隔。
0.35 s 候选量化为 0.36 s；超预算的段会延长并在 `requested_s/duration_s` 中体现。
LJ01/RJ01 先以 0.60 s 和 0.35 s 候选分别遍历六个朝向，再以 0.20 s 候选采三个正常蹬伸朝向；每轮轮换压缩深度。
默认 LJ01/RJ01 有 6 个超预算延长段，默认各包没有缩幅段；若以后收紧预算，缩幅记录在 `amplitude_scale`。
多正弦各分量按总幅值上界归一，频率和相位保存在 manifest；保持边沿独立标 waveform=9，边沿处导数未定义。

## 生成、预览与录制

从宿主机 RMCS 仓库根目录执行 `.script/host/rmcs zsh`，启动并进入已有开发容器；以下命令从容器的 `/workspaces/RMCS` 执行。
SSH 别名 `remote` 使用容器内的配置。先用 `ssh -o BatchMode=yes -o ConnectTimeout=8 remote 'hostname'` 核对连接；地址变化时用 `.script/set-remote <机器人IP>` 更新。
随后编译。生成器不会连接机器人。

```bash
.script/build-rmcs --packages-select rmcs_msgs rmcs_core rmcs_rl rmcs_bringup
python3 .script/identification/prepare_v6_pair_recording.py \
  --run LJ01 \
  --output rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-v6-LJ01-identification.yaml \
  --preview-dir /tmp/v6-LJ01
```

`--run` 可选表中十个标签。源码配置目录已提供默认 L01 profile；其余通过同一生成器产生，避免复制轨迹代码。
预览直接运行生产 C++ `wheel_leg_recording_preview`，输出精确 nominal manifest、50 Hz CSV 和标定快照。
主机也可用 g++ 编译 `rmcs_core/tools/wheel_leg_recording_preview.cpp`，通过 `--preview-bin` 指定；无需 ROS 或 MuJoCo。
图形可在有 numpy/matplotlib 的环境中生成：

```bash
python3 .script/identification/plot_v6_pair_recording.py \
  /tmp/v6-LJ01/reference-50hz.csv /tmp/v6-LJ01/preview.png --run LJ01
```

曲线预览不是带载仿真，不提供未经计算的气簧/重力力矩预测或可达性信用。
[默认 manifest 与图片](artifacts/v6_pair_recording_v1/README.md) 已保存，CSV 可按上述命令重建。

安装生成的 profile 并同步构建产物后，现场仍用既有遥控器门控和 recording-ready 握手：

```bash
.script/build-rmcs --packages-select rmcs_bringup
```

开发容器终端 A 保持以下同步进程运行（`watch` 不会自动退出，只保留一个同步实例）：

```bash
.script/sync-remote
```

另开开发容器终端 B，从 `/workspaces/RMCS` 执行；保持遥控器双下，`wait-sync` 成功后才启动：

```bash
.script/wait-sync && \
.script/identification/remote-wheel-leg-identify start-v6 LJ01
# 等到 Recorder subscribed and active / Recording MCAP 提示，再拨两侧 MIDDLE。
# 执行结束/中断后拨两侧 DOWN，再封包：
.script/identification/remote-wheel-leg-identify stop
```

`stop` 只终止 rosbag 并封包，controller 仍运行，不能代替遥控器双下失能。
`status` 列出 screen、最近 run 和 topic 类型；开关、电机及反馈状态另用 robot_status 查询。

`record-v6 LJ01` 用于录制已加载的同一个 profile；不会重启 executor。
启动端比对本地/远端 profile、core、MSG/IDL 哈希，核对运行中的全部 controller/recorder 参数及只读 plan manifest。
新协议节点拒绝在线修改参数，以免 ROS 参数读回值和实际已编译计划不一致；更改配置用新 profile、重启、新 run。
一次中断/重启必须新建 bag，导出器拒绝混合多个运行 repetition。
`start-v6` 把 controller stdout 保存到该 run 的 `controller.log`；`record-v6` 接续既有进程时无法回溯其原 stdout。

run 目录保存 profile、calibration.json、trajectory-plan.json、recording-ready.json、core 二进制、源代码 tar、MSG/IDL 和哈希。
manifest 中 entry/from 是占位初态；真正入口来自 preflight 结束时的新鲜反馈，实际参考、中心、等待和跳过过程以 MCAP 为准。

## 导出与 real2sim 接口

消息新增 recording protocol version、连续 model 角、实际 FK beta、请求 beta、实际下发参考 beta、FK 大腿方向、参考更新 tick、局部实耗时、中心、成功/未覆盖段、跳跃周期和相位。
已有 CAN 原始字节/sequence/主机接收时戳、提交字节/时戳、温度、IMU 等字段继续保留；电压未知仍为 NaN。

封包成功后，在开发容器的 `/workspaces/RMCS` 下载最近一次完整 run；`record/` 已由仓库忽略。不要只复制 `.mcap`，消息定义快照必须保留在 `bag/` 的父目录。

```bash
recording_remote_dir="$(ssh remote 'cat /root/identification_bags/.active_run')"
mkdir -p record/v6
scp -r "remote:${recording_remote_dir}" record/v6/
recording_local_dir="record/v6/${recording_remote_dir##*/}"
```

进入容器时是 zsh，加载对应环境再导出。若使用其他终端，请将 `recording_local_dir` 设置为刚下载的 run 目录：

```bash
source /opt/ros/jazzy/setup.zsh
source rmcs_ws/install/local_setup.zsh
python3 .script/identification/export_identification_bag.py \
  --bag "${recording_local_dir}/bag" \
  --profile "${recording_local_dir}/profile.yaml" \
  --calibration "${recording_local_dir}/calibration.json" \
  --output "${recording_local_dir}/response_1khz.npz"
```

原始 MCAP 保留，派生输出为 `response_1khz.npz` 和同名 `.json`。本地主机可在 RMCS 的 `record/v6/` 查看下载结果。

schema=5 的 JSON 同时携带完整 controller 配置和 recording QC。保留完整 1 kHz 原始时序，paired age/skew 质量筛选独立于原始数据。
QC 报告实际 FK 角箱持续时间与参考角箱、到位成功/失败、中心历史、逐周期逐相位误差、限幅持续时间、反馈估计力矩分箱、驱动象限和正负机械功估计。
FK 不是独立角传感器，反馈力矩不是测力计，负机械功不能自动等同电气回馈。

L01/L02/LJ01（右侧同理）全部为 fit；L03/LJ02（右侧同理）全部为 holdout，**不随机拆相邻 tick，也不从拟合 run 伪造独立留出**。
旧 bag 的 MSG/IDL 布局不同，仍必须用对应版本的类型解码；导出器会拒绝不匹配的 CDR 布局。

新 q/dq/tau 已经是 V6。后续训练仓库 `scripts/identify_pair_dynamics.py` 如接入这些输出，指定 V6 bundle，坐标额外变换取 `--coordinate-sign 1`；旧包到 V6 的 −1 不得再重复。
本次不运行拟合、不更新冻结训练参数、不改变现有 PPO。
整圈和气簧载荷需要固定基座非线性闭链回放，旧局部 affine-load fitter 会明确拒绝此协议。

B01/BJ01 仍需要独立四轴协调入口和作用域；JL/JC 需要相应加载/接触装置。本版不生成这些 profile，不把两个单侧 controller 并发当成有效双侧，也不赋予架空动作真实跳高或着地信用。

## 验证结果

2026-09-30，在 `dev/wheel_leg_rl` 完成以下离线验证：

- 开发容器内 `rmcs_msgs/rmcs_core/rmcs_rl/rmcs_bringup` 四包编译通过。
- `rmcs_core` 96 个、`rmcs_rl` 46 个 C++ 用例通过；colcon 连同测试包装共 152 项，零失败、零跳过。
- 主机 identification 脚本回归：121 通过、7 跳过（两个 ROS MCAP 用例及五个已删除旧控制方式的用例）。
- ROS 容器内 profile/远端启动握手/导出/新协议专项回归：104 通过、5 个旧控制方式用例跳过。旧协议和新 `RJ02` 均完成真实 rosbag2 MCAP 序列化→读取→NPZ/JSON 导出，校验 uint64 时间戳、连续 model 角、到位事件、跳跃相位和整包留出。
- 十个默认 profile/manifest 和 50 Hz 参考由同一生产 planner 生成；额外验证 55–102° 显式准入的 L01/LJ01 离线生成。默认准入保持 65–100°。

这些结果验证软件计划与记录链，尚无本协议的实机运行、负载仿真或新的物理覆盖数据。
