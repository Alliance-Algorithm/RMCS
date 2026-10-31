# v2.0.2 传统自起候选

本实现位于独立工作树 `/home/noir/.codex/worktrees/rmcs-traditional-recovery/RMCS`、分支 `dev/wheel_leg_tradition`。它已接入传统控制的组件图和 DM 速度伺服路径，尚未完成全部倒地姿态的自起验证。默认 `enabled: false`、`simulation_only: true`、`hardware_mapping_verified: false`、`handoff_enabled: false`。没有启动实车、CAN 控制或修改正在训练的 RL 工程。

## 轨迹与控制链

`SelfRightingCore` 是无 ROS、无仿真依赖的 C++ 核心。Isaac Sim 通过 `self_righting_abi.cpp` 的 ABI v6 调用同一个核心，组件 `WheelLegSelfRightingController` 直接包含它，不复制一套 Python 控制律。

状态顺序为 **7 翻正 → 3 收腿 → 2 将轮子放到身下 → 1 伸腿起身 → 4 接稳 → 5 完成**。本来正向趴地时跳过 7；已经站立时可直接接稳。左右腿各自从测得的角度开始规划，支持同前、同后、左前右后、左后右前。腿的前后位置由完整闭链几何换算，不能把两个髋电机角度当作整个腿形。

每段使用有加减速的关节空间速度轨迹，默认巡航 1.5 rad/s、参考加速度 3 rad/s²；分段交界从上一参考继续，避免用跟踪误差重新定位参考。位置外环生成有界的速度命令，电机端执行 `Kd * (v_cmd - v_actual) + feedforward`。前馈包含速度积分项、与 Isaac 相同的气弹簧补偿及可关闭的静态支撑补偿；支撑补偿只在起身/接稳时渐入，属于静态负载近似。

四轴顺序固定为 CAD 的 `L_joint1, LL_joint1, R_joint1, RR_joint1`，对应传统接口的左髋、左膝、右髋、右膝。膝角表覆盖 41–109°，站姿使用 v202 资产的约 63.672° 膝角、−0.656° 轮心轨道角。生成文件 `recovery_geometry_v202.hpp` 包含模型及气弹簧来源哈希，不适用于旧 V5/MuJoCo 模型。

翻正阶段包含有限次数的前后支撑、非对称腿形候选。设置 `flip_asymmetry_knee_deg` 后，严格侧躺的后续尝试会保持下侧腿、单独摆上侧腿。**这些侧躺候选尚未成功，不能把存在状态分支当作功能通过。** 自由云台仍保持失能，尚未实现云台辅助。

## 3.14 rad/s 的边界

速度命令限制在所设巡航速度内，巡航必须小于 3.14 rad/s。实测任一四腿轴速度超过 3.140001 rad/s，核心锁存故障并撤除主动输出；接近上限时先制动。电机前馈及估算总驱动力矩限制为 40 N·m。

DM 采用原 MIT 报文的 `Kp=0, Kd<=5` 速度伺服。保留原协议的速度量程 45 rad/s，另在命令编码前向内收一个量化码，保证有符号量化也不会把边界命令编码成超过 3.14 rad/s。不能通过改协议量程来伪造限速。

**命令限速不等于接触冲击下的实际转速保证。** 当前仿真成功案例还依赖 PhysX 的 2.8 rad/s 原生关节限速；取消该限速的对照出现实际超速。该理想限速在接触冲击下也出现过瞬时超限。DM 电机内部限速/制动、实际输出轴映射及报文反馈之间的动态力矩尚未通过台架或实车确认。软件检测发生在超速反馈之后，不能称作硬件速度保护已验证。

## 传统控制接入

- 自起输出位于 `/wheelleg/recovery/`，包含 `active/ready/phase/fault`、四轴参考角/参考速度、四轴速度伺服及前馈、两个轮力矩。
- `WheelLegDriveBridge` 仍是四个腿电机及轮力矩的唯一写入者。标定优先，自起其次，普通轨迹随后；自起故障时保持占用并撤力，不能回落到旧轨迹继续运动。
- 自起占用时，普通 `WheellegTrajectoryController` 清空自身参考与积分；站立状态机清空稳定计时，避免翻正阶段被当成已站稳。交接后普通轨迹从实际角度重新规划。
- 保留 DM 启动握手、遥控新鲜度、底层反馈新鲜度及原轮输出许可。等待 DM ready 时保持自起占用但输出为零，避免与普通轨迹争夺使能。
- 双开关 DOWN 清除全部自起参考、积分、故障、完成标志及使能状态。遥控失效时撤除输出。无遥控接口的仿真图需要显式 `/wheelleg/recovery/request`，不会因缺少遥控而自动运动。
- `handoff_enabled` 默认关闭。即使有限窗口达到完成门，也没有证据证明已能可靠接入整个 v202 传统站立/行驶控制。

故障码：1 非有限/非法反馈，2 反馈失效，3 实际超速，4 总时间到，5 段末角度未到达，6 接稳时再次倾倒，8 翻正尝试耗尽，9 DM 曾 ready 后掉线。故障只由明确复位清除；后续异常不覆盖首个核心故障原因。

## 配置和构建

`rmcs_bringup/config/wheel-leg-self-righting-v202.yaml` 是关闭状态的配置片段，不是可直接替代现有组件图的完整车辆配置。先从原图生成独立研究配置：

```bash
python3 .script/compose-wheelleg-recovery.py \
  --base rmcs_ws/src/rmcs_bringup/config/wheelleg_sim.yaml \
  --output /tmp/wheelleg-recovery-research.yaml
```

合成器检查状态估计、普通轨迹和唯一驱动桥，保留原组件，只追加关闭的自起组件；拒绝覆盖已有文件或重复自起组件。**这里的 `wheelleg_sim.yaml` 仍然是旧 MuJoCo/V5 仿真图，不能启用 v202 几何控制器用于该模型。** 此命令只验证图合成，不授权启动。

遵循仓库构建入口：

```bash
.script/build-rmcs --packages-up-to rmcs_core rmcs_bringup \
  --cmake-args -DBUILD_TESTING=ON
# 进入构建环境并加载安装空间后
cd rmcs_ws/build/rmcs_core
ctest --output-on-failure -R 'wheelleg_(self_righting_core|recovery_velocity_packet)'
```

本次 Docker/ROS Jazzy 实际构建 7 个包通过，两个 CTest 通过。核心测试覆盖四种前后组合的参考连续性及有跟踪滞后的分段切换、速度/力矩上界、超速/过期/非法反馈、故障锁存和复位；报文测试直接调用生产 `DmMotor` 编码器，检查反向、量化边界、速度增益及估算总力矩、非法请求归零。它们不是车辆行为或硬件测试。

独立 Isaac 工程在 `/home/noir/.codex/worktrees/9085/isaac_wheeled_rl_train`，分支 `dev/traditional-recovery-v202`。其 `docs/TRADITIONAL_SELF_RIGHTING_20261031.md` 和 `reports/traditional_self_righting_20261031/summary002.json` 记录实际成功窗口、失败姿态、碰撞模型负例及仍未满足的准入条件。必须先解决严格侧躺、倒扣随机云台覆盖、CAD 相交与持续接稳，再讨论实车启用。
