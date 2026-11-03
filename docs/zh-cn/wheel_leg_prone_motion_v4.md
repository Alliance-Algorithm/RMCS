# ProneMotion-v4 当前部署

当前入口为 `wheel-leg-infantry-prone-v4.yaml`：1000 Hz executor、200 Hz PC 侧 PD、
100 Hz 策略。主控制器为 `WheelLegRlController`，观测由 `WheelLegObservation`
通过 `policy/prone_motion.hpp` 构造；硬件仅发送 MIT 纯力矩。
完整结构、接口与验证见[轮腿重构说明](wheel_leg_refactor.md)。

冻结的 35 维观测、初始化动作历史、局部里程计、嵌入 ONNX 的训练归一化及
`paired_branch107_v1` 策略目标投影继续保留。机械零点仍为膝内角 110°，
实测机械域为 40°–110°；策略冻结域与机械域分别拥有定义，不能互相替换。
腿 P/D=160/2.5、气簧增益=1、轮速 P=0.5；轮目标限制 ±90 rad/s。
只有实际用于 PD 的动作才进入下一帧动作历史。

默认关闭脚本自起；显式启用后使用传统工作区最新的共享自起核心，并保留
`simulation_only` / `hardware_mapping_verified` 限制。这是独立记录的行为迁移。
策略本身的自起能力与脚本自起验证分开评价。

模型来源、更新数、导出及 Python/C++ 推理对照见
[两路训练部署](rl_training_pair_20261103.md)。随仓库模型仍是未完成训练的候选，
没有实车验收。旧 1000 Hz PD 入口、旧模型和旧验证工具已删除。
