# RL 部署接口

通用模型加载和异步推理由独立子模块 `rmcs_rl` 负责。车辆坐标、机构、
观测、脚本自起和 PC 侧 PD 由 `rmcs_core` 负责。当前轮腿结构及验证结果见
[轮腿重构说明](wheel_leg_refactor.md)，两套模型的训练来源见
[两路训练部署](rl_training_pair_20261103.md)。

`RlBridge` 的动作发布 partner → 主控制器 → 观测组件 → 推理提交构成单向调度链。
推理线程返回的结果在后续周期发布，避免 executor 循环依赖。
启停、复位代次、结果时龄和有限性通过通用桥处理；控制器拥有最终输出许可。
显式配置的 enable/reset 接口必须配对，缺失输入在启动阶段报错。

YAML 中使用 `rl_model_path`、张量维度、观测合同及 `expected_metadata.<key>`
声明模型要求。相对模型路径以安装后的 `rmcs_rl` share 目录为基准。
轮腿两套适配器使用 raw action；缩放、绕圈和动作历史在适配器与主控制器处理。
修改模型时须一起核对观测、频率、归一化和动作约定。

Deformable 继续通过同一桥向 `DeformableRlSuspension` /
`DeformableSuspensionArbiter` 提供动作。其 transformer 模型、历史和动作契约未改变，
实际插件集成回归已通过。

旧 `wheel-leg-infantry-rl` / `wheel-leg-infantry-unified-rl`、旧策略模型和专用工具
已退出当前构建。历史结果仍记录各自当时的源码和配置，不代表当前接口。
通用配置说明见子模块 [README](../../rmcs_ws/src/rmcs_rl/README.md)。
