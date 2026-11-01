# v2.0.2 气簧补偿

`WheelLegDriveBridge` 的普通腿 PD 路径支持 RL 同源的测量气簧模型。生成头文件 `wheelleg_gas_spring_v202.hpp` 保留每侧 281 个几何节点、16 个实测轴向力节点及模型 SHA。测量覆盖 56°–101°；40°–110°几何范围内的其余部分使用 RL 相同的端点趋势延拓，右侧力曲线使用已授权的左侧对称假设，不称为重新辨识。

四个 API 编码器经显式符号和 110° 零位偏置映射到 CAD 主动轴坐标，再分别计算各腿 `tau_hold = -J_length(q)^T F(compression(q))`，同时作用于髋和膝驱动轴。多圈反馈只规范化相对绕圈，不发送电机置零命令。边界容许 1e-12 rad 双精度舍入误差；更大的越界与 RL 一样返回零补偿。

顺序：PD + 既有外部前馈 + 气簧保持力矩 → 总电机力矩限幅至不超过 40 N·m → RL 同源的外向相对力矩投影（0.03 rad 保护带）→ 映回 API。无效几何的一侧补偿为零并保留外向保护；非有限输入、安全停机、未使能与复位撤除主动输出。补偿没有积分状态。自起已有 `recovery_v202::compensation`，标定透传也由独立路径负责，二者在桥中提前返回，不重复加补偿。

`wheelleg-gas-spring-v202.yaml` 是启用增益 1 的配置片段，绑定源模型 SHA 和四轴标定；普通桥默认关闭该选项。为现有 **v2.0.2 真机传统图** 生成新配置：

```bash
python3 .script/compose-wheelleg-gas-spring.py \
  --base /path/to/current-v202-hardware.yaml \
  --output /tmp/wheelleg-v202-gas-spring.yaml
```

合成器要求唯一驱动桥和 WheelLeg 硬件节点，拒绝重复使能、覆盖及旧 MuJoCo/V5 图。它不启动 RMCS。若自定义图另有气簧前馈生产者，不能再重复接入普通桥。偏置沿用确认的接线及 110° 参考，独立实车多点校验仍待完成。

诊断为 `/wheelleg/gas_spring/geometry_valid` 及四路 `/wheelleg/gas_spring/<joint>/holding_torque_nm`。插件类与唯一硬件力矩生产者保持不变。

同源生成入口是调参仓库 `scripts/sync_rl_real2sim.py`。传统 Isaac 另外移植腿电机的被动库仑摩擦；该力不发送给真机，也不在这里添加库仑摩擦抵消。验证回执在调参仓库 `docs/validation/real2sim_20261102/`，数学/构建/接线检查不等于实车行为验收。本次没有硬件或 PPO 操作。
