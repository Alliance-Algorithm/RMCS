# 腿部系统辨识激励调整（2026-10-27）

本次 CSV 没有真正启动扫频。66196 条完整记录、66.4749 秒内，`state=6`、
`stop_reason=2`、`dr16_fresh=0`、拨杆 UNKNOWN、使能请求为零。
因此不能用此数据推断 PD 较小或估计机械阻尼、执行器延迟。
完整机器可读检查见 [CSV 核对](unusable_joint_sweep_audit.json)。

新关节扫频已加入低速与高速激励：

| 模式 | 激励 | 主要用途 |
|---|---|---|
| 1（左上右中） | 0.2 / 0.6 / 1.5 / π rad/s，正反各两圈 | 摩擦、阻尼、方向和速度差异；高速平台 2 s/圈 |
| 2（左中右上） | β=45–105°、3/12/30°/s，随后局部 0.1–10 Hz 扫频 | 开合方向的速度响应和跟踪 |
| 3（双上） | β=50/75/100°局部双轴力矩探针及验证段，之后 2 s/圈高速旋转；60/90°高速验证 | 局部耦合参数、独立输入、局部/高速延迟验证 |

主机采集 PD 初值由 60/2 改为 120/4，主机与驱动最大力矩均为 40 Nm。
参考角度留在 40–110°参考范围内，当前 β 计划范围为 45–105°；遥控和反馈路径按当前配置执行。
这组 PD 初值尚未经过实车验证，不称为优化结果。

新增离线工具可拒绝无激励数据，在完整局部训练/验证段上拟合 M/B/库仑项/局部载荷，
分别估计局部与高速命令到电流反馈的等效延迟，然后生成带延迟、限幅的离线 PD 候选。
候选不会自动写回配置。局部模型不等于整车 real2sim 标定，尤其不能把耦合 B 矩阵直接当作
四个独立关节的 URDF damping。方法、使用命令及适用范围见
[扫频文档](../../docs/zh-cn/wheel_leg_frequency_sweep.md)。

仅完成离线构建、内存反馈控制器测试及合成信号拟合验证。本次未启动实车、未进行物理 CAN 时序或 RL 仿真验收，
因此没有输出实车优化 PD 或合格的真实动力学参数。

验证结果：

- 通过 `.script/build-rmcs --packages-select rmcs_core rmcs_bringup` 完成两包构建。
- 扫频相关 6 个 CTest 目标通过，包含 23 个 GTest 用例；包内已有结果合计 56 tests、0 errors、0 failures。
- 离线分析 6 项测试通过：未启动数据拒绝、共线激励拒绝、已知耦合参数回收、已知延迟回收、
  超过浮点整数精度的纳秒时间戳保存、完整分析及无命令帧的负对照。
- 核对插件注册、YAML 参数/235 个唯一采集信号和双下复位；关节 CSV 正式输出 243 列。

本次文件改动：`rmcs_ws/src/rmcs_core/src/identification/wheel_leg_sweep_plan.hpp`（轨迹）、
`rmcs_ws/src/rmcs_core/src/identification/wheel_leg_frequency_sweep_controller.cpp`（力矩探针与日志）、
`rmcs_ws/src/rmcs_bringup/config/wheel-leg-joints-sweep.yaml`（采集参数）、
`rmcs_ws/src/rmcs_core/test/wheel_leg_frequency_sweep.cpp`（回归）、
`.script/identification/analyze_joint_sweep.py` 与 `joint_sweep_analysis_test.py`（离线分析与测试），
以及扫频文档和本报告。插件名称、生产 RL 配置及双下复位逻辑沿用当前实现。

后续按操作要求将关节扫频两层限幅调为 40 Nm，注释执行器每 5 秒打印组件耗时占比的调用，
并增加每轮正常结束时仅打印一次的 `Sweep complete` 日志。三个遥控模式按上表选择，
当前配置锁存运行，双下中止；完成后回双中可清除完成锁存，再选下一模式。
