# 腿部级联 P/P 参数与代码更新（2026-10-27）

按用户最终选择，采用外环位置 P、内环速度 P，不使用 I，内环 D 保持 0。
这组值是可用于下一轮验证的起点，不能称为已经辨识出的动态最优解。
本次没有启动车辆。

| 参数 | 髋 | 膝 | 单位 |
|---|---:|---:|---|
| 外环位置 P | 40 | 40 | s⁻¹ |
| 内环速度 P | 4 | 4 | Nm·s/rad |
| 内环 I / D | 0 / 0 | 0 / 0 | 连续时间定义 |
| 速度目标绝对上限 | 8.5 | 8.5 | rad/s |
| 最终电机力矩绝对上限 | 40 | 40 | Nm |

扫频控制律（q、dq、力矩均先在模型坐标计算）：

```text
e = q_ref - q
dq_target = clip(dq_ref + 40*e, -8.5, 8.5)
tau_model = 4*(dq_target - dq) + tau_probe_model
tau_api = clip(model_sign * tau_model, -min(40, motor_max_torque), +min(40, motor_max_torque))
```

独立力矩探针继续在速度控制器之后叠加，最终仍共同受 40 Nm 限幅。
没有给气簧负载凭空增加恒定偏置。遥控器当前档位语义、完成日志和周期日志关闭设置保留。
速度目标限幅不代表实测速度硬上限，也不代表电机内置了速度控制：两环均在主机运行，MIT 为纯力矩模式。

## 来自数据的依据

重新读取 `左腿关节不同夹角扫频.csv`，只统计两个运行中完整的局部激励/独立验证段：
50°用 stage 2/3，75°用 7/8，100°用 12/13；没有混入整腿旋转段和过渡段。
原始控制器为位置 PD 120/4。悬空且水平，因此按用户说明不加入重力项。

| 参考 β | 实测 β 中位数 | 髋/膝模型力矩中位数 | 髋/膝电机坐标误差中位数 |
|---:|---:|---:|---:|
| 50° | 62.64° | +17.39 / −19.03 Nm | +8.29 / −9.12° |
| 75° | 87.92° | +15.47 / −16.84 Nm | +7.38 / −8.07° |
| 100° | 101.92° | +2.15 / −2.41 Nm | +1.01 / −1.18° |

P/P 没有增加额外动态状态：不限幅时展开为
`tau = (40*4)*e + 4*(dq_ref-dq) + probe`，等效位置刚度 160、速度阻尼 4。
选择 160 是在原 120 的基础上提高三分之一，4 保留已采集过的速度阻尼，避免用尚未通过独立验证的 J/B 数字整定新带宽。
两环采样率相同不表示已经证实内环带宽明显高于外环；内环速度阶跃验证仍需补充。

原高速旋转 stage 4/5、9/10、14/15 的参考速度绝对值 P99 为髋 3.403、膝 3.349 rad/s；
实测为 3.418、3.374 rad/s。按最大有效直流负载约 19.03 Nm，速度环为供给它需要约
`19.03/4=4.76 rad/s` 的误差。`3.42+4.76≈8.18 rad/s`，因此速度目标限幅选 8.5，
而不是把速度目标直接限制到 2 秒/圈的 π rad/s，否则会挤掉气簧负载需要的速度环误差余量。
这个估算没有覆盖所有气簧姿态和瞬时动态力矩，仍需观察两个限幅标志。

不加内环 D 的依据：在膝关节气簧数据的 stage 38/39、41/42 中，原始 1 ms 速度差分形成的
加速度绝对值 P99 约为髋 44、膝 88 rad/s²。它包含真实加速度、异步采样和量化，不能全部当作噪声；
但没有证据证明额外加速度反馈有收益，因此不直接叠加 D。
位置控制中的阻尼项已经由内环速度 P 提供；这里的 D=0 指内环速度微分为零。

没有 I 和气簧前馈时，静止平衡仍满足 `e=tau_load/160`。
按相同负载估计，50°点髋/膝仍可能约 +6.23/−6.81°，75°约 +5.54/−6.03°。
这些是电机坐标的局部静态估算，不能直接当作新的机械 β 误差或实测改善率；实际负载会随姿态改变。
不要通过无限增大 P 来宣称消除气簧误差。

数据摘要、源文件 SHA-256、各段行数、原始差分诊断和记录状态上的限幅比例见
[leg_cascade_pp.json](leg_cascade_pp.json)。记录状态上的新命令计算不是新控制器的闭环性能回放。
右腿采用相同起始增益，尚无同等独立辨识支持，必须分别验证。

## 代码与配置

- `wheel-leg-joints-sweep.yaml` 已选择 `cascade_pp` 并写入以上数值，继续用原来的三个扫频模式。
- `WheelLegFrequencySweepController` 新增速度目标/误差/限幅与两环增益记录；每周期先清零，停止、失联、故障和完成当周期速度目标归零。
- 原 paired identification 控制器支持 `control_law: cascade_pp`，四轴参数名为
  `kp_position`、`kp_velocity`、`velocity_target_limit`；仍要求 `ki_velocity`、`integral_limit`、
  `gravity_feedforward_model` 为零。冻结的 V6 calibrated recording 继续要求原始 PD。
- `WheelLegController` 可选 `leg_control_law: cascade_pp`，四轴参数为 `leg_position_kp`、
  `leg_velocity_kp`、`leg_velocity_target_limit`。RL 只有保持的位置目标，不对离散 actor 目标差分生成速度前馈。
  RL 的速度目标和最终力矩都保留相对角边界处理，保留电机侧限幅和故障停止。
- 原 RL/unified YAML 继续 `rl_pd`。`wheel-leg-infantry-cascade-pp.yaml` 是显式启用级联的实验配置，
  使用现有 actor，执行器变化后的策略行为尚未验收；不能将它当成训练/部署契约已经匹配。
  Unified exact-contract 路径拒绝该实验模式，硬件 MIT position_pd 也不能和主机级联同时启用。

复算命令：

```bash
/home/noir/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  .script/identification/tune_leg_cascade_pp.py \
  --data-dir /home/noir/Documents/workspace/wheel_leg扫频 \
  --cache-dir /home/noir/.cache/rmcs-sweep-archive-20261027 \
  --output reports/wheel_leg_sweep_identification_20261027/leg_cascade_pp.json
```

## 验证

构建使用开发容器中的 `.script/build-rmcs --packages-select rmcs_core rmcs_bringup`。
离线测试使用 `colcon test --merge-install --packages-select rmcs_core`。
新增测试检查 P/P 等效关系、速度限幅、非有限量拒绝、无积分残留和气簧静差；
左右扫频集成测试逐行核对速度目标、速度误差、模型/API 力矩符号、非活动侧零输出和停止归零。
RL 集成测试覆盖相对角边界、力矩上限和反馈过期停止。
最终运行结果见同目录 `leg_cascade_pp_validation.json`。以上均为离线代码/假传感器验证，不能证明实机跟踪质量。
