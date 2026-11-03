# 2026-11-03 传统自起验证

[方案与测试表](../../traditional_self_righting_20261103.md)；[每 0.3 s 的 33 帧观察](video_0p3s/README.md)。

`verification.json` 是最终源码、配置与全部 5 个原生工况的总回执。`build.log` / `tests.log` 对应自起 5 项测试；`traditional_build.log` / `traditional_tests.log` 对应共享传统控制 4 项测试，全部通过。`configuration.json` / `composition.log` 仅验证插件、51 个参数、唯一实例及静态图，不表示已运行完整 ROS 图。

每个 `native/<case>/` 含 `receipt.json`、完整 1 ms `receipt.npz`、`native.log`、审计 `verification.json`、轨迹图及 TensorBoard events。NPZ 的 `trace` 为控制窗口，`all_joints` / `roots` 每 30 ms 采样，可用于回放观察；回放不替代原生物理测试。原运行路径保留在回执中，归档副本位置以本目录为准。

| case | 结果 |
| --- | --- |
| `rear_prone` | 正趴后伸腿，自起 10.132 s + 传统控制 10 s，通过 |
| `front_prone` | 正趴前伸腿，自起 10.041 s + 传统控制 10 s，通过 |
| `forward30` | 前倾 30° 失能沉降后正趴，自起 10.314 s + 传统控制 10 s，通过 |
| `forward75` | 前倾 75° 沉降后倾斜倒扣，4.649 s 实测腿轴超速故障 3，失败 |
| `inverted180` | 倒扣安放，沉降后倾斜倒扣，5.206 s 实测腿轴超速故障 3，失败 |

这五项是确定初态测试，没有统计成功率资格。三条通过均完整运行交接后 10 s，失败未接管传统控制。所有工况仅初始化安放写物理状态，之后零力矩沉降 3 s，再仅用电机力矩驱动原生 PhysX。没有启动 PPO、实车/CAN 或云台主动控制。

范围：v202 整车原生闭链、实体气簧、已辨识轮响应与腿库仑摩擦，CPU 1 ms、TGS 64/32；既有 plant **关闭连杆间自碰撞**，云台固定零位。测试衔接生产共享 C++ 内核，未运行完整 ROS 调度图和真实电机映射。整体配置 `enabled: false`、`simulation_only: true`、`hardware_mapping_verified: false`；达到接稳门后的仿真交接开关为 true。

`trace` 列：时间、阶段、pitch、roll、根高度、真实前向速度、最大闭链误差、左右轮接触合力、左右腿长、17 维编码器/IMU packet、6 电机命令力矩、膝机械越界量、气簧越界量。packet 顺序为 6 位置、6 速度、pitch、pitch rate、yaw、yaw rate、比力模；四腿轴顺序左髋/左膝/右髋/右膝，随后左右轮。
