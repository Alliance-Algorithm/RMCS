# V6 原生物理与生产 C++ 接管回执

运行说明与判定见 [部署仿真文档](../../wheel_leg_v6_cpp_simulation_20261003.md)。模型、合同、CAD manifest、源码摘要及实际 bridge 参数均记录在各次 `report.json` 中。

| 目录 / 文件 | 用途 |
| --- | --- |
| `validation.json` | 261 个 C++ 测试及直接 / 100 ms / 200 ms 的组件传感器与 actor 接口检查 |
| `matrix_paced/` | 完整 10 项控制矩阵，9 项通过；430 mm 高度未通过。含 33,770 帧 ONNX 动作对照 |
| `takeover_direct/`、`takeover_100ms/`、`takeover_200ms/` | 相同四种工程初态的三组完整接管运行，每个回合 16 s |
| `takeover_comparison/` | 原始轨迹重新计算的接管指标与图；除混合时间外，记录的配置和源码摘要一致 |
| `height_high_margin_zero/` | 仅仿真取消 0.03 rad 软件保护裕度的 430 mm 诊断，物理限位保持不变；不是生产配置 |
| `gui_preview/` | 3 s GUI 渲染检查；短跑不计验收通过 |
| `smoke_02/` | 修复前静态 PREPARE 无法接管并超过 45° 的失败记录 |
| `matrix_nominal/` | 首次完整矩阵；包含主机时序间隔触发反馈保护的失败，不能代替最终矩阵 |

Git 保存报告、统计及图片。完整 200 Hz `stand*.json`、运动和高度案例原始轨迹保留在本地，并由本目录 `.gitignore` 排除。新 checkout 可按文档重新运行；`compare_v6_takeover.py` 需要原始轨迹，不只读取汇总数据。源码摘要相同不等价于运行二进制的独立证明。

这些运行使用理想 IMU 与编码器反馈，没有驱动实机，没有验证 BMI088 EKF、CAN 时序或 USB 链路。四种接管初态仅覆盖 upright 端点，**没有执行完整 V6 倒地自起序列**；接管 12/12 通过不能作为倒地自起成功率。
