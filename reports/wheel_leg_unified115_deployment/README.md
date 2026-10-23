# unified115 当前部署移植回执

新入口：`wheel-leg-infantry-unified-rl.yaml`，模型为
`rmcs_rl/models/wheel_leg/self_wake_V1.0.onnx`。旧模型及旧 YAML 保留。
来源包：`isaac_wheeled_rl_train/models/unified_getup115_test_200hz_20261023`。

当前移植保留此前删除脚本自起的修改，只合并导出包对应的完整观测帧、
站立轮坐标位移、已执行动作历史和桥接合同检查；没有恢复脚本代码。
三个冻结观测 helper 的内容哈希与源包一致。其余控制器在当前代码上合并，
以 `contract.json` 中的源码及 YAML 哈希绑定本次结果，
不宣称与包内父/子仓库提交 ID 相同。

部署参数：200 Hz 策略、1000 Hz host PD、35 维单帧、6 动作；
腿部 raw ±16、偏移 ±4 rad，轮子 raw ±9、速度 ±90 rad/s。
默认高度 0.305 m，航向保持关闭。机械标定与硬件配置沿用当前车辆配置；
反馈时效、机械限位、40/4.5 N·m 力矩帽及故障锁存继续生效。

验证结果：

- `build-rmcs --packages-select rmcs_rl rmcs_core rmcs_bringup --cmake-args -DBUILD_TESTING=ON`
  成功。现有 CMake/gtest 及 ament 头文件安装提示保留在 `build.log`。
- `colcon test --merge-install --packages-select rmcs_rl rmcs_core` 全通过，
  随后补充的 unified 错误配置/故障锁存用例单独重建并运行通过。
  最终 25 个独立 gtest，0 失败、0 跳过，详情见 `tests.json`。
- 生产控制器、真实 ONNX、桥、执行器使用内存传感器检查倒置/侧倒接入、
  零力矩握手、首帧历史、已执行历史、复位重启、陈旧反馈停机及故障锁存。
  最终运行 71 帧，序列化误差和已执行历史误差均为 0。
- 新模型 256 组冻结输入最大 raw action 误差 `1.9073486328125e-6`。
- 观测 helper 回放训练侧 35,158 帧，其中 35,137 帧有效；无效拒绝完全一致。
  完整 35 维及位移槽最大误差 `4.76837158203125e-7`，
  测量首帧历史最大误差 `1.9073486328125e-6`。
  `observation_parity.json` 记录输入、训练侧期望及当前输出哈希。

合同复查工具为 `rmcs_core/tools/verify_wheel_leg_unified.py`，
命令和模型观测说明见 `docs/zh-cn/rl_deployment_refactor.md`。
工具不会修改 ONNX 或创建硬件组件。

仅完成软件部署及离线接口验证，没有启动 CAN 或真实车辆，也没有运行新的
物理仿真。模型来源包的 `behavior_qualified=false`、`hardware_tested=false`
保持不变；本回执不能作为闭环自起行为验收。
