# 2026-11-04 传统控制真机配置离线验证

回执中的工作树路径、`head`、`changes_committed` 和 `changes_pushed` 均记录测试发生时的状态；发布提交由 Git 历史确认。临时工作树清理后，这些路径仍作为原始来源保留。

- [verification.json](verification.json)：工作树、分支、源码/配置 SHA256、构建及测试结果、未验证范围。
- [profile_source.json](profile_source.json)：硬件配置来源与 v202 模型、姿态、气簧表哈希。
- [build2.log](build2.log)：新增插件和测试首次成功构建，含 `rmcs_bringup` 安装。
- [build_full.log](build_full.log)：随后完整 `--packages-up-to rmcs_core rmcs_bringup` 依赖构建，6 包通过。
- [build_final.log](build_final.log)：初始化锁定 FastTF 子模块并清理 CMake 缓存后，最终 7 包构建通过。
- [dependency_paths.json](dependency_paths.json)：最终 CMake 依赖均绑定本传统工作树。
- [install_check.json](install_check.json)：仅加载 ROS 与本工作树安装空间后，实际包前缀及安装 YAML 的来源和 SHA256。
- [finaltests.log](finaltests.log)：最终全部 9 项 CTest 通过（IMU 内含 11 项 gtest，colcon 共 20 项结果）。
- [offline_parity.json](offline_parity.json)：新旧内核及参数逐项核对，两个正向工况各 20000 包回放、16 维输出差值为零。
- [replay_profile.py](replay_profile.py)：生成上述离线等价回执的脚本快照；使用本机原生 Isaac 环境内的 NumPy/PyYAML 和 `wheelleg_tuning.native`，不启动 Kit 或硬件。

集成测试读取实际 `wheel-leg-traditional-v202.yaml` 并加载生产自起、传统控制与广播插件，以内存传感器替代硬件。覆盖安装映射、遥控解锁、DM 就绪、MIT 速度伺服、接稳交接、独立整圈、固定零指令、过期反馈、超速、复位和遥控失效。没有实例化 `hardware::WheelLeg`，没有打开 CAN；该结果不能替代实车验证。

此前物理资格范围见 `/home/noir/Documents/workspace/wheelleg_controller_tuning/docs/SELF_RIGHTING_RETEST_20261103.md`：两个正向工况通过自起后 20 s 站立，180° 倒扣仍触发故障 3。本轮未重新运行 Isaac 物理或 PPO。
