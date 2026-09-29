# 辨识代码收敛与验证

2026-09-28。清理范围仅辨识生产路径；本轮右腿使用启动时归档的原库，未在录包中替换程序。当前清理版本已本机构建测试，尚未同步到机器人。

- 腿控制器保留完整机构多频轨迹、50Hz目标保持、1kHz PC纯PD、所选单侧输出与原生使能/反馈接口；移除已废弃串级PI、hold/LUT规划、bounded probe、rotation-chirp分支。
- 控制器1146→760行，规划公共头876→182行；删除生产sweep头。轨迹使用std::optional直接持有；反馈、执行器、组件和录包沿用RMCS现有接口，不另造线程/控制框架。
- 显式PD为Kp位置误差−Kd实测速度，不能直接换成基于误差差分的通用PID微分而改变Kd语义。保留最小代数函数；接口中历史积分/前馈字段输出0以保持bag schema兼容。
- 生产入口只留start-pd-loaded、record-pd-loaded、start-wheels、stop、status。三个旧profile移到[历史profile](artifacts/pair_multiband_v2/legacy_profiles/)，旧实现/测试存[源码归档](artifacts/pair_multiband_v2/legacy_implementation_20260928.tar.gz)，旧bag仍按各自源码回放。离线旧profile校验保留用于历史数据审计。
- 当前腿profile右160/2.5，轮profile0.6/0/0、±4.5Nm。双下优先、反馈和驱动故障退出保留；本轮没有更改自起状态机及RL接管职责。

验证：用户指定build-rmcs构建rmcs_core、rmcs_bringup成功；5个C++ suite合计67项全部通过（DM27、规划9、腿控制器23、轮规划4、轮控制器4）。新增验证完整671s序列完成释放、必须双下后重启、右腿路由只驱动右侧；原50Hz保持/1kHz反馈重算仍通过。配置/启动Python检查48项通过、5项因需要ROS环境的条件跳过，bash语法检查通过。历史测试数减少来自对应废弃实现归档，不把减少用例伪称覆盖增加。

这是本地清理验证，不等于新清理库已完成台架实测。右腿新bag库SHA仍为baeb5a55…，回放使用归档源码；不混用当前工作树推断录包时的控制器。
