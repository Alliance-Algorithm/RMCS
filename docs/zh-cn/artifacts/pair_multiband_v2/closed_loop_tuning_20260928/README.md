# 离线探索制品的适用范围

当前任务已明确为 RL 下层 PC PD 的执行器辨识。权威核对见 `rl_pd_contract_audit.json` 与上层文档 `wheel_leg_rl_pd_identification_20260928.md`。

- `response_1khz.npz/.json`：保留完整采样率的原包字段，不是训练执行器参数。
- `loop_analysis.json`：采集版本 P＋PI 串级的代数/频谱及反馈报告动态诊断。
- `passive_surrogates.json`：5–90 Hz 有效双轴局部模型；残差使用实测状态，不等于完整闭链自由回放误差。
- `gain_screen.json`、`pd_gain_screen.json`、`surrogate_replays.*`：1 kHz 局部模型控制探索，不复现 RL 的 200 Hz 力矩保持与 50 Hz 目标保持；纯 PD 探索也非 RL 的无速度前馈形式。不能据此发布 RL 增益或写入训练。
- `audit_rl_pd_contract.py`：重算 687,301 个运行样本的原控制代数，核对训练/部署 PD 源码与归档合同，输出带源文件哈希的审计结果。

本目录未包含通过验证的 nonlinear MuJoCo 闭链拟合或正式训练参数，不用于自动部署。

用户最终确认的新采集时序为 50 Hz 目标、1000 Hz PD。新增 `rl_pd_1khz_screen.json` 是该时序的局部敏感性筛查；`pd_recording_candidate.json` 记录候选与局限，`pd_recording_ready_050017.json` 记录真实运行参数握手。旧 `rl_pd_200hz_screen.json` 只保留作旧时序对照。
