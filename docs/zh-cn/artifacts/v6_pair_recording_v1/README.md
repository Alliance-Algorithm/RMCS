# V6 单侧名义参考预览

生成日期：2026-09-30，协议 `pair_v6_recording_v1`，默认准入 65–100°，PD 160/2.5，50 Hz reference / 1000 Hz control。
每个 `*-plan.json` 来自生产 C++ planner；包含完整 segment、候选/实际持续时间、缩幅、频率与相位、标定及 V6 asset 哈希。
`summary.json` 汇总十个 run。图片是名义目标，不包含实测反馈、到位等待、动态中心或载荷预测。

|类型|左右计划|图片|
|---|---|---|
|静载/慢速/整圈|[L01](L01-plan.json) / [R01](R01-plan.json)|[预览](L01-preview.png)|
|动态/阶跃|[L02](L02-plan.json) / [R02](R02-plan.json)|[预览](L02-preview.png)|
|独立留出|[L03](L03-plan.json) / [R03](R03-plan.json)|[预览](L03-preview.png)|
|跳跃拟合|[LJ01](LJ01-plan.json) / [RJ01](RJ01-plan.json)|[预览](LJ01-preview.png)|
|跳跃留出|[LJ02](LJ02-plan.json) / [RJ02](RJ02-plan.json)|[预览](LJ02-preview.png)|

默认左右目标轴角互为镜像，时长相同；实测能力和拟合参数仍须分别验证。
复现命令及实际录制/导出流程见 [实施说明](../../wheel_leg_v6_recording_20260930.md)。
