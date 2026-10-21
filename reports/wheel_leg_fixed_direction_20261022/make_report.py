#!/usr/bin/env python3
"""Freeze source identities and summarize the final build and fixture results."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
analysis = json.loads((ROOT / "analysis.json").read_text())
sim = json.loads((ROOT / "isaac_final/report.json").read_text())
positive = [r for r in sim["results"] if r["integral_gain"] > 0]
negative = [r for r in sim["results"] if r["integral_gain"] == 0]
log = (ROOT / "final_build_tests.log").read_text()
assert "19 tests, 0 errors, 0 failures" in log
assert all(row["handover"] and row["mechanical_guard_passed"] for row in positive)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


sources = [
    "rmcs_ws/src/rmcs_bringup/config/wheel-leg-infantry-rl.yaml",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/joint_reference_recovery_controller.cpp",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/joint_reference_recovery_controller.hpp",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/configuration.cpp",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/leg_target_winding.hpp",
    "rmcs_ws/src/rmcs_core/src/controller/chassis/wheel_leg/recovery_tuck_controller.cpp",
    "rmcs_ws/src/rmcs_core/test/wheel_leg_recovery_tuck.cpp",
    "rmcs_ws/src/rmcs_core/tools/wheel_leg_tuck_probe.cpp",
    "rmcs_ws/src/rmcs_core/tools/probe_wheel_leg_tuck.py",
]
patch = subprocess.check_output(["git", "diff", "--no-ext-diff"], cwd=REPO)
(ROOT / "implementation.patch").write_bytes(patch)
provenance = dict(
    git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
    git_branch=subprocess.check_output(["git", "branch", "--show-current"], cwd=REPO, text=True).strip(),
    source_sha256={name: sha(REPO / name) for name in sources},
    input_sha256=analysis["source_sha256"], library_sha256=sha(ROOT / "libtrajectory_final.so"),
    simulation=sim, build_log_sha256=sha(ROOT / "final_build_tests.log"),
    deployed_ros_prefix="/workspaces/RMCS/rmcs_ws/install",
    software_gtests=16, ctest_suites=3, colcon_count_including_suites=19,
    positive_handover_and_mechanical_guard_pass_count=len(positive),
    positive_strict_settled_geometry_and_contact_pass_count=sum(r["passed"] for r in positive),
    source_patch_includes_existing_previous_turn_changes=True,
    hardware_motion_started=False, actor_in_physics_probe=False,
    physics_self_collisions_enabled=False,
)
assert provenance["library_sha256"] == sim["library_sha256"]
assert provenance["source_sha256"][sources[-1]] == sim["script_sha256"]
(ROOT / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
table = []
for row in sim["results"]:
    event = row["handover"]
    knees = " / ".join(f"{v:.2f}" for v in event["knee_deg"])
    table.append(f"| {row['case']} | {event['time_s']:.3f} | {knees} | "
                 f"{max(abs(v) for v in event['wheel_com_error_mm']):.2f} | "
                 f"{'通过' if row['mechanical_guard_passed'] else '失败'} | "
                 f"{'通过' if row['passed'] else '未通过'} |")
text = """# 固定反向自起轨迹与三倍速度（2026-10-22）

已把官方配置的自起流程改为 **保持当前朝向缩腿 → 固定方向旋转 → 当帧全部交给 RL**。缩腿和旋转参考速度均为 **1 → 3 rad/s**。构建已写入实际启动路径 `rmcs_ws/install`，16 项 GTest 全部通过；colcon 连同 3 个 CTest 套件计数为 19 项、零失败。本次没有启动实车或发送电机指令。

## 本次 CSV 与报错

CSV 有 7904 个完整样本、235 列，末尾 1 行不完整，分析中排除。四次尝试只出现 FOLD 与 PREPARE，**没有一行进入 RL 状态**。每次折腿约 0.705–0.710 秒，旋转约 0.50–0.645 秒就因 InvalidFeedback 停止；停止前还剩约 0.86–0.88 rad，左膝约 51.7°，右膝约 39.2–39.6°。

附件日志明确给出右腿实测差角 `-0.1734 / -0.1719 rad`，低于允许下限 `-0.1706 rad`。这是当前分支上的真实差角越界，不能用补整圈或放宽保护掩盖。CSV 最后几个完整样本早于日志的精确越界帧。反馈新鲜、传感器有效，所以这几次停机的直接原因不是 RL 推理超时或支撑门槛。

失败旋转在模型坐标上是 **左侧两电机增角，右侧两电机减角**。新配置 `recovery_tuck_rotation_direction: -1.0` 固定走相反分支：**左侧共同减角，右侧共同增角**。不再按腿在前后选择较短路线。CSV 和日志不能证明接管后 actor 的伸腿行为，因为本次根本未接管。

## 实现与交接

- 两段轨迹均直接使用 `recovery_tuck_speed_rad_s: 3.0`；折腿最短阶段时间同时由 60 ms 缩为 20 ms。三倍指参考速度，同一条路径的软件回归验证时长约为原来的三分之一；固定方向可能绕得更远，因此不同路线的总耗时不保证都缩短三分之二。
- 折腿保持每侧电机对的共同朝向；旋转阶段保持 40.5°目标夹角。只在折腿结束时选一次整圈分支，之后按该分支持续展开 IMU 修正，避免重新取近造成突然换向。
- 此模式直接使用 FOLD → PREPARE → RL，不进入 PLANT、THRUST、CAPTURE 或 BLEND；超过旧 55°分界也不会改用站立或蹬腿目标。无效几何会停止输出，不能退回伸腿分支。失能和重启清除旋转锚点与积分。
- 折腿与最终交接复用 0.20 rad 跟随误差容许值，计划剩余小于 0.005 rad。最终交接仍需倾角不超过 40°、驱动和反馈/策略有效；不等待高度、接地探测、静止或速度稳定，也不增加交接后脚本动作。交接后的电机动作属于 actor 输出。
- 保留真实机械限位、反馈失联检查与腿 40 Nm / 轮 4.5 Nm 上限。新增回归明确验证日志中的负向越界不会被整圈连续化抹掉。

## Isaac Sim 6.0.0.1 闭链结果

最终代码下，12 个正向场景均到达 RL 阶段，且每个 1 ms 采样的差角都在机械保护范围内。11 个通过末段严格几何和承重检查；保留 CSV 侧倾的落地夹具未通过双轮承重检查。另保留关闭积分的负对照。

| 场景 | 进入 RL 阶段 / s | 当时物理膝角 L / R / ° | 当时最大轮心前后偏差 / mm | 全程机械角检查 | 末段严格检查 |
|---|---:|---:|---:|---|---|
TABLE

CSV 起点使用第 5025 行原始四电机角度与重力方向，连续求解被动关节闭合，不将右膝截断到 105°；其初始实测约 105.2°。悬挂夹具约 1.854 秒进入 RL 阶段，两膝约 41.97°，轮心偏差约 3.4 mm；保留原始倾斜的落地夹具约 1.883 秒交接，膝角约 41.9° / 40.0°。

后者被夹具固定在原始侧倾约 5°，左轮悬空、右轮承担约 136 N；其双轮承重检查明确未通过。不能把这条结果称为稳定整车自起成功。完整结果以 JSON 为准。

**验证边界：** 使用真实闭链 USD、六个闭环约束、气弹簧目录力曲线、地面接触、生产 C++ 轨迹和实际反馈。机身姿态由夹具固定，落地夹具只允许竖直运动；资产自身过滤自碰撞，本试验不证明零件与机身无干涉。进入 RL 阶段后由测试夹具 PD 保持最后腿位，未运行物理环境中的 actor。因而“进入 RL 阶段”证明软件交接路径，不能证明自由机身 RL 平衡或实车成功。气弹簧仍是先验，未识别实车摩擦/载荷。

初次 `isaac_first` 在长路径越过整圈时暴露 PhysX 独立关节角跳变；测试入口此前缺少实车已有的整圈连续化。修复夹具后保留原失败日志，并使用相同 `wind_leg_feedback` 入口完成 `isaac_normalized`、`isaac_additional` 与最终全场景复验，没有放宽机械保护。

## 文件

- `input.csv`：本次输入副本。工作过程中原始路径被移动到回收站，从同名文件保留分析副本，SHA-256 见 `analysis.json`；未恢复或修改原文件。
- `analysis.json`、`analyze.py`：四次失败分段、原始坐标与可复算分析。
- `isaac_final/report.json`、各场景 `*_samples.json`、`isaac_final.log`：最终闭链测量与边界。
- `fixed_direction_evidence.png`：CSV 失败轨迹与新反向夹具的电机共同角/膝角对照。
- `final_build_tests.log`：实际部署路径的最终构建与 16 项 GTest。
- `provenance.json`、`implementation.patch`：源码/输入/资产/探针摘要及当前可审阅补丁；补丁含先前尚未提交的修复。
"""
text = text.replace("TABLE", "\n".join(table))
(ROOT / "README.md").write_text(text)
print(json.dumps({k: provenance[k] for k in (
    "software_gtests", "positive_handover_and_mechanical_guard_pass_count",
    "positive_strict_settled_geometry_and_contact_pass_count", "hardware_motion_started")}, indent=2))
