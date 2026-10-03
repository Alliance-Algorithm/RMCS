#!/usr/bin/env python3
"""Summarize paired Isaac recovery tests, retaining every failed replica."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "before", "python", "cpp"):
        parser.add_argument("--" + name, type=Path, required=True)
    options = parser.parse_args()
    paths = {"cpp_before": options.before, "python_reference": options.python, "cpp_aligned": options.cpp}
    reports = {name: json.loads((path / "report.json").read_text()) for name, path in paths.items()}
    for key in ("policy_sha256", "asset_manifest_sha256"):
        if len({report[key] for report in reports.values()}) != 1:
            raise ValueError("Comparison changes " + key)
    case_sets = [{(t["pose"], t["method"], t["replica"]) for t in r["trials"]} for r in reports.values()]
    if not all(cases == case_sets[0] for cases in case_sets):
        raise ValueError("Unpaired pose/method/replica sets")
    cpp = reports["cpp_aligned"]
    poses = list(dict.fromkeys(t["pose"] for t in cpp["trials"]))
    outcomes = {name: {"total": len(r["trials"]), "strict_success": sum(t["success"] for t in r["trials"]),
                       "ever_stable_rl": sum(t["reached_stable_rl"] for t in r["trials"]),
                       "failures": dict(Counter(t["failure"] or "terminal_stability_dwell"
                                                for t in r["trials"] if not t["success"]))}
                for name, r in reports.items()}
    per_pose = {pose: {name: sum(t["success"] for t in report["trials"] if t["pose"] == pose)
                       for name, report in reports.items()} for pose in poses}
    tick = np.load(options.cpp / "cpp_ticks.npz")
    profile = json.loads((options.cpp / "simulation_profile.json").read_text())
    geometry_failures = []
    for i, trial in enumerate(cpp["trials"]):
        if trial["failure"] != "invalid_feedback":
            continue
        k = int(np.flatnonzero(tick["outputs"][:, i, 7] == 1)[0])
        sample, sides = tick["inputs"][k, i], []
        for side, table in enumerate(profile["sides"]):
            delta = sample[2 * side + 1] - sample[2 * side]
            low, high = table["delta"][0], table["delta"][-1]
            sides.append({"delta_rad": float(delta), "lut_bounds_rad": [low, high],
                          "excess_rad": float(max(low - delta, delta - high, 0.))})
        geometry_failures.append({"pose": trial["pose"], "replica": trial["replica"],
                                  "time_s": k / cpp["feedback_hz"], "sides": sides})
    parity = json.loads((options.cpp / "sensor_parity.json").read_text())
    summary = {"date": "2026-10-03", "horizon_s": cpp["arguments"]["seconds"],
               "feedback_hz": cpp["feedback_hz"], "physics_hz": cpp["physics_hz"],
               "policy_sha256": cpp["policy_sha256"], "asset_manifest_sha256": cpp["asset_manifest_sha256"],
               "runs": outcomes, "per_pose": per_pose, "geometry_failures": geometry_failures,
               "sensor_parity": parity, "hardware_ready": False, "v6_stop105_tested": False}
    (options.root / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    figure, ax = plt.subplots(figsize=(16, 6), constrained_layout=True)
    labels = ["upright", "upright\nrelease", "crouch", "crouch\nrelease", "pitch\n+45", "pitch\n-45",
              "front\n90", "back\n90", "left\nside", "right\nside", "front\nsplit A", "front\nsplit B",
              "back\nsplit A", "back\nsplit B", "belly\nlegs up", "back\nlegs up"]
    x = np.arange(len(poses))
    for offset, name, title, color in ((-.25, "cpp_before", "C++ before", "#89909b"),
                                     (0., "python_reference", "Python reference", "#37956c"),
                                     (.25, "cpp_aligned", "C++ aligned", "#3f78c4")):
        result = outcomes[name]
        ax.bar(x + offset, [per_pose[p][name] for p in poses], .24,
               label=f"{title}: {result['strict_success']}/{result['total']}", color=color)
    ax.set(xticks=x, xticklabels=labels, ylim=(0, 5.7), yticks=range(6), ylabel="Strict successes / 5 replicas",
           title="Isaac V5 232 mm | 200 Hz control, 400 Hz physics | 12 s per trial")
    ax.grid(axis="y", alpha=.2)
    ax.set_axisbelow(True)
    ax.legend(loc="upper center", ncol=3)
    figure.savefig(options.root / "alignment_comparison.png", dpi=160)
    plt.close(figure)
    lines = ["# 自起 C++ 对齐与 Isaac 闭环复测", "", "更新：2026-10-03。分支 `dev/wheel_leg_rl`。", "",
             "使用原 V5 232 mm、40–110°机械资产和相同 V5 ONNX。105°只是模拟 DM 编码器零位，不是V6止挡资产。",
             "控制/反馈200 Hz、物理400 Hz、策略50 Hz；每例12 s，动作预算8 s，混合接管200 ms。", "",
             "| 实现 | 末尾严格通过 | 曾连续稳定 RL ≥1 s |", "| --- | ---: | ---: |"]
    for name, label in (("cpp_before", "修改前 C++"), ("python_reference", "Python 原参考（10-03重跑）"),
                        ("cpp_aligned", "最终 C++")):
        r = outcomes[name]
        lines.append(f"| {label} | {r['strict_success']}/{r['total']} | {r['ever_stable_rl']}/{r['total']} |")
    lines += ["", "![逐姿态对照](alignment_comparison.png)", "",
              "严格通过要求末尾连续1 s：100% RL输出、倾角<10°、高度305±20 mm、平面速度<0.2 m/s、"
              "角速度<0.5 rad/s、两轮接触力>2 N、非轮接触力<5 N，且无机构失效。评分真值不送入控制器。", "",
              "| 姿态 | 修改前 C++ | Python | 最终 C++ |", "| --- | ---: | ---: | ---: |"]
    for pose, a in per_pose.items():
        lines.append(f"| {pose} | {a['cpp_before']}/5 | {a['python_reference']}/5 | {a['cpp_aligned']}/5 |")
    lines += ["", "最终失败分母：`" + json.dumps(outcomes["cpp_aligned"]["failures"], ensure_ascii=False) + "`。",
              "6次反馈失效均是主动轴角差超过 LUT 外的2 mrad容差，右侧超过表端点约2.31–9.51 mrad；保护未删除。"
              "8次超时主要是CAPTURE回弹，仰躺双腿朝上0/5，Python也只有1/5。另8次曾稳定RL满1 s，"
              "但末尾连续稳定时间不足1 s，未剔除。", "",
              "修复了 IMU+髋/被动膝/轮轴合成制动、标定轴投影、气簧节点导数、条件高度选路、FOLD跟踪域、"
              "ORBIT/CAPTURE接触条件、着地判据、15 ms连续探测时钟、LUT节点选择及BLEND预算。"
              "新电流/发送时序验证、8°接管门槛、机械保护及实机禁用默认值保留。", "",
              f"同输入回放检查{parity['checked_sensor_rows']}条有效记录，最大误差`"
              + json.dumps(parity["maximum_absolute_error"]) + "`；此项检查数值运动学，不声称全部状态机位级一致。", "",
              "`cpp_ticks.npz`保存200 Hz的29列输入和39列输出；`traces_000.npz`保存50 Hz评分和诊断。"
              "每次保留源码、SHA、profile、命令和TensorBoard事件。中断运行另存，不计入完成试验。", ""]
    for name, path in paths.items():
        lines.append(f"- `{name}`：[原始报告]({path.resolve()}/report.json)。")
    lines += ["", "复跑参数见同目录`*.command.json`。同输入检查入口："
              "`.script/simulation/check_recovery_sensor_parity.py --training-repo <训练仓库> --run <C++运行目录>`。", "",
              "尚未验证完整RMCS硬件图、CAN/USB/BMI088 EKF、实测热预算和V6止挡资产；此次未驱动实机。"
              "`recovery_enabled`和readiness仍为false。"]
    (options.root / "README.md").write_text("\n".join(lines) + "\n")
    print(json.dumps(outcomes, indent=2))


if __name__ == "__main__":
    main()
