#!/usr/bin/env python3
"""Summarize paired Isaac recovery reports and retain their full denominators."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path


def summarize(path):
    data = json.loads(path.read_text())
    if data["status"] != "evaluated" or abs(data["simulated_seconds"] - 12.) > 1e-5:
        raise ValueError(f"Incomplete simulation: {path}")
    groups = defaultdict(list)
    for trial in data["trials"]:
        groups[trial["pose"]].append(trial)
    return {"path": str(path), "report_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "strict_pass": sum(bool(t["success"]) for t in data["trials"]),
            "ever_stable_rl": sum(bool(t["reached_stable_rl"]) for t in data["trials"]),
            "total": len(data["trials"]), "policy_sha256": data["policy_sha256"],
            "asset_manifest_sha256": data["asset_manifest_sha256"],
            "poses": {pose: {"pass": sum(bool(t["success"]) for t in trials),
                             "ever_stable": sum(bool(t["reached_stable_rl"]) for t in trials),
                             "total": len(trials), "failure": dict(Counter(t["failure"] or "none" for t in trials))}
                      for pose, trials in groups.items()}}


def plot(root):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    fig, axes = plt.subplots(2, 2, figsize=(12, 7), layout="constrained")
    for name, label, color in [("python_reference", "Python reference", "#3977b6"),
                               ("cpp_current_200hz_final", "Current C++", "#cc503e"),
                               ("cpp_diagnostic_wheel_brake_200hz", "C++ + diagnostic wheel braking", "#369367")]:
        data = json.loads((root / name / "report.json").read_text())
        index = next(i for i, t in enumerate(data["trials"]) if t["pose"] == "front_down_90")
        trace = np.load(root / name / "traces_000.npz")["values"][:, index]
        time = (np.arange(len(trace)) + 1) * .02
        axes[0, 0].plot(time, np.rad2deg(trace[:, 1]), label=label, color=color)
        axes[0, 1].plot(time, trace[:, 0], color=color)
    axes[0, 0].axhline(8., color="#777777", linestyle="--", linewidth=1)
    axes[0, 0].set(title="Forward fall: body tilt", ylabel="deg", ylim=(0, 180), xlim=(1, 10))
    axes[0, 0].legend(fontsize=8)
    axes[0, 1].axhspan(.27, .36, color="#777777", alpha=.12)
    axes[0, 1].set(title="Forward fall: true body height (scoring only)", ylabel="m", xlim=(1, 10))
    data = json.loads((root / "cpp_current_1000hz/report.json").read_text())
    index = next(i for i, t in enumerate(data["trials"]) if t["pose"] == "upright")
    trace = np.load(root / "cpp_current_1000hz/traces_000.npz")
    feedback = np.load(root / "cpp_current_1000hz/cpp_feedback.npz")["values"][:, index]
    values = trace["values"][:, index]
    time = (np.arange(len(values)) + 1) * .02
    axes[1, 0].plot(time, np.rad2deg(values[:, 1]), color="#3977b6", label="Body tilt")
    axes[1, 0].axhline(8., color="#777777", linestyle="--", linewidth=1, label="8 deg gate")
    axes[1, 0].set(title="1 kHz sensitivity: already upright", ylabel="deg", ylim=(0, 10))
    axes[1, 0].legend(fontsize=8)
    axes[1, 1].step(time, feedback[:, 16], where="post", color="#cc503e", label="Support confirmed")
    axes[1, 1].step(time, feedback[:, 13], where="post", color="#369367", alpha=.7, label="Contact candidate")
    axes[1, 1].set(title="1 kHz sensitivity: support evidence", ylim=(-.1, 1.1), ylabel="bool")
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.set_xlabel("simulation time / s")
        axis.grid(alpha=.2)
    fig.savefig(root / "handover_comparison.png", dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    names = ["python_matrix_16x5", "cpp_matrix_16x5", "python_reference",
             "cpp_current_200hz_final", "cpp_diagnostic_wheel_brake_200hz", "cpp_current_1000hz"]
    result = {name: summarize(root / name / "report.json") for name in names}
    assert len({value["policy_sha256"] for value in result.values()}) == 1
    assert len({value["asset_manifest_sha256"] for value in result.values()}) == 1
    (root / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    old, cpp = result["python_matrix_16x5"], result["cpp_matrix_16x5"]
    rows = ["# RMCS C++ 自起的 Isaac 闭环复验（2026-10-02）", "",
            "当前 C++ 自起控制器与支撑观察器尚未通过这轮完整回归。"
            "相同 16 姿态×5 个重复、12 s 观察窗口下，旧 Python 参考末尾严格通过 **62/80**，"
            "当前 C++ 为 **18/80**；曾连续稳定 RL 满 1 s 分别 **73/80、26/80**。", "",
            "8° 是机体相对正立方向的总倾角，`acos(-g_z)`；不是膝角，也不是仅前后 pitch。"
            "旧脚本在 BLEND 之前已用腿 PD、轮平衡与制动力矩扶正，ONNX 在此前仅作影子计算。"
            "当前 C++ 有倒地回弹和支撑探测未确认两类失败，不能仅归因为 8° 太严。", "",
            "## 成对矩阵", "", "|姿态|Python 末尾严格通过|当前 C++ 末尾严格通过|C++ 曾稳定 RL|",
            "|---|---:|---:|---:|"]
    for pose, old_row in old["poses"].items():
        row = cpp["poses"][pose]
        rows.append(f"|`{pose}`|{old_row['pass']}/5|{row['pass']}/5|{row['ever_stable']}/5|")
    rows.extend(["", "## 差异定位", "",
        "- 前后 90°倒地及四种劈叉在当前 C++ 完整矩阵中均 0/5。14 工况单次诊断中，"
        "当前前倒于约 3.10 s 进入 CAPTURE，随后回弹到倒置并耗尽 8 s 主动预算；"
        "同轮 Python 约 5.39 s 才开始 BLEND，当时实际倾角约 5.2°。",
        "- M3508 的轮输出轴反馈速度已经可读（驱动内换算 15.8 减速比）。"
        "它是轮相对机构的转速，区别于旧脚本重建的轮整体角速度；后者叠加 IMU、髋、被动膝及轮自身转动。"
        "当前 observer 始终将 `world_wheel_omega_valid` 置 false，因此 THRUST／壳体接触时该项制动力矩为零。",
        "- 仅在测试适配器中提供旧的传感器重建轮角速度，8°门槛不变，14 工况末尾通过由 6/14 变为 8/14。"
        "这支持轮制动缺失是一个因素；该诊断不是已接入部署的修复，也没有解决全部失败。",
        "- 该制动诊断中三种劈叉已扶正到 8° 内，仍因支撑探测未确认而超时。"
        "例如一例在 7 s 时倾角约 2.2°、高度约 0.304 m，`contact_candidate=true`、"
        "`support_confirmed=false`。应审查探测脉冲、新反馈响应及采样噪声，不能把接触候选直接当成支撑确认。",
        "- 控制／观测提高到 1 kHz、物理 2 kHz 的 14 工况敏感性测试为 0/14，均超时。"
        "其中已有正立工况仍无法通过支撑确认；该频率不是当前部署的 200 Hz PD 配置。",
        "- 另有闭链 LUT 范围保护退出。测试保留了保护，没有靠删掉检查提高分数。"
        "CAD 滑块 q 的坐标原点可为负；适配器同时平移 q 与 s0，保持压缩量和导数不变，以匹配非负滑块标定接口。", "",
        "## 条件与边界", "",
        "Isaac Sim 6.0（本机包版本6.0.0.1）／CPU PhysX，200 Hz 反馈与控制、400 Hz 物理、50 Hz ONNX；"
        "合成 105°编码器零参考、IMU／编码器扰动、24 V 条件力矩包络（100 rpm、额定20 Nm、峰值40 Nm）、"
        "气簧与真实 PhysX 接触。该资产仍是 V5 232 mm、40–110°机械域，不能把105°编码器零参考称为V6止挡资产。",
        f"策略 SHA256：`{cpp['policy_sha256']}`。资产 manifest SHA256：`{cpp['asset_manifest_sha256']}`。", "",
        "控制器只接收模拟 IMU、编码器及上一拍 PhysX 实际施加的轮力矩作为电流反馈代理；"
        "接触力、根高度与根速度只用于评分。仅 reset 写初态，其后不写根姿态翻正。"
        "模拟排队／电流响应是理想模型，没有验证 CAN／USB、BMI088 EKF、完整 RMCS Component 图或实测热预算。"
        "完整矩阵保留失败分母，严格通过要求观察窗口末尾仍连续满足站高、倾角、速度、双轮接触与机壳清离满1 s。",
        "单次14工况与80环境矩阵的布局及扰动序列不同，只各自成对比较，不跨矩阵宣称配对成功率。",
        "这轮未修改部署控制算法、未更换策略、未开启任何实机 readiness 或自起使能。", "",
        "## 入口与证据", "",
        "旧入口：训练仓库 `scripts/inspect_v5_activation.py`；动作实现 `ActivationBench.step`，"
        "传感器重建 `src/wheeled_tasks/chassis/recovery_observer.py`。",
        "新入口：RMCS `.script/simulation/inspect_recovery_isaac.py`；它直接编译部署的"
        " `recovery_controller.cpp`、`recovery_observer.cpp` 为测试桥，不另写一份 Python FSM。",
        "每组 `*.command.json` 保留完整 argv，在训练仓库目录用记录的 Isaac Python 执行；"
        "C++ 文件快照、参数表、源码 SHA 在组目录，原始50 Hz轨迹在 `traces_000.npz`，"
        "C++ 门控反馈在 `cpp_feedback.npz`，TensorBoard event 只写文件，没有启动本机 TensorBoard 服务。",
        "C++ 原始报告的列名／时钟元数据来自旧参考入口，已按 `cpp_metadata.json` 修正，"
        "修正前报告保留为 `report_before_metadata_repair.json`，物理轨迹与判分未改。"
        "可复跑入口现用父进程在 Kit 退出后完成报告元数据写入。", "",
        "[汇总 JSON](./summary.json) · [完整 Python 报告](./python_matrix_16x5/report.json) · "
        "[完整 C++ 报告](./cpp_matrix_16x5/report.json) · [C++ 命令](./cpp_matrix_16x5.command.json)", "",
        "![接管对比](./handover_comparison.png)", ""])
    (root / "README.md").write_text("\n".join(rows))
    plot(root)
    print(f"Python {old['strict_pass']}/{old['total']}; C++ {cpp['strict_pass']}/{cpp['total']}; {root / 'README.md'}")


if __name__ == "__main__":
    main()
