"""Summarize frozen component and Isaac fixture evidence; no hardware access."""
from pathlib import Path
import hashlib
import json
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
repo = root.parents[1]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
reports = [json.loads((root / d / 'report.json').read_text()) for d in ('isaac_final', 'isaac_extended')]
rows = [r for report in reports for r in report['results']]
positive = [r for r in rows if r['integral_gain'] > 0]
handover_knees = [v for r in positive for v in r['handover']['knee_deg']]
handover_error = max(abs(v) for r in positive for v in r['handover']['wheel_com_error_mm'])
files = subprocess.check_output(['git', 'diff', '--name-only'], cwd=repo, text=True).splitlines()
patch = subprocess.check_output(['git', 'diff', '--binary'], cwd=repo)
(root / 'implementation.patch').write_bytes(patch)
tests = []
for p in (repo / 'rmcs_ws/build/rmcs_core/test_results/rmcs_core').glob('*.gtest.xml'):
    tree = ET.parse(p).getroot()
    tests.append(dict(file=str(p.relative_to(repo)), tests=int(tree.attrib['tests']),
                      failures=int(tree.attrib.get('failures', 0)), errors=int(tree.attrib.get('errors', 0))))
receipt = dict(base_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
    source_sha256={p: sha(repo / p) for p in files}, implementation_patch_sha256=sha(root / 'implementation.patch'),
    model_sha256=sha(repo / 'rmcs_ws/src/rmcs_rl/models/wheel_leg/recovery_candidate.onnx'),
    installed_library_sha256=sha(repo / 'rmcs_ws/build/rmcs_core/librmcs_core.so'),
    installed_prefix='/workspaces/RMCS/rmcs_ws/install',
    probe_library_sha256=sha(root / 'libtrajectory.so'),
    tests=tests, actual_gtest_count=sum(t['tests'] for t in tests),
    isaac_positive_cases=len(positive), isaac_positive_passes=sum(r['passed'] for r in positive),
    handover_knee_range_deg=[min(handover_knees), max(handover_knees)],
    handover_max_sagittal_error_mm=handover_error,
    scope='No CAN/hardware launch. Component test uses real ONNX and mocked sensors. Isaac fixtures hold body attitude, use actual closed chains, and hold the legs with fixture PD after the RL-phase event; no free-body actor recovery acceptance.')
(root / 'provenance.json').write_text(json.dumps(receipt, indent=2, ensure_ascii=False)+'\n')
lines = ['| 场景 | 交接时间 / s | 交接膝角 L / R | 交接最大前后偏差 / mm | 夹具保持后的膝角 L / R |',
         '|---|---:|---:|---:|---:|']
for r in positive:
    h = r['handover']
    lines.append(f"| {r['case']} | {h['time_s']:.3f} | {h['knee_deg'][0]:.2f} / {h['knee_deg'][1]:.2f} | {max(map(abs,h['wheel_com_error_mm'])):.2f} | {r['final_knee_deg'][0]:.2f} / {r['final_knee_deg'][1]:.2f} |")
text = '''# 双中立即失能与自起接管修复（2026-10-22）

修改已在实际启动脚本使用的 `rmcs_ws/install` 完成构建。12 项腿部回归与 2 项组件集成通过；colcon 汇总为 17 项，其中包含 3 个 CTest 套件计数。Isaac Sim 6.0.0.1 的 10 个正向闭链夹具场景全部完成脚本到 RL 阶段的交接。

## 直接失能的原因

最新日志中 `rl_bridge valid=1`、`sensor_valid=1`、`feedback_fresh=1`，失能位置是旧 `action.cpp:238` 的机械差角检查。右腿直接拿编码器分支算得 `-4.9422 rad`，加一个整圈后实际应为 `1.3409853 rad`，落在标定上限 `1.3889088544 rad` 内。此前支撑观察器修复了这个分支，但主控制器的保护与 PD 使用原始分支，导致同一帧各处解释不一致。

现在在读取模型反馈时统一恢复 paired-107 分支，观测、恢复、PD 和机械检查使用同一组角度。只恢复整圈，不截断真实机械越界；真实越界会打印腿侧、实测值与上下限，并记录 InvalidFeedback。

此前 CSV 中另外两个卡点也已处理：FOLD 在负载下因差角误差大于 0.24 rad 无法推进；最终收腿积分要求所有轴误差先小于 0.15 rad，实际最小误差约 0.160 rad，导致补偿一直无法启用。旧 CSV 的证据保留在相邻 `wheel_leg_csv_20261021_155021` 报告中。

## 当前接管行为

官方 YAML 开启 `recovery_trajectory_takeover: true`。近直立起步（倾角小于 55°）使用两段有速率限制的轨迹：先保持当前腿朝向折叠，再把折叠后的轮心转到整个机器质心的重力下方。目标膝角保持 40.5°。大倾角起步仍使用原有翻转／侧摆阶段，进入捕获后使用最终收腿轨迹。

折腿轨迹推进后不再等待旧的实测差角门槛。最终交接只要求：

- 最后一段计划轨迹剩余小于 0.005 rad；
- 实测关节相对执行参考的最大跟随误差小于 0.20 rad（这是主动关节角，不是膝夹角）；
- 机身总倾角不超过 40°，反馈、驱动与策略输出有效。

满足后当帧把全部六轴交给 RL，混合时间为 0，并立即释放驾驶指令。不再要求接触探测通过、条件高度在 0.20–0.29 m、加速度支持证据、陀螺／腿速／轮速低于旧接管阈值、40–41°与 5 mm 的严格几何就绪，也没有站稳等待。严格几何就绪仍作为诊断数据保留。

积分改为跟踪正在执行的限速参考，折腿阶段即可启用；逐轴抗饱和，补偿限制 20 Nm，总腿力矩仍限制 40 Nm。失能、失败和正式 RL 阶段清除补偿。反馈最大年龄改为 40 ms、跨源时差 30 ms，控制间隔容许 40 ms，覆盖 CSV／日志中的约 22–24 ms 间歇；更长的失联、无效数据和 RL 阶段超过 45°倾角仍会停止输出。

CSV 和广播已加入 21 个诊断量：收腿有效／启用／严格就绪、左右膝角与轮心偏差、每轴执行参考／最终目标／补偿力矩、轨迹剩余量与跟随误差。阶段转换也会打印上述关键数值。

## 验证

组件测试使用真实 `WheelLegController`、`RlBridge`、ONNX 和执行器，输入由内存夹具提供。按日志构造左／右差角 `1.2945 / -4.9422 rad`，无支持加速度、gyro=3 rad/s 时，41.25°倾角保持 PREPARE；改为约 34.4°后，完成 `FOLD → PREPARE → RL`，全轴释放。另验证 reset／重启、策略有效性、超时反馈失能和输出力矩界限。它不构造硬件传输。

Isaac 使用实际闭链 USD、六个闭环约束、气弹簧力与地面接触；五个原始悬挂姿态、一个对齐承重姿态、两个 104°张角姿态及两个腿朝前／朝后的落地姿态。所有正向场景交接时膝角为 **{kmin:.2f}–{kmax:.2f}°**，轮心到质心的最大前后偏差为 **{err:.2f} mm**。因此交接没有被静态 40–41°门槛拖住。

{table}

对齐承重夹具两轮平均约 68.15 / 68.09 N，保持阶段没有其它部件承重；闭链最大闭合误差约 0.051 mm。关闭积分的承重负对照最终约 41.20°，没有通过严格 40–41°几何验证，但仍能按新的宽容接管条件交接。

初次试验保存在 `isaac_trajectory`：终点剩余量为 0.02 rad 时，承重保持膝角约 41.15°，严格验证未过。最终改为完整走到 0.005 rad 内，最终正向场景全部通过；没有改宽严格几何验证的标准。

**证据边界：** Isaac 机身姿态由夹具固定，承重夹具允许竖直运动。到达 RL 阶段后由夹具 PD 保持最后腿位以验证几何，表中“夹具保持后”不表示 RL 运行结果。气弹簧仍是目录曲线先验，最大曲线夹持暴露约 0.8 mm。这些测试不能证明自由机身 RL 自起或实车已成功。本次没有启动实车、发送 CAN 指令或训练策略。

- 实际部署目录构建与测试：`deployment_build_tests.log`
- 闭链试验：`isaac_final/report.json`、`isaac_extended/report.json` 及逐时刻 samples
- 软件与资产摘要：`provenance.json`
- 可审阅补丁：`implementation.patch`
'''.format(kmin=min(handover_knees), kmax=max(handover_knees), err=handover_error, table='\n'.join(lines))
(root/'README.md').write_text(text)
print(json.dumps({k:receipt[k] for k in ('actual_gtest_count','isaac_positive_cases','isaac_positive_passes','handover_knee_range_deg','handover_max_sagittal_error_mm')},indent=2))
