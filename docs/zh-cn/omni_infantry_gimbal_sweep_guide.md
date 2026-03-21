# omni-infantry 云台扫频使用指南

本文记录 `omni-infantry` 云台扫频的完整使用流程，适用于：

- 采集 yaw / pitch 开环扫频数据
- 估计 `扭矩 -> 角速度` 频响
- 基于扫频结果整定 `OmniInfantryPlannerGimbalController`

当前代码库中的扫频组件为：

- `rmcs_core::controller::gimbal::OmniInfantryGimbalSweep`

对应实现文件：

- `rmcs_ws/src/rmcs_core/src/controller/gimbal/omni_infantry_gimbal_sweep.cpp`

## 1. 组件行为说明

`OmniInfantryGimbalSweep` 的行为与正常云台控制器不同，使用前需要先明确：

- 它会直接接管 `/gimbal/yaw/control_torque` 和 `/gimbal/pitch/control_torque`
- 它不接入 `OmniInfantryPlannerGimbalController`
- 扫频信号是开环扭矩 chirp，不经过角度环 / 速度环
- 它会将 `/gimbal/yaw/control_angle_error` 和 `/gimbal/pitch/control_angle_error` 固定为 `0.0`
- 仅当遥控器处于 `左中右上` 时，扫频自动开始
- 只要离开 `左中右上`，扫频就会停止并复位输出

因此，扫频期间机器人不会执行正常云台闭环控制。采集前必须确认机械固定、人员避让和安全边界。

## 2. 使用前准备

建议在以下条件满足后再开始扫频：

- 目标车已经通过 `ssh: remote` 可达
- 本地工作区能够正常 `build-rmcs`
- 已经启动远端同步进程，或准备启动
- 云台和底盘处于安全固定状态
- 已确认扫频时不会误触发射击链路

特别说明：

- 如果 yaw 轴机械链路为 `电机 -> 同步带 -> 云台(IMU)`，且同步带已经松弛，`IMU 角速度` 闭环通常更容易引入延迟和自激
- 本项目中后续整定 yaw 控制时，已经改为优先使用电机侧角速度闭环
- 做扫频时建议优先看 `扭矩 -> 电机/云台 yaw 角速度`，不要先盯角度曲线

## 3. 临时切换到扫频组件

正常情况下，`omni-infantry.yaml` 中的云台组件是：

```yaml
- rmcs_core::controller::gimbal::OmniInfantryPlannerGimbalController -> gimbal_controller
```

做扫频时，临时切换为：

```yaml
- rmcs_core::controller::gimbal::OmniInfantryGimbalSweep -> gimbal_controller
```

`gimbal_controller` 的参数改为扫频参数，例如：

```yaml
gimbal_controller:
  ros__parameters:
    duration: 10.0

    yaw_torque_bias: 0.0
    pitch_torque_bias: 0.0

    yaw_torque_amplitude: 20.0
    pitch_torque_amplitude: 0.0

    yaw_start_frequency: 0.5
    yaw_end_frequency: 6.0
    pitch_start_frequency: 0.5
    pitch_end_frequency: 6.0

    output_csv_path: /tmp/rmcs/gimbal_sweep.csv
    csv_decimation: 1
```

参数含义：

- `duration`：单次 chirp 时长，单位秒
- `*_torque_bias`：扭矩偏置
- `*_torque_amplitude`：扭矩振幅
- `*_start_frequency` / `*_end_frequency`：起止扫频频率
- `output_csv_path`：远端 CSV 输出路径
- `csv_decimation`：CSV 抽样倍率，`1` 表示每个控制周期都记

建议初始设置：

- 先只扫 yaw，`pitch_torque_amplitude = 0.0`
- 先从中等振幅开始，例如 `yaw_torque_amplitude = 10 ~ 20`
- 先扫 `0.5 ~ 6 Hz`

## 4. 构建、同步与启动

典型流程如下。

先在单独终端启动同步进程：

```bash
RMCS_PATH=/workspaces/dev-sp-vision sync-remote
```

注意：

- `sync-remote` 是常驻监视进程，不会自行退出
- 不要在同一个终端里等待它返回后再执行后续命令
- 建议保持该终端常驻，另开终端执行构建、`wait-sync` 和 `attach-remote -r`

再构建并等待同步完成：

```bash
RMCS_PATH=/workspaces/dev-sp-vision build-rmcs
wait-sync
```

最后重启并附着远端控制进程：

```bash
attach-remote -r
```

如果启动正常，日志中应出现类似输出：

```text
[gimbal_controller]: Started gimbal torque sweep, csv output: /tmp/rmcs/gimbal_sweep.csv
```

这说明扫频组件已经开始接管扭矩输出并写入 CSV。

## 5. 采集与停止

采集过程中，建议至少覆盖多个完整周期：

- 若 `duration = 10 s`，建议至少采 `30 ~ 60 s`
- 这样能覆盖 3 到 6 个完整 chirp 周期

在附着界面中按 `Ctrl+C` 可以停止 RMCS，同时终止扫频并关闭 CSV。正常情况下日志中会出现：

```text
[gimbal_controller]: Stopped gimbal torque sweep
```

停止后再进行回传，避免拿到尚未 flush 完整的文件。

## 6. 回传 CSV

默认输出路径为：

```text
/tmp/rmcs/gimbal_sweep.csv
```

可以直接拷回本地：

```bash
scp remote:/tmp/rmcs/gimbal_sweep.csv /workspaces/dev-sp-vision/gimbal_sweep.csv
```

如果要保留多轮实验结果，建议按日期和轮次重命名，例如：

```bash
scp remote:/tmp/rmcs/gimbal_sweep.csv \
  /workspaces/dev-sp-vision/gimbal_torque_sweep_20260316_round2.csv
```

## 7. CSV 字段说明

当前扫频 CSV 头如下：

```text
update_count,elapsed_s,cycle_index,cmd_yaw_torque,cmd_pitch_torque,actual_world_yaw,actual_world_pitch,actual_yaw_velocity_imu,actual_pitch_velocity_imu,yaw_control_angle_error,pitch_control_angle_error,yaw_frequency_hz,pitch_frequency_hz
```

主要字段解释：

- `elapsed_s`：当前周期内的时间
- `cycle_index`：当前是第几个 chirp 周期
- `cmd_yaw_torque`：yaw 注入扭矩
- `actual_world_yaw`：世界系 yaw 角
- `actual_yaw_velocity_imu`：IMU 测得的 yaw 角速度
- `yaw_frequency_hz`：当前时刻的瞬时扫频频率

注意：

- `actual_world_yaw` 在跨过 `±pi` 时会发生包裹
- 角度域图像在高幅值、长时间扫频下通常不适合直接做精确频响拟合
- 速度域分析一般比角度域更稳定

## 8. 数据预处理建议

常见预处理步骤如下：

1. 去掉启动初期不稳定段
2. 只保留完整 chirp 周期
3. 再做频段统计或模型拟合

例如，去掉前 15 秒：

```bash
/workspaces/dev-sp-vision/.venv/bin/python - <<'PY'
from pathlib import Path
import csv

src = Path('/workspaces/dev-sp-vision/gimbal_sweep.csv')
dst = Path('/workspaces/dev-sp-vision/gimbal_sweep_trimmed_15s.csv')
duration = 10.0
trim_seconds = 15.0

with src.open('r', newline='', encoding='utf-8') as f_in, dst.open('w', newline='', encoding='utf-8') as f_out:
    reader = csv.DictReader(f_in)
    writer = csv.DictWriter(f_out, fieldnames=reader.fieldnames)
    writer.writeheader()
    for row in reader:
        total_elapsed = float(row['cycle_index']) * duration + float(row['elapsed_s'])
        if total_elapsed < trim_seconds:
            continue
        writer.writerow(row)
PY
```

如果首个完整稳定周期从第 `2` 个周期开始，后续分析时建议只使用 `cycle_index = 2..N-1`。

## 9. 推荐分析方法

建议优先做以下分析：

- `cmd_yaw_torque -> actual_yaw_velocity_imu` 的幅频 / 相频估计
- 分频段计算 `扭矩 -> 角速度` 增益
- 估计等效惯量 `J`
- 估计阻尼项 `B`
- 估计 cross-axis coupling，例如 `yaw 扭矩 -> pitch 角速度`

建议不要把第一次分析重点放在：

- `cmd_yaw_torque -> actual_world_yaw` 的高频精确拟合

因为 yaw 角本身会受到：

- `±pi` 包裹
- 长时间累积漂移
- 固定件松动 / 结构回差

这些因素更适合看趋势，不适合直接做一阶或二阶精确辨识。

## 10. 结果解释建议

### 10.1 如果 `扭矩 -> yaw 角速度` 相位接近 `-90 deg`

这通常说明该频段主要是惯量主导，可以近似为：

```text
omega(s) / torque(s) ≈ 1 / (J s + B)
```

此时可以根据频响反推 yaw 输出轴等效惯量 `J`。

### 10.2 如果低频段明显偏离模型

常见原因：

- 静摩擦
- 机械松弛 / 回差
- 初始固定不充分
- IMU 零偏和结构缓慢摆动

低频段不稳定时，常见做法是：

- 直接丢弃 `0.5 ~ 1 Hz` 的结果
- 用 `1.0 Hz` 或 `1.5 Hz` 以上数据做线性模型拟合

### 10.3 如果 `yaw_velocity_kp` 一加就震荡

优先怀疑：

- 电机与云台之间存在柔性传动
- IMU 角速度链路延迟较大
- 同步带松弛导致的附加相位滞后

这时不要继续一味提高速度环增益，应优先：

- 改用电机侧角速度闭环
- 降低角度环大误差输出
- 对大角度阶跃单独观察是否出现尾端衰减振荡

## 11. 基于扫频结果整定 yaw 的建议

推荐顺序：

1. 先做开环扫频，识别 yaw 轴 `J / B / 延迟`
2. 再恢复 `OmniInfantryPlannerGimbalController`
3. 优先整定 yaw 速度环
4. 再整定 yaw 角度环
5. 最后在 planner 轨迹跟踪场景下评估 `yaw_acc_ff_gain`

实践中常见规律：

- 若大角度阶跃时略超调并带一小段衰减振荡，通常是角度环给出的速度参考过猛
- 若 `yaw_velocity_kp` 再往上加立刻震荡，通常不是“阻尼不够”，而是柔性链路延迟已经到上限
- 此时应优先调小 `yaw_angle_output_max`，而不是继续加 `yaw_velocity_kp`

## 12. 恢复正常控制器

扫频结束后，记得把 YAML 切回正常云台控制器：

```yaml
- rmcs_core::controller::gimbal::OmniInfantryPlannerGimbalController -> gimbal_controller
```

然后恢复正常的 `gimbal_controller` 参数，重新构建、同步、重启。

不要把扫频组件长期留在运行配置中。

## 13. 常见问题

### 13.1 日志没有出现 `Started gimbal torque sweep`

先检查：

- YAML 是否真的切到了 `OmniInfantryGimbalSweep`
- `plugins.xml` 中是否注册了该组件
- 远端是否已经同步到最新 `install`
- 遥控器当前是否处于 `左中右上`

### 13.2 CSV 没生成

先检查：

- `output_csv_path` 是否为空
- 父目录是否可创建
- RMCS 是否在运行中就被异常打断

### 13.3 数据前段很乱

通常原因：

- 机械固定还没稳定
- 刷入新代码后立刻开始采集
- 操作人员仍在接触车体或云台

处理方式：

- 明确延时几秒后再纳入分析
- 或直接裁掉前 `10 ~ 20 s`

### 13.4 yaw 角看起来跳变

这是正常现象。`actual_world_yaw` 是角度，不是连续展开角。跨过 `±pi` 时会发生包裹。  
做频响拟合时优先使用 yaw 角速度。

## 14. 建议保留的实验记录

每轮扫频建议至少记录：

- 采集日期
- 机器人编号
- 机械状态，例如同步带是否松弛
- 扫频参数
- 是否裁掉前段数据
- 最终使用了哪些周期做拟合
- 得到的 `J / B / 延迟`
- 基于结果修改了哪些 yaw 参数

这样后续才能把“扫频结果”和“实机控制手感”稳定关联起来。
