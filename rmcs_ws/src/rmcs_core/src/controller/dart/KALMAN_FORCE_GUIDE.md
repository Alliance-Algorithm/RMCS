# 卡尔曼滤波力闭环使用指南

## 概述

在 `launch_prepare` 阶段的力闭环控制中，可以选择使用卡尔曼滤波后的力值替代原始传感器读数，以获得更平滑、更准确的控制。

## 架构说明

**重要**：卡尔曼滤波只在 Controller 层做一次，Manager 层只是选择使用哪个数据源。

```
力传感器 (ch1+ch2, 单位: g)
    ↓
ForceKalmanFilter 组件 (Controller 层)
    ├─ 预测步骤：弹簧动力学模型 (前馈)
    └─ 更新步骤：传感器测量修正 (后馈)
    ↓
/dart/kalman/filtered_force (N)
/dart/kalman/force_rate (N/s)
    ↓
DartManager (Manager 层)
    ├─ use_kalman_force=false → 读原始传感器 (g)
    └─ use_kalman_force=true  → 读卡尔曼输出 (N)
    ↓
LaunchPreparationTask
    ↓
ForceScrewCalibrationAction (统一的 PID 控制)
    ├─ 原始模式：读 current_force_ch1/ch2，单位 g，乘 0.00981 转 N
    └─ 滤波模式：读 filtered_force，单位已是 N
    ↓
force_screw_motor 速度指令
```

**关键点**：
- **ForceKalmanFilter**：Controller 层组件，做卡尔曼滤波 + 弹簧模型
- **ForceScrewCalibrationAction**：Manager 层 Action，只做 PID 控制，支持两种输入源
- **没有重复的卡尔曼滤波**：滤波只在 Controller 层做一次

## 启用步骤

### 1. 启用卡尔曼滤波组件

编辑 `dart-guidance.yaml`，在 `components` 列表中添加：

```yaml
rmcs_executor:
  ros__parameters:
    components:
      # ... 其他组件 ...
      - rmcs_core::controller::dart::ForceKalmanFilter -> force_kalman
```

### 2. 配置卡尔曼滤波参数

取消注释并调整 `force_kalman` 配置块：

```yaml
force_kalman:
  ros__parameters:
    update_rate_hz: 1000.0

    # 过程噪声（调大 → 更信任传感器）
    Q_force: 1.0
    Q_rate: 10.0

    # 测量噪声（调大 → 更信任模型）
    R_sensor: 5.0

    # 弹簧模型参数（需标定）
    spring_K1: 5000.0
    spring_K2: 2000.0
    spring_exponent: 1.5

    # 能量转换参数
    eta_unloading: 0.90
    friction_coeff: 0.85
    projectile_mass: 0.2

    # 查表数据（可选，实测标定后填入）
    lookup_table_force: [50.0, 100.0, 150.0, 200.0, 250.0, 300.0]
    lookup_table_disp:  [0.008, 0.017, 0.027, 0.038, 0.050, 0.063]

    enable_logging: false
```

### 3. 启用 DartManager 的卡尔曼力闭环

在 `dart_manager` 配置中：

```yaml
dart_manager:
  ros__parameters:
    enable_force_calibration: true       # 启用力闭环
    use_kalman_force: true               # 使用卡尔曼滤波值
    kalman_rate_feedforward: false       # 可选：使用 dF/dt 前馈
    kalman_rate_gain: 0.005              # 前馈增益（如果启用）
    
    # 其他力闭环参数
    force_tolerance: 5.0
    force_settle_ticks: 1000
    force_timeout_ticks: 1200
    force_kp: 1.0
    force_ki: 0.1
    force_kd: 0.01
```

## 参数调节

### 前馈/后馈比例调节

有两种方式：

#### 方式 1：自动模式（推荐）

通过 Q 和 R 调节：

```yaml
# 更信任传感器（后馈权重大）
Q_force: 10.0    # 大
R_sensor: 1.0    # 小

# 更信任模型（前馈权重大）
Q_force: 0.1     # 小
R_sensor: 10.0   # 大
```

#### 方式 2：手动权重模式

```yaml
use_manual_fusion_weight: true
manual_fusion_weight: 0.5    # 0=纯前馈, 1=标准, >1=更信任传感器
```

### 力变化率前馈

当力值快速变化时，可以用 dF/dt 提前补偿：

```yaml
kalman_rate_feedforward: true
kalman_rate_gain: 0.005      # 建议从 0.001 开始调
```

**物理意义**：如果 dF/dt < 0（力在下降），增加正向电机速度补偿，提前对抗力的下降趋势。

## 对比测试

### A/B 测试流程

1. **基线测试**（原始传感器）
   ```yaml
   enable_force_calibration: true
   use_kalman_force: false
   ```
   记录：稳定时间、超调量、最终误差

2. **卡尔曼测试**（滤波后）
   ```yaml
   enable_force_calibration: true
   use_kalman_force: true
   ```
   记录：相同指标

3. **对比指标**
   - 稳定时间（settle_ticks）
   - 力值波动（标准差）
   - 超调量
   - 最终误差

### 查看实时数据

```bash
# 原始力值
ros2 topic echo /force_sensor/channel_1/weight
ros2 topic echo /force_sensor/channel_2/weight

# 卡尔曼滤波后
ros2 topic echo /dart/kalman/filtered_force
ros2 topic echo /dart/kalman/force_rate

# 储能和速度估计
ros2 topic echo /dart/kalman/estimated_energy
ros2 topic echo /dart/kalman/estimated_velocity
```

## 故障排查

### 问题：卡尔曼输出不稳定

**原因**：Q/R 参数不匹配

**解决**：
1. 先用 `manual_fusion_weight: 1.0`（标准卡尔曼）
2. 如果仍不稳定，增大 `R_sensor`（更信任模型）
3. 如果响应太慢，增大 `Q_force`（更信任传感器）

### 问题：力闭环超时

**原因**：PID 参数不合适或目标力值不可达

**解决**：
1. 检查 `force_kp`, `force_ki`, `force_kd`
2. 检查 `force_timeout_ticks` 是否足够
3. 启用日志查看实时力值：
   ```yaml
   enable_logging: true
   ```

### 问题：卡尔曼输入未就绪

**日志**：`use_kalman_force=true but Kalman inputs not ready`

**原因**：`ForceKalmanFilter` 组件未启用或未运行

**解决**：
1. 确认 `components` 列表包含 `ForceKalmanFilter`
2. 检查组件是否成功加载：
   ```bash
   ros2 topic list | grep kalman
   ```

## 高级功能

### 查表标定

实测不同力值下的弹簧位移，填入查表：

```yaml
lookup_table_force: [100.0, 200.0, 300.0, 400.0]  # 实测力值 (N)
lookup_table_disp:  [0.015, 0.032, 0.051, 0.072]  # 对应位移 (m)
```

查表比数学模型更准确，因为考虑了实际安装、预紧、摩擦等因素。

### 速度估计

卡尔曼滤波器同时输出飞行器脱离速度估计：

```bash
ros2 topic echo /dart/kalman/estimated_velocity
```

可用于：
- 验证发射一致性
- 调整弹簧预紧力
- 预测弹道

## 总结

| 模式 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| 原始传感器 | 简单，无额外计算 | 噪声大，可能震荡 | 传感器质量好，环境稳定 |
| 卡尔曼滤波 | 平滑，抗噪声，可预测 | 需要调参，依赖模型 | 传感器噪声大，需要高精度 |
| 卡尔曼+前馈 | 响应快，超调小 | 调参复杂 | 动态响应要求高 |

**推荐起点**：
```yaml
use_kalman_force: true
Q_force: 1.0
R_sensor: 5.0
kalman_rate_feedforward: false
```

逐步调优，观察效果。
