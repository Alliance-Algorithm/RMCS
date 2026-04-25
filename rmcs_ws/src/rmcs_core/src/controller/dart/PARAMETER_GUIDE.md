# 参数配置指南（统一使用克作为力单位）

## 单位约定

- **力**：克 (g) - 与传感器输出一致
- **位移**：米 (m)
- **弹簧刚度**：g/m
- **能量**：焦耳 (J)
- **速度**：m/s

## 参数分类

### 1. 必须校准的参数

这些参数每个系统不同，必须实测：

```yaml
spring_model:
  ros__parameters:
    # 丝杆顶端角度（rad）
    screw_top_angle_rad: 0.0
    
    # 预紧力（g）
    preload_force_g: 0.0
    
    # 弹簧线性刚度（g/m）
    spring_K1: 500000.0
    
    # 弹簧非线性刚度（g/m^n）
    spring_K2: 200000.0
    
    # 非线性指数
    spring_exponent: 1.5
```

**校准方法**：

1. **screw_top_angle_rad**：
   - 手动将扳机移到最上端
   - `ros2 topic echo /dart/force_screw_motor/angle`
   - 记录读数

2. **preload_force_g**：
   - 扳机在最上端时
   - `ros2 topic echo /force_sensor/channel_1/weight`
   - `ros2 topic echo /force_sensor/channel_2/weight`
   - `preload_force_g = ch1 + ch2`

3. **spring_K1, spring_K2, spring_exponent**：
   - 方法1：查阅弹簧规格书，转换单位（N/m → g/m，乘以 ~102）
   - 方法2：采集多组（位移，力）数据，拟合曲线

### 2. 必须调优的参数

这些参数需要根据实际噪声情况调整：

```yaml
force_kalman_v2:
  ros__parameters:
    # 模型权重（0-1）
    model_weight: 0.5
    
    # 卡尔曼噪声参数（g^2）
    Q_force: 100000.0      # ~100g 标准差
    Q_rate: 500000.0       # ~700g/s 标准差
    R_sensor: 20000.0      # ~140g 标准差（±1N 传感器噪声）
```

**调优方法**：

- **model_weight**：
  - 0.0 = 纯传感器（忽略模型）
  - 1.0 = 纯模型（忽略传感器）
  - 0.5 = 平衡（推荐起点）
  - 观察 `/dart/kalman/innovation` 和 `/dart/kalman/kalman_gain` 调整

- **Q_force, Q_rate**（过程噪声）：
  - 增大 → 更信任传感器测量
  - 减小 → 更信任模型预测
  - 默认值基于 ±1N 传感器噪声

- **R_sensor**（测量噪声）：
  - 基于传感器规格：±1N ≈ ±100g
  - 方差 = (100g)^2 = 10000 g^2
  - 保守估计用 20000 g^2

### 3. 可选调优的参数

这些参数有合理的默认值（硬编码），通常不需要修改：

```yaml
spring_model:
  ros__parameters:
    # 相位检测阈值（g）
    force_threshold_idle: 5000.0    # ≈50N，进入 LOADING
    force_threshold_peak: 15000.0   # ≈150N，进入 HOLDING
    holding_tolerance: 1000.0       # ≈10N，HOLDING 稳定性
```

**何时需要调整**：
- 不同镖体的力范围差异很大
- 相位检测不准确（误判或漏判）

### 4. 硬编码的常量

这些参数是物理常量或算法设计，不需要配置：

```cpp
// 物理常量（机械设计固定）
constexpr double SCREW_PITCH_M = 0.002;        // 2mm/360°
constexpr double PULLEY_RATIO = 2.0;           // 1:2
constexpr double PROJECTILE_MASS_KG = 0.2;     // 发射体质量
constexpr double ETA_LOADING = 0.95;           // 加载效率
constexpr double ETA_UNLOADING = 0.90;         // 卸载效率
constexpr double FRICTION_COEFF = 0.85;        // 摩擦系数

// 算法参数（经验值）
constexpr int HOLDING_DURATION_TICKS = 50;     // HOLDING 确认帧数
constexpr size_t MIN_CALIBRATION_SAMPLES = 100;
constexpr size_t MAX_CALIBRATION_SAMPLES = 1000;
constexpr size_t FORCE_HISTORY_SIZE = 100;
```

## 推荐配置模板

### 最小配置（首次使用）

```yaml
spring_model:
  ros__parameters:
    screw_top_angle_rad: 0.0      # TODO: 校准
    preload_force_g: 0.0          # TODO: 校准
    spring_K1: 500000.0           # TODO: 根据弹簧规格调整
    spring_K2: 200000.0           # TODO: 根据弹簧规格调整
    spring_exponent: 1.5
    enable_logging: true          # 首次使用建议启用

force_kalman_v2:
  ros__parameters:
    model_weight: 0.5
    Q_force: 100000.0
    Q_rate: 500000.0
    R_sensor: 20000.0
    enable_logging: true          # 首次使用建议启用
```

### 生产配置（调优后）

```yaml
spring_model:
  ros__parameters:
    screw_top_angle_rad: 1.234    # 已校准
    preload_force_g: 10000.0      # 已校准（≈100N）
    spring_K1: 510000.0           # 已拟合
    spring_K2: 180000.0           # 已拟合
    spring_exponent: 1.5
    enable_logging: false

force_kalman_v2:
  ros__parameters:
    model_weight: 0.6             # 已调优（更信任模型）
    Q_force: 150000.0             # 已调优
    Q_rate: 600000.0              # 已调优
    R_sensor: 15000.0             # 已调优
    enable_logging: false
```

## 单位转换参考

| 物理量 | 克单位 | 牛顿单位 | 转换 |
|--------|--------|----------|------|
| 力 | 10000 g | 100 N | × 0.00981 |
| 弹簧刚度 | 500000 g/m | 5000 N/m | × 0.00981 |
| 力方差 | 100000 g² | 10 N² | × (0.00981)² |

## 调试技巧

### 1. 验证校准

```bash
# 扳机在顶端，观察预测力是否接近预紧力
ros2 topic echo /dart/dynamics/predicted_force
ros2 topic echo /dart/dynamics/measured_force
# 两者应该接近 preload_force_g
```

### 2. 观察相位转换

```bash
ros2 topic echo /dart/dynamics/phase
# 0=IDLE, 1=LOADING, 2=HOLDING, 3=RELEASE
```

### 3. 检查卡尔曼滤波效果

```bash
# 观察新息（应该接近0）
ros2 topic echo /dart/kalman/innovation

# 观察卡尔曼增益（0-1之间）
ros2 topic echo /dart/kalman/kalman_gain

# 对比滤波前后
ros2 topic echo /dart/dynamics/measured_force
ros2 topic echo /dart/kalman/filtered_force
# filtered_force 应该更平滑
```

### 4. 启用日志

```yaml
spring_model:
  ros__parameters:
    enable_logging: true
    log_interval_ticks: 500  # 每500次更新打印一次

force_kalman_v2:
  ros__parameters:
    enable_logging: true
    log_interval_ticks: 500
```

日志输出示例：
```
[Model] phase=2 x=0.0123m F_pred=12345g F_meas=12400g | calib: F_max=12380g n=567
[KFv2] F_model=12345g F_meas=12400g F_filt=12370g dF/dt=123g/s | innov=55g K=0.456
```

## 常见问题

### Q: 为什么使用克而不是牛顿？

**A**: 
1. 传感器直接输出克，避免频繁转换
2. 减少浮点运算误差
3. 配置文件中的数值更直观（10000g vs 100N）
4. 只在计算能量时转换为牛顿（乘以 0.00981）

### Q: 如何确定弹簧参数？

**A**: 
1. 查阅弹簧规格书（推荐）
2. 实测：采集多组（位移，力）数据，用最小二乘法拟合
3. 使用 HOLDING 阶段的校准数据反推

### Q: model_weight 如何选择？

**A**:
- 传感器噪声大 → 增大 model_weight (0.6-0.8)
- 模型不准确 → 减小 model_weight (0.2-0.4)
- 不确定 → 从 0.5 开始，观察 innovation 调整
