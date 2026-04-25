# 重构后的弹簧动力学卡尔曼滤波系统

## 架构设计

### 两层架构：先验 + 后验

```
丝杆电机多圈角度 (/dart/force_screw_motor/angle)
    ↓
┌─────────────────────────────────────────────────────────┐
│ SpringDynamicsModelV2 (先验 - PRIOR)                    │
│                                                          │
│ 1. 运动学映射：                                          │
│    angle → trigger_disp → spring_elongation             │
│    (螺距2mm/360°, 滑轮比1:2)                            │
│                                                          │
│ 2. 弹簧力模型：                                          │
│    F_predicted = K1·x + K2·|x|^n + F_preload            │
│                                                          │
│ 3. 相位检测：                                            │
│    IDLE → LOADING → HOLDING → RELEASE                   │
│                                                          │
│ 4. HOLDING 阶段校准：                                    │
│    采样 100-1000 个力值，取均值                          │
└─────────────────────────────────────────────────────────┘
    ↓ 输出
    - /dart/dynamics/predicted_force (模型预测力)
    - /dart/dynamics/measured_force (传感器测量力)
    - /dart/dynamics/spring_elongation (弹簧伸长量)
    - /dart/dynamics/phase (当前相位)
    - /dart/dynamics/calibrated_max_force (校准最大力)
    ↓
┌─────────────────────────────────────────────────────────┐
│ ForceKalmanFilterV2Refactored (后验 - POSTERIOR)        │
│                                                          │
│ 卡尔曼滤波：融合模型预测与传感器测量                      │
│                                                          │
│ 预测步骤：                                               │
│   X_pred = α·F_model + (1-α)·F_kinematic                │
│   (α = model_weight, 默认 0.5)                          │
│                                                          │
│ 更新步骤：                                               │
│   X_post = X_pred + K·(F_measured - F_pred)             │
│   K = Kalman gain (自适应计算)                           │
└─────────────────────────────────────────────────────────┘
    ↓ 输出
    - /dart/kalman/filtered_force (最优估计力)
    - /dart/kalman/force_rate (力变化率)
    - /dart/kalman/innovation (新息)
    - /dart/kalman/kalman_gain (卡尔曼增益)
```

## 关键改进

### 相比原始设计

1. **职责分离**
   - SpringDynamicsModelV2：负责物理建模和相位检测
   - ForceKalmanFilterV2Refactored：负责传感器融合

2. **避免重复**
   - 不再在卡尔曼滤波器中重复实现弹簧模型
   - 不再重复实现相位检测逻辑

3. **模块化**
   - 可以单独使用 SpringDynamicsModelV2 进行模型验证
   - 可以单独使用 ForceKalmanFilterV2Refactored 进行滤波测试

4. **可维护性**
   - 弹簧参数只需在一处配置（spring_model）
   - 卡尔曼参数独立配置（force_kalman_v2）

## 接口定义

### SpringDynamicsModelV2

**输入**：
- `/force_sensor/channel_1/weight` (int, g)
- `/force_sensor/channel_2/weight` (int, g)
- `/dart/force_screw_motor/angle` (double, rad)

**输出**：
- `/dart/dynamics/spring_elongation` (double, m) - 弹簧伸长量
- `/dart/dynamics/predicted_force` (double, N) - 模型预测力
- `/dart/dynamics/measured_force` (double, N) - 传感器测量力
- `/dart/dynamics/stored_energy` (double, J) - 储存能量
- `/dart/dynamics/estimated_velocity` (double, m/s) - 预测速度
- `/dart/dynamics/phase` (int) - 当前相位 (0=IDLE, 1=LOADING, 2=HOLDING, 3=RELEASE)
- `/dart/dynamics/calibrated_max_force` (double, N) - 校准最大力
- `/dart/dynamics/calibration_samples` (int) - 校准采样数

### ForceKalmanFilterV2Refactored

**输入**：
- `/dart/dynamics/predicted_force` (double, N)
- `/dart/dynamics/measured_force` (double, N)
- `/dart/dynamics/spring_elongation` (double, m)
- `/dart/dynamics/phase` (int)

**输出**：
- `/dart/kalman/filtered_force` (double, N) - 滤波后的力
- `/dart/kalman/force_rate` (double, N/s) - 力变化率
- `/dart/kalman/innovation` (double, N) - 新息（测量-预测）
- `/dart/kalman/kalman_gain` (double) - 卡尔曼增益

## 配置指南

### 1. 启用组件

在 `dart-guidance.yaml` 中：

```yaml
rmcs_executor:
  ros__parameters:
    components:
      # 先启用弹簧模型（先验）
      - rmcs_core::controller::dart::SpringDynamicsModelV2 -> spring_model
      # 再启用卡尔曼滤波（后验）
      - rmcs_core::controller::dart::ForceKalmanFilterV2Refactored -> force_kalman_v2
```

### 2. 校准 SpringDynamicsModelV2

**必须校准的参数**：

```yaml
spring_model:
  ros__parameters:
    # 1. 丝杆顶端角度
    screw_top_angle_rad: 0.0  # TODO: 扳机在最上端时，读取电机角度

    # 2. 预紧力
    preload_force_N: 0.0      # TODO: 在顶端位置，读取力传感器并转换为N
```

**校准步骤**：
1. 手动控制扳机移到最上端
2. `ros2 topic echo /dart/force_screw_motor/angle` → 填入 `screw_top_angle_rad`
3. `ros2 topic echo /force_sensor/channel_1/weight` 和 `channel_2/weight`
4. 计算：`preload_force_N = (ch1 + ch2) × 0.00981`

### 3. 调优 ForceKalmanFilterV2Refactored

**model_weight 参数**：

```yaml
force_kalman_v2:
  ros__parameters:
    model_weight: 0.5  # 0=纯传感器, 1=纯模型, 0.5=平衡
```

- **传感器噪声大** → 增大 model_weight (0.6-0.8)
- **模型不准确** → 减小 model_weight (0.2-0.4)
- **默认平衡** → 0.5

**噪声参数**：

```yaml
force_kalman_v2:
  ros__parameters:
    Q_force: 10.0   # 力过程噪声
    Q_rate: 50.0    # 力变化率噪声
    R_sensor: 2.0   # 传感器测量噪声
```

## 使用方法

### 在 DartManager 中使用滤波后的力

```cpp
// 原始代码
register_input("/force_sensor/channel_1/weight", force_ch1_);
register_input("/force_sensor/channel_2/weight", force_ch2_);
double F_raw = (force_ch1_ + force_ch2_) * 0.00981;

// 使用卡尔曼滤波后的力
register_input("/dart/kalman/filtered_force", filtered_force_);
double F_filtered = *filtered_force_;  // 已经是 N 单位

// 可选：使用力变化率进行前馈
register_input("/dart/kalman/force_rate", force_rate_);
double feedforward = *force_rate_ * feedforward_gain;
```

### 使用校准的最大力

```cpp
// 在 HOLDING 阶段读取校准值
register_input("/dart/dynamics/calibrated_max_force", calibrated_max_force_);
register_input("/dart/dynamics/calibration_samples", calibration_samples_);

if (*calibration_samples_ >= 100) {
    // 校准完成，使用校准值
    double F_target = *calibrated_max_force_;
}
```

## 验证与调试

### 1. 验证弹簧模型

```bash
# 观察模型预测 vs 实际测量
ros2 topic echo /dart/dynamics/predicted_force
ros2 topic echo /dart/dynamics/measured_force

# 观察弹簧伸长量
ros2 topic echo /dart/dynamics/spring_elongation

# 观察相位转换
ros2 topic echo /dart/dynamics/phase
```

预期：
- 扳机在顶端：spring_elongation ≈ 0, predicted_force ≈ preload_force_N
- 扳机下行：spring_elongation 增大，predicted_force 增大
- HOLDING 阶段：phase = 2，calibration_samples 增长

### 2. 验证卡尔曼滤波

```bash
# 观察滤波效果
ros2 topic echo /dart/kalman/filtered_force
ros2 topic echo /dart/kalman/innovation
ros2 topic echo /dart/kalman/kalman_gain
```

预期：
- filtered_force 比 measured_force 更平滑
- innovation 接近 0（模型与测量一致）
- kalman_gain 在 0-1 之间（自适应调整）

### 3. 启用日志

```yaml
spring_model:
  ros__parameters:
    enable_logging: true

force_kalman_v2:
  ros__parameters:
    enable_logging: true
```

日志输出：
```
[Model] phase=2 x=0.0123m F_pred=234.5N F_meas=236.1N | calib: F_max=235.8N n=567
[KFv2] F_model=234.5N F_meas=236.1N F_filt=235.3N dF/dt=12.3N/s | innov=1.6N K=0.456
```

## 性能优势

### 相比原始传感器

1. **噪声抑制**：±1N 噪声被卡尔曼滤波平滑
2. **物理一致性**：融合弹簧模型，确保力值符合物理规律
3. **动态响应**：提供 dF/dt 用于前馈控制

### 相比 V1 版本

1. **模块化**：弹簧模型和卡尔曼滤波分离，易于维护
2. **无重复**：避免重复实现相位检测和运动学映射
3. **可扩展**：可以独立改进弹簧模型或滤波算法

### 相比纯模型

1. **鲁棒性**：传感器测量修正模型误差
2. **自适应**：卡尔曼增益自动调整模型/传感器权重
3. **实时校准**：HOLDING 阶段自动校准最大力

## 常见问题

### Q: 为什么要分成两个组件？

**A**: 职责分离原则
- SpringDynamicsModelV2：物理建模专家，负责从运动学预测力
- ForceKalmanFilterV2Refactored：传感器融合专家，负责最优估计

这样可以：
- 独立验证模型准确性
- 独立调优滤波参数
- 复用模型输出（如 phase, calibrated_max_force）

### Q: model_weight 如何选择？

**A**: 取决于模型准确性和传感器噪声
- 模型准确 + 传感器噪声大 → model_weight = 0.7-0.8
- 模型不准 + 传感器稳定 → model_weight = 0.2-0.3
- 不确定 → model_weight = 0.5（默认）

观察 innovation 和 kalman_gain 来调优。

### Q: 如何知道校准是否完成？

**A**: 观察 calibration_samples
```bash
ros2 topic echo /dart/dynamics/calibration_samples
```
- < 100：校准中
- >= 100：校准完成，可以使用 calibrated_max_force

### Q: 滤波后的力还是抖动？

**A**: 调整噪声参数
1. 减小 R_sensor（更信任传感器）
2. 增大 model_weight（更信任模型）
3. 检查弹簧模型参数是否准确

## 文件清单

- `spring_dynamics_model_v2.cpp` - 弹簧动力学模型（先验）
- `force_kalman_filter_v2_refactored.cpp` - 卡尔曼滤波器（后验）
- `plugins.xml` - 组件注册
- `dart-guidance.yaml` - 配置文件
- 本文档 - 使用指南
