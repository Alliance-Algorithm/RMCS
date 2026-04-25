# 卡尔曼滤波力闭环完整实现指南

## 实现状态

✅ **已完成**：
1. ForceKalmanFilter 组件代码 (`force_kalman_filter.cpp`)
2. SpringDynamicsModel 组件代码 (`spring_dynamics_model.cpp`)
3. plugins.xml 注册
4. CMakeLists.txt 自动编译
5. dart-guidance.yaml 配置（基于53次发射数据优化）

## 配置说明

### 优化策略

基于两组测试数据（53次发射）的分析：
- 通道1波动：±70-110g (±0.7-1.1N)
- 通道2波动：±60-80g (±0.6-0.8N)
- **关键发现**：这些波动是真实机械状态（摩擦、间隙、温度），不是传感器噪声

### 参数设置

```yaml
force_kalman:
  ros__parameters:
    # 高度信任传感器模式
    Q_force: 10.0      # 大 → 承认模型预测不准
    Q_rate: 50.0       # 大 → 力变化率不可预测
    R_sensor: 2.0      # 小 → 传感器测量真实状态
    
    # 卡尔曼增益 K ≈ 10/(10+2) ≈ 0.83
    # 结果：83%权重给传感器，17%给模型预测
```

### 启用方法

已在 `dart-guidance.yaml` 第16行启用：
```yaml
components:
  - rmcs_core::controller::dart::ForceKalmanFilter -> force_kalman
```

## 编译和运行

### 1. 编译

```bash
cd ~/RMCS_dart/RMCS/rmcs_ws
colcon build --packages-select rmcs_core
source install/setup.zsh
```

### 2. 运行

```bash
ros2 launch rmcs_bringup dart_guidance.launch.py
```

### 3. 验证组件加载

```bash
# 检查 Kalman 输出 topic
ros2 topic list | grep kalman

# 应该看到：
# /dart/kalman/filtered_force
# /dart/kalman/force_rate
# /dart/kalman/estimated_energy
# /dart/kalman/estimated_velocity
```

### 4. 实时监控

```bash
# 原始力值
ros2 topic echo /force_sensor/channel_1/weight
ros2 topic echo /force_sensor/channel_2/weight

# 卡尔曼滤波后
ros2 topic echo /dart/kalman/filtered_force

# 力变化率
ros2 topic echo /dart/kalman/force_rate

# 能量和速度估计
ros2 topic echo /dart/kalman/estimated_energy
ros2 topic echo /dart/kalman/estimated_velocity
```

## 在 DartManager 中使用

### 启用卡尔曼力闭环

编辑 `dart-guidance.yaml` 中的 `dart_manager` 配置：

```yaml
dart_manager:
  ros__parameters:
    enable_force_calibration: true
    use_kalman_force: true          # 使用卡尔曼滤波值
    
    # 可选：使用力变化率前馈
    kalman_rate_feedforward: false   # 先测试基础版本
    kalman_rate_gain: 0.005         # 如果启用前馈
    
    # PID 参数（可能需要重新调整）
    force_kp: 1.0
    force_ki: 0.1
    force_kd: 0.01
```

## 对比测试方案

### A/B 测试

**测试A：原始传感器**
```yaml
use_kalman_force: false
```

**测试B：卡尔曼滤波**
```yaml
use_kalman_force: true
```

### 评估指标

1. **稳定时间**：达到目标力值所需时间
2. **超调量**：最大超调百分比
3. **稳态误差**：稳定后的平均误差
4. **力值波动**：稳定阶段的标准差

### 数据记录

启用日志：
```yaml
force_kalman:
  ros__parameters:
    enable_logging: true  # 每500个tick记录一次
```

或使用 ForceSensorRecorder：
```yaml
force_sensor_recorder:
  ros__parameters:
    enable_recording: true
    output_file: "kalman_test_results.csv"
```

## 参数调优指南

### 如果滤波响应太慢

```yaml
Q_force: 20.0      # 增大（更信任传感器）
R_sensor: 1.0      # 减小（降低传感器噪声假设）
```

### 如果滤波输出抖动

```yaml
Q_force: 5.0       # 减小（更信任模型）
R_sensor: 5.0      # 增大（增加平滑）
```

### 如果想完全跟随传感器

```yaml
use_manual_fusion_weight: true
manual_fusion_weight: 2.0   # 或更大
```

## 高级功能

### 查表标定

实测不同力值下的弹簧位移，替换默认值：

```yaml
lookup_table_force: [100.0, 150.0, 200.0, 250.0, 300.0]  # 实测力值 (N)
lookup_table_disp:  [0.015, 0.025, 0.037, 0.051, 0.067]  # 对应位移 (m)
```

**标定方法**：
1. 用力传感器测量不同预紧力
2. 用卡尺/激光测距仪测量对应的弹簧压缩量
3. 填入表格（至少5个点，覆盖工作范围）

### 速度估计验证

```bash
# 发射后查看估计速度
ros2 topic echo /dart/kalman/estimated_velocity

# 对比实际飞行距离反推的速度
# v = sqrt(2 * g * h)  # 自由落体
```

## 故障排查

### 问题1：topic 不存在

**原因**：组件未加载

**解决**：
```bash
# 检查组件列表
ros2 param get /rmcs_executor components

# 应该包含 force_kalman
```

### 问题2：滤波值始终为0

**原因**：力传感器输入未就绪

**解决**：
```bash
# 检查力传感器 topic
ros2 topic hz /force_sensor/channel_1/weight
ros2 topic hz /force_sensor/channel_2/weight

# 应该有 ~1000Hz 输出
```

### 问题3：编译错误

**原因**：C++23 特性或依赖问题

**解决**：
```bash
# 清理重新编译
cd ~/RMCS_dart/RMCS/rmcs_ws
rm -rf build/ install/ log/
colcon build --packages-select rmcs_core --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 预期效果

### 成功指标

1. ✅ 卡尔曼滤波输出平滑但响应快速
2. ✅ 力闭环稳定时间减少 10-30%
3. ✅ 超调量减少
4. ✅ 发射一致性提高（速度标准差减小）

### 数据对比

**原始传感器**（预期）：
- 稳定时间：~1.2秒
- 力值波动：±1-2N
- 速度标准差：~0.3 m/s

**卡尔曼滤波**（目标）：
- 稳定时间：~0.9秒
- 力值波动：±0.5-1N
- 速度标准差：~0.2 m/s

## 下一步

1. **编译测试**：确认无编译错误
2. **空载测试**：不发射，只观察滤波效果
3. **单次发射**：验证力闭环工作
4. **连续发射**：收集统计数据（建议20次以上）
5. **参数优化**：根据实测数据微调 Q/R
6. **查表标定**：实测力-位移关系，提高精度

## 参考文档

- `KALMAN_FORCE_GUIDE.md` - 详细使用指南
- `force_kalman_filter.cpp` - 实现代码
- `spring_dynamics_model.cpp` - 弹簧模型参考
