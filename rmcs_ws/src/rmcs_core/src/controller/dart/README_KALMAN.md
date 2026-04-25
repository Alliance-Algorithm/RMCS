# 弹簧动力学卡尔曼滤波系统

## 文件结构

### 代码文件

#### V1 版本（独立实现）
- `spring_dynamics_model.cpp` - 原始弹簧模型（基于力传感器反推位移）
- `force_kalman_filter.cpp` - 独立卡尔曼滤波器（带查找表）
- `KALMAN_FORCE_GUIDE.md` - V1 使用指南

#### V2 版本（推荐使用）
- `spring_dynamics_model_v2.cpp` - **弹簧模型（基于丝杆角度正推力）**
  - 读取丝杆电机多圈角度
  - 通过运动学映射计算弹簧伸长量
  - 预测弹簧力（先验）
  - 检测相位并在 HOLDING 阶段校准
  
- `force_kalman_filter_v2.cpp` - **卡尔曼滤波器（融合模型与传感器）**
  - 使用 SpringDynamicsModelV2 的预测作为先验
  - 融合传感器测量作为后验
  - 输出最优力估计和力变化率

- `KALMAN_V2_GUIDE.md` - **V2 完整使用指南**

## 架构对比

### V1：独立实现
```
传感器 → 卡尔曼滤波 → 滤波后的力
         ↑
      查找表（需预先标定）
```

### V2：两层架构（推荐）
```
丝杆角度 → SpringDynamicsModelV2 → 预测力（先验）
                                      ↓
传感器测量 ────────────────→ ForceKalmanFilterV2 → 最优力估计（后验）
```

## 快速开始

### 1. 启用组件

在 `dart-guidance.yaml` 中：

```yaml
rmcs_executor:
  ros__parameters:
    components:
      - rmcs_core::controller::dart::SpringDynamicsModelV2 -> spring_model
      - rmcs_core::controller::dart::ForceKalmanFilterV2 -> force_kalman_v2
```

### 2. 校准参数

```yaml
spring_model:
  ros__parameters:
    screw_top_angle_rad: 0.0  # 扳机在最上端时的电机角度
    preload_force_N: 0.0      # 顶端位置的预紧力
```

### 3. 使用滤波后的力

```cpp
register_input("/dart/kalman/filtered_force", filtered_force_);
double F = *filtered_force_;  // 最优力估计
```

## 关键改进

V2 相比 V1：
- ✅ 无需查找表标定
- ✅ 基于物理运动学（丝杆 + 滑轮）
- ✅ 模块化设计（模型与滤波分离）
- ✅ 自动相位检测和校准
- ✅ 更准确的先验

## 详细文档

- **V2 使用指南**：`KALMAN_V2_GUIDE.md`
- V1 使用指南：`KALMAN_FORCE_GUIDE.md`

## 维护说明

- 推荐使用 V2 版本
- V1 版本保留用于兼容性和对比验证
- 所有新功能开发基于 V2
