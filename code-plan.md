# 主动悬挂 ADRC 方案 — 实现步骤

## 修改文件清单

| 文件 | 变更类型 | 说明 |
|------|---------|------|
| `rmcs_core/src/controller/chassis/deformable_joint_controller.cpp` | **重写** | PID 级联 → ADRC 内核 + 双模式切换 |
| `rmcs_core/src/controller/chassis/deformable_joint_sweep_recorder.cpp` | 修改 | 增加 ESO z3/z2 信号记录 |
| `rmcs_bringup/config/deformable-infantry-v2.yaml` | 修改 | ADRC 参数替换 PID 参数 |
| `plugins.xml` | 无修改 | `DeformableJointController` 已注册 |

---

## Step 1: 重写 DeformableJointController（ADRC 内核）

**文件**: `deformable_joint_controller.cpp`

### 1a. 替换 PID 头文件为 ADRC

```cpp
// 移除:
#include "controller/pid/pid_calculator.hpp"

// 添加:
#include "controller/adrc/TD.hpp"
#include "controller/adrc/ESO.hpp"
#include "controller/adrc/NLESF.hpp"
```

### 1b. 成员变量替换

```cpp
// 移除 PID 成员:
// pid::PidCalculator angle_pid_, velocity_pid_;
// pid::PidCalculator suspension_angle_pid_, suspension_velocity_pid_;

// 添加 ADRC 成员:
adrc::TD td_;
adrc::ESO eso_;
adrc::NLESF nlesf_;

// 双模式配置:
adrc::TD::Config normal_td_config_;
adrc::TD::Config suspension_td_config_;
adrc::ESO::Config normal_eso_config_;
adrc::ESO::Config suspension_eso_config_;
adrc::NLESF::Config normal_nlesf_config_;
adrc::NLESF::Config suspension_nlesf_config_;

double b0_ = 1.0;
double kt_ = 1.0;
double last_u_ = 0.0;
double output_min_, output_max_;
double suspension_output_min_, suspension_output_max_;
double torque_feedforward_gain_ = 0.0;
double suspension_torque_feedforward_gain_ = 0.0;

// ESO z3 输出（用于扫频记录和调试）
OutputInterface<double> eso_z3_output_;
OutputInterface<double> eso_z2_output_;
```

### 1c. 构造函数 — 加载 ADRC 参数

接口注册保持与当前 `DeformableJointController` 完全一致:
```cpp
register_input(get_parameter("measurement_angle").as_string(), measurement_angle_);
register_input(get_parameter("measurement_velocity").as_string(), measurement_velocity_);
register_input(get_parameter("setpoint_angle").as_string(), setpoint_angle_);
// setpoint_velocity, mode_input, suspension_torque — 同现有逻辑
register_output(get_parameter("control").as_string(), control_torque_, nan_);
```

新增 ESO 输出（可选，用于扫频记录）:
```cpp
if (has_parameter("eso_z3_output"))
    register_output(get_parameter("eso_z3_output").as_string(), eso_z3_output_, nan_);
if (has_parameter("eso_z2_output"))
    register_output(get_parameter("eso_z2_output").as_string(), eso_z2_output_, nan_);
```

加载通用 ADRC 参数:
```cpp
const double dt = load_parameter_or(*this, "dt", 0.001);
b0_ = load_parameter_or(*this, "b0", 1.0);
kt_ = load_parameter_or(*this, "kt", 1.0);
```

加载正常模式参数:
```cpp
normal_td_config_.h = load_parameter_or(*this, "td_h", dt);
normal_td_config_.r = load_parameter_or(*this, "td_r", 300.0);

normal_eso_config_.h = dt;
normal_eso_config_.b0 = b0_;
normal_eso_config_.w0 = load_parameter_or(*this, "eso_w0", 250.0);
normal_eso_config_.auto_beta = load_parameter_or_bool(*this, "eso_auto_beta", true);
normal_eso_config_.z3_limit = load_parameter_or(*this, "eso_z3_limit", 1e9);

normal_nlesf_config_.k1 = load_parameter_or(*this, "k1", 30.0);
normal_nlesf_config_.k2 = load_parameter_or(*this, "k2", 17.0);
normal_nlesf_config_.alpha1 = load_parameter_or(*this, "alpha1", 0.75);
normal_nlesf_config_.alpha2 = load_parameter_or(*this, "alpha2", 0.7);
normal_nlesf_config_.delta = load_parameter_or(*this, "delta", 0.02);

output_min_ = load_parameter_or(*this, "output_min", -200.0);
output_max_ = load_parameter_or(*this, "output_max", 200.0);
```

加载悬挂模式参数（fallback 到正常模式值）:
```cpp
suspension_td_config_.h = normal_td_config_.h;
suspension_td_config_.r = load_parameter_or(*this, "suspension_td_r", normal_td_config_.r);

suspension_eso_config_ = normal_eso_config_;
suspension_eso_config_.w0 = load_parameter_or(*this, "suspension_eso_w0", normal_eso_config_.w0);

suspension_nlesf_config_.k1 = load_parameter_or(*this, "suspension_k1", normal_nlesf_config_.k1);
suspension_nlesf_config_.k2 = load_parameter_or(*this, "suspension_k2", normal_nlesf_config_.k2);
suspension_nlesf_config_.alpha1 = load_parameter_or(*this, "suspension_alpha1", normal_nlesf_config_.alpha1);
suspension_nlesf_config_.alpha2 = load_parameter_or(*this, "suspension_alpha2", normal_nlesf_config_.alpha2);
suspension_nlesf_config_.delta = load_parameter_or(*this, "suspension_delta", normal_nlesf_config_.delta);

suspension_output_min_ = load_parameter_or(*this, "suspension_output_min", output_min_);
suspension_output_max_ = load_parameter_or(*this, "suspension_output_max", output_max_);
```

初始化 ADRC:
```cpp
td_ = adrc::TD(normal_td_config_);
eso_ = adrc::ESO(normal_eso_config_);
nlesf_ = adrc::NLESF(normal_nlesf_config_);
```

### 1d. update() — ADRC 核心循环

```cpp
void update() override {
    if (!std::isfinite(*measurement_angle_) || !std::isfinite(*setpoint_angle_)) {
        disable_output_();
        return;
    }

    // 模式切换
    const bool suspension_mode = *suspension_mode_;
    if (suspension_mode != last_suspension_mode_) {
        switch_adrc_mode_(suspension_mode);
        last_suspension_mode_ = suspension_mode;
    }

    // ADRC 核心计算
    const auto td_out = td_.update(*setpoint_angle_);
    const auto eso_out = eso_.update(*measurement_angle_, last_u_);

    const double e1 = td_out.x1 - eso_out.z1;
    const double e2 = td_out.x2 - eso_out.z2;

    const auto nlesf_out = nlesf_.compute(e1, e2, eso_out.z3, b0_);
    double u = kt_ * nlesf_out.u;

    // 可选: 悬挂力矩前馈
    const double torque_ff_gain =
        suspension_mode ? suspension_torque_feedforward_gain_ : torque_feedforward_gain_;
    if (std::isfinite(*suspension_torque_) && torque_ff_gain != 0.0)
        u += torque_ff_gain * *suspension_torque_;

    // 限幅（模式相关）
    const double u_min = suspension_mode ? suspension_output_min_ : output_min_;
    const double u_max = suspension_mode ? suspension_output_max_ : output_max_;
    u = std::clamp(u, u_min, u_max);

    if (!std::isfinite(u)) {
        disable_output_();
        return;
    }

    *control_torque_ = u;
    last_u_ = u;

    // 输出 ESO 状态供记录
    if (eso_z3_output_.active()) *eso_z3_output_ = eso_out.z3;
    if (eso_z2_output_.active()) *eso_z2_output_ = eso_out.z2;
}
```

### 1e. 模式切换

```cpp
void switch_adrc_mode_(bool suspension) {
    // 切换 ADRC 参数，但保留 ESO/TD 状态（平滑过渡）
    if (suspension) {
        td_.set_config(suspension_td_config_);
        eso_.set_config(suspension_eso_config_);
        nlesf_.set_config(suspension_nlesf_config_);
    } else {
        td_.set_config(normal_td_config_);
        eso_.set_config(normal_eso_config_);
        nlesf_.set_config(normal_nlesf_config_);
    }
    // 注意: 不调用 reset() — ESO 和 TD 的状态需要保留以实现平滑过渡
}
```

---

## Step 2: 修改 SweepRecorder 记录 ADRC 信号

**文件**: `deformable_joint_sweep_recorder.cpp`

### 2a. 新增输入接口

```cpp
// 可选输入（不影响现有功能）
if (has_parameter("eso_z3")) {
    register_input(get_parameter("eso_z3").as_string(), eso_z3_);
    has_eso_z3_ = true;
}
if (has_parameter("eso_z2")) {
    register_input(get_parameter("eso_z2").as_string(), eso_z2_);
    has_eso_z2_ = true;
}
```

### 2b. CSV 列新增

```cpp
// CSV 头
csv_stream_ << "...,measured_torque,frequency_hz,phase_rad,eso_z3,eso_z2\n";

// 数据行
csv_stream_ << ... << ',' << *frequency_hz_ << ',' << *phase_rad_
            << ',' << (has_eso_z3_ ? *eso_z3_ : nan_)
            << ',' << (has_eso_z2_ ? *eso_z2_ : nan_)
            << '\n';
```

---

## Step 3: YAML 配置更新

**文件**: `deformable-infantry-v2.yaml`

### 关节控制器参数（每个关节）

```yaml
lf_joint_controller:
  ros__parameters:
    # 接口（不变）
    measurement_angle: /chassis/left_front_joint/physical_angle
    measurement_velocity: /chassis/left_front_joint/physical_velocity
    setpoint_angle: /chassis/left_front_joint/target_physical_angle
    setpoint_velocity: /chassis/left_front_joint/target_physical_velocity
    mode_input: /chassis/left_front_joint/suspension_mode
    suspension_torque: /chassis/left_front_joint/suspension_torque
    control: /chassis/left_front_joint/control_torque

    # ESO 输出（扫频记录用）
    eso_z3_output: /chassis/left_front_joint/eso_z3
    eso_z2_output: /chassis/left_front_joint/eso_z2

    # ADRC 通用参数
    dt: 0.001
    b0: 0.60          # ← 从扫频辨识确定
    kt: 1.0

    # 正常模式（刚性跟踪）
    td_r: 50.0
    td_h: 0.001
    eso_w0: 250.0
    eso_auto_beta: true
    k1: 30.0
    k2: 17.0
    alpha1: 0.75
    alpha2: 0.7
    delta: 0.02
    output_min: -200.0
    output_max: 200.0

    # 悬挂模式（柔顺弹簧）
    suspension_td_r: 10.0
    suspension_eso_w0: 80.0
    suspension_k1: 8.0
    suspension_k2: 5.0
    suspension_alpha1: 0.75
    suspension_alpha2: 0.7
    suspension_delta: 0.02
    suspension_output_min: -50.0
    suspension_output_max: 50.0

    # 力矩前馈增益
    torque_feedforward_gain: 0.0
    suspension_torque_feedforward_gain: 0.0
```

### 扫频记录器参数新增

```yaml
joint_sweep_recorder:
  ros__parameters:
    # ... 现有参数 ...
    eso_z3: /chassis/left_front_joint/eso_z3
    eso_z2: /chassis/left_front_joint/eso_z2
```

---

## Step 4: 扫频辨识 b0 流程

### 4a. 运行扫频

使用现有 `DeformableJointSweepController` + 改造后的 `DeformableJointController (ADRC)` + 增强的 recorder。

先用一组保守 ADRC 参数（低增益避免失控）：
```yaml
b0: 1.0       # 初始猜测
eso_w0: 100.0  # 中等带宽
k1: 10.0
k2: 5.0
output_min: -100.0
output_max: 100.0
```

### 4b. 从 CSV 辨识 b0

系统模型: `theta_ddot = b0 * u + f(t)`  (f = 总扰动)

方法 1 — 时域最小二乘:
```python
# Python 后处理
theta_ddot = np.gradient(np.gradient(actual_angle, dt), dt)
b0_estimate = np.mean(theta_ddot / control_torque)  # 在扰动平稳段取平均
```

方法 2 — 频域 (更精确):
```python
# Bode 图: H(jw) = theta(jw) / u(jw)
# 低频渐近线增益 |H(0)| ≈ 1/(b0 * w^2)
# 从频率响应曲线拟合 b0
```

### 4c. 从 b0 推导悬挂参数

```
期望弹簧刚度 K [N·m/rad]   →  suspension_k1 ≈ K * b0
期望阻尼系数 D [N·m·s/rad] →  suspension_k2 ≈ D * b0
ESO 带宽                    →  suspension_eso_w0 ≈ 系统带宽 * 3 (从 Bode 图)
力矩限幅                    →  suspension_output_max ≈ mg/4 * rod_length + 安全裕量
```

---

## Step 5: 编译和验证

```bash
build-rmcs --packages-select rmcs_core
```

### 验证清单

- [ ] 编译通过
- [ ] 正常模式: 关节精确跟踪目标角度（与 PID 性能对比）
- [ ] 悬挂模式: 手推关节可以柔顺偏移，松手后缓慢恢复
- [ ] 模式切换: normal ↔ suspension 切换无冲击（ESO 状态保持）
- [ ] 悬空测试: 抬起轮组 → 关节自动伸展到目标位置
- [ ] 触地测试: 放下轮组 → 柔顺吸收冲击
- [ ] 扫频: CSV 数据包含 eso_z3, eso_z2 列
- [ ] value_broadcaster 可监控 ESO z3 实时值
