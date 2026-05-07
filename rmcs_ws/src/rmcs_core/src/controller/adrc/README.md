# ADRC V2 说明（公式 + 变量含义）

对应代码文件：
- `TD.hpp`
- `ESO.hpp`
- `NLESF.hpp`
- `adrc_controller_v2.cpp`

## 1. 总体结构

单通道 ADRC 每周期执行顺序：

1. TD: `r -> x1, x2`
2. ESO: `y, u(k-1) -> z1, z2, z3`
3. NLESF: `e1, e2, z3, b0 -> u`
4. 输出缩放与限幅: `u_out = clamp(kt * u, output_min, output_max)`

其中：
- `r`: 参考输入（目标）
- `y`: 测量反馈
- `u`: 控制器内部输出
- `u_out`: 组件最终输出

---

## 2. TD（Tracking Differentiator）

### 2.1 状态与作用

- `x1`: 平滑后的参考轨迹
- `x2`: 平滑后的参考导数

TD 作用：把突变目标 `r` 变成平滑的 `(x1, x2)`，降低控制冲击。

### 2.2 连续形式

```text
x1_dot = x2
x2_dot = fhan(x1 - r, x2, r_td, h)
```

### 2.3 离散更新（代码实现）

```text
x1(k+1) = x1(k) + h * x2(k)
x2(k+1) = x2(k) + h * fh
fh = fhan(x1 - r, x2, r_td, h)
```

### 2.4 fhan 子公式

```text
d  = r_td * h^2
a0 = h * x2
y  = (x1 - r) + a0
a1 = sqrt(d * (d + 8 * abs(y)))

if abs(y) > d:
    a = a0 + sign(y) * (a1 - d) / 2
else:
    a = a0 + y

if abs(a) <= d:
    fhan = -r_td * a / d
else:
    fhan = -r_td * sign(a)
```

### 2.5 TD 变量物理含义

| 变量 | 含义 | 典型影响 |
|---|---|---|
| `r` | 原始目标输入 | 上层给定目标 |
| `x1` | 平滑目标 | 作为位置/主误差基准 |
| `x2` | 平滑目标导数 | 作为速度/辅误差基准 |
| `r_td` | TD 跟踪速度参数 | 大则快，小则更平滑 |
| `h` | TD 步长 | 通常与 `dt` 同量级 |
| `fh` | TD 内部“加速度”输出 | 驱动 `x2` 更新 |

---

## 3. ESO（Extended State Observer）

### 3.1 状态定义

- `z1`: 对输出 `y` 的估计
- `z2`: 对输出一阶导的估计
- `z3`: 对总扰动的估计（外扰 + 未建模项 + 参数误差）

### 3.2 观测误差

```text
e = z1 - y
```

### 3.3 连续形式（LESO）

```text
z1_dot = z2 - beta1 * e
z2_dot = z3 + b0 * u - beta2 * e
z3_dot = -beta3 * e
```

### 3.4 离散更新（代码实现）

```text
z1(k+1) = z1(k) + h * (z2 - beta1 * e)
z2(k+1) = z2(k) + h * (z3 + b0 * u - beta2 * e)
z3(k+1) = z3(k) + h * (-beta3 * e)
```

### 3.5 带宽法自动整定

当 `eso_auto_beta = true`：

```text
beta1 = 3 * w0
beta2 = 3 * w0^2
beta3 = w0^3
```

### 3.6 ESO 变量物理含义

| 变量 | 含义 | 典型影响 |
|---|---|---|
| `y` | 传感器测量值 | 观测器输入 |
| `u` | 控制输入（本实现用上拍输出） | 进入系统模型输入项 |
| `b0` | 标称控制增益 | 控制量到输出变化的比例 |
| `w0` | ESO 带宽 | 大则估计快但更敏感噪声 |
| `beta1/2/3` | 观测器注入增益 | 决定误差收敛速度 |
| `z3` | 总扰动估计 | 用于控制补偿 |

---

## 4. NLESF（Nonlinear Error State Feedback）

### 4.1 误差定义

```text
e1 = x1 - z1
e2 = x2 - z2
```

- `e1`: 轨迹误差（主误差）
- `e2`: 导数误差（阻尼/动态误差）

### 4.2 非线性函数 fal

```text
fal(e, alpha, delta) =
    e / delta^(1 - alpha),           if |e| <= delta
    |e|^alpha * sign(e),             if |e| >  delta
```

含义：
- 小误差线性化，抑制抖动；
- 大误差非线性增益，提高收敛速度。

### 4.3 控制律

先计算反馈控制项：

```text
u0 = k1 * fal(e1, alpha1, delta)
   + k2 * fal(e2, alpha2, delta)
```

再做扰动补偿：

```text
u = (u0 - z3) / b0
```

最后内部限幅：

```text
u = clamp(u, u_min, u_max)
```

### 4.4 NLESF 变量物理含义

| 变量 | 含义 | 典型影响 |
|---|---|---|
| `k1` | 主误差增益 | 大则位置误差收敛更快 |
| `k2` | 导数误差增益 | 大则阻尼更强 |
| `alpha1/alpha2` | fal 指数 | 改变非线性强度 |
| `delta` | fal 线性区阈值 | 小则更“非线性” |
| `u0` | 未补偿控制量 | 仅反馈项 |
| `u` | 补偿后控制量 | 实际控制基础输出 |

---

## 5. `adrc_controller_v2.cpp` 里的整合关系

每周期核心计算：

```text
td_out  = TD.update(reference)
eso_out = ESO.update(measurement, last_u)

e1 = td_out.x1 - eso_out.z1
e2 = td_out.x2 - eso_out.z2

nlesf_out = NLESF.compute(e1, e2, eso_out.z3, b0)
u = nlesf_out.u

u_out = clamp(kt * u, output_min, output_max)
last_u = u_out
```

### error-input 模式

若未提供 `setpoint`/`target`，则输入 `measurement` 被当作误差 `e = r - y`，内部等效转换：

```text
reference  = 0
measurement = -e
```

---

## 6. 参数接口（简化 YAML 友好）

### 6.1 必要参数

| 参数 | 说明 |
|---|---|
| `measurement` | 测量输入通道名 |
| `control` | 控制输出通道名 |

### 6.2 可选输入

| 参数 | 说明 |
|---|---|
| `setpoint` 或 `target` | 参考输入通道名 |

### 6.3 通用参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `dt` | `0.001` | 控制周期（秒） |
| `b0` | `1.0` | 标称增益 |
| `kt` | `1.0` | 输出缩放 |
| `output_min` | `-inf` | 最终输出下限 |
| `output_max` | `+inf` | 最终输出上限 |
| `init_reference` | `0.0` | TD 初始参考 |
| `init_measurement` | `0.0` | ESO 初始测量 |

### 6.4 TD 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `td_h` | `dt` | TD 步长 |
| `td_r` | `300.0` | TD 跟踪速度 |
| `td_max_vel` | `+inf` | TD 速度限幅 |
| `td_max_acc` | `+inf` | TD 加速度限幅 |

### 6.5 ESO 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `eso_w0` | `80.0` | ESO 带宽 |
| `eso_auto_beta` | `true` | 自动生成 beta |
| `eso_beta1` | `3*w0` | 手动 beta1 |
| `eso_beta2` | `3*w0^2` | 手动 beta2 |
| `eso_beta3` | `w0^3` | 手动 beta3 |
| `eso_z3_limit` | `1e9` | 扰动估计限幅 |

### 6.6 NLESF 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `k1` | `50.0` | 主误差增益 |
| `k2` | `5.0` | 导数误差增益 |
| `alpha1` | `0.75` | fal 指数 1 |
| `alpha2` | `1.25` | fal 指数 2 |
| `delta` | `0.01` | fal 线性区阈值 |
| `u_min` | `-inf` | NLESF 内部下限 |
| `u_max` | `+inf` | NLESF 内部上限 |

---

## 7. 变量总览速查

| 类别 | 变量 |
|---|---|
| 参考轨迹 | `r, x1, x2` |
| 测量与观测 | `y, z1, z2, z3, e` |
| 反馈误差 | `e1, e2` |
| 控制量 | `u0, u, u_out` |
| 参数 | `b0, w0, beta1, beta2, beta3, k1, k2, alpha1, alpha2, delta, h` |

