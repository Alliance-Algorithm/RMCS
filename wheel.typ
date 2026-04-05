#import "./rmcs_notebook/template/template.typ": *

== Deformable Infantry V2 独立轮解算

本文针对 `deformable infantry v2` 底盘，推导四个轮各自独立轮距、独立关节分速度条件下的统一 Jacobian 解算公式。

该底盘基于舵轮底盘构建，基础舵轮模型可参考 `rmcs_notebook/chapters/algorithm/steering_wheel.typ`。与普通舵轮不同的是，本底盘的每个轮模块均沿固定对角射线做径向伸缩，因此轮心位置、轮心速度、底盘速度观测、目标舵向以及动力学分配都需要在每轮独立几何下重新推导。

=== 建模前提

已知前提如下：

- `deformable infantry v2` 是在舵轮底盘基础上，融合关节分速度与可变轮距实现解算的底盘
- 当前目标是推导适用于 `DeformableWheelController` 的完整独立轮模型
- 四个轮模块的顺序为：左前、左后、右后、右前
- 四个关节各自所在射线都与底盘坐标轴成 $45 deg$
- 关节角 $alpha_i$ 的物理语义为“连杆与地面的夹角”
- 关节运动范围上限为 $62.5 deg$，下限为 $8 deg$

在上述角度定义下：

- 当 $alpha_i = 62.5 deg$ 时，轮距较小
- 当 $alpha_i = 8 deg$ 时，轮距较大

本文不再沿用 rigid swerve 的闭式公式直接替换 $R -> R_i$，而是从每个轮模块独立的轮心位置与轮心速度出发，统一推导观测与控制公式。

=== 独立轮几何建模

建立底盘坐标系 $B$：

- $x$ 轴向前
- $y$ 轴向左
- $z$ 轴向上

设第 $i$ 个轮模块所在固定射线与 $x$ 轴夹角为 $phi_i$，则有：

$
  phi_i = cases(
      pi / 4 comma quad & i = 1 comma \
    3 pi / 4 comma quad & i = 2 comma \
    - 3 pi / 4 comma quad & i = 3 comma \
      - pi / 4 comma quad & i = 4 .
  )
$

定义第 $i$ 个轮模块的径向单位向量与切向单位向量：

$
  vb(e)_(r i) = mat(cos(phi_i), sin(phi_i))
$

$
  vb(e)_(t i) = mat(- sin(phi_i), cos(phi_i))
$

其中，$vb(e)_(r i)$ 表示底盘中心指向轮心的方向，$vb(e)_(t i)$ 表示底盘绕中心旋转时轮心的瞬时切向方向。

设第 $i$ 个关节的物理角为 $alpha_i$，则第 $i$ 个轮的轮距定义为：

$
  R_i = R_0 + L cos(alpha_i)
$

其中：

- $R_0$ 为固定基座半径
- $L$ 为连杆长度

于是第 $i$ 个轮心相对底盘中心的位置向量为：

$
  vb(p)_i = R_i vb(e)_(r i)
$

对时间求导得：

$
  dot(vb(p))_i = dot(R_i) vb(e)_(r i)
$

其中：

$
  dot(R_i) = - L sin(alpha_i) dot(alpha_i)
$

继续求导得：

$
  ddot(vb(p))_i = ddot(R_i) vb(e)_(r i)
$

$
  ddot(R_i) = - L cos(alpha_i) dot(alpha_i)^2 - L sin(alpha_i) ddot(alpha_i)
$

=== 轮心速度

设底盘中心平动速度为：

$
  vb(v) = mat(v_x, v_y)
$

底盘角速度为：

$
  omega = dot(theta)
$

定义二维平面内的旋转算子：

$
  J = mat(0, -1; 1, 0)
$

则第 $i$ 个轮心的绝对速度为：

$
  vb(v)_i = vb(v) + omega J vb(p)_i + dot(vb(p))_i
$

代入 $vb(p)_i = R_i vb(e)_(r i)$ 后得：

$
  vb(v)_i = vb(v) + omega R_i vb(e)_(t i) + dot(R_i) vb(e)_(r i)
$

展开成坐标形式：

$
  v_(i x) = v_x - omega R_i sin(phi_i) + dot(R_i) cos(phi_i)
$

$
  v_(i y) = v_y + omega R_i cos(phi_i) + dot(R_i) sin(phi_i)
$

该式表明每个轮心速度由三部分叠加得到：

- 底盘平动速度
- 底盘转动带来的切向速度
- 关节伸缩带来的径向速度

=== 观测底盘速度

设第 $i$ 个舵轮的滚动方向单位向量为：

$
  vb(s)_i = mat(cos(zeta_i), sin(zeta_i))
$

其法向单位向量为：

$
  vb(n)_i = mat(- sin(zeta_i), cos(zeta_i))
$

若轮不发生侧滑，则满足约束：

$
  vb(n)_i dot.op vb(v)_i = 0
$

若轮半径为 $r$，轮电机角速度为 $omega_i$，则滚动约束为：

$
  vb(s)_i dot.op vb(v)_i = r omega_i
$

代入轮心速度公式：

$
  vb(s)_i dot.op vb(v) + omega R_i vb(s)_i dot.op vb(e)_(t i) + dot(R_i) vb(s)_i dot.op vb(e)_(r i) = r omega_i
$

注意到：

$
  vb(s)_i dot.op vb(e)_(r i) = cos(zeta_i - phi_i)
$

$
  vb(s)_i dot.op vb(e)_(t i) = sin(zeta_i - phi_i)
$

故可写为：

$
  cos(zeta_i) v_x + sin(zeta_i) v_y + R_i sin(zeta_i - phi_i) omega = r omega_i - dot(R_i) cos(zeta_i - phi_i)
$

这就是第 $i$ 个独立轮的标量滚动方程。

将四个轮的滚动方程联立，写为矩阵形式：

$
  A x = b
$

其中：

$
  x = mat(v_x, v_y, omega)
$

第 $i$ 行定义为：

$
  A_i = mat(cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i))
$

$
  b_i = r omega_i - dot(R_i) cos(zeta_i - phi_i)
$

由于共 $4$ 个方程、$3$ 个未知数，属于超定线性方程组，可采用最小二乘解：

$
  hat(x) = (A^T A)^(-1) A^T b
$

若考虑不同轮的可信度，可进一步引入权重矩阵 $W$，采用加权最小二乘：

$
  hat(x) = (A^T W A)^(-1) A^T W b
$

其中权重可用于表示：

- 某个轮打滑时可信度下降
- 某个轮接地较差
- 某个轮传感器异常

=== 目标舵向与目标轮速

给定目标底盘平动速度 $vb(v)^* = mat(v_x^*, v_y^*)$、目标角速度 $omega^*$，以及期望关节状态决定的 $R_i$ 与 $dot(R_i)$，第 $i$ 个轮的目标轮心速度为：

$
  vb(v)_i^* = vb(v)^* + omega^* R_i vb(e)_(t i) + dot(R_i) vb(e)_(r i)
$

展开为：

$
  v_(i x)^* = v_x^* - omega^* R_i sin(phi_i) + dot(R_i) cos(phi_i)
$

$
  v_(i y)^* = v_y^* + omega^* R_i cos(phi_i) + dot(R_i) sin(phi_i)
$

于是目标舵向定义为：

$
  zeta_i^* = tan^(-1) (v_(i y)^*, v_(i x)^*)
$

目标轮速定义为：

$
  omega_i^* = frac(|vb(v)_i^*|, r)
$

与普通舵轮不同的是，这里的目标舵向不再从 rigid swerve 的闭式公式直接展开，而是直接由目标轮心速度的方向给出。

=== 舵向角速度

设：

$
  vb(u)_i = vb(v)_i
$

则第 $i$ 个轮的舵向角满足：

$
  zeta_i = tan^(-1) (u_(i y), u_(i x))
$

其时间导数为：

$
  dot(zeta_i) = frac(u_(i x) dot(u)_(i y) - u_(i y) dot(u)_(i x), u_(i x)^2 + u_(i y)^2)
$

因此需要继续求 $dot(vb(u))_i$。

由：

$
  vb(u)_i = vb(v) + omega R_i vb(e)_(t i) + dot(R_i) vb(e)_(r i)
$

求导得：

$
  dot(vb(u))_i = vb(a) + dot(omega) R_i vb(e)_(t i) + omega dot(R_i) vb(e)_(t i) + ddot(R_i) vb(e)_(r i)
$

其中：

$
  vb(a) = mat(a_x, a_y)
$

$
  dot(omega) = ddot(theta)
$

展开为坐标形式：

$
  dot(u)_(i x) = a_x - dot(omega) R_i sin(phi_i) - omega dot(R_i) sin(phi_i) + ddot(R_i) cos(phi_i)
$

$
  dot(u)_(i y) = a_y + dot(omega) R_i cos(phi_i) + omega dot(R_i) cos(phi_i) + ddot(R_i) sin(phi_i)
$

因此完整的舵向角速度公式为：

$
  dot(zeta_i) = frac(u_(i x) dot(u)_(i y) - u_(i y) dot(u)_(i x), u_(i x)^2 + u_(i y)^2)
$

与 rigid swerve 相比，这里新增的关键项为：

- $omega dot(R_i)$
- $ddot(R_i)$

这两项正是“边变形边旋转”时不可忽略的耦合项。

=== 独立轮动力学分配

若第 $i$ 个轮沿滚动方向 $vb(s)_i$ 提供驱动力 $f_i$，则整车平面动力学满足：

$
  sum_(i = 1)^4 f_i vb(s)_i = m vb(a)
$

$
  sum_(i = 1)^4 (vb(p)_i times vb(s)_i) f_i = J dot(omega)
$

其中二维叉乘标量为：

$
  vb(p)_i times vb(s)_i = R_i sin(zeta_i - phi_i)
$

因此可写为矩阵形式：

$
  B vb(f) = vb(tau)
$

其中：

$
  vb(f) = mat(f_1, f_2, f_3, f_4)
$

$
  vb(tau) = mat(m a_x, m a_y, J dot(omega))
$

第 $i$ 列定义为：

$
  B_i = mat(cos(zeta_i), sin(zeta_i), R_i sin(zeta_i - phi_i))
$

若采用最小范数分配，则：

$
  vb(f) = B^T (B B^T)^(-1) vb(tau)
$

轮电机目标力矩为：

$
  tau_i = r f_i
$

这给出了独立轮模型下的轮力矩分配公式。若后续需要加入摩擦、功率与关节约束，应在该分配模型基础上继续扩展，而不应回退到 rigid swerve 的旧闭式公式。

=== 退化情况与实现注意事项

本文推导的独立轮模型应满足以下退化条件：

1. 当四个轮满足 $R_i = R$ 且 $dot(R_i) = 0$ 时，应退化为普通舵轮底盘公式。
2. 当 $omega = 0$ 时，每个轮的目标方向应仅由平动速度与径向伸缩速度叠加决定。
3. 当 $vb(v) = 0$ 且 $omega != 0$ 时，轮心速度应为切向速度与径向速度叠加。
4. 当某个轮满足 $vb(v)_i = 0$ 时，$tan^(-1)$ 与 $dot(zeta_i)$ 将发生奇异，需要在实现中单独做降级处理。

=== 结论

对于 `deformable infantry v2` 这类四轮独立可变轮距底盘，更合理的解算方式不是在 rigid swerve 闭式公式上直接替换 $R -> R_i$，而是：

- 先定义每个轮独立的轮心位置 $vb(p)_i$
- 再定义每个轮独立的轮心速度 $vb(v)_i$
- 所有底盘速度观测、目标舵向、目标轮速、舵向角速度和轮力分配，都从统一的独立轮几何模型出发推导

这样才能正确覆盖：

- 前后不同高
- 四轮独立姿态
- 变形过程中的舵向连续控制
- 高速旋转与关节变形叠加
