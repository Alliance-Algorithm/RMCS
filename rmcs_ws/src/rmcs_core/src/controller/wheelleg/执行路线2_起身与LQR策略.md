# 执行路线 2：起身与平衡控制策略

> 独立于 `执行路线1_数学解析.md`（那里只留几何/机械参数/质心公式）。
> 本文记录**起身/平衡控制律**：系统结构、完整约束、任务优先级、协调前馈，
> 以及**从控制量到电机命令量的闭环**。
> 标注 **[实测]** = 已从模型算出；**[已核对]** = 已用 MJCF/几何数值验证；**[待定]** = 设计选择。

## 1. 问题定义

- 起身 = 从初始姿态 $S_0$（轮 + 机身同时着地，[待定]）**统一闭环**收敛到目标静态位形 $S^\*$。
- **不使用开环脚本、不做分段**：全部用**软优先级任务**表达（不可行时按权重退让）。
- **目标全是相对量**：只要求整机内部稳态（腿型、姿态、CoM 相对接触点、速度零），
  **不约束世界位置 / 轮轴绝对位置**。轮速是积分从动量，不规划。

## 2. 目标静态位形 $S^\*$ [实测]

约束：髋高 $0.400$ m、$\theta=0$、$\text{CoM}_x=$ 接触点 $x$、$v=0$。数值解（L 腿，R 腿镜像）：

| 量 | 值 |
|---|---|
| 髋角 $q_1$ | $0.7147$ rad $=40.95°$ |
| 膝角 $q_2$ | $0.9199$ rad $=52.71°$ |
| 腿长 $L_0=\lvert HA\rvert$ | $0.3405$ m |
| 轮-髋水平偏移 $x_e$ | $+0.0117$ m |
| 髋高 | $0.4000$ m |
| 机身原点高 | $0.4008$ m |
| 整机质心高 | $0.3239$ m |
| $\text{CoM}_x-$接触点$_x$ | $\approx 0$ |
| 降级角 $\varphi,\beta$ | $54.03°,\ -94.98°$ |
| 腿角 $\alpha_L$（由 $L_0\sin\alpha_L=h-r$ 反推） | $\approx 86.9°=1.516$ rad |

## 3. 完整约束清单 [实测 / 已核对]

### 3.1 几何约束

| 约束 | 值 |
|---|---|
| 腿长 | $0.043\sim0.460$ m |
| 髋高 | $\approx0.10\sim0.52$ m；目标 $0.38\sim0.42$ m |
| 膝伸直奇异 | $L_0\to L_1+L_2=0.46$，$\partial L_0/\partial\beta\to0$（避开） |
| 四杆可达 | linkage knee $\in[0.258,3.141]$ $\Leftrightarrow$ $\beta\in[-2.883,0]$ |

### 3.2 力矩约束

模型已是**真闭链（A1）**：曲柄与大腿是髋处同轴的**两个独立关节**，曲柄经两级四杆驱动膝
（`models/wheelleg/build_closed_chain.py`）。因此不再需要串行近似的力矩反射，两个电机各自限幅：

| 执行器 | 限幅 |
|---|---|
| 大腿 motor（joint1） | $40$ Nm |
| 曲柄 motor | $40$ Nm |
| 轮电机（减速后） | $47.3$ Nm |
| 轮-地摩擦（静态，$\mu=2$） | $\mu M g r\approx15.0$ Nm |
| **轮有效限幅** | $\min(47.3,\,15.0)=\mathbf{15.0}$ Nm（摩擦先到） |

> 四杆速度比 $r=d\beta/dq_{crank}\in[0.08,0.918]<1$，膝端力矩被放大（$\tau_{knee}=\tau_{crank}/|r|$）；
> 该关系现在由**闭链模型自身**实现。串行近似旧版用手工反射（髋 80 / 膝 $40/|r|$），A1 下简化为 40/40。
> 曲柄关节与大腿耦合：$c_{eff}=c_{asm}+q_{crank}\pm q_{thigh}$（左 +，右 −），桥中据此把膝参考换算成曲柄目标。

### 3.3 摩擦 / 不打滑

- 轮-地 $\mu\approx2$；水平外力**只能由轮-地提供**，腿蹬地的水平反力受此限。

### 3.4 终端速度

- $v=0$（整车速度为零）。位置自由 $\ne$ 速度自由，需显式收速度。

## 4. 系统结构与执行器权限 [已核对]

矢状面坐标 $(x_w,\theta,L_0,\alpha_L)$，**4 DOF**；输入 3，矩阵 $B$ 为 $4\times3$，秩 3
$\Rightarrow$ **欠驱动 1 维**。权限：

| 量 | 执行器 |
|---|---|
| 腿长 $L_0$ | 曲柄（经四杆） |
| 腿角 $\alpha_L$ | 大腿 |
| 平衡模态 $(x_w,\theta)$ | 轮 |
| 底移 $x_A$ / 轮速 | **从动**（积分态） |

关键事实：

- **$\theta$ 欠驱动**：髋/曲柄力矩是**内力矩**，只能在机身与腿之间重分配角动量；
  绝对 pitch 必须靠**外力**（轮-地）。轮是唯一外部力矩源。
- $\text{CoM}_x = A_x + d(L_0,\alpha_L,\theta)$，其中 $d$ **与轮绝对位置无关**。
  故"静态稳定（CoM 与轮轴同轴）"是**相对条件**，由**腿**满足，**轮无权限**；
  轮平移 $A_x$ 时 $\text{CoM}_x$ 同步平移，$d$ 不变。

## 5. 任务坐标与雅可比 [已核对]

三层坐标：

- 电机 $q=(q_1,q_2,q_3)$（大腿、曲柄/膝、轮）
- 机械 $(\psi_1,\beta,\phi_w)$（大腿绝对角、膝角、轮角）
- 任务 $y=(L_0,\ \alpha_L,\ \omega_w)$

正解（路线 1 §4）：

$$L_0=\sqrt{L_1^2+L_2^2-2L_1L_2\cos\beta},\qquad
\alpha_L=\psi_1+\operatorname{atan2}\!\big(L_2\sin\beta,\ L_1-L_2\cos\beta\big)$$

雅可比 $J_{leg}=\partial(L_0,\alpha_L)/\partial(q_1,q_2)$，目标位形实测：

$$J_{leg}=\begin{bmatrix}0 & 0.1538\\ -1 & 0.5795\end{bmatrix}
\qquad(\text{数值 vs 解析最大误差 }1.9\text{e-}4)$$

力映射（虚功 / 功率平衡）：

$$\begin{bmatrix}\tau_{thigh}\\\tau_{crank}\end{bmatrix}
=J_{leg}^{\mathsf T}\begin{bmatrix}F_L\\ \tau_\alpha\end{bmatrix}+\tau_g,
\qquad \tau_{wheel}=\tau_w$$

**用 $J^{\mathsf T}$ 不用 $J^{-\mathsf T}$**：腿伸直 $\partial L_0/\partial\beta\to0$ 时逆映射发散；
转置有界、保无源性。

## 5.5 相对髋关节的重力矩（等效腿坐标）[推导 + 已核对]

**坐标系与记号**：原点取在髋关节 $H$，$x$ 轴沿机身（前为正），$y$ 轴与 $x$ 垂直。任意点 $P$ 的坐标记为
$P=(x_P,\ y_P)$，即**相对 $H$ 的坐标**（故 $x_H=y_H=0$）。重力竖直向下：$\boldsymbol g=(0,\,-g)$。

**等效腿**：腿长 $L=\lvert HA\rvert$，髋–轮轴连线 $H\to A$ 与 $x^+$ 成角 $\theta$，则轮轴

$$A=(x_A,\ y_A)=(L\cos\theta,\ L\sin\theta).$$

**膝关节**（由路线1 §3 的正解）。记

$$a=\frac{L^2+L_2^2-L_1^2}{2L},\qquad
\rho=L-a=\frac{L^2+L_1^2-L_2^2}{2L},\qquad
b=\sigma\sqrt{L_2^2-a^2}\quad(\sigma=\pm1\ \text{装配分支}),$$

则

$$K=(x_K,\ y_K)=(\rho\cos\theta+b\sin\theta,\ \ \rho\sin\theta-b\cos\theta).$$

**大腿、小腿质心**（$C_1$ 沿 $\widehat{HK}$ 偏 $d_1$ 转 $\beta_1$；$C_2$ 沿 $\widehat{KA}$ 偏 $d_2$ 转 $\beta_2'=\beta_2-\beta_3$）：

$$x_{C_1}=\frac{d_1}{L_1}\big(x_K\cos\beta_1-y_K\sin\beta_1\big),\qquad
y_{C_1}=\frac{d_1}{L_1}\big(x_K\sin\beta_1+y_K\cos\beta_1\big),$$

$$x_{C_2}=x_K+\frac{d_2}{L_2}\big((x_A-x_K)\cos\beta_2'-(y_A-y_K)\sin\beta_2'\big),\qquad
y_{C_2}=y_K+\frac{d_2}{L_2}\big((x_A-x_K)\sin\beta_2'+(y_A-y_K)\cos\beta_2'\big).$$

展开成只含 $\theta$ 的形式：

$$x_{C_1}=\frac{d_1}{L_1}\big[\rho\cos(\theta+\beta_1)+b\sin(\theta+\beta_1)\big],$$

$$x_{C_2}=\rho\cos\theta+b\sin\theta+\frac{d_2}{L_2}\big[a\cos(\theta+\beta_2')-b\sin(\theta+\beta_2')\big].$$

**机身质心**相对 $H$ 固定（路线1 §2）：$x_b=x_{C_b}=0.005$。

**机身 + 腿的合成质心**：

$$x_{C_{b\ell}}=\frac{m_b\,x_b+m_1\,x_{C_1}+m_2\,x_{C_2}}{m_b+m_1+m_2}.$$

**重力矩（绕髋横轴）**：重力 $=m_i g$ 竖直向下，力臂为该质心的 $x$ 坐标，故

$$\tau_g^{H}=-g\big(m_b\,x_b+m_1\,x_{C_1}+m_2\,x_{C_2}\big)
=-(m_b+m_1+m_2)\,g\,x_{C_{b\ell}}.$$

**数值核对**（$S^\*$，$L=0.3405$，$\theta=\alpha_L+\pi=4.7481$）：
$x_{C_1}=0.04292$、$x_{C_2}=0.06536$（与路线1 §3 逐位一致），$\tau_g^{H}\approx-1.38$ Nm。

**注**：以上为**腿（大腿+小腿）+ 机身**；轮质量在 $A$（$x_A$）、地面反力经轮传入。若要整机静力矩，
再加 $-m_w g\,x_A$ 与接触项。分支取 $\sigma=-1$。

## 5.6 两同轴髋电机的静态保持力矩 [推导 + 已核对]

沿用 §5.5 的坐标系（原点 $H$、$x$ 沿机身、$y$ 垂直）。两个**同轴**髋电机：
大腿电机 $\tau_1$、曲柄电机 $\tau_c$，**都在髋轴 $H$ 上**。对**机身**的反力矩为 $-(\tau_1+\tau_c)$，
对**腿**的正力矩为 $+(\tau_1+\tau_c)$。

**方程 ①（机身绕髋）**。机身质心 $x_b$（§5.5，$=0.005$），重力矩 $-m_b g x_b$；静止：
$$-m_b g x_b-(\tau_1+\tau_c)=0
\quad\Longrightarrow\quad
\boxed{\ \tau_1+\tau_c=m_b\,g\,x_b\ }$$

**方程 ②（腿+轮绕髋）**。腿两段质心 $x_{C_1},x_{C_2}$，轮在 $x_A$，地面反力 $N=Mg$ 经轮传入：
$$\boxed{\ \tau_1+\tau_c=x_A\,(N-m_w g)-g\,(m_1 x_{C_1}+m_2 x_{C_2})\ }$$

**两式一致的条件**：$\text{①}=\text{②}$ $\Longleftrightarrow$ 整车质心过接触点（$d=0$）。
数值（$S^\*$，$L=0.3405,\theta=4.7481$）：① $=0.530$ Nm，② $=0.622$ Nm，差 $0.092$ Nm，
恰等于 $S^\*$ 的 CoM 残差（$\approx0.73$ mm）——**推导自洽**，$S^\*$ 本身不是严格 $d=0$。

**结论**：
- **只靠两个同轴电机**可以抵消**机身相对腿**的重力矩（髋）：其**和** $=m_b g x_b$。
- 但 $\tau_1+\tau_c$ 是机身+腿的**内力矩**，改变不了整机角动量；**整机绝对 pitch（$\theta=0$）仍需地面/轮**。
- 从"和"到各自的**分裂** $\tau_1,\tau_c$：需腿的内部（膝/四杆）平衡，即
  $$\tau_{q_1}=\tau_1+\tau_c,\qquad \tau_{q_2}=\tau_c/r\ (r=\mathrm d\beta/\mathrm d q_{crank}),$$
  其中 $\tau_{q_2}$ 由小腿+轮绕膝的平衡（经四杆）给出 [待补]。

## 6. 控制目标：约束与优先级 [待定：次序]

| 优先 | 任务 | 表达式 | 主执行器 |
|---|---|---|---|
| 0 | **静态稳定** | $d=(C_{\text{veh}}-A)_x\to0$ | 腿（$\alpha_L$） |
| 1 | **机身水平** | $\theta\to0$ | 轮 |
| 2 | **起身驱动** | $L_0\to0.3405$（髋高 $\to0.4$） | 腿 |
| 3 | **速度阻尼** | $\dot{(\cdot)}\to0$（CoM/膝/轮） | 全 |

说明与 [待定]：

- **无世界位置任务**；轮速从动。
- 优先级次序 [待定]：物理上 $d$ 是"倒不倒"的判据，建议 $d$ **不低于** $\theta$；
  若坚持 $\theta$ 最高，需接受"姿态正确但 $d$ 偏"的风险。
- 机身着地阶段：$d=0$、$\theta=0$ 未必良定，靠**软权重自然退让**（保证全程统一、不分段）。

## 7. 配合：连接函数（协调前馈） [待定：实现]

**定义**：由腿型变换 $(L_0,\alpha_L)(t)$ 实时算出**轮需要输出的力矩/速度**，
以协调腿动对平衡模态的扰动。

**必要性**：腿型变换通过 (1) 驱动反力矩、(2) 惯性项扰动机身/平衡模态；
内力矩无法改变绝对 $\theta$，必须由轮-地实时补偿。

**数学形式**：约束系统逆动力学前馈——

$$M\ddot q+C+G=Bu+J_c^{\mathsf T}\lambda,\qquad \ddot\theta=0,\ d=0$$

解出 $u_{wheel}$；用 **MuJoCo `mj_inverse`** 可直接算，无需手推 $M,C,G$。

**形态**：

- **力矩版**（推荐）：动力学协调；轮现为力矩控制。
- **速度版**：运动学协调 $\dot\phi_w=f(\dot L_0,\dot\alpha_L)$，仅用于防滑（世界位置自由后非必要）。

**必须叠反馈**：

$$\tau_w=\underbrace{\tau_w^{\text{coord}}(L_0,\alpha_L,\dot L_0,\dot\alpha_L)}_{\text{连接函数前馈}}
+\underbrace{\text{反馈}(\theta,\dot\theta,d)}_{\text{平衡环}}$$

## 8. 从控制量到命令量的闭环 [核心]

```
                ┌────────────── 状态测量 ──────────────┐
编码器 q1,q2,q3 │  IMU θ,θ̇  │  四杆正解 → (L0,αL)      │  实时 CoM (路线1 §3)
                └──────────────────────────────────────┘
                              │
                              ▼  状态 x = [x_w, θ, L0, αL, ω_w, 速率]
        ┌──────────── 任务层（软优先级）────────────┐
        │  d*  = 0        (静态稳定)                 │
        │  θ*  = 0        (姿态)                     │
        │  L0* = 0.3405   (起身驱动 / 髋高日程)      │
        │  v*  = 0        (速度阻尼)                 │
        └───────────────────────────────────────────┘
                              │  任务误差
                              ▼  分配层：WBC-QP / 任务空间律
        ┌──────────── 任务广义力 Γ = [F_L, τα, τw] ────────────┐
        │  约束：髋 80 / 膝 40/|r| / 轮 min(47.3,15) / 腿长 / 奇异 │
        └───────────────────────────────────────────────────────┘
                              │
                              ▼  电机映射
        腿：[τ_thigh, τ_crank]ᵀ = J_legᵀ [F_L, τ_α]ᵀ + τ_g
        轮：τ_wheel = τ_w^coord(连接函数) + τ_w^fb(平衡环)
                              │
                              ▼  限幅（§3.2）
        ┌────────── 命令量：6 路电机力矩 ──────────┐
        │  仿真：data->ctrl[actuator]（torque）      │
        │  实机：电机力矩命令                        │
        └───────────────────────────────────────────┘
                              │
                              ▼  执行器 → MuJoCo / 实机
                              └────── 反馈回到状态测量 ──────┘
```

要点：

- **闭环闭合**：测量 → 任务误差 → 分配 → 电机力矩 → 执行 →（扰动/模型误差）→ 测量。
- **前馈 + 反馈**：连接函数提供前馈；平衡环/任务反馈吸收模型误差与扰动。
- **限幅**：`wheelleg_simulation.cpp` 已实现髋 80 / 膝 $40/\lvert r\rvert$ / 轮 $\min(47.3,15)$。
- **接口**：现桥接受 `hip_angle/knee_angle/wheel_velocity(+wheel_torque)`；
  本方案要任务量/力矩直通，需新增 `/wheelleg/control/{hip_torque,knee_torque}` 或等价接口 [待定]。

## 9. 与 LQR / 分层的关系

- **平衡点 LQR** = §6 任务在 $S^\*$ 邻域的线性化实现（可替换现有 ADRC 平衡环）：
  在 $S^\*$ 线性化得 $A,B$，ZOH 离散解 Riccati 得 $K$，$u=u_{eq}-K(x-x_{eq})$，$u_{eq}\approx0$。
  线性化来源：MuJoCo 数值（`mjd_transitionFD`/有限差分，先用）或解析 reduced（需补 $M,C,G$）。
- **全程统一**靠**软优先级**，不靠状态机；机身着地/离地是接触自然切换，控制器不切段。
- 大偏差起身段如需更强，可把分配层升级为 **NMPC（MuJoCo 预测）**，连接函数隐式包含其中。

## 10. 待定项

1. **初始姿态 $S_0$**：轮 + 机身同时着地的具体关节/位姿。
2. **§6 优先级次序**：$d$ 与 $\theta$ 谁最高。
3. **§7 连接函数实现**：MuJoCo `mj_inverse` 还是解析 $M,C,G$。
4. **架构**：两层（显式连接函数）还是 flat NMPC（隐式）。
5. **接口**：新增力矩直通 vs 保持参考 + 内层力矩映射。
6. **$Q/R$ / 任务权重**与终端阈值。

## 11. 模型（A1 真闭链）

- **生成**：`convert_urdf.py` → `wheelleg_serial.xml`（串行近似）→ `build_closed_chain.py` → `wheelleg.xml`（真闭链）。
- **结构**：每腿新增 `crank / lower_coupler / rocker / upper_coupler` 刚体；髋处同轴两关节；膝被动；
  D、E 销点用 `connect` 等式约束闭合（`neq=4`）。共 16 body / 20 dof / 6 actuator。
- **执行器**：`{thigh, crank, wheel} × {L,R}`（曲柄驱动膝）。
- **几何校验**：装配闭环残差 ~1e-7；扫曲柄（±1 rad）闭环残差 L~5e-7、R~6e-6；曲柄↔膝角与解析四杆一致。
- **初始位形**：启动即置 $S^\*$（髋 0.7147 / −0.7147，曲柄 0.4912 / 0.4164，被动连杆闭合），
  避免落地抽搐。**标定必须在全零装配位形做**（否则 knee offset 错）。
- **动力学标定**（`calibrate_model.py`，运行时由 `apply_calibrated_dynamics_` 施加）：
  | 项 | 值 | 来源 |
  |---|---|---|
  | 约束刚度 `constraint_timeconst/dampratio` | 0.002 / 1.0 | 扫参：≤0.002 闭环紧且稳定 |
  | 关节阻尼 `joint_damping` | 0.1 | 扫参：能量单调衰减 |
  | 髋反射转子惯量 `hip_armature` | 5e-3 | **估计**（$J_{rotor}\cdot N^2$，N=9，转子惯量未知）|
- **关节增益**：曲柄 $M\approx5.4\text{e-}3$，需独立低增益（`crank_kp=150, crank_kd=2`）；髋 `250/6`。
  离散稳定条件 $k_d\,\Delta t/M<2$。
- **已知问题**：当前轨迹控制器 + ADRC 按串行模型整定，闭链下不能站住（会倒向静态倾斜）；
  抽搐已消除，平衡属 P1。

## 12. 与现有代码的衔接

- **力矩/摩擦限幅**：`wheelleg_simulation.cpp`（髋 40 / 曲柄 40 / 轮 15）。
- **真闭链模型**：`models/wheelleg/{build_closed_chain.py, wheelleg.xml, wheelleg_serial.xml}`。
- **四杆与雅可比**：`controller/wheelleg/leg_transmission.hpp`、`leg_kinematics.hpp`（正解/逆解/$J_{leg}$）。
- **桥接口**：`/wheelleg/control/{hip_torque, crank_torque, wheel_torque}`（力矩直通）、`/wheelleg/state/{leg_length,leg_angle,com_offset,com_x,com_z}`。
- **CoM 实时公式**：`执行路线1_数学解析.md` §3。
- **参考时序**：`WheelLegTrajectoryController` 产出 `/wheelleg/reference/{hip_angle,knee_angle,leg_length,wheel_velocity,pitch}`（桥把膝参考换算成曲柄目标）。
