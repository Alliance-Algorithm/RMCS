# 全向底盘模型

以全向轮底盘的中心为原点，从原点出发，向两相邻轮构成的射线为 $x,y$ 轴，建立坐标系。

从位于 $x$ 轴正方向的轮开始，逆时针依次遍历位于 $y$ 正，$x$ 负，$y$ 负处的轮，分别记其编号为 $0,1,2,3$ 。

对于任意轮电机 $i$，设轮半径为 $r$，观测其输出轴转速为 $\omega_i$。若不考虑打滑，底盘在此处相对地面速度的可观测分量为 $v_i=-\omega_i r$（垂直于全向轮的速度分量不可观测）。

本文轮电机输出轴转速的正方向，以拇指指向与原点向外的射线同向的右手定则决定。

观测速度的平移分量：
$$v_{tx}=v_3-v_1$$
$$v_{ty}=v_0-v_2$$

于是底盘观测平移速度向量：
$$\bm{v_t}=\left[\begin{aligned}v_{tx}\\v_{ty}\end{aligned}\right]=\left[\begin{aligned}\left(\omega_1-\omega_3\right)r\\\left(\omega_2-\omega_0\right)r\end{aligned}\right]$$

观测速度的旋转分量：
$$v_{rx}=v_0+v_2$$
$$v_{ry}=v_1+v_3$$

对于未打滑的对称底盘，不难发现：
$$v_{rx}=v_{ry}$$

为减小观测误差，考虑：
$$v_r=\frac{v_{rx}+v_{ry}}{2}$$

$v_r$ 本质上是对轮差速，设 $R$ 为原点到轮的距离，则 $v_r$ 可转化为更直观的底盘观测角速度 $\omega_r$：
$$\omega_r=\frac{v_r}{R}=-\frac{\left(\omega_0+\omega_1+\omega_2+\omega_3\right)r}{2R}$$

# 简单闭环控制

假定底盘功率无限，电机能提供的扭矩无限，我们可以简单地对观测速度的平移和旋转分量进行闭环控制。

设底盘目标平移速度向量 $\bm{V_t}$，则平移速度误差为 $\bm{V_t}-\bm{v_t}$。

经过一纯 $p$ 控制环后，得到未限制的底盘目标平移控制力 $\bm{F_t}=K_p\left(\bm{V_t}-\bm{v_t}\right)$。

设底盘目标旋转速度 $\Omega_r$，则旋转速度误差为 $\Omega_r-\omega_r$。

经过一纯 $p$ 控制环后，得到未限制的底盘目标旋转控制力矩 $\tau_r=K_p\left(\Omega_r-\omega_r\right)$。

于是对 $x$ 轴上的轮，其在接地点处对底盘的目标作用力：
$$\begin{aligned}
F_0&=\frac{F_{ty}}{2}+\frac{\tau_r}{4R}\\
F_2&=-\frac{F_{ty}}{2}+\frac{\tau_r}{4R}\\
\end{aligned}$$

得出 $x$ 轴电机的目标控制力矩：
$$\begin{aligned}
\tau_0&=-F_0 r=-\frac{F_{ty}r}{2}-\frac{\tau_r r}{4R}\\
\tau_2&=-F_2 r=\frac{F_{ty}r}{2}-\frac{\tau_r r}{4R}\\
\end{aligned}$$

同理得 $y$ 轴电机的目标控制力矩：
$$\begin{aligned}
\tau_1&=\frac{F_{tx}r}{2}-\frac{\tau_r r}{4R}\\
\tau_3&=-\frac{F_{tx}r}{2}-\frac{\tau_r r}{4R}\\
\end{aligned}$$

注意到，若设：
$$\bm{\tau_t}=-\frac{\bm{F_t}r}{2}$$
$$\tau_r=-\frac{\tau_r r}{4R}$$

则各电机的目标控制力矩可简化为：
$$\begin{aligned}
\tau_0&=\tau_r+\tau_{ty}\\
\tau_1&=\tau_r-\tau_{tx}\\
\tau_2&=\tau_r-\tau_{ty}\\
\tau_3&=\tau_r+\tau_{tx}\\
\end{aligned}$$

# 增加约束

显然底盘功率并非无限，电机能提供的扭矩也并非无限。因此实际控制力矩不可能与期望相符。

## 约束条件1

对于平移控制力矩，我们要求它小于等于上文计算的控制力矩，同时必须大于等于 $0$。

设 $\tau_{t\text{max}}=|\bm{\tau_t}|$，$\theta$ 为 $\bm{\tau_t}$ 的方向角。

将问题转化为二维线性规划 $(\text{LP})$ 问题，决策变量为 $\tau_t,\tau_r$，约束条件之一为：

$$\left\{\begin{aligned}\tau_t&\leq \tau_{t\text{max}}\\\tau_t&\geq 0\end{aligned}\right.$$


可重新定义限制后的目标平移控制力矩 $\left[\begin{aligned}\tau_{tx}\\\tau_{ty}\end{aligned}\right]=\left[\begin{aligned}\tau_t\cos(\theta)\\\tau_t\sin(\theta)\end{aligned}\right]$。

## 约束条件2

对于旋转控制力矩，在不同底盘模式下应分类讨论。

对于跟随模式，由角度环计算出目标旋转速度后，速度环计算出目标旋转力矩 $\tau_r'$。

若 $\tau_r'\geq0$，则 $\tau_{r\text{min}}=0$，$\tau_{r\text{max}}=\tau_r'$。

若 $\tau_r'<0$，则 $\tau_{r\text{min}}=\tau_r'$，$\tau_{r\text{max}}=0$。

对于小陀螺模式，应由最高速度环计算出 $\tau_{r\text{max}}$，由最低速度环计算出 $\tau_{r\text{min}}$。

$\tau_{r\text{min}}\leq\tau_r\leq\tau_{r\text{max}}$ 是约束条件之一。

## 约束条件3

单个电机的控制力矩 $\tau$ 总有上限。同时为防止打滑而浪费功率，电机的输出力矩也应限制在一个较小范围。

设轮电机的最大控制力矩为 $\tau_{\text{max}}$，则有：
$$\begin{aligned}
|\tau_0|&=\left|\tau_r+\tau_t\sin(\theta)\right|<\tau_{\text{max}}\\
|\tau_1|&=\left|\tau_r-\tau_t\cos(\theta)\right|<\tau_{\text{max}}\\
|\tau_2|&=\left|\tau_r-\tau_t\sin(\theta)\right|<\tau_{\text{max}}\\
|\tau_3|&=\left|\tau_r+\tau_t\cos(\theta)\right|<\tau_{\text{max}}\\
\end{aligned}$$

若设 $\left\{\begin{aligned}x&=\tau_t\\y&=\tau_r\end{aligned}\right.$，易证该不等式的可行域是沿 $x,y$ 轴对称的菱形，其上顶点恒为 $\left(0,\sqrt{\tau_{\text{max}}}\right)$，右顶点为 $\left(0,\frac{\sqrt{\tau_{\text{max}}}}{\cos\left((\theta+\frac{\pi}{4})\pmod{\frac{\pi}{2}}-\frac{\pi}{4}\right)}\right)$。

## 约束条件4

电机功率：
$$P\left(\tau,\omega\right)=k_1\tau^2+\omega\tau+k_2\omega^2$$

于是 $x$ 轴电机总功率：
$$\begin{aligned}
P_x&=P\left(\tau_0,\omega_0\right)+P\left(\tau_2,\omega_2\right)\\
&=2k_1\tau_{ty}^2+2k_1\tau_{rx}^2+\left(\omega_0-\omega_2\right)\tau_{ty}+\left(\omega_0+\omega_2\right)\tau_{rx}+k_2\left(\omega_0^2+\omega_2^2\right)
\end{aligned}$$

$y$ 轴电机总功率：
$$\begin{aligned}
P_y&=P\left(\tau_1,\omega_1\right)+P\left(\tau_3,\omega_3\right)\\
&=2k_1\tau_{tx}^2+2k_1\tau_{ry}^2+\left(\omega_3-\omega_1\right)\tau_{tx}+\left(\omega_1+\omega_3\right)\tau_{ry}+k_2\left(\omega_1^2+\omega_3^2\right)
\end{aligned}$$

所有电机总功率（$P_c$ 表示无负载时底盘的静态功耗）：
$$\begin{aligned}
P&=P_x+P_y+P_c\\
&=2k_1\tau_t^2+4k_1\tau_r^2\\
&+\left(\left(\omega_0-\omega_2\right)\sin(\theta)+\left(\omega_3-\omega_1\right)\cos(\theta)\right)\tau_t+
  \left(\omega_0+\omega_1+\omega_2+\omega_3\right)\tau_r\\
&+k_2\left(\omega_0^2+\omega_1^2+\omega_2^2+\omega_3^2\right)+P_c\\
\end{aligned}$$

设底盘功率上限为 $P_{\text{max}}$，设 $P_{\text{remain}}\left(\tau_t,\tau_r\right)=P_{\text{max}}-P$。则 $P_{\text{remain}}\left(\tau_t,\tau_r\right)>0$ 是约束条件之一。

容易发现 $P_{\text{remain}}\left(\tau_t,\tau_r\right)=0$ 是关于 $\tau_t,\tau_r$ 的二元二次函数。其符合圆锥曲线的一般方程 $A\tau_t^2+B\tau_t\tau_r+C\tau_r^2+D\tau_t+E\tau_r+F=0$。显然其判别式 $B^2-4AC<0$，因此方程为椭圆型，不等式 $P_{\text{remain}}\left(\tau_t,\tau_r\right)>0$ 的可行域为椭圆。

由于椭圆不满足线性规划问题的定义，该问题转化为非线性规划问题 $(\text{NLP})$。

## 目标函数

设需要极大化的目标函数 $f\left(\tau_t,\tau_r\right)=a\tau_t+b\tau_r$ 。

对于小陀螺模式下的底盘，应优先保证平移控制力矩。取 $a=1$，$b=0$，$f\left(\tau_t,\tau_r\right)=\tau_t$ 。

对于跟随模式下的底盘，应优先保证旋转控制力矩。取 $a=0$，$b=1$，$f\left(\tau_t,\tau_r\right)=\tau_r$ 。

# 问题求解

问题的数学形式为：

$$\begin{aligned}
\text{maximize }&f\left(\tau_t,\tau_r\right)=a\tau_t+b\tau_r\\
\text{subject to }
&|\tau_0|=\left|\tau_r+\tau_t\sin(\theta)\right|<\tau_{\text{max}}\\
&|\tau_1|=\left|\tau_r-\tau_t\cos(\theta)\right|<\tau_{\text{max}}\\
&|\tau_2|=\left|\tau_r-\tau_t\sin(\theta)\right|<\tau_{\text{max}}\\
&|\tau_3|=\left|\tau_r+\tau_t\cos(\theta)\right|<\tau_{\text{max}}\\
&\tau_t\geq 0\\
&\tau_t\leq\tau_{t\text{max}}\\
&\tau_r\geq\tau_{r\text{min}}\\
&\tau_r\leq\tau_{r\text{max}}\\
&P_{\text{remain}}\left(\tau_t,\tau_r\right)>0\\
\end{aligned}$$

注意到除椭圆外其余不等式均为线性，又因线性规划的可行域一定是凸集，且椭圆也为凸集，凸集的交集一定为凸集，因此可行域为凸集，同时目标函数为线性，可知问题实际上是一个凸规划问题，局部最优解是它的全局最优解。

## 求线性可行域按逆时针排序的角点序列

设 $\left\{\begin{aligned}x&=\tau_t\\y&=\tau_r\end{aligned}\right.$，将问题转换到二维笛卡尔坐标系下。

### 约束条件3
从约束条件开始，易证其可行域是沿 $x,y$ 轴对称的菱形，上顶点恒为 $\left(0,\sqrt{\tau_{\text{max}}}\right)$，右顶点为 $\left(0,\frac{\sqrt{\tau_{\text{max}}}}{\cos\left((\theta+\frac{\pi}{4})\pmod{\frac{\pi}{2}}-\frac{\pi}{4}\right)}\right)$。

### 约束条件1
由 $\tau_t\geq0$，截取菱形的右半部分，得到一沿 $x$ 轴对称的三角形。从其右顶点开始，逆时针记录其顶点序列为多边形 $P=\{P_1,P_2,P_3\}$。

使用 [$\text{Sutherland–Hodgman}$](https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm) 算法，遍历剩余约束条件，对多边形进行裁剪。算法一次裁剪的伪代码如下：

```
for (int i = 0; i < input_points.count; i += 1) do
    Point current_point = input_points[i];
    Point prev_point = input_points[(i − 1) % input_points.count];

    Point Intersecting_point = ComputeIntersection(prev_point, current_point, clip_edge)

    if (current_point inside clip_edge) then
        if (prev_point not inside clip_edge) then
            output_points.add(Intersecting_point);
        end if
        output_points.add(current_point);

    else if (prev_point inside clip_edge) then
        output_points.add(Intersecting_point);
    end if
done
```

第一次裁剪：
$P$ 作为初始序列 `input_points`，条件 $\tau_t\leq\tau_{t\text{max}}$ 作为裁剪边 `clip_edge` 输入，执行算法。

### 约束条件2

第二次裁剪：
将上一次裁剪的输出 `output_points` 作为初始序列 `input_points`，条件 $\tau_{r\text{min}}\leq\tau_r$ 作为裁剪边 `clip_edge` 输入，执行算法，完成第二次裁剪。

第三次裁剪：
同理将条件 $\tau_r\leq\tau_{r\text{max}}$ 作为裁剪边输入，完成第三次裁剪。

## 用椭圆对线性可行域进行裁剪

### 仿射变换

对于可行域为椭圆的不等式 $P_{\text{remain}}\left(\tau_t,\tau_r\right)>0$，注意到其圆锥曲线的一般方程 $A\tau_t^2+B\tau_t\tau_r+C\tau_r^2+D\tau_t+E\tau_r+F=0$ 的交叉项系数 $B$ 为 $0$。
这意味着这是一个无旋转的椭圆，可以利用仿射变换变换坐标系，将椭圆变成圆以简化计算。

$$\begin{aligned}
a&=\frac{-4ACF+AE^2+CD^2}{4A^2C}\\
&=\frac{\frac{\left(\left(\omega_0-\omega_2\right)\sin(\theta)+\left(\omega_3-\omega_1\right)\cos(\theta)\right)^2}{8k_1}+\frac{\left(\omega_0+\omega_1+\omega_2+\omega_3\right)^2}{16k_1}-k_2\left(\omega_0^2+\omega_1^2+\omega_2^2+\omega_3^2\right)-P_c}{2k_1}\\
c&=\frac{-4ACF+AE^2+CD^2}{4AC^2}=\frac{a}{2}\\
\\
x_0&=\frac{\left(\omega_2-\omega_0\right)\sin(\theta)+\left(\omega_1-\omega_3\right)\cos(\theta)}{4k_1}\\
y_0&=-\frac{\omega _0+\omega _1+\omega _2+\omega _3}{8 k_1}\\
\end{aligned}$$