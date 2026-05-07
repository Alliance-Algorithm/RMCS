# Middle Omni 连续下两级台阶软悬挂方案

## 1. 目标

当前 `Down_Stairs` 使用固定姿态：

```cpp
down_stairs_trajectory
    .set_end_point(std::vector<double>{1.109164, 1.55792, 1.55792, 1.109164})
    .set_total_step(800);
```

四个值顺序仍然是：

- `lf`
- `lb`
- `rb`
- `rf`

这组角度可以继续作为下台阶的 nominal pose，但不能再把它当成“落地后刚性保持的终点”。  
真正需要的是：

- 不用 ToF
- 用 IMU 和关节/电机状态识别离地与落地
- 在落地前先给前馈
- 在落地后做方向自适应缓冲
- 连续两级台阶之间保留剩余柔性

## 2. 核心思路

软悬挂不再理解成“沿固定方向缩一下”，而是：

```text
q_cmd = q_nominal + dq_preload + dq_impact
```

其中：

- `q_nominal`：当前固定下台阶姿态
- `dq_preload`：离地后、触地前的前馈预加载
- `dq_impact`：触地后根据当前冲击方向生成的动态缓冲量

缓冲方向不固定，而是由三类模式组合：

- `heave`：竖向压缩
- `pitch`：车头前俯冲击
- `roll`：左右不同时落地冲击

也就是：

```text
dq_impact = dq_heave + dq_pitch + dq_roll
```

这里仍然建议：

- `lf/rf` 作为主缓冲轴
- `lb/rb` 作为辅助配合轴

## 3. 为什么这次不用 ToF

你这次明确要求无 ToF 方案。  
因此事件判断不再基于“距离地面多远”，而改成基于：

- IMU 姿态
- IMU 角速度
- IMU 加速度
- middle omni 轮速/扭矩
- 腿部编码器角度
- 腿部关节速度
- `control_theta_error` 及其变化率

本质上改成“动力学事件检测”，而不是“几何测距检测”。

## 4. 如何从 `bmi088.hpp` 获得 pitch

这一部分是最关键的。

## 4.1 先说明现状

`rmcs_ws/src/rmcs_core/src/hardware/device/bmi088.hpp` 当前已经提供：

- `ax() / ay() / az()`
- `gx() / gy() / gz()`
- `q0() / q1() / q2() / q3()`

并且 `update_status()` 会：

1. 读取加速度和陀螺仪原始值
2. 做单位换算
3. 执行 Mahony AHRS
4. 更新四元数 `q0 q1 q2 q3`

但是它**没有直接提供 `pitch()` 成员函数**。  
所以当前如果你要拿 pitch，有两种做法：

1. 在 `engineer.cpp` 里根据四元数现算 pitch，然后发布出去
2. 在 `bmi088.hpp` 里直接补一个 `pitch()` 接口

我更建议第 1 种，因为你现在的系统已经是 `hardware -> topic -> controller` 这条链路。

## 4.2 当前 `engineer.cpp` 已经在算 yaw，可以按同样方式算 pitch

当前 `engineer.cpp` 的 `update_imu()` 里已经有：

```cpp
*yaw_imu_velocity = bmi088_.gz();
*yaw_imu_angle    = std::atan2(
    2.0 * (bmi088_.q0() * bmi088_.q3() + bmi088_.q1() * bmi088_.q2()),
    1.0 - 2.0 * (bmi088_.q2() * bmi088_.q2() + bmi088_.q3() * bmi088_.q3()));
```

那么 pitch 直接按同一套四元数转欧拉角公式补上即可。

## 4.3 pitch 公式

如果使用常见的 ZYX 欧拉角定义，则：

```cpp
pitch = asin(2 * (q0*q2 - q3*q1))
```

为了避免浮点误差导致 `asin()` 输入越界，建议写成：

```cpp
const double sinp = std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0);
const double pitch = std::asin(sinp);
```

输出单位是：

- `rad`

## 4.4 推荐做法：在 `engineer.cpp` 里发布 `pitch_imu_angle`

### 第一步：增加输出接口

在 `ArmBoard` 里仿照 `yaw_imu_angle` 增加一个 pitch 输出，例如：

```cpp
OutputInterface<double> pitch_imu_angle;
```

### 第二步：在构造函数里注册输出

在这里：

```cpp
engineer.register_output("yaw_imu_velocity", yaw_imu_velocity, NAN);
engineer.register_output("yaw_imu_angle", yaw_imu_angle, NAN);
```

后面补上：

```cpp
engineer.register_output("pitch_imu_angle", pitch_imu_angle, NAN);
```

如果你后面还要做更完整的控制，也建议同时补：

```cpp
engineer.register_output("roll_imu_angle", roll_imu_angle, NAN);
engineer.register_output("imu_gx", imu_gx, NAN);
engineer.register_output("imu_gy", imu_gy, NAN);
engineer.register_output("imu_gz", imu_gz, NAN);
engineer.register_output("imu_ax", imu_ax, NAN);
engineer.register_output("imu_ay", imu_ay, NAN);
engineer.register_output("imu_az", imu_az, NAN);
```

### 第三步：在 `update_imu()` 里实际计算 pitch

把 `update_imu()` 改成类似这样：

```cpp
void update_imu() {
    bmi088_.update_status();

    const double q0 = bmi088_.q0();
    const double q1 = bmi088_.q1();
    const double q2 = bmi088_.q2();
    const double q3 = bmi088_.q3();

    *yaw_imu_velocity = bmi088_.gz();
    *yaw_imu_angle    = std::atan2(
        2.0 * (q0 * q3 + q1 * q2),
        1.0 - 2.0 * (q2 * q2 + q3 * q3));

    const double sinp = std::clamp(2.0 * (q0 * q2 - q3 * q1), -1.0, 1.0);
    *pitch_imu_angle  = std::asin(sinp);
}
```

### 第四步：在 `leg_controller.cpp` 中读取

在 `LegController` 构造函数中增加：

```cpp
register_input("pitch_imu_angle", pitch_imu_angle_);
```

成员变量增加：

```cpp
InputInterface<double> pitch_imu_angle_;
```

之后就可以直接这样用：

```cpp
double pitch = *pitch_imu_angle_;
```

这就是当前工程里最自然、也最稳妥的“调用 IMU pitch”的方式。

## 4.5 另一种做法：直接在 `bmi088.hpp` 里补 `pitch()` 函数

如果你想让调用更直接，也可以在 `Bmi088` 里加：

```cpp
double pitch() const {
    const double sinp = std::clamp(2.0 * (q0_ * q2_ - q3_ * q1_), -1.0, 1.0);
    return std::asin(sinp);
}
```

然后在 `engineer.cpp` 中直接写：

```cpp
*pitch_imu_angle = bmi088_.pitch();
```

但是这种方法要注意两点：

1. `bmi088.hpp` 里需要包含 `std::clamp` 所需头文件
2. 你只是让接口变短了，底层逻辑本质没变，还是先 `update_status()` 再读 pitch

所以从当前代码风格看，我仍然更建议把 pitch 的计算放在 `engineer.cpp`。

## 4.6 一个很重要的细节：先 `update_status()`，再取 pitch

顺序必须是：

```cpp
bmi088_.update_status();
// 然后再读 q0 q1 q2 q3
```

如果你没先调用 `update_status()`，那你拿到的四元数不是这一周期最新的姿态。

## 4.7 当前坐标映射会直接影响 pitch 正负号

`engineer.cpp` 里当前有：

```cpp
bmi088_.set_coordinate_mapping(
    [](double x, double y, double z) { return std::make_tuple(-x, -y, +z); });
```

这意味着：

- pitch 正负方向不是“芯片原始坐标”决定的
- 而是“映射后坐标”决定的

所以你必须上车实测确认：

- 当车头向下俯时，pitch 是增大还是减小

这个符号一定要在控制前确认好。  
否则后面你写前馈和缓冲方向时，很容易把“减冲击”写成“主动砸地”。

## 4.8 当前 IMU 装在机械臂上时的注意点

你说这个 IMU 装在机械臂上，但能反映车体的 `roll/pitch/yaw`。  
这一点在工程上必须满足下面任一条件：

1. IMU 相对车体是刚性固定的
2. 或者你已经做了“IMU 姿态 -> 车体姿态”的补偿

否则：

- 机械臂自己的运动会污染 pitch
- 腿部控制读到的就不是车体真实前俯角

本方案默认你最终喂给腿控的 `pitch_imu_angle` 是**车体 pitch**，不是机械臂局部 pitch。

## 5. 无 ToF 的落地控制结构

## 5.1 建议状态机

建议把 `Down_Stairs` 做成以下状态：

- `prepare_drop_1`
- `preload_1`
- `touchdown_1_absorb`
- `transfer_to_drop_2`
- `preload_2`
- `touchdown_2_absorb`
- `settle_exit`

含义很简单：

- `prepare_drop_1`：走到 nominal pose
- `preload_1`：第一次触地前打开缓冲窗口
- `touchdown_1_absorb`：第一次吸收冲击
- `transfer_to_drop_2`：第一次后不要完全回弹
- `preload_2`：第二次触地前再次预加载
- `touchdown_2_absorb`：第二次吸收冲击
- `settle_exit`：结束后再慢慢恢复

## 5.2 离地前前馈

没有 ToF 后，前馈不靠测距，而靠“动力学上已经开始掉下去”的迹象。

建议主要看：

- `pitch`
- `pitch` 变化率
- middle omni 轮速/扭矩变化
- 腿部关节误差变化

前馈本质上就是：

- 不要等撞到地才让腿退
- 而是在将要落地时先释放一部分刚性

## 5.3 触地后吸收

触地后缓冲方向由三类模式叠加：

- `heave`：两侧一起压缩
- `pitch`：前部主导压缩
- `roll`：左右差动压缩

建议形式：

```text
dq_impact = dq_heave + dq_pitch + dq_roll
```

其中：

- `lf/rf` 为主补偿
- `lb/rb` 为小比例配合

## 5.4 连续两级台阶时不要第一次后立刻回刚性

第一次吸收完成后，建议保留残余柔性：

```text
dq_residual = alpha * dq_peak_1
```

`alpha` 可以从 `0.2 ~ 0.4` 起调。

目的：

- 第一次之后不一直塌着
- 但也不马上回到刚性满顶
- 给第二次台阶保留缓冲余量

## 6. 你真正需要采的信号

如果只做最小可用版本，建议先采这些：

- `pitch_imu_angle`
- `yaw_imu_velocity` 或者后续补出来的 `imu_gy / imu_gz`
- `lf/lb/rb/rf` 角度
- 四个腿关节速度
- 两侧 middle omni 速度
- 两侧 middle omni 扭矩
- 四个腿关节 `control_theta_error`

第一阶段里，**最关键的是先把 `pitch_imu_angle` 打通**。

## 7. 最推荐的实际落地顺序

1. 先在 `engineer.cpp` 补 `pitch_imu_angle` 输出
2. 在 `leg_controller.cpp` 注册并读到这个 pitch
3. 上车确认 pitch 正负号
4. 先做最简单的 `preload + touchdown_absorb`
5. 再做连续两级台阶的 `dq_residual`
6. 最后再扩展到 `roll` 和更完整的冲击模式判定

## 8. 一句话结论

这次方案最重要的不是改掉 `{1.109164, 1.55792, 1.55792, 1.109164}` 这组下台阶姿态，而是：

**保留这组 nominal pose，同时通过 `bmi088_.update_status()` 更新四元数，在 `engineer.cpp` 里用 `pitch = asin(2*(q0*q2 - q3*q1))` 算出 `pitch_imu_angle` 并发布，再让腿控基于这个 pitch 做离地前前馈和落地后方向自适应缓冲。**
