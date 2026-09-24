# V5历史平地策略：12486-update候选

本包提供此前已验证部分平地移动／旋转能力的原始ONNX，便于部署接口对接和仿真复现。

- 模型：`policy.onnx`，204185字节，FP32，opset 17，无外置权重文件。
- SHA-256：`ae58b862be5547195d8c4b3e71aa9be37b147792ebc903c68f032f341d92be6d`。
- 对应checkpoint：12486 updates；SHA `e6307dd7419c052a96285e58ce6a6624b8832e6c5986aadf568287bfcfc37f81`。
- 保留原导出文件字节，ONNX的全部8个权重／偏置张量与checkpoint actor逐元素一致。
- 角色：历史部分能力候选。原始`artifact_selection.json`的`accepted_stage`为null，完整阶段未通过。

## 1. 模型结构

`float32[batch,35] obs → Linear(35,256) → ELU → Linear(256,128) → ELU → Linear(128,64) → ELU → Linear(64,6) → float32[batch,6] actions`

三个隐藏层、四个全连接层，50758个FP32参数。ONNX实际图为4个Gemm＋3个Elu。
无循环状态、历史帧堆叠、运行均值归一化或输出tanh；上一动作通过输入的六个显式槽位提供。
输出为确定性动作均值，未包含动作裁剪、PD、执行器限幅、critic或训练探索噪声。

训练critic为`81 → 256 → 128 → 64 → 1`，62209参数；部署不运行critic。

## 2. 输入与输出

机体系X前、Y左、Z上；角度rad、角速度rad/s、速度m/s，高度m。

| 输入索引 | 内容 |
|---|---|
| 0,1,2 | vx命令、vy参考命令、yaw命令；普通平移／旋转的vy为0 |
| 3 | 目标高度×5；本候选平移／旋转用0.305m，即1.525 |
| 4–6 | 机体系IMU角速度×0.5 |
| 7–9 | 机体系单位重力方向，水平时约[0,0,-1] |
| 10–13 | 四腿主动轴角度相对q0的wrap到[-π,π]偏差，策略顺序 |
| 14–15 | 两轮角度占位，始终0 |
| 16–21 | 六轴输出端角速度×0.1，策略顺序 |
| 22–27 | 上一周期裁剪后的归一化策略动作，策略顺序 |
| 28–34 | 本候选使用普通上下文[1,0,0,0,0,0,0] |

拼接后整体裁剪到[-100,100]，输入连续FP32数组。Actor不输入实测车体线速度、实测高度、接触力或被动膝角。

策略顺序P：`[L_joint1, LL_joint1, R_joint1, RR_joint1, L_joint3, R_joint3]`。
采集／控制顺序C：`[L_joint1, LL_joint1, L_joint3, R_joint1, RR_joint1, R_joint3]`。
P=C[[0,1,3,4,2,5]]；C=P[[0,1,4,2,3,5]]。

P顺序名义位置：`[0.42, -0.13742282595254576, -0.42, 0.13741557625658019, 0, 0]`。
编码器需先转换为模型输出端坐标，不能把上电位置直接清零作为q0。

输出前四维裁剪到±3，然后`q_des=q0+0.25*a`；后两维裁剪到±9，然后`wheel_speed_target=10*a` rad/s。
50Hz更新策略目标，PC以训练对应的200Hz参考计算四腿PD（Kp=60、Kd=2）和两轮速度环（Kd=0.2），再按模型电机包络限幅。
六轴最终发力矩／电流适配指令；四DM使用`control_torque`模式A。输出不是直接力矩，也不是发给DM模式B的位置命令。
硬件传动／符号／零位与CAN绑定独立标定；相关模型参数仍保留原合同中的未实测标志。

## 3. 有记录支持的能力

`evaluation.json`保存同一checkpoint的27案例固定评测，每案例4个episode，18项全部判据通过。
评测目标和阈值见`evaluation.contract.json`，不能用动作允许范围代替已验证能力。

在名义高度0.305m下：

- 站立通过，高度MAE约1.93mm，最大漂移约10.44mm。
- 平移通过的档位：前进0.5、2、3m/s；后退2、3m/s。
- 低速启停通过；弧线`vx=0.25,wz=±0.3`及`vx=0.5,wz=±0.6`通过。
- 正反`wz=±1rad/s`慢转通过；正yaw的4、2π、8、4π rad/s旋转通过。
- 原地0.29–0.32m的指定高度轨迹通过，但两个端点独立驻留没有全判据通过。

未通过：后退0.5m/s、前后1m/s的速度精度；负yaw的−4、−2π、−8、−4π rad/s高度精度；低／高位驻留部分判据。
因此这里不承诺连续整个速度域均达标，也不将正向旋转结果当作反向结果。
本包没有旋转平移、4–5m/s、高度0.21–0.35m全域运动、跳跃或复杂地形的完整能力验收。

## 4. 离线验证／推理

需要Python、NumPy、ONNX Runtime；无需Isaac Sim、CUDA或PyTorch。

```bash
python infer_example.py
```

示例校验模型／合同／机械manifest／控制先验的SHA，执行CPU推理，比较已保存的`io_fixture.json`，打印目标和模型侧力矩，不连接硬件。
本仓库环境也可运行：

```bash
/home/yukikaze/isaacsim60-venv/bin/python models/v5_flat_12486/infer_example.py
```

在此目录内执行`sha256sum -c SHA256SUMS`可核对整包。
`v5_policy_io.py`是与训练侧对照过的NumPy观测／动作／力矩数学参考。
真实循环应按20ms组装输入、推理并保存裁剪动作；每个低层周期用新q/dq重新计算力矩。

## 5. 文件含义

- `policy.onnx.json`：原始导出metadata及SHA，未改写。
- `policy.onnx.contract.json`：原始**训练阶段**合同，与导出metadata的contract SHA对应。
- `evaluation.json`／`evaluation.contract.json`：历史固定评测结果及其**评测合同**；评测合同与训练合同各自保留。
- `manifest.json`：机械资产身份、轴序和q0；`own_v40_v2.json`仅提供所复用的控制数学先验。
- `agent_config.json`：该checkpoint实际训练网络／算法配置。
- `artifact_selection.json`：原始候选角色，保留完整阶段未通过的事实。
- `provenance.json`／`verification.json`：本次交付来源、图结构、参数一致性和ORT验证记录。
- `io_fixture.json`：固定35D输入、原始6D输出、裁剪动作、位置／速度目标及模型侧力矩，供对接方逐项比对。

`own_v40_v2.json`中的旧观测布局不是本模型接口；本包的35D／6D、50Hz、V5轴序以`policy.onnx.contract.json`为准。
完整部署说明在训练仓库`docs/V54_DEPLOYMENT_INTERFACE.md`。完整仿真资产在`model/纯底盘_v5/urdf/`，不属于此轻量ONNX包。
