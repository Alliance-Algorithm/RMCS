# WheelBipe jump-v4 部署控制

`wheel-leg-infantry-wheelbipe-jump-v4.yaml` 对应 Isaac 训练提交
`0f5cbd3805c2dab0c0595fe43ae61ad34d6f5a29` 的 `foundation-jump-v4`。
正式合同为 `rmcs_ws/src/rmcs_bringup/config/contracts/wheelbipe_jump_v4.json`；
生成的 `wheel_leg/model/jump_v4.hpp` 包含同一提交的机械分支、气簧曲线和控制参数。
这只是部署接口与控制实现，模型的跳跃、落地、基础能力和实车表现必须另行验收。

## 配置与模型

旧 `wheel-leg-infantry-wheelbipe.yaml` 和 `wheelbipe` profile 继续使用原参数。
新 YAML 的状态估计、控制器和观测组件统一选择 `wheelbipe-jump-v4`。
策略为 50 Hz、反馈 PD 为 200 Hz、executor 为 1000 Hz，腿 PD 为 80/1.5。
新配置默认需要 `models/wheel_leg/wheelbipe_jump_v4.onnx`；仓库不附带已通过验收的新模型。
导出后应明确设置 `rl_model_path`，并记录 ONNX 与导出回执的 SHA-256。

导出使用训练输出目录里的 `receipt.json` 和对应检查点。完整学习状态检查点还需要同名
`.pt.json` 边车文件。脚本校验训练提交、模型、课程、PD、补偿、目标权限、预算和检查点身份。
预算读取回执，不再硬编码要求旧的 20000 更新；当前课程为 10000 更新。

```bash
python3 rmcs_ws/src/rmcs_core/tools/export_wheelbipe_jump.py \
  --checkpoint /path/to/saved_checkpoint.pt \
  --receipt /path/to/train/receipt.json \
  --output /path/to/wheelbipe_jump_v4.onnx
```

未完成的检查点仅能显式添加 `--allow-pilot` 导出，用于离线检查。
其元数据 `pilot=true` 会被正式 YAML 拒绝；导出成功也不代表行为通过。
新旧模型的模型 SHA、观测版本、控制合同、气簧表、PD、目标权限和补偿方式均会核验。
YAML 动作映射须保持腿 `scale=0.5` 且无额外裁剪，轮 `scale=10`、裁剪 ±90 rad/s。
控制器启动时再次核验这些映射，防止修改 YAML 后静默改变控制含义。

## 请求、目标与力矩

已使能且正常直立、推理有效、驱动就绪、未自旋，且指令满足
`|vx|≤0.5 m/s`、横移和转向为零时，Ctrl 上升沿或遥控器两拨杆中位至左中右上的
切换沿提出一次跳跃请求。忙时的新请求不排队，复位、失能、故障和驱动掉线会清除请求。
静止与前后移动使用同一协议；请求接受后保持该次前进/后退指令。
接受门槛还要求倾角约 12° 内、角速度模小于 0.8 rad/s，过期动作由原推理超时保护停止。

默认顶点参考为机体高度 0.46 m，可在合同的 0.42–0.54 m 范围设置 `jump.target_apex`。
高度指令按训练公式，从 0.305 m 用 0.35 s 下蹲至 0.25 m，再用二次曲线蹬至
0.39 m；预测腾空时保持 0.39 m，之后返回站立参考。
actor 的七维上下文为普通任务 `[1,0,0,0,0,0,0]`，跳跃请求为
`[0,0,0,0,1,apex,elapsed_seconds]`。当前合同的请求窗口为 3.7 s。
时序和补偿资格仅来自外部请求及经过时间，接触力、实际顶点和仿真奖励状态不进入控制。

每个 PD 步根据当前编码器重新计算成对目标的共同绕圈和相对角投影。
主动请求期间，左右腿各自的**相对目标角边界**两端扩展 0.2 rad；投影修正由两轴均分，
对应每轴最多多保留约 0.1 rad 的边界目标。普通运动和被动腾空不扩展。
这允许虚拟平衡目标适度越界；真实角度仍按机械分支和 0.03 rad 保护裕度限制向外力矩。
没有放宽机械关节、驱动位置范围或电机总力矩。

物理气簧保留，全局气簧抵消仍关闭。只在请求时钟的预测腾空窗口内，且相对目标误差
与 PD 都要求收腿时，按当前编码器计算抵抗气簧伸腿的补偿：误差 0.02–0.10 rad 平滑开启，
向内 PD 为 0–2 N·m 时平滑开启，模型增益 0.75，每轴额外补偿最多 16 N·m。
同腿两轴补偿反向等大；共同转腿、伸腿、PD 制动和几何查表域外不获得收腿补偿。
先将补偿加入未裁剪 PD，再统一裁剪至腿 ±40 N·m，随后执行真实相对角保护与硬件力矩帽。
轮仍采用原速度环和 ±4.5 N·m 软件帽。

落地的 0.3 s 二次曲线属于训练奖励参考，部署端没有额外脚本接管电机，收腿和缓冲动作
由 actor 输出。训练中的双轮收腿 4 cm 成功阈值与 8 cm 稠密目标同样不是部署强制轨迹。

## 诊断与离线验证

`/chassis/jump/` 提供状态、时间、参考顶点、高度、接受次数和左右补偿门控。
`/wheel_leg/rl/pd_target`、`pd_requested_effort`、`tuck_compensation` 及上下文加入 200 Hz
数据记录，轴序为左髋、左辅助、右髋、右辅助、左轮、右轮，角度与力矩使用模型输出坐标。
各电机 `control_torque` 继续使用硬件 API 坐标，腿力矩符号与模型相反。

在开发容器中按当前工作树构建，测试仅创建控制、观测和伪反馈组件，不创建硬件组件：

```bash
RMCS_PATH=/path/to/worktree .script/build-rmcs --packages-up-to rmcs_core rmcs_rl
cd rmcs_ws
colcon test --merge-install --packages-select rmcs_core --return-code-on-test-failure
```

冻结训练源与 C++ 的数值对照需要 PyTorch 和 NumPy，输入为任意包含该提交的 Isaac Git 仓库：

```bash
python3 rmcs_ws/src/rmcs_core/test/wheelbipe_jump_parity.py \
  --training-repo /path/to/isaac_wheeled_rl_train \
  --revision 0f5cbd3805c2dab0c0595fe43ae61ad34d6f5a29 \
  --test-binary rmcs_ws/build/rmcs_core/wheelbipe_jump_test
```

更新训练合同后，显式运行 `rmcs_ws/src/rmcs_core/tools/sync_wheelbipe_jump_contract.py --training-repo ... --revision ...`
重建 JSON 和头文件，并同步 YAML 与导出规则，再重新做数值对照和组件测试。
