# DM右侧随机未使能：实车日志与启动修复

## 日志能确定的事实

用户从双下切换为左下右中（URDF零位），提供了以下首次异常：

```text
1790453730.527650570
controller=1 reason=none
LH [status=1 age_ms=0.668276 velocity=0.010989 command=0]
LK [status=1 age_ms=0.648461 velocity=0.010989 command=0]
RH [status=0 age_ms=8.42709  velocity=0.010989 command=0]
RK [status=1 age_ms=3.40834  velocity=0.010989 command=0]
```

当时角度为 `[9.28,-65.81,-35.84,36.06]°`，控制器接受该闭链姿态。右髋反馈新鲜且明确回报失能；四台同时使能这一运动条件不成立，因此全部VEL输出被置零。其他已使能电机的速度内环维持零速度，表现为抵住当前位置，不能据此判断模式变成了保持位置。

这段片段没有使能请求起点或此前status变化，因此单凭它不能区分“始终未使能”和“短暂使能后退回0”。新日志明确记录这两种情况。

## 找到的缺口

1. 旧序列在启动50、60、70、80、90 ms附近发送五轮FC，到100 ms后停止重试；某台保持status=0时，只能一直发送零速度。
2. 原FC周期在每条DM总线上连续排入两条零VEL、两条系统帧、两条零VEL，共六帧。左侧ID1、右侧ID2；MST_ID也分别为1、2。接收到了状态反馈，不能证明之前的FC或VEL已经送达。
3. 用户确认实车 **CAN自动重发关闭**。本机 `librmcs/firmware/rmcs_board/app/src/can/can.hpp` 同样设置 `disable_auto_retransmission=true`；CAN发送使用非阻塞FIFO，没有应用层发送完成重试。M_CAN在DAR模式下，仲裁失利也可取消该次发送，不需要DM同时报告通信故障。依据[Bosch M_CAN手册3.1.7](https://www.bosch-semiconductors.com/media/ip_modules/pdf_2/m_can/mcan_users_manual_v331.pdf)。

“左侧响应参与仲裁、右侧FC丢失，再被固定启动窗口遗留”符合观察。尚无总线抓包，不能把它当作当次每一帧已经实测确认的事件。

## 本次RMCS修改

- 将固定次数序列改成逐台反馈确认：先持续零速度及清错50 ms，再使能。
- 同总线一次只给一台发系统命令，左右分开周期发送；系统帧先于本轮VEL排入发送队列。FC后持续发送零VEL。周期更新中的DM发送峰值由每总线六帧降为三帧。
- 每台必须有自己的FC之后的新反馈，不能用旧status=1完成确认。四台同时就绪至少50 ms，且该观察期末都有更新反馈，才打开运动门控。
- 启动阶段只重试尚未确认的电机，单台FC间隔至少100 ms，总时限2 s；不重复使能已确认的电机，也不对仍有故障码的电机持续发FC。
- 启动超时与运行中失去就绪均锁住零速度。运行中不自动发FC，也不因后来收到status=1而自动恢复运动；双下/取消使能请求清除状态，下一次请求重新开始。
- 双下期间也持续发送零VEL，FD按左右错开重发；状态、重试次数、稳定确认计时、故障锁存全部复位。

没有改DM控制模式、CAN ID、offset、配对选弧和RL目标，也没有改或烧录板卡固件。关闭自动重发的底层机制仍存在，因此这是RMCS端的确认与发送时序修复，不代表USB/CAN物理发送已变成可靠交付。

实现文件：

- `rmcs_ws/src/rmcs_core/src/hardware/device/dm_joint_enable_sequence.hpp`
- `rmcs_ws/src/rmcs_core/src/hardware/wheel-leg-infantry-rl.cpp`
- `rmcs_ws/src/rmcs_core/src/hardware/device/dm_motor.hpp`（提供反馈接收时间）

没有新增组件或YAML参数；已有插件连接不变。

## 如何看新日志

请求打开仅显示 `request=enable`，避免把请求误当成电机已使能。运动允许前必须出现：

```text
[joint_enable] startup confirmed: four fresh status=1 stable for 50 ms;
FC attempts LH,LK,RH,RK=...
```

失败时保留首次快照，并附加：

| 字段 | 含义 |
| --- | --- |
| `phase=startup_timeout` | 2 s内未完成启动确认 |
| `phase=running_unavailable` | 已经开放运动后失去就绪 |
| `ever_active` | 本次请求是否曾允许运动 |
| `pending_mask=4` | 右髋未确认/不可用 |
| `pending_mask=8` | 右膝未确认/不可用 |
| `pending_mask=12` | 右侧两台未确认/不可用 |
| `FC_attempts` | LH、LK、RH、RK各自发送FC次数 |

`/rmcs/service/robot_status` 同时提供这些状态。`v_des=0` 本身不能说明目标角度是否为零；本次四路速度为零的直接原因是使能门控关闭。

## 验证

开发容器内 `rmcs_core` 编译通过。三组回归测试通过：DM帧六项gtest、使能序列九项gtest、闭链几何测试。使能测试覆盖右髋/右膝/两台同时漏FC、短暂status=1、旧反馈、启动超时、运行中失能、故障码、不同时钟步长及双下复位。

[实际驱动与组件的故障注入结果](wheel_leg_dm_startup_validation.json)：使用本次日志的四个角度，分别让右髋、右膝、右侧两台的FC连续丢失到550 ms。三组均在约632 ms完成确认；确认前非零VEL为0次，运行中再次失能后自动恢复为0次。此测试注入模拟CAN反馈，未运行真实CAN总线，也不模拟物理运动。

[新使能序列的Isaac闭链回归](wheel_leg_direction_after_startup_fix.json)：四实例方向对照通过；当前联合选弧最差终点误差约0.000478 rad（0.0274°），控制器故障0次，启动门控未开启时非零VEL为0次。此前16实例报告使用旧使能序列，保留为历史验证结果。

复现故障注入（先编译测试桥接）：

```bash
/home/noir/miniconda3/envs/isaaclab/bin/python \
  rmcs_ws/src/rmcs_core/tool/validate_dm_startup.py \
  --output /tmp/wheel_leg_dm_startup_validation.json
```

代码未在实车重新使能验证。
