# 可调重载底盘

`adjustable_heavy.yaml` 加载四个组件：

- `AdjustableHeavy`：读取开发板、电机、IMU 和遥控器反馈，内部的 `heavy_command_` 发送电机命令。
- `AdjustableChassisController`（`adjustable_chassis/chassis_controller.cpp`）：处理遥控启停、行走和手动爬阶流程，输出底盘目标速度、功率上限及六路升降目标角度。
- `SteeringWheelController`：根据底盘速度要求计算转向和行走电机扭矩。
- `HeavyStabilizingController`：位置环、速度环串级计算六路升降电机扭矩。

本配置不启动机械臂或 MoveIt 节点。当前没有小陀螺、自动调平或自动台阶检测。

## 遥控操作

启动后先将左右拨杆都拨到 `DOWN`，之后按下表操作。遥控失联会停止输出，并要求重新双 `DOWN` 解锁。

| 操作 | 行为 |
| --- | --- |
| 左右拨杆都 `DOWN` | 停止行走及升降扭矩，完成解锁检查 |
| 左拨杆 `MIDDLE` | 普通行走，左摇杆前后/左右控制车体坐标系中的平移 |
| 左拨杆 `UP` | 进入手动爬阶模式；进入时右拨杆的位置不推进阶段 |
| 爬阶中右拨杆从其他档拨到 `UP` | 推进一个阶段，持续保持上档不会连续推进 |
| 左拨杆从 `UP` 回到 `MIDDLE` | 退出爬阶，以当时的升降反馈角度继续保持 |
| 左拨杆 `DOWN`，右拨杆其他档 | 停止输出 |

普通行走默认速度上限为 `0.5 m/s`，爬阶为 `0.15 m/s`，旋转速度目标始终为零。爬阶只允许前后移动；全伸出和最终保持阶段禁止行走。每次使能时，普通行走的升降目标从当前反馈角度开始，不自动回到零角度。

停机发送零电流，不提供机械锁止。四个转向零点、电机方向、速度与功率参数需要根据实物确认。

## 手动爬阶

流程为：`LIFT → APPROACHING → PRE_CLIMBING → SLAVE_COLLAPSE → MASTER_COLLAPSE → HOLD → IDLE`。

1. 进入 `LIFT`，六路升降伸到各自配置的伸出角。
2. 手动推进到 `APPROACHING`，用摇杆低速靠近台阶。
3. 确认位置后推进到 `PRE_CLIMBING`，前主升降回收一个台阶高度。
4. 配合摇杆移动，推进到 `SLAVE_COLLAPSE`，中间从升降回收。
5. 再推进到 `MASTER_COLLAPSE`，后主升降回收。
6. 再推进到 `HOLD`，保持 `hold_ticks` 个更新周期后结束。结束后上层继续保持最后的有效角度，不自动卸力，也不自动开始下一次爬阶。

目前没有传感器自动判断靠近或到位，操作者需要确认各阶段动作完成后再推进。主升降数组顺序是 **左前、左后、右后、右前**；前组下标为 `0、3`，后组为 `1、2`。

三组 `front_master`、`back_master`、`slave` 各自需要以下参数：

- `*_retracted_angle`、`*_extended_angle`：输出轴收回和伸出角，单位 rad。
- `*_lead`：输出轴每转动一弧度对应的升降距离，单位 m/rad，必须大于零。
- `step_height`：台阶高度，单位 m，必须大于零且不超过各组行程。

回收角度按 `台阶高度 / lead` 换算，支持伸出角大于或小于收回角的两种方向。

这些机械参数按当前要求全部保留为 `0.0`。因此默认配置可以生成普通行走指令并保持当前升降角度，但爬阶模式显示 `UNCALIBRATED`、不执行行走或升降序列。填入实测值后才会启用上述流程。

## 构建与验证

```bash
bash .script/build-rmcs --packages-up-to rmcs_core rmcs_bringup
source rmcs_ws/install/setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=adjustable_heavy
```

启动会连接真实开发板，空序列号表示不按序列号筛选。无硬件验证使用 `rmcs_core` 的测试数据源，覆盖遥控失联、四向指令、组件接口配对和手动爬阶阶段；测试里的非零机构参数仅用于模拟，不能作为实机标定值。
