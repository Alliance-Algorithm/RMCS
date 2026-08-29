# LK MG4010Ei10 单电机测试（dm_motor_test）运行手册

测试目标：一块 `rmcs_board_lite` + can0 上 **LK MG4010Ei10（ID=1，帧 ID 0x141）** 电机，
由 **DR16 遥控**控制；验证电机可用性并核对参数（扭矩常数 Kt、减速比、编码器零点等）。

## 1. 硬件连接

- `rmcs_board_lite` 板 USB 连 PC（`board_serial` 留空 = 匹配任意板子）
- 电机挂在板子 **can0** 口，**LK 电机 ID = 1**（用 LK 上位机配置；帧 ID = 0x140+1 = 0x141，命令/反馈同 ID）
- DR16 接收机接板子 **DBus** 口
- 电机供电（LK 4010 系列按标称电压）；上电后用 LK 上位机确认电机 ID 与编码器零点

## 2. 构建

### 2.0 USB 权限（首次必做，在**主机**终端执行，不要在 docker 容器里）

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="a11c", MODE="0666"' | sudo tee /etc/udev/rules.d/95-rmcs-slave.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

拔插一次板子。容器里若仍报 `Access denied (-3 / ERROR_ACCESS)`，临时方案：

```bash
sudo chmod 666 /dev/bus/usb/$(lsusb | awk '/a11c:a801/{print $2"/"$4}' | tr -d ':')
```

（WSL2 用户需先用 usbipd 把设备 attach 到 WSL，见 `docs/zh-cn/wsl2_develop_guide.md`。）

### 2.1 编译

```bash
cd /workspaces/RMCS/rmcs_ws
colcon build --packages-select rmcs_core
```

> 若报 `mavlink/v2.0/common/mavlink.h` 缺失：`sudo apt-get install -y ros-jazzy-mavlink`
> 装完需 `colcon build --packages-select rmcs_core --cmake-clean-cache` 重新配置（已装但配置缓存旧的场景）。

## 3. 启动

```bash
source /workspaces/RMCS/rmcs_ws/install/setup.bash
ros2 launch rmcs_bringup rmcs.launch.py robot:=dm_motor_test
# 或：launch-rmcs robot:=dm_motor_test
```

## 4. DR16 操作（SWA=左开关，SWD=右开关）

| 开关状态 | 行为 |
|---|---|
| 双下（左 DWN + 右 DWN） | 电机**失能**（0xA1 电流 0） |
| 其余任意组合 | 电机**启动/使能**（0x88） |
| 双中（左 MID + 右 MID） | **位置速度**：目标角度=左摇杆X×position_scale，限速=\|左摇杆Y\|×velocity_scale（0xA3） |
| 左上右中（左 UP + 右 MID） | **速度**：v_des=右摇杆Y×velocity_scale，扭矩限幅=velocity_torque_limit（0xA2） |
| 左中右上（左 MID + 右 UP） | **扭矩(MIT)**：t_ff=右摇杆Y×torque_scale（0xA1） |
| 遥控丢失 >500ms | 自动失能（安全保护） |

**安全顺序**：遥控先上电并保持双下 → 启动程序（电机保持 DISABLED）→ 再拨模式开关。

## 5. 日志解读（每秒一行，`log_rate` 可调）

```
[dm motor test] dr16=Y swL=MID swR=UP mode=MIT en=Y
    sticks: L=(+0.00,+0.00) R=(+0.00,+0.30) knob=+0.00
    cmd:    angle=+0.000 rad v_des=+0.000 rad/s t_ff=+1.350 Nm vel_limit=+0.000 rad/s tq_limit=+0.000 Nm
    fb:     angle=+0.123 rad raw=1234 vel=+0.450 rad/s torque=+1.290 Nm temp=32.0 C max_torque=4.50 Nm rx=Y
    frame:  [A1 00 00 00 0C 00 00 00]
```

- `rx=Y/N`：是否收到电机反馈帧（N = 帧 ID 不对 / 未启动 / 接线问题）
- `angle/raw`：输出轴角度(rad)与编码器原始值；`temp` 电机温度；`max_torque` 由驱动按型号给出（4010i10 = 4.5Nm）
- `frame`：实际下发的 8 字节 LK 命令帧（0xA1 扭矩 / 0xA2 速度 / 0xA3 角度 / 0x88 启动）
- 模式切换、使能/失能切换、遥控丢失都会即时打一行日志

状态服务（一次看全）：

```bash
ros2 service call /rmcs/service/dm_motor_test_status std_srvs/srv/Trigger "{}"
```

## 6. 参数核对清单（测试项目核心）

- [ ] **扭矩常数 Kt**：驱动内 `Type::kMG4010Ei10` 用 **Kt=0.07 Nm/A（电机侧）**、减速比 **10:1**，
      输出侧 ≈ 0.7 Nm/A；反馈扭矩 = Kt × 电流 × 10。实测核对：扭矩(MIT)模式下发已知 t_ff，
      对比 LK 上位机电流 → τ/I 是否 ≈ 0.7 Nm/A（输出侧）。
- [ ] **峰值扭矩**：4010i10 `max_torque = 4.5 Nm`（驱动参考值，来自厂商文档）
- [ ] **编码器零点**：`encoder_zero_point` 须与 LK 上位机标定一致（0 位时 angle≈0）
- [ ] **电机 ID / 帧 ID**：电机内 ID=1 → 帧 ID 0x141；`motor_id` 参数须与之一致
- [ ] 反接方向：若 `torque` 与 `t_ff` 符号相反，设 `motor_reversed: true`
- [ ] `multi_turn_angle`：位置模式建议 true（多圈绝对角度）

## 7. 常见问题

| 现象 | 排查 |
|---|---|
| `rx=N` | 帧 ID 与电机 ID 不符；接线；电机未启动(0x88)前无反馈 |
| `en=N` | 遥控双下或未连接；检查 DBus 接线 |
| 电机不转 | 先确认 `en=Y` 且 t_ff/v_des≠0；LK 需先发 0x88 启动才会执行动作 |
| 上电即报错 | 固件版本校验失败时，把 `RmcsBoardLite` 构造改成 wheel-leg 的 `AdvancedOptions{.dangerously_skip_version_checks=true}` |
| 角度乱跳 | 编码器零点不对；多圈/单圈模式与上位机设置不一致 |
