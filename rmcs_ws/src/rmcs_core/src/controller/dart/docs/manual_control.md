# 手动控制

当遥控器的left-switch处于up状态时，进入手动控制模式，各个控制器不再听从上层的命令控制，直接有遥控器的摇杆输入等来控制

## 具体命令

1. 左上右中：
- left-joystic.x控制同步带速度，控制速度为belt-control-sensitive*该通道的值
- right-joystic.x方向控制trigger carriage的速度，控制速度为carriage-control-sensitive*该通道的值；
- rotary_knob_switch控制trigger，up为lock，down为free

2. 左上右上
- left-joystic.x控制填装升降速度，也是灵敏度x通道值
- right-joystic.y控制yaw电机的速度，同样是灵敏度乘通道值
- rotary_knob_switch控制limit-servo，down上升沿时触发一次FillingCommand::LIMIT_PULSE_FILL

3. 左上右下
- left-joystic.x控制4z底盘整体升降
- right-joystic.y控制yaw电机的速度