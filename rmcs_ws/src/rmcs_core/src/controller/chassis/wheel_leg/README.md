#Wheel-leg Controller

desire state: [s ds phi d_phi theta_l d_theta_l theta_r d_theta_r theta_b d_theta_b]

## reset
启动时收腿至最短后再启动lqr计算。

## Wheel-leg Mode
balanceless: 锁死关节，仅控轮子移动
follow： yaw轴角度、角速度
spin： yaw轴角速度

### balanceless
非平衡模式，关节以位置模式固定在0点，仅控制轮毂，不进行lqr计算。

### follow

## Special Situation
rescue tip over: 关闭lqr，仅关节
jump: 跳跃
climb: 爬台阶

