# 编码器位置闭环

1. 在 `dart_manager` 上配置四个参数：`launch_carriage_position_1/2/3/4`，对应第 1-4 发飞镖发射时 carriage 应该 goto 的相对零点编码器位置。

2. `dart-carriage-calibrate` 中，`CARRIAGE_CALIBRATE` 完成后追加 `CARRIAGE_GOTO` 到 `launch_carriage_position_1`。

3. `dart-fire` 中，`TRIGGER_FREE` 完成后追加 `CARRIAGE_GOTO`，再递增 `fire_count`：
   - `fire_count == 0`：goto `launch_carriage_position_2`
   - `fire_count == 1`：goto `launch_carriage_position_3`
   - `fire_count >= 2`：goto `launch_carriage_position_4`

4. 如果四个位置参数缺失或不是有限数，相关任务以 `CONFIGURATION_ERROR` 失败，不下发机构动作。

5. `CARRIAGE_GOTO` 需要 encoder ready、setpoint 有限且滑台已完成标定；否则 trigger controller 返回 `FAILED`。

6. 闭环反馈方向：上行速度为正，但 encoder 数值减小；因此位置闭环用 `motor_encoder - target_encoder` 作为误差。
