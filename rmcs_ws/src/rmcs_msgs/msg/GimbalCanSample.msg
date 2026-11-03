# Atomic diagnostic sample from one received LK motor CAN frame.
# The monotonic timestamp is captured in the CAN receive callback and is the
# time base used to align the offline identification data.
int64 can_receive_monotonic_ns
uint8 can_channel
uint32 can_id
int32 raw_angle
float64 current_amp
# Current-derived torque estimate; this is not an independent sensor reading.
float64 torque_nm
bool online
# Monotonic time and quantized command sent by the same controller process.
int64 command_monotonic_ns
float64 command_torque_nm
