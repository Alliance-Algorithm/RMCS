# One executor iteration; arrays in API order [left_hip, left_knee, right_hip,
# right_knee, left_wheel, right_wheel]. Model vectors use [LH, LK, RH, RK].
# The DM configuration parameters use a DIFFERENT order: [LH, RH, LK, RK].
# All *_steady_ns fields use the host's steady clock epoch, not ROS time.
builtin_interfaces/Time stamp
uint64 control_steady_ns
uint64 tick
# 0=left, 1=right: which mounted hip/aux pair is the experiment's subject.
# This labels the recording; motor authority belongs to the controller graph.
uint8 selected_side
# 0=paired leg drive; 1=wheel drive. Compare like-for-like runs only.
uint8 experiment_kind
uint32 repetition_id
# 0=unconfirmed (e.g. enable/system frames), 1=selected hip+knee MIT torque
# frames and fresh DM status 0 for both inactive drives, 2=opposite pair
# commanded or motor-internal PD; 3=both wheels current-commanded while all
# four DM drives report disabled. Status 0 is a drive report, not an
# independently measured shaft torque or CAN transmission acknowledgement.
uint8 actuation_scope

# Last received CAN frame for each axis (sequence starts at 1); 0 means no frame.
# Feedback values are the latest values decoded by the hardware status component.
# Receive callbacks are asynchronous; sequence/time and decoded values can
# straddle a frame boundary if a callback runs during the status update.
uint64[6] feedback_steady_ns
uint64[6] feedback_sequence
# Raw eight bytes per axis, flattened in API axis order. A repeated sequence
# repeats cached bytes: it is NOT another device sample. No CAN device clock is
# available in this board SDK; feedback_steady_ns is host callback time.
uint8[48] feedback_frame_bytes
float64[6] q_api
float64[6] dq_api
float64[6] torque_fb_api
# Wheel axes: raw M3508 reported motor current in A; DM MIT frames do not
# expose current, so their entries are NaN. Never infer DM current from torque.
float64[6] feedback_current_a
# 0=unavailable, 1=DM drive-reported estimated torque,
# 2=M3508 raw current converted to output-shaft torque using configured 15.8 ratio.
uint8[6] feedback_torque_source
float64[6] max_torque_api
# DM temperature is reported by the driver. Wheel temperature is unavailable
# unless a future hardware output binds it; NaN means not measured.
float64[6] temperature_c
float64[4] dm_rotor_temperature_c
float64 supply_voltage_v
# -1 means no DM frame yet. DM order [LH, LK, RH, RK].
int32[4] dm_fault
int32[4] dm_status

# Optional controller-provided model-coordinate telemetry; NaN/-1 when absent.
# q_ref_model is the outer position reference, dq_ref_model is its trajectory
# derivative, and velocity_target_model is the position-loop OUTPUT after its
# speed limit. They are not interchangeable in a cascaded position/velocity loop.
float64[4] q_ref_model
float64[4] dq_ref_model
float64[4] velocity_target_model
float64[4] position_error_model
float64[4] speed_error_model
float64[4] tau_preclip_model
float64[4] torque_integral_model
# Explicit gravity/spring feedforward term, zero until fitted from actual rig data.
float64[4] tau_gravity_ff_model
# Optional API-frame control interfaces. Motor-internal PD may also provide
# control_angle; tx_kind=3 then identifies the different control mode.
float64[4] q_ref_api
float64[4] velocity_target_api
# Wheel output-shaft speed references, rad/s; NaN when not controlled.
float64[2] wheel_velocity_target_api
# PC requested torque before software limiting, in API reference/units. NaN
# when an axis has no active controller; model-coordinate preclip is above.
float64[6] tau_preclip_api
bool[6] torque_limited
# Controller output in API sign/units; NaN when no control_torque interface.
float64[6] tau_cmd_api
# Quantized effort encoded in the torque/current frame submitted to the board,
# in API units. The board SDK does not acknowledge an actual CAN transmission.
# NaN for no frame, DM system frames, and MIT position-PD frames (the latter
# produce additional motor-internal effort which cannot be measured here).
float64[6] tau_frame_api
# Host-side submission time (after can_transmit), NOT verified CAN transmission
# or motor execution time. The SDK can silently drop on buffer allocation failure.
# 0 when no frame submission was attempted in this iteration.
uint64[6] tx_queued_steady_ns
# Submitted frame kind: 0=none; 1=controlled torque/current; 2=system;
# 3=MIT position-PD; 4=disabled DM neutral MIT feedback poll (never excitation).
uint8[6] tx_kind
# Exact eight submitted CAN bytes per axis (the same 0x200 frame is repeated
# for each wheel); CAN bus 0/1/2 and CAN ID. These are host-side submissions,
# not verified driver reception. Bus=255 and ID=0 mean no submitted frame.
uint8[48] tx_frame_bytes
uint8[6] tx_can_bus
uint32[6] tx_can_id

int32 phase
# 0=none; 40..45=missing axis timestamp; 50..55=future axis timestamp;
# 60..65=axis feedback timeout; 70..75=nonfinite axis state;
# 80=aggregate stale despite individual stamps; 81=IMU stale;
# other codes identify controller/trajectory stop conditions.
int32 failure_reason
int32 segment_id
# 0=training excitation, 1=independent validation, 2=not running.
uint8 segment_role
# 0=dwell, 1=slow move, 2=two-tone, 3=bounded step,
# 4=rounded triangle, 5=two-tone validation, 6=linear chirp, 7=full-turn grid,
# 8=paired multisine (check segment_role for its train/validation assignment),
# 9=50 Hz held small edge (derivative undefined at the update).
# Chirp validation is labeled by segment_role=1; wheel stages may use 0/1/5.
uint8 segment_waveform
# Wheel stage only: 0=inactive, 1=velocity loop, 2=enabled zero-current coast.
# M3508 has no DM-style 0xFD command: mode 2 is NOT an ESC disable.
uint8 wheel_mode
bool feedback_fresh
# Actual hardware enable gate after remote/request checks; not DM acknowledgement.
bool enable_requested
# Cumulative recorder queue overflow/publish failures; gaps also visible in tick.
uint64 dropped_samples

# IMU orientation (xyzw) and body angular rate [rad/s]. NaN until first valid
# IMU sample; imu_last_ns=0 until the board produces a valid gyro update.
float64[4] imu_quaternion_xyzw
float64[3] imu_gyro
uint64 imu_last_ns
float64[3] imu_acceleration_mps2
uint64 imu_acceleration_steady_ns
# Raw board quarter-microsecond ticks [accelerometer, gyroscope], wrap at 2^32.
uint32[2] imu_board_timestamp_quarter_us

# Version 1: calibrated V6 single-side recording. Version 0: legacy/wheel run.
# Geometry-derived beta is an FK estimate, not an independent knee sensor.
# The exact mapping, calibration provenance and run identity live in profile.yaml.
uint32 recording_protocol_version
float64[4] q_unwrapped_model
float64 inner_knee_fk_deg
float64 inner_knee_requested_deg
float64 inner_knee_reference_deg
float64 thigh_orientation_fk_deg
# Metadata belongs to the currently held 50 Hz reference. Tick names its most
# recent executor update; segment time is actual local elapsed time at that update.
uint64 reference_update_tick
float64 segment_elapsed_s
float64 reference_center_delta_rad
# -1=not applicable, 0=awaiting arrival, 1=qualified group,
# 3=transition rebased after skipping an uncovered group (actual refs recorded).
# A latched skipped_arrival_segment identifies an uncovered group after timeout.
int32 configuration_admission
int32 skipped_arrival_segment
int32 jump_cycle_id
# 0=none, 1=compress, 2=loaded, 3=extend, 4=tuck, 5=reach,
# 6=airborne buffer shape, 7=settle, 8=gap. No label implies measured contact.
int32 jump_phase

# Latched last qualified hold, including static holds immediately followed by a new group.
int32 qualified_arrival_segment
