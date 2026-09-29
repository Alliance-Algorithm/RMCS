import { Input } from "./types.ts";

// Copy into Foxglove's User Scripts sidebar after opening the original MCAP.
// This transform is stateless, so seeking does not alter angle reconstruction.
export const inputs = ["/wheel_leg/identification/sample"];
export const output = "/foxglove_script/pd_view";

type Output = {
  side: string;
  phase: number;
  segment: number;
  hip_target_deg: number;
  hip_measured_deg: number;
  knee_drive_target_deg: number;
  knee_drive_measured_deg: number;
  hip_error_deg: number;
  knee_drive_error_deg: number;
  motor_difference_target_deg: number;
  motor_difference_measured_deg: number;
  hip_velocity_rad_s: number;
  knee_drive_velocity_rad_s: number;
  hip_request_nm: number;
  knee_drive_request_nm: number;
  hip_reported_torque_nm: number;
  knee_drive_reported_torque_nm: number;
};

export default function script(
  event: Input<"/wheel_leg/identification/sample">,
): Output | undefined {
  const m = event.message;
  // Error telemetry reconstructs the actual continuous motor coordinates only
  // while the controller is executing; idle/final samples zero these fields.
  if (m.phase !== 2 || m.experiment_kind !== 0) return undefined;
  const h = m.selected_side === 0 ? 0 : 2;
  const k = h + 1;
  const degrees = 180 / Math.PI;
  const rh = m.q_ref_model[h]!;
  const rk = m.q_ref_model[k]!;
  const eh = m.position_error_model[h]!;
  const ek = m.position_error_model[k]!;
  const qh = rh - eh;
  const qk = rk - ek;
  return {
    side: m.selected_side === 0 ? "left" : "right",
    phase: m.phase,
    segment: m.segment_id,
    hip_target_deg: rh * degrees,
    hip_measured_deg: qh * degrees,
    knee_drive_target_deg: rk * degrees,
    knee_drive_measured_deg: qk * degrees,
    hip_error_deg: eh * degrees,
    knee_drive_error_deg: ek * degrees,
    motor_difference_target_deg: (rk - rh) * degrees,
    motor_difference_measured_deg: (qk - qh) * degrees,
    hip_velocity_rad_s: m.dq_api[h]!,
    knee_drive_velocity_rad_s: m.dq_api[k]!,
    hip_request_nm: m.tau_cmd_api[h]!,
    knee_drive_request_nm: m.tau_cmd_api[k]!,
    hip_reported_torque_nm: m.torque_fb_api[h]!,
    knee_drive_reported_torque_nm: m.torque_fb_api[k]!,
  };
}
