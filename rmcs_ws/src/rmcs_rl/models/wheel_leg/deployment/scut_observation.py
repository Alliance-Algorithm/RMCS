"""SCUT-style single-frame interface built only from encoders, IMU and commands.

No simulator contact, passive-joint, spring, terrain or root velocity input is
accepted here. The seven context entries are commanded modes and a command
clock, not a contact-driven phase estimate. Ordering follows V14's 28+7 layout.
"""
import torch


POLICY_FROM_CONTROL = (0, 1, 3, 4, 2, 5)
CONTROL_FROM_POLICY = (0, 1, 4, 2, 3, 5)


def build_sensor18(omega, gravity, motor_q, motor_dq, nominal_q):
    """Only the encoder/IMU slice; commands and privileged state stay current."""
    delta = motor_q - nominal_q
    delta = torch.atan2(delta.sin(), delta.cos())[:, POLICY_FROM_CONTROL]
    delta = torch.cat((delta[:, :4], torch.zeros_like(delta[:, 4:])), -1)
    return torch.cat((omega * .5, gravity, delta, motor_dq[:, POLICY_FROM_CONTROL] * .1), -1).clamp(-100., 100.)


def build_scut35(omega, gravity, commands, motor_q, motor_dq, previous_control_action,
                 nominal_q, jump_request, jump_height_command, command_elapsed,
                 lateral_command=None):
    count = len(motor_q)
    lateral = torch.zeros_like(commands[:, 0]) if lateral_command is None else lateral_command
    velocity_command = torch.stack((commands[:, 0], lateral, commands[:, 1]), -1)
    delta = motor_q - nominal_q
    delta = torch.atan2(delta.sin(), delta.cos())[:, POLICY_FROM_CONTROL]
    delta = torch.cat((delta[:, :4], torch.zeros_like(delta[:, 4:])), -1)
    context = torch.zeros(count, 7, device=motor_q.device, dtype=motor_q.dtype)
    context[:, 0] = (~jump_request).to(motor_q.dtype)
    context[:, 4] = jump_request.to(motor_q.dtype)
    context[:, 5] = jump_height_command * jump_request * 5.
    context[:, 6] = command_elapsed.clamp(0., 5.) * jump_request
    return torch.cat((velocity_command, commands[:, 2:3] * 5., omega * .5, gravity,
                      delta, motor_dq[:, POLICY_FROM_CONTROL] * .1,
                       previous_control_action[:, POLICY_FROM_CONTROL], context), -1).clamp(-100., 100.)


def build_reference36(omega, gravity, commands, motor_q, motor_dq, previous_control_action,
                      nominal_q, jump_request, jump_height_command, reference, lateral_command=None):
    frame = build_scut35(omega, gravity, commands, motor_q, motor_dq, previous_control_action,
        nominal_q, jump_request, jump_height_command, reference.elapsed, lateral_command)
    frame[:, 28] = (~jump_request & (reference.terrain_mode == 0)).to(frame.dtype)
    frame[:, 29] = reference.terrain_mode
    frame[:, 30] = reference.height_delta * 5.
    frame[:, 31] = reference.vertical_velocity
    frame[:, 34] = reference.elapsed.clamp(0., 10.)
    return torch.cat((frame, reference.acceleration[:, None]), -1).clamp(-100., 100.)


def build_manual35(omega, gravity, commands, motor_q, motor_dq, previous_control_action,
                   nominal_q, jump_request, jump_height_command, reference, lateral_command=None):
    """Activate reserved mode slots without exposing hidden reference derivatives."""
    frame = build_scut35(omega, gravity, commands, motor_q, motor_dq, previous_control_action,
        nominal_q, jump_request, jump_height_command, reference.elapsed, lateral_command)
    stair = (reference.terrain_mode != 0) & ~jump_request
    recover = stair & (reference.phase == reference.STEP_LOWER)
    frame[:, 28] = (~jump_request & ~stair).to(frame.dtype)
    frame[:, 29] = (stair & ~recover).to(frame.dtype)
    # Reserved command slot: preset landing-platform rise, never sensed terrain.
    frame[:, 30] = jump_request * getattr(reference, "landing_delta", torch.zeros_like(jump_height_command)) * 5.
    frame[:, 31] = recover.to(frame.dtype)
    frame[:, 33] = torch.where(stair, commands[:, 2] * 5., frame[:, 33])
    frame[:, 34] = reference.elapsed.clamp(0., 5.)
    return frame
