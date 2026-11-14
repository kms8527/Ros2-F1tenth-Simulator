"""Causal simulator plant: residual longitudinal + E2E lateral dynamics."""
from pathlib import Path

import numpy as np
import torch

from .dynamic_mlp_residual import step as residual_step


def load_lateral_model(path):
    torch.set_num_threads(1)
    model = torch.jit.load(str(Path(path)), map_location="cpu").eval()
    with torch.no_grad():
        output = model(torch.zeros(1, 17, dtype=torch.float32))
    if tuple(output.shape) != (1, 2):
        raise ValueError(f"hybrid lateral model must map (N,17) to (N,2): {path}")
    return model


def step(state, steer_command, speed_command, command_history, actuator_steer,
         actuator_speed_reference, residual_weights, residual_mean,
         residual_std, lateral_model, *, vx_history=None, dt=.04,
         current_accel=None, position_speed_scale=1.0, **residual_kwargs):
    """Advance one coherent 40 ms hybrid transition.

    ``residual_step`` supplies the deployable longitudinal ``vx``/``ax``.
    The trained E2E branch supplies ``dvy/dt`` and ``dr/dt`` conditioned on
    that residual ax. Pose is integrated once from the combined next state.
    """
    current = np.asarray(state, dtype=np.float32)
    (longitudinal, history, applied_steer, speed_reference, residual_imu,
     next_vx_history) = residual_step(
        current, steer_command, speed_command, command_history,
        actuator_steer, actuator_speed_reference, residual_weights,
        residual_mean, residual_std, vx_history=vx_history, dt=dt,
        current_accel=current_accel, position_speed_scale=position_speed_scale,
        **residual_kwargs)

    feature = np.concatenate((
        current[[3, 7, 5]],
        np.asarray((steer_command, speed_command), dtype=np.float32),
        np.asarray(history, dtype=np.float32).reshape(10),
        np.asarray((residual_imu[1], applied_steer), dtype=np.float32),
    )).astype(np.float32)
    with torch.no_grad():
        lateral = lateral_model(torch.from_numpy(feature[None])).numpy()[0]

    dt = np.float32(dt)
    next_vx = np.float32(longitudinal[3])
    next_vy = np.float32(current[7] + dt*lateral[0])
    next_yaw_rate = np.float32(current[5] + dt*lateral[1])
    next_beta = np.float32(np.arctan2(next_vy, next_vx))
    yaw = np.float32(current[4])

    output = current.copy()
    output[0] = current[0] + position_speed_scale * (
        next_vx*np.cos(yaw) - next_vy*np.sin(yaw)) * dt
    output[1] = current[1] + position_speed_scale * (
        next_vx*np.sin(yaw) + next_vy*np.cos(yaw)) * dt
    output[2] = np.float32(steer_command)
    output[3] = next_vx
    output[4] = np.arctan2(np.sin(yaw + next_yaw_rate*dt),
                           np.cos(yaw + next_yaw_rate*dt))
    output[5] = next_yaw_rate
    output[6] = next_beta
    output[7] = next_vy

    # dvy/dt = ay - vx*r in body-FLU coordinates.
    ay = np.float32(lateral[0] + current[3]*current[5])
    hybrid_imu = np.asarray((next_yaw_rate, residual_imu[1], ay), np.float32)
    return (output, history, applied_steer, speed_reference, hybrid_imu,
            next_vx_history)
