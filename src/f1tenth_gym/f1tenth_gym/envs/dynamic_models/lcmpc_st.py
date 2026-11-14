"""Single-track plant identical to the MDN-LCMPC prediction model.

This reproduces ``dynamics::DynamicModel(..., ORIGINALMODEL, ...)`` and
``dynamics::KinematicModel`` from MDN_SLCMPC_module/control/MPC/src/dynamics.cpp:

    state  x = [px, py, delta, v, yaw, omega, beta]   (same layout as gym ST)
    input  u = [steering angle (rad), longitudinal acceleration command (m/s^2)]

Differences from the stock gym ``st`` model, all chosen to match the MPC:
  * steering is the commanded angle (no steering-velocity PID / delay buffer),
  * longitudinal dynamics are ``v_dot = a * (1 - Cm0 * v)``,
  * below ``v_thresh`` the MPC kinematic model is used (beta = 0).
The ODE is integrated with RK4 at the simulator time step.
"""

import numpy as np
from numba import njit

G = 9.81


@njit(cache=True)
def _deriv(x, steer, accel, mu, cs_f, cs_r, lf, lr, h, m, iz, cm0, v_thresh):
    yaw = x[4]
    v = x[3]
    omega = x[5]
    beta = x[6]
    f = np.zeros(7)
    if v > v_thresh:
        rear_val = G * lr - accel * h
        front_val = G * lf + accel * h
        vv = v + 1e-8
        f[0] = v * np.cos(yaw + beta)
        f[1] = v * np.sin(yaw + beta)
        f[3] = accel * (1.0 - cm0 * v)
        f[4] = omega
        f[5] = (mu * m / (iz * (lf + lr))) * (
            lf * cs_f * steer * rear_val
            + beta * (lr * cs_r * front_val - lf * cs_f * rear_val)
            - (omega / vv) * (lf * lf * cs_f * rear_val + lr * lr * cs_r * front_val))
        f[6] = (mu / (vv * (lr + lf))) * (
            cs_f * steer * rear_val
            - beta * (cs_r * front_val + cs_f * rear_val)
            + (omega / vv) * (cs_r * lr * front_val - cs_f * lf * rear_val)) - omega
    else:
        f[0] = v * np.cos(yaw)
        f[1] = v * np.sin(yaw)
        f[3] = accel
        f[4] = v * np.tan(steer) / (lf + lr)
    return f


@njit(cache=True)
def step(state, steer_cmd, accel_cmd, dt, mu, cs_f, cs_r, lf, lr, h, m, iz,
         cm0, v_thresh, s_max, a_max, d_max, v_max):
    steer = min(max(steer_cmd, -s_max), s_max)
    accel = min(max(accel_cmd, -d_max), a_max)
    x = state.copy()
    x[2] = steer
    k1 = _deriv(x, steer, accel, mu, cs_f, cs_r, lf, lr, h, m, iz, cm0, v_thresh)
    k2 = _deriv(x + 0.5 * dt * k1, steer, accel, mu, cs_f, cs_r, lf, lr, h, m, iz, cm0, v_thresh)
    k3 = _deriv(x + 0.5 * dt * k2, steer, accel, mu, cs_f, cs_r, lf, lr, h, m, iz, cm0, v_thresh)
    k4 = _deriv(x + dt * k3, steer, accel, mu, cs_f, cs_r, lf, lr, h, m, iz, cm0, v_thresh)
    x = x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x[2] = steer
    x[3] = min(max(x[3], 0.0), v_max)
    if x[3] <= v_thresh:
        # MPC NextStep(): kinematic regime has no slip; report kinematic yaw rate.
        x[5] = x[3] * np.tan(steer) / (lf + lr)
        x[6] = 0.0
    x[4] = np.arctan2(np.sin(x[4]), np.cos(x[4]))
    return x
