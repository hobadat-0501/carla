"""Open-loop MPC forecasts and horizon-wise comparison with CARLA states."""
from __future__ import annotations

import csv
import math

import numpy as np


def _dyn_step(s, u, dt, mass, lf, lr, caf=60e3, car=60e3):
    """Euler step of the linear-tire dynamic bicycle model in matlab/models.m."""
    x, y, psi, vx, vy, yaw_rate = s
    accel, delta = u
    xs = max(vx, 1.0)
    alpha_f = math.atan2(vy + lf * yaw_rate, xs) - delta
    alpha_r = math.atan2(vy - lr * yaw_rate, xs)
    ff, fr = -caf * alpha_f, -car * alpha_r
    dx = vx * math.cos(psi) - vy * math.sin(psi)
    dy = vx * math.sin(psi) + vy * math.cos(psi)
    dvx = yaw_rate * vy + accel
    dvy = -yaw_rate * vx + 2.0 / mass * (ff * math.cos(delta) + fr)
    iz = mass * lf * lr
    dr = 2.0 / iz * (lf * ff - lr * fr)
    return s + dt * np.array([dx, dy, yaw_rate, dvx, dvy, dr])


def forecast_plan(origin_t, z0, dynamic0, controls, controller, mass, lf, lr):
    """Generate kinematic-controller and dynamic-model predictions at td steps."""
    if controls is None or len(controls) == 0:
        return []
    z = np.asarray(z0, dtype=float).copy()
    d = np.asarray(dynamic0, dtype=float).copy()
    rows = []
    for step, u in enumerate(np.asarray(controls), start=1):
        z = controller._step(z, u, controller.td)
        d = _dyn_step(d, u, controller.td, mass, lf, lr)
        rows.append({"origin_t_s": float(origin_t), "horizon_step": step,
                     "horizon_s": step * controller.td,
                     "plan_accel_mps2": float(u[0]), "plan_delta_rad": float(u[1]),
                     "kin_x_m": z[0], "kin_y_m": z[1], "kin_psi_rad": z[2],
                     "kin_speed_mps": z[3], "dyn_x_m": d[0], "dyn_y_m": d[1],
                     "dyn_psi_rad": d[2], "dyn_vx_mps": d[3], "dyn_vy_mps": d[4],
                     "dyn_speed_mps": math.hypot(d[3], d[4])})
    return rows


def write_prediction_errors(path, forecasts, actual_states):
    """Interpolate measured trajectory at forecast times and save errors per horizon."""
    fields = ["origin_t_s", "horizon_step", "horizon_s", "plan_accel_mps2", "plan_delta_rad", "actual_t_s",
              "kin_x_m", "kin_y_m", "kin_psi_rad", "kin_speed_mps",
              "kin_position_error_m", "kin_heading_error_rad", "kin_speed_error_mps",
              "dyn_x_m", "dyn_y_m", "dyn_psi_rad", "dyn_vx_mps", "dyn_vy_mps", "dyn_speed_mps",
              "dyn_position_error_m", "dyn_heading_error_rad", "dyn_speed_error_mps",
              "actual_x_m", "actual_y_m", "actual_psi_rad", "actual_vx_mps", "actual_vy_mps", "actual_speed_mps"]
    states = np.asarray(actual_states, dtype=float)
    if len(states):
        states[:, 3] = np.unwrap(states[:, 3])
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        if len(states) < 2:
            return
        times = states[:, 0]
        for row in forecasts:
            target = row["origin_t_s"] + row["horizon_s"]
            if target < times[0] or target > times[-1]:
                continue
            j = int(np.searchsorted(times, target, side="right"))
            j = min(max(j, 1), len(times)-1)
            t0, t1 = times[j-1], times[j]
            f = 0.0 if t1 == t0 else (target-t0)/(t1-t0)
            a = states[j-1] + f*(states[j]-states[j-1])
            actual_speed = math.hypot(a[4], a[5])
            def angle_error(pred): return math.atan2(math.sin(pred-a[3]), math.cos(pred-a[3]))
            kin_pos = math.hypot(row["kin_x_m"]-a[1], row["kin_y_m"]-a[2])
            dyn_pos = math.hypot(row["dyn_x_m"]-a[1], row["dyn_y_m"]-a[2])
            writer.writerow({**row, "actual_t_s": target, "actual_x_m": a[1], "actual_y_m": a[2],
                "actual_psi_rad": a[3], "actual_vx_mps": a[4], "actual_vy_mps": a[5], "actual_speed_mps": actual_speed,
                "kin_position_error_m": kin_pos, "kin_heading_error_rad": angle_error(row["kin_psi_rad"]),
                "kin_speed_error_mps": row["kin_speed_mps"]-actual_speed,
                "dyn_position_error_m": dyn_pos, "dyn_heading_error_rad": angle_error(row["dyn_psi_rad"]),
                "dyn_speed_error_mps": row["dyn_speed_mps"]-actual_speed})
