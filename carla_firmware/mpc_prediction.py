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
    """Forecast both models on their own td grids over the same 1.6 s plan."""
    if controls is None or len(controls) == 0:
        return []
    controls = np.asarray(controls, dtype=float)
    dynamic_td = .1
    kinematic_td = .2
    rows = []
    # Dynamic prediction: Euler integration at td=100 ms. Reuse/hold the
    # optimized command for its duration when the controller plan has td=200 ms.
    d = np.asarray(dynamic0, dtype=float).copy()
    d_nsub = max(1, int(round(controller.td / dynamic_td)))
    d_dt = controller.td / d_nsub
    horizon_step = 0
    for u in controls:
        for _ in range(d_nsub):
            d = _dyn_step(d, u, d_dt, mass, lf, lr)
            horizon_step += 1
            rows.append({"model": "dynamic", "origin_t_s": float(origin_t),
                         "horizon_step": horizon_step, "td_s": d_dt,
                         "horizon_s": horizon_step*d_dt, "plan_accel_mps2": float(u[0]),
                         "plan_delta_rad": float(u[1]), "pred_x_m": d[0], "pred_y_m": d[1],
                         "pred_psi_rad": d[2], "pred_vx_mps": d[3], "pred_vy_mps": d[4],
                         "pred_speed_mps": math.hypot(d[3], d[4])})

    # Kinematic prediction: MATLAB's nominal Euler model with td=200 ms.
    # If the controller optimizes 100 ms actions, average each pair so both
    # models follow the same planned acceleration/steering over 200 ms.
    k_controls_per_step = max(1, int(round(kinematic_td / controller.td)))
    z = np.asarray(z0, dtype=float).copy()
    kin_controls = [np.mean(controls[i:i+k_controls_per_step], axis=0)
                    for i in range(0, len(controls), k_controls_per_step)]
    for step, u in enumerate(kin_controls, start=1):
        z = controller._step(z, u, kinematic_td)
        rows.append({"model": "kinematic", "origin_t_s": float(origin_t),
                     "horizon_step": step, "td_s": kinematic_td,
                     "horizon_s": step*kinematic_td, "plan_accel_mps2": float(u[0]),
                     "plan_delta_rad": float(u[1]), "pred_x_m": z[0], "pred_y_m": z[1],
                     "pred_psi_rad": z[2], "pred_vx_mps": z[3], "pred_vy_mps": 0.0,
                     "pred_speed_mps": z[3]})
    return rows


def write_prediction_errors(path, forecasts, actual_states):
    """Interpolate measured trajectory at forecast times and save errors per horizon."""
    fields = ["model", "origin_t_s", "horizon_step", "td_s", "horizon_s", "plan_accel_mps2", "plan_delta_rad",
              "actual_t_s", "pred_x_m", "pred_y_m", "pred_psi_rad", "pred_vx_mps", "pred_vy_mps",
              "pred_speed_mps", "position_error_m", "heading_error_rad", "speed_error_mps",
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
            writer.writerow({**row, "actual_t_s": target, "actual_x_m": a[1], "actual_y_m": a[2],
                "actual_psi_rad": a[3], "actual_vx_mps": a[4], "actual_vy_mps": a[5], "actual_speed_mps": actual_speed,
                "position_error_m": math.hypot(row["pred_x_m"]-a[1], row["pred_y_m"]-a[2]),
                "heading_error_rad": angle_error(row["pred_psi_rad"]),
                "speed_error_mps": row["pred_speed_mps"]-actual_speed})
