"""Small SI 3-DOF bicycle model for replaying measured steering inputs."""
import json
import math
from pathlib import Path

import numpy as np


def validate_parameters(p):
    vals = [p["wheelbase_m"], p["mass_kg"], p["lateral"]["cf_over_mass"],
            p["lateral"]["cr_over_mass"], p["lateral"]["iz_over_mass"]]
    if not all(math.isfinite(float(x)) and x > 0 for x in vals):
        raise ValueError("Geometry, mass, tire stiffness, and yaw inertia must be positive and finite")
    if not 0 < p["lateral"]["lf_fraction"] < 1:
        raise ValueError("lf_fraction must be between zero and one")
    low_speed = float(p.get("low_speed_end_mps", .8))
    if not math.isfinite(low_speed) or low_speed <= .3:
        raise ValueError("low_speed_end_mps must be finite and greater than 0.3 m/s")
    if any(not math.isfinite(float(v)) or float(v) < 0 for v in p["longitudinal"].values()):
        raise ValueError("Longitudinal gains must be finite and nonnegative")


def longitudinal_acceleration(vx, throttle, brake, p):
    longitudinal = p["longitudinal"]
    speed = max(float(vx), 0.0)
    rolling = longitudinal["rolling_accel"] * math.tanh(speed / .2)
    acceleration = (longitudinal["traction_gain"] * throttle -
                    longitudinal["brake_gain"] * brake - rolling -
                    longitudinal["linear_drag"] * speed)
    return 0.0 if speed == 0 and acceleration < 0 else acceleration


def derivative(z, throttle, brake, delta, p):
    X, Y, psi, vx, vy, yaw_rate = z
    lat, lon = p["lateral"], p["longitudinal"]
    L = p["wheelbase_m"]
    lf, lr = lat["lf_fraction"] * L, (1 - lat["lf_fraction"]) * L
    speed = max(vx, .8)
    af = delta - math.atan2(vy + lf * yaw_rate, speed)
    ar = -math.atan2(vy - lr * yaw_rate, speed)
    fyf = lat["cf_over_mass"] * af
    fyr = lat["cr_over_mass"] * ar
    ax = longitudinal_acceleration(vx, throttle, brake, p)
    ay = fyf * math.cos(delta) + fyr
    yaw_acc = (lf * fyf * math.cos(delta) - lr * fyr) / lat["iz_over_mass"]
    blend = min(1.0, max(0.0, (max(vx, 0.0) - .3) / (p.get("low_speed_end_mps", .8) - .3)))
    yaw_kinematic = max(vx, 0.0) * math.tan(delta) / p["wheelbase_m"]
    lateral_rate = ay - yaw_rate * vx
    yaw_rate_change = yaw_acc
    low_lateral_rate = (lr * yaw_kinematic - vy) / .1
    low_yaw_rate_change = (yaw_kinematic - yaw_rate) / .1
    return np.array([vx * math.cos(psi) - vy * math.sin(psi),
                     vx * math.sin(psi) + vy * math.cos(psi), yaw_rate,
                     ax + yaw_rate * vy,
                     blend * lateral_rate + (1-blend) * low_lateral_rate,
                     blend * yaw_rate_change + (1-blend) * low_yaw_rate_change])


def advance(state, dt, throttle, brake, delta, p, max_step=.002):
    validate_parameters(p)
    if dt <= 0 or max_step <= 0:
        raise ValueError("dt and max_step must be positive")
    n = max(1, math.ceil(dt / max_step))
    h, z = dt / n, np.asarray(state, dtype=float).copy()
    for _ in range(n):
        f = lambda x: derivative(x, throttle, brake, delta, p)
        k1 = f(z); k2 = f(z + h * k1 / 2); k3 = f(z + h * k2 / 2); k4 = f(z + h * k3)
        z += h * (k1 + 2*k2 + 2*k3 + k4) / 6
        z[3] = max(0.0, z[3])
        if not np.all(np.isfinite(z)):
            raise FloatingPointError("Vehicle model diverged")
    return z


def load_parameters(path):
    p = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_parameters(p)
    return p
