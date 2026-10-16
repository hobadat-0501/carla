"""Command line interface for the rewritten CARLA simulation firmware."""
from __future__ import annotations

import argparse
import json
import math
from datetime import datetime
from pathlib import Path

from .engine import run_simulation


ROOT = Path(__file__).resolve().parent.parent


def parser(profile):
    p = argparse.ArgumentParser(description="CARLA 0.9.15 real-time simulation firmware")
    p.add_argument("--mode", choices=("pad", "attach"), default="pad")
    p.add_argument("--map", dest="map_name", help="load a built-in CARLA map, for example Town03")
    p.add_argument("--spawn-index", type=int, help="vehicle spawn point index on --map")
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=2000)
    p.add_argument("--timeout", type=float, default=30 if profile in ("realtime", "scenario") else 90)
    p.add_argument("--vehicle", default="vehicle.tesla.model3")
    p.add_argument("--wheelbase", type=float)
    p.add_argument("--dt", type=float, default=.01 if profile != "basic" else .02)
    p.add_argument("--duration", type=float, default=0 if profile == "custom" else 60 if profile in ("realtime", "scenario") else 18)
    p.add_argument("--out", type=Path)
    p.add_argument("--tire-friction", type=float)
    p.add_argument("--mass-scale", type=float, default=1.0)
    p.add_argument("--rain-visual", action="store_true")
    p.add_argument("--no-model", action="store_true")
    p.add_argument("--no-telemetry", action="store_true")
    p.add_argument("--lf-fraction", type=float, default=.45)
    p.add_argument("--wait", type=float, default=60)
    p.add_argument("--role-name", default="hero")
    p.add_argument("--actor-id", type=int)
    p.add_argument("--param-refresh", type=float, default=1.0)
    p.add_argument("--throttle-ramp-per-s", type=float, default=0.5,
                   help="limit throttle command slew rate for smoother acceleration")
    p.add_argument("--print-hz", type=float, default=5.0)
    p.add_argument("--max-speed-mps", type=float, default=10 if profile == "basic" else 35 if profile == "maneuver" else 0)
    p.add_argument("--fast", action="store_true")
    p.add_argument("--keep-vehicle", action="store_true")
    p.add_argument("--no-spectator-follow", action="store_true",
                   help="leave the CARLA spectator camera where it is")
    p.add_argument("--no-hud", action="store_true", help="disable the in-world live telemetry HUD")
    if profile == "basic":
        p.add_argument("--case", choices=("doc", "ngang", "phoi_hop"), default="ngang")
        p.add_argument("--speed-mps", type=float, default=2.0)
        p.add_argument("--throttle", type=float, default=.18)
        p.add_argument("--steer-amplitude", type=float, default=.06)
        p.add_argument("--steer-hz", type=float, default=.2)
        p.add_argument("--stabilize-s", type=float, default=2.0)
        p.add_argument("--max-speed", type=float, default=10.0)
    elif profile == "maneuver":
        p.add_argument("--maneuver", choices=("jturn", "double_pulse"), required=True)
        p.add_argument("--speed-kmh", type=float, choices=(30, 50, 80), required=True)
        p.add_argument("--mu-label", type=float, choices=(.4, 1.0), required=True)
        p.add_argument("--delta-deg", type=float, default=2.0)
        p.add_argument("--direction", type=int, choices=(-1, 1), default=1)
        p.add_argument("--brake-pulse", type=float, default=0)
        p.add_argument("--maneuver-duration", type=float, default=6)
        p.add_argument("--warmup-limit-s", type=float, default=40)
        p.add_argument("--repeat-s", type=float, default=0)
        p.add_argument("--stabilize-s", type=float, default=2)
    elif profile == "scenario":
        p.add_argument("--scenario", required=True, choices=(
            "suite", "straight", "speed_step", "coastdown", "steer_ramp", "jturn",
            "double_pulse", "sine_dwell", "constant_radius", "braking"))
        p.add_argument("--speed-kmh", type=float, default=25)
        p.add_argument("--step-speed-kmh", type=float, default=35)
        p.add_argument("--delta-deg", type=float, default=2)
        p.add_argument("--direction", type=int, choices=(-1, 1), default=1)
        p.add_argument("--brake-pulse", type=float, default=1.0)
        p.add_argument("--radius-m", type=float, default=60)
        p.add_argument("--sine-hz", type=float, default=.7)
        p.add_argument("--maneuver-duration", type=float)
        p.add_argument("--warmup-limit-s", type=float, default=40)
        p.add_argument("--stabilize-s", type=float, default=2)
        p.add_argument("--repeat-s", type=float, default=0)
    elif profile == "custom":
        p.add_argument("--script", type=Path, required=True,
                       help="JSON file with map, spawn_index and timed speed/steering steps")
    else:
        p.add_argument("--maneuver", choices=("jturn", "double_pulse", "none"), default="double_pulse")
        p.add_argument("--speed-kmh", type=float, default=30)
        p.add_argument("--delta-deg", type=float, default=2)
        p.add_argument("--direction", type=int, choices=(-1, 1), default=1)
        p.add_argument("--brake-pulse", type=float, default=0.0)
        p.add_argument("--repeat-s", type=float, default=0)
        p.add_argument("--reset-s", type=float, default=1)
        p.add_argument("--warmup-limit-s", type=float, default=40)
        p.add_argument("--stabilize-s", type=float, default=2)
        p.add_argument("--plot", action="store_true")
        p.add_argument("--plot-window", type=float, default=20)
    return p


def run(profile, argv=None):
    p = parser(profile)
    args = p.parse_args(argv)
    if profile == "custom":
        try:
            script = json.loads(args.script.expanduser().read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            p.error(f"Cannot read custom script {args.script}: {exc}")
        if not isinstance(script, dict) or not isinstance(script.get("steps"), list) or not script["steps"]:
            p.error("Custom script must be a JSON object with a non-empty 'steps' array")
        for index, step in enumerate(script["steps"], start=1):
            if not isinstance(step, dict):
                p.error(f"Step {index} must be an object")
            try:
                speed = float(step["speed_kmh"])
                steer = float(step["steer_deg"])
                duration = float(step["duration_s"])
            except (KeyError, TypeError, ValueError):
                p.error(f"Step {index} requires numeric speed_kmh, steer_deg, duration_s")
            if not all(math.isfinite(value) for value in (speed, steer, duration)):
                p.error(f"Step {index} values must be finite")
            if not (0 <= speed <= 100 and abs(steer) <= 30 and duration > 0):
                p.error(f"Step {index}: speed_kmh must be 0..100, steer_deg -30..30, duration_s > 0")
            step["name"] = str(step.get("name") or f"step_{index:02d}")
            step["speed_kmh"], step["steer_deg"], step["duration_s"] = speed, steer, duration
        args.custom_steps = script["steps"]
        args.custom_script = script
        args.vehicle = str(script.get("vehicle") or args.vehicle)
        args.map_name = args.map_name or script.get("map", "Town06")
        args.spawn_index = args.spawn_index if args.spawn_index is not None else int(script.get("spawn_index", 0))
    if args.spawn_index is None:
        args.spawn_index = 0
    if profile == "scenario" and args.map_name is None:
        args.map_name = "Town03"
    if profile == "scenario" and args.maneuver_duration is None:
        args.maneuver_duration = 3.0 if args.scenario == "constant_radius" else 6.0
    args.profile = profile
    args.realtime = profile in ("realtime", "scenario", "custom") and not args.fast
    args.log_hz = args.print_hz
    args.phase_start = 0.0
    args.max_steer_deg = 30.0
    if args.out is None:
        args.out = ROOT / "ket_qua" / f"{profile}_{datetime.now():%Y%m%d_%H%M%S_%f}"
    finite_values = [args.dt, args.duration, args.timeout, args.wait, args.param_refresh,
                     args.throttle_ramp_per_s, args.print_hz,
                     args.mass_scale, args.lf_fraction]
    if args.wheelbase is not None:
        finite_values.append(args.wheelbase)
    if not all(math.isfinite(x) for x in finite_values):
        p.error("Time values must be finite")
    if not (0 < args.dt <= (.05 if profile == "basic" else .02) and args.duration >= 0 and 1 <= args.timeout <= 600
            and 1 <= args.port <= 65535):
        p.error("Invalid timestep, duration, or timeout")
    if args.wheelbase is not None and (not math.isfinite(args.wheelbase) or args.wheelbase <= 0):
        p.error("--wheelbase must be positive and finite")
    if not (0 < args.lf_fraction < 1 and args.wait >= 0 and args.param_refresh > 0
            and args.throttle_ramp_per_s > 0
            and 0 < args.print_hz <= 60):
        p.error("Invalid telemetry, wait, or logging parameter")
    if args.tire_friction is not None and not (0 < args.tire_friction <= 10):
        p.error("--tire-friction must be in (0, 10]")
    if not math.isfinite(args.mass_scale) or args.mass_scale <= 0:
        p.error("--mass-scale must be positive and finite")
    if args.mode == "attach" and profile == "basic" and args.mass_scale != 1.0:
        p.error("attach mode is read-only; mass-scale is only valid in pad mode")
    if profile == "basic" and not (args.speed_mps > 0 and 0 <= args.throttle <= .5 and
                                    abs(args.steer_amplitude) <= .15 and args.steer_hz > 0
                                    and args.stabilize_s >= 0 and args.max_speed > 0):
        p.error("Invalid basic simulation parameters")
    if profile == "basic":
        args.max_speed_mps = args.max_speed
    if profile in ("maneuver", "realtime", "scenario") and not (5 <= args.speed_kmh <= 100 and 0 < args.delta_deg <= 10):
        p.error("Invalid maneuver speed or steering angle")
    if profile in ("maneuver", "realtime", "scenario") and not (0 <= args.brake_pulse <= 1):
        p.error("--brake-pulse must be normalized in [0, 1]")
    if profile == "scenario" and not (5 <= args.step_speed_kmh <= 100 and args.radius_m > 0
                                       and args.sine_hz > 0 and args.maneuver_duration > 0
                                       and args.warmup_limit_s > 0 and args.repeat_s >= 0):
        p.error("Invalid scenario parameter")
    if profile == "scenario" and args.mode != "pad":
        p.error("scenario tests require --mode pad")
    if profile == "custom" and args.mode != "pad":
        p.error("custom scripts require --mode pad")
    if args.spawn_index < 0:
        p.error("--spawn-index must be nonnegative")
    if profile == "maneuver" and (args.maneuver_duration <= 0 or args.warmup_limit_s <= 0):
        p.error("Maneuver and warmup durations must be positive")
    if profile == "realtime" and not (.2 <= args.reset_s <= 10 and args.repeat_s >= 0
                                       and args.warmup_limit_s > 0 and args.stabilize_s >= 0):
        p.error("--reset-s must be 0.2..10 and --repeat-s nonnegative")
    if profile == "scenario":
        args.maneuver = args.scenario
        args.reset_s = 1.0
        args.plot = False
        args.plot_window = 20
        args.max_speed_mps = 0
    run_simulation(args)
    return 0


def main(profile, argv=None):
    try:
        return run(profile, argv)
    except KeyboardInterrupt:
        print("Stopped by user; written samples remain in the output folder.")
        return 130
    except Exception as exc:
        print(f"ERROR [{type(exc).__name__}]: {exc}")
        return 1
