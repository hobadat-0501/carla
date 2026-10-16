"""New experiment runner for the CARLA simulation firmware."""
from __future__ import annotations

import csv
import math
import queue
import time
import traceback
from pathlib import Path

import numpy as np
from .api import CarlaFirmware, WHEEL_NAMES, WHEEL_TELEMETRY_FIELDS
from .runtime import (actor_state, cleanup, configure_world, physics_report,
                           tick_checked, validate_carla, write_json)
from .model import advance as advance_model

SCENARIO_SUITE = (
    ("straight", 3.0), ("speed_step", 5.0), ("coastdown", 4.0),
    ("recovery", 5.0), ("steer_ramp", 3.0), ("jturn", 4.0),
    ("double_pulse", 4.0), ("sine_dwell", 4.0),
    ("constant_radius", 3.0), ("braking", 5.0),
)


def _body(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c + y * s, -x * s + y * c


def _road_xodr():
    return '''<?xml version="1.0" standalone="yes"?>
<OpenDRIVE><header revMajor="1" revMinor="4" name="firmware_pad" version="1.00" date="2026-10-16" north="200" south="-200" east="1200" west="-1200"/>
<road name="firmware_pad" length="2400" id="1" junction="-1"><planView><geometry s="0" x="-1200" y="0" hdg="0" length="2400"><line/></geometry></planView>
<elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>
<lanes><laneSection s="0"><left><lane id="1" type="driving" level="false"><link/><width sOffset="0" a="50" b="0" c="0" d="0"/></lane></left><center><lane id="0" type="none" level="false"><roadMark sOffset="0" type="none" weight="standard" color="standard" width="0.12"/></lane></center><right><lane id="-1" type="driving" level="false"><link/><width sOffset="0" a="50" b="0" c="0" d="0"/></lane></right></laneSection></lanes></road></OpenDRIVE>'''


def _controls(args, row, elapsed, phase):
    if args.profile == "basic":
        if elapsed < args.stabilize_s:
            return 0.0, 1.0, 0.0, "stabilize"
        t = elapsed - args.stabilize_s
        if args.case == "doc":
            steer = 0.0
        elif args.case == "ngang":
            steer = args.steer_amplitude * math.sin(2 * math.pi * args.steer_hz * t)
        else:
            ramp = min(1.0, max(0.0, (t - 5.0) / .5))
            step = ramp if t < 11 else 0.0
            steer = step + args.steer_amplitude * math.sin(2 * math.pi * args.steer_hz * t)
        speed_error = args.speed_mps - row["vx_mps"]
        throttle = min(args.throttle, max(0.0, .2 * speed_error))
        return throttle, 0.0, max(-.15, min(.15, steer)), "drive"

    if args.profile == "custom":
        step = args.active_step
        speed_error = step["speed_kmh"] / 3.6 - row["vx_mps"]
        throttle = max(0.0, min(.65, 2.5 * speed_error))
        brake = max(0.0, min(.4, -.25 * speed_error))
        steer = max(-1.0, min(1.0, step["steer_deg"] / max(args.max_steer_deg, .1)))
        return throttle, brake, -steer, step["name"]

    if elapsed < 2.0:
        return 0.0, 1.0, 0.0, "stabilize"
    scenario = getattr(args, "scenario", None) if args.profile == "scenario" else None
    case = getattr(args, "suite_segment", "straight") if scenario == "suite" else scenario
    target_kmh = args.speed_kmh
    if case == "speed_step" and getattr(args, "suite_segment_t", elapsed - args.phase_start) >= .5:
        target_kmh = args.step_speed_kmh
    target = target_kmh / 3.6
    error = target - row["vx_mps"]
    # The previous low proportional gain left a steady speed error of ~4 km/h
    # against CARLA's rolling/drive resistance, preventing maneuver warmup.
    throttle = max(0.0, min(.65, 2.5 * error))
    brake = max(0.0, min(.4, -.25 * error))
    if phase == "warmup":
        return throttle, brake, 0.0, "warmup"
    if phase == "cruise":
        return throttle, brake, 0.0, "cruise"
    t = getattr(args, "suite_segment_t", elapsed - args.phase_start) if scenario == "suite" else elapsed - args.phase_start
    angle = math.radians(args.delta_deg) * args.direction
    steer_angle = 0.0
    if case == "coastdown":
        return 0.0, 0.0, 0.0, "coastdown"
    if case == "braking":
        if t >= .5:
            return 0.0, args.brake_pulse, 0.0, "braking"
        return throttle, 0.0, 0.0, "brake_approach"
    if case == "constant_radius":
        angle = math.atan2(args.wheelbase_m, args.radius_m) * args.direction
        steer_angle = angle
    elif case == "steer_ramp":
        ramp = min(1.0, max(0.0, t / .75))
        steer_angle = angle * ramp if t < 2.75 else 0.0
    elif case == "sine_dwell":
        half_wave = .5 / args.sine_hz
        start = .5
        if start <= t < start + half_wave:
            steer_angle = angle * math.sin(math.pi * (t - start) / half_wave)
        elif start + half_wave <= t < start + half_wave + .5:
            steer_angle = angle
        elif start + half_wave + .5 <= t < start + 2 * half_wave + .5:
            steer_angle = -angle * math.sin(math.pi * (t - (start + half_wave + .5)) / half_wave)
        elif start + 2 * half_wave + .5 <= t < start + 2 * half_wave + 1.0:
            steer_angle = -angle
    elif case == "jturn" or (scenario is None and args.maneuver == "jturn"):
        if 0.5 <= t < 3.0:
            steer_angle = angle
    elif case == "double_pulse" or (scenario is None and args.maneuver == "double_pulse"):
        if .5 <= t < 1.5:
            steer_angle = angle
        elif 2.0 <= t < 3.0:
            steer_angle = -angle
    norm = max(-1.0, min(1.0, steer_angle / max(math.radians(args.max_steer_deg), .01)))
    if args.brake_pulse and 2.0 <= t < 2.5:
        brake = args.brake_pulse
    return throttle, brake, -norm, "maneuver"


class _OnlineBicycle:
    """Low-cost online axle-stiffness fit and one-second shadow rollout."""
    def __init__(self, wheelbase, mass, lf_fraction, extent):
        self.L, self.mass, self.fraction = wheelbase, mass, lf_fraction
        self.lf, self.lr = wheelbase * lf_fraction, wheelbase * (1 - lf_fraction)
        self.iz_m = ((2 * extent[0])**2 + (2 * extent[1])**2) / 12.0
        self.samples, self.prev, self.coeff = [], None, None
        self.state, self.age = None, 0.0
        self.errors = []

    def update(self, row, dt, reset_s):
        if self.prev is not None and row["vx_mps"] > 3:
            af = row["delta_equiv_rad"] - math.atan2(row["vy_mps"] + self.lf * row["r_radps"], row["vx_mps"])
            ar = -math.atan2(row["vy_mps"] - self.lr * row["r_radps"], row["vx_mps"])
            if max(abs(af), abs(ar)) > .003 and abs(row["ay_mps2"]) < 12:
                self.samples.append(([af, ar], row["ay_mps2"]))
                if len(self.samples) > 5000:
                    self.samples.pop(0)
            if len(self.samples) >= 60 and len(self.samples) % 20 == 0:
                A = np.asarray([x[0] for x in self.samples], dtype=float)
                b = np.asarray([x[1] for x in self.samples], dtype=float)
                estimate, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
                if rank == 2 and np.all(np.isfinite(estimate)) and np.all(estimate > 0):
                    self.coeff = estimate
        self.prev = row
        if self.coeff is None or row["vx_mps"] < .8:
            self.state = None
            return self._empty()
        reset = self.state is None or self.age >= reset_s
        if reset:
            self.state = np.array([row["X_m"], row["Y_m"], row["psi_rad"], row["vx_mps"],
                                   row["vy_mps"], row["r_radps"]], dtype=float)
            self.age = 0.0
        else:
            params = {"wheelbase_m": self.L, "mass_kg": self.mass,
                      "low_speed_end_mps": .8,
                      "lateral": {"cf_over_mass": float(self.coeff[0]),
                                  "cr_over_mass": float(self.coeff[1]),
                                  "lf_fraction": self.fraction, "iz_over_mass": self.iz_m},
                      "longitudinal": {"traction_gain": max(row["ax_mps2"], 0),
                                       "brake_gain": max(-row["ax_mps2"], 0),
                                       "rolling_accel": 0.0, "linear_drag": 0.0}}
            self.state = advance_model(self.state, dt, 1.0 if row["ax_mps2"] >= 0 else 0.0,
                                       1.0 if row["ax_mps2"] < 0 else 0.0,
                                       row["delta_equiv_rad"], params, max_step=min(.002, dt))
            self.age += dt
        epsi = math.atan2(math.sin(self.state[2] - row["psi_rad"]), math.cos(self.state[2] - row["psi_rad"]))
        err = {"model_active": 1, "model_age_s": self.age, "model_X_m": self.state[0],
               "model_Y_m": self.state[1], "model_psi_rad": self.state[2], "model_vx_mps": self.state[3],
               "model_vy_mps": self.state[4], "model_r_radps": self.state[5],
               "err_vy_mps": self.state[4] - row["vy_mps"],
               "err_r_radps": self.state[5] - row["r_radps"], "err_psi_rad": epsi,
               "err_pos_m": math.hypot(self.state[0] - row["X_m"], self.state[1] - row["Y_m"])}
        self.errors.append(err)
        return err

    @staticmethod
    def _empty():
        return {k: None for k in ("model_age_s", "model_X_m", "model_Y_m", "model_psi_rad",
                                  "model_vx_mps", "model_vy_mps", "model_r_radps", "err_vy_mps",
                                  "err_r_radps", "err_psi_rad", "err_pos_m")} | {"model_active": 0}


class _LivePlot:
    """Persistent artists; redraw at the console sampling rate, not every frame."""
    PANELS = (("speed_kmh",), ("vy_mps", "model_vy_mps"), ("r_radps", "model_r_radps"),
              ("delta_equiv_rad",), ("Fy_total_est_N",), ("front_slip_est_rad", "rear_slip_est_rad"))

    def __init__(self, window_s):
        import matplotlib.pyplot as plt
        self.plt, self.window = plt, window_s
        plt.ion()
        self.fig, axes = plt.subplots(3, 2, figsize=(10, 7))
        self.axes = list(axes.flat)
        self.data = {"t_s": []}
        for keys in self.PANELS:
            for key in keys:
                self.data[key] = []
        self.lines = {}
        for ax, keys in zip(self.axes, self.PANELS):
            for key in keys:
                line, = ax.plot([], [], label=key)
                self.lines[key] = line
            ax.grid(alpha=.3)
            if len(keys) > 1:
                ax.legend(fontsize=7)
        self.fig.suptitle("CARLA realtime state and shadow model")

    def update(self, row):
        t = row["t_s"]
        self.data["t_s"].append(t)
        for key in self.data:
            if key != "t_s":
                value = row.get(key)
                self.data[key].append(float("nan") if value is None else value)
        cutoff = t - self.window
        while self.data["t_s"] and self.data["t_s"][0] < cutoff:
            for values in self.data.values():
                values.pop(0)
        for line in self.lines.values():
            line.set_data(self.data["t_s"], self.data[line.get_label()])
        for ax in self.axes:
            ax.relim(); ax.autoscale_view()
            if self.data["t_s"]:
                ax.set_xlim(max(0, self.data["t_s"][-1] - self.window), self.data["t_s"][-1] + .01)
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.plt.pause(.001)


def run_simulation(args):
    import carla

    out = Path(args.out).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / "run_status.json", {"result": "incomplete", "rows": 0})
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        versions = validate_carla(client)
    except BaseException as exc:
        write_json(out / "run_status.json", {"result": "interrupted" if isinstance(exc, KeyboardInterrupt) else "error",
                    "rows": 0, "error_type": type(exc).__name__, "message": str(exc)})
        (out / "error_traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        raise
    world = None
    old_settings = None
    actors = []
    collision_queue = queue.SimpleQueue()
    try:
        if args.mode == "pad":
            map_name = getattr(args, "map_name", None)
            if map_name:
                world = client.load_world(map_name)
            else:
                world = client.generate_opendrive_world(_road_xodr(), carla.OpendriveGenerationParameters(
                    vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=.6,
                    smooth_junctions=True, enable_mesh_visibility=True))
            old_settings = configure_world(world, args.dt)
            world.set_weather(carla.WeatherParameters.WetNoon if args.rain_visual else carla.WeatherParameters.ClearNoon)
            if map_name:
                spawn_points = world.get_map().get_spawn_points()
                if not spawn_points:
                    raise RuntimeError(f"Map {map_name!r} has no vehicle spawn points")
                if args.spawn_index >= len(spawn_points):
                    raise ValueError(f"--spawn-index {args.spawn_index} out of range; map has {len(spawn_points)} spawn points")
                spawn = spawn_points[args.spawn_index]
            else:
                waypoint = world.get_map().get_waypoint_xodr(1, -1, 100.0)
                if waypoint is None:
                    raise RuntimeError("Could not locate spawn waypoint on generated test road")
                spawn = waypoint.transform
                spawn.location.z += .5
            blueprint = world.get_blueprint_library().find(args.vehicle)
            blueprint.set_attribute("role_name", "carla_firmware_ego")
            vehicle = world.try_spawn_actor(blueprint, spawn)
            if vehicle is None:
                raise RuntimeError(f"Unable to spawn {args.vehicle}")
            actors.append(vehicle)
            vehicle.set_autopilot(False)
            vehicle.apply_control(carla.VehicleControl(brake=1.0))
            collision_bp = world.get_blueprint_library().find("sensor.other.collision")
            collision_sensor = world.spawn_actor(collision_bp, carla.Transform(), attach_to=vehicle)
            if collision_sensor is not None:
                actors.append(collision_sensor)
                collision_sensor.listen(lambda event: collision_queue.put({
                    "frame": int(event.frame), "other_actor_id": int(event.other_actor.id),
                    "impulse_Ns": [float(event.normal_impulse.x), float(event.normal_impulse.y),
                                   float(event.normal_impulse.z)]}))
            if args.tire_friction is not None:
                physics = vehicle.get_physics_control()
                for wheel in physics.wheels:
                    wheel.tire_friction = args.tire_friction
                vehicle.apply_physics_control(physics)
        else:
            world = client.get_world()
            deadline = time.monotonic() + args.wait
            vehicle = None
            while time.monotonic() < deadline:
                if args.actor_id is not None:
                    vehicle = world.get_actor(args.actor_id)
                else:
                    vehicle = next((a for a in world.get_actors().filter("vehicle.*")
                                    if a.attributes.get("role_name") == args.role_name), None)
                if vehicle is not None:
                    break
                time.sleep(.2)
            if vehicle is None:
                raise TimeoutError("No matching vehicle found in attach mode")

        api = CarlaFirmware(client, world, vehicle)
        if args.profile == "basic" and args.mass_scale != 1.0:
            current_mass = float(vehicle.get_physics_control().mass)
            api.apply_physics(mass_kg=current_mass * args.mass_scale)
        if args.mode == "pad":
            for _ in range(3):
                tick_checked(world, None, args.dt, args.timeout)
        physics, params = physics_report(vehicle, args.wheelbase)
        args.wheelbase_m = params["wheelbase_m"]
        max_steer = max(float(w.max_steer_angle) for w in list(physics.wheels)[:2])
        args.max_steer_deg = max_steer
        params.update(client_version=versions[0], server_version=versions[1], map=world.get_map().name,
                      mode=args.mode, capabilities=api.get_capabilities(),
                      weather=api.get_weather(),
                      telemetry_limitation="Published 0.9.15 and 0.9.16 Python APIs do not document a per-wheel force/load/slip getter")
        write_json(out / "tham_so_carla.json", params)
        if args.profile == "custom":
            write_json(out / "kich_ban_da_chay.json", args.custom_script)
        write_json(out / "metadata.json", {"vehicle": vehicle.type_id, "actor_id": vehicle.id,
                    "versions": {"client": versions[0], "server": versions[1]}, "mode": args.mode,
                    "scenario": getattr(args, "scenario", getattr(args, "maneuver", None)),
                    "map": world.get_map().name,
                    "wheelbase_m": params["wheelbase_m"], "mass_kg": params["mass"],
                    "mu_label": getattr(args, "mu_label", None),
                    "mass_scale": getattr(args, "mass_scale", 1.0),
                    "tire_friction_requested": args.tire_friction,
                    "coordinate_convention": "body X forward, Y left; SI units"})
        if args.mode == "pad" and not getattr(args, "map_name", None):
            (out / "test_road.xodr").write_text(_road_xodr(), encoding="utf-8")

        columns = ["frame", "t_s", "t_sim_s", "X_m", "Y_m", "Z_m", "psi_rad", "roll_deg", "pitch_deg",
                   "target_speed_kmh", "target_steer_deg",
                   "vx_mps", "vy_mps", "vz_mps", "speed_kmh", "r_radps", "ax_mps2", "ay_mps2",
                   "beta_rad", "angular_z_snapshot_raw", "r_yaw_difference_radps",
                   "steer_fl_deg", "steer_fr_deg", "steer_fl_deg_carla", "steer_fr_deg_carla",
                   "delta_equiv_rad", "delta_conversion", "throttle", "brake", "steer_norm",
                   "throttle_applied", "brake_applied", "steer_applied_norm", "throttle_next",
                   "brake_next", "steer_next_norm", "phase", "phase_next", "gear", "mass_kg",
                   "tire_friction_mean", "Fx_total_est_N", "Fy_total_est_N", "telemetry_source",
                   "front_slip_est_rad", "rear_slip_est_rad", "front_slip_rad", "rear_slip_rad",
                   "Fy_front_tel_N", "Fy_rear_tel_N", "front_slip_tel", "rear_slip_tel"]
        columns += [f"{name}_{field}" for name in WHEEL_NAMES for field in WHEEL_TELEMETRY_FIELDS]
        columns += ["model_active", "model_age_s", "model_X_m", "model_Y_m", "model_psi_rad",
                    "model_vx_mps", "model_vy_mps", "model_r_radps", "err_vy_mps", "err_r_radps",
                    "err_psi_rad", "err_pos_m"]
        online_model = None if args.no_model else _OnlineBicycle(params["wheelbase_m"], float(physics.mass),
                            args.lf_fraction, params["bounding_box_extent_m"])
        plotter = _LivePlot(args.plot_window) if getattr(args, "plot", False) else None
        path = out / ("states_realtime.csv" if args.profile in ("realtime", "scenario", "custom") else "states.csv")
        sim_start = None
        wall_start = time.monotonic()
        previous_frame = None
        previous_psi = None
        last_spectator_frame = -10
        last_hud_frame = -10
        last_path_point = None
        phase = "warmup" if args.mode == "pad" else "attach"
        last_throttle_command = 0.0
        speed_stable = 0
        mass, friction = float(physics.mass), sum(float(w.tire_friction) for w in physics.wheels) / 4
        termination_reason = "duration_complete"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            frame_count = 0
            while True:
                if args.mode == "attach":
                    snapshot = world.wait_for_tick(args.timeout)
                snapshot, state, transform, velocity, acceleration, angular = actor_state(world, vehicle)
                if not args.no_spectator_follow and snapshot.frame - last_spectator_frame >= 5:
                    vehicle_transform = vehicle.get_transform()
                    camera_location = vehicle_transform.transform(carla.Location(x=-14.0, z=7.0))
                    world.get_spectator().set_transform(carla.Transform(
                        camera_location,
                        carla.Rotation(pitch=-22.0, yaw=vehicle_transform.rotation.yaw)))
                    last_spectator_frame = snapshot.frame
                now_sim = float(snapshot.timestamp.elapsed_seconds)
                if sim_start is None:
                    sim_start = now_sim
                elapsed = now_sim - sim_start
                if args.profile == "custom":
                    cursor = 0.0
                    for step in args.custom_steps:
                        if elapsed < cursor + step["duration_s"]:
                            args.active_step = step
                            phase = step["name"]
                            break
                        cursor += step["duration_s"]
                    else:
                        termination_reason = "duration_complete"
                        break
                yaw = math.radians(float(transform.rotation.yaw))
                vx, raw_vy = _body(float(velocity.x), float(velocity.y), yaw)
                ax, raw_ay = _body(float(acceleration.x), float(acceleration.y), yaw)
                vy, ay = -raw_vy, -raw_ay
                fl = fr = 0.0
                if hasattr(carla, "VehicleWheelLocation"):
                    fl = float(vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel))
                    fr = float(vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel))
                delta = -math.radians((fl + fr) * .5)
                control = vehicle.get_control()
                base = {"vx_mps": vx, "vy_mps": vy, "r_radps": -float(angular.z), "delta_equiv_rad": delta}
                psi = -yaw
                yaw_difference = (float("nan") if previous_psi is None else
                                  math.atan2(math.sin(psi - previous_psi), math.cos(psi - previous_psi)) /
                                  max(now_sim - last_sim_time, 1e-9))
                tel = api.get_telemetry(state=base, wheelbase_m=params["wheelbase_m"],
                                        lf_fraction=args.lf_fraction, native=not args.no_telemetry)
                row = {"frame": snapshot.frame, "t_s": elapsed, "t_sim_s": now_sim,
                       "X_m": float(transform.location.x), "Y_m": -float(transform.location.y),
                       "Z_m": float(transform.location.z), "psi_rad": psi,
                       "roll_deg": float(transform.rotation.roll), "pitch_deg": float(transform.rotation.pitch),
                       "target_speed_kmh": (args.active_step["speed_kmh"] if args.profile == "custom" else float("nan")),
                       "target_steer_deg": (args.active_step["steer_deg"] if args.profile == "custom" else float("nan")),
                       "vx_mps": vx, "vy_mps": vy, "vz_mps": float(velocity.z),
                       "speed_kmh": 3.6 * math.hypot(vx, vy), "r_radps": base["r_radps"],
                       "ax_mps2": ax, "ay_mps2": ay,
                       "beta_rad": math.atan2(vy, vx) if abs(vx) > .01 else 0.0,
                       "angular_z_snapshot_raw": float(angular.z), "r_yaw_difference_radps": yaw_difference,
                       "steer_fl_deg": fl, "steer_fr_deg": fr, "steer_fl_deg_carla": fl,
                       "steer_fr_deg_carla": fr, "delta_equiv_rad": delta,
                       "delta_conversion": "mean_front_wheel_angle",
                       "throttle": float(control.throttle), "brake": float(control.brake),
                       "steer_norm": float(control.steer), "gear": int(control.gear), "mass_kg": mass,
                       "throttle_applied": float(control.throttle), "brake_applied": float(control.brake),
                       "steer_applied_norm": float(control.steer),
                       "tire_friction_mean": friction, "Fx_total_est_N": mass * ax,
                       "Fy_total_est_N": mass * ay, "phase": phase, "telemetry_source": tel["source"],
                       "front_slip_est_rad": tel["estimates"]["front_slip_rad"],
                       "rear_slip_est_rad": tel["estimates"]["rear_slip_rad"]}
                row.update({key: (float("nan") if value is None else value)
                            for key, value in tel["wheel_fields"].items()})
                for key in ("front_slip_est_rad", "rear_slip_est_rad"):
                    if row[key] is None:
                        row[key] = float("nan")
                row["front_slip_rad"] = row["front_slip_est_rad"]
                row["rear_slip_rad"] = row["rear_slip_est_rad"]
                def finite_pair(a, b):
                    return (a + b) if math.isfinite(a) and math.isfinite(b) else float("nan")
                row["Fy_front_tel_N"] = finite_pair(row["FL_lat_force"], row["FR_lat_force"])
                row["Fy_rear_tel_N"] = finite_pair(row["RL_lat_force"], row["RR_lat_force"])
                row["front_slip_tel"] = finite_pair(row["FL_lat_slip"], row["FR_lat_slip"]) / 2
                row["rear_slip_tel"] = finite_pair(row["RL_lat_slip"], row["RR_lat_slip"]) / 2
                if online_model is not None:
                    sample_dt = (now_sim - last_sim_time) if previous_frame is not None else args.dt
                    try:
                        row.update(online_model.update(row, sample_dt, getattr(args, "reset_s", 1.0)))
                    except (ValueError, FloatingPointError, np.linalg.LinAlgError):
                        row.update(online_model._empty())
                        online_model.state = None
                else:
                    row.update({key: (float("nan") if value is None else value)
                                for key, value in _OnlineBicycle._empty().items()})
                next_control = None
                finish_maneuver = False
                if args.mode == "pad":
                    if args.profile == "custom":
                        next_control = _controls(args, row, elapsed, phase)[:3]
                    elif args.profile in ("maneuver", "realtime", "scenario"):
                        if phase == "warmup":
                            if elapsed >= args.stabilize_s:
                                speed_stable = speed_stable + 1 if abs(args.speed_kmh / 3.6 - vx) < .14 else 0
                                if speed_stable >= math.ceil(1 / args.dt):
                                    phase, args.phase_start = "maneuver", elapsed
                            if elapsed > args.warmup_limit_s + args.stabilize_s and phase == "warmup":
                                raise TimeoutError("Could not reach maneuver entry speed before warmup timeout")
                        elif phase == "cruise" and args.profile in ("realtime", "scenario") and args.repeat_s > 0 and elapsed - args.phase_start >= args.repeat_s:
                            phase, args.phase_start = "maneuver", elapsed
                            args.direction *= -1
                        if args.profile == "scenario" and args.scenario == "suite" and phase != "warmup":
                            suite_elapsed = elapsed - args.phase_start
                            suite_length = sum(duration for _, duration in SCENARIO_SUITE)
                            if suite_elapsed >= suite_length:
                                termination_reason = "duration_complete"
                                break
                            cursor = 0.0
                            for segment, segment_duration in SCENARIO_SUITE:
                                if suite_elapsed < cursor + segment_duration:
                                    phase = segment
                                    args.suite_segment = segment
                                    args.suite_segment_t = suite_elapsed - cursor
                                    break
                                cursor += segment_duration
                        next_control = _controls(args, row, elapsed, phase)[:3]
                        finish_maneuver = phase == "maneuver" and elapsed - args.phase_start >= getattr(args, "maneuver_duration", 6.0)
                    else:
                        thr, brk, steer, next_phase = _controls(args, row, elapsed, phase)
                        phase = next_phase
                        next_control = (thr, brk, steer)
                    row["phase_next"] = phase
                    row["throttle_next"], row["brake_next"], row["steer_next_norm"] = next_control
                else:
                    row["phase_next"] = phase
                    row["throttle_next"], row["brake_next"], row["steer_next_norm"] = (
                        float("nan"), float("nan"), float("nan"))
                if args.mode == "pad" and next_control is not None:
                    desired_throttle, desired_brake, desired_steer = next_control
                    max_change = args.throttle_ramp_per_s * args.dt
                    if desired_brake > 0:
                        applied_throttle_command = 0.0
                    else:
                        applied_throttle_command = last_throttle_command + max(
                            -max_change, min(max_change, desired_throttle - last_throttle_command))
                    next_control = (applied_throttle_command, desired_brake, desired_steer)
                    row["throttle_next"] = applied_throttle_command
                writer.writerow(row)
                frame_count += 1
                previous_frame = snapshot.frame
                previous_psi = psi
                last_sim_time = now_sim
                if frame_count % 20 == 0:
                    stream.flush()

                if args.mode == "pad":
                    if finish_maneuver:
                        if args.profile in ("realtime", "scenario") and args.repeat_s > 0:
                            phase = "cruise"
                            args.phase_start = elapsed
                        else:
                            termination_reason = "duration_complete"
                            break
                    thr, brk, steer = next_control
                    vehicle.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=steer))
                    last_throttle_command = thr
                    if args.max_speed_mps > 0 and vx > args.max_speed_mps:
                        termination_reason = "cutoff_speed"
                        break
                    if args.duration > 0 and elapsed >= args.duration:
                        break
                    tick_checked(world, snapshot.frame, args.dt, args.timeout)
                    if not collision_queue.empty():
                        termination_reason = "cutoff_collision_event"
                        break
                    if args.realtime and args.mode == "pad":
                        target = wall_start + (elapsed + args.dt)
                        delay = target - time.monotonic()
                        if delay > 0:
                            time.sleep(delay)
                else:
                    if args.duration > 0 and elapsed >= args.duration:
                        break
                if args.log_hz > 0 and frame_count % max(1, round(1 / (args.log_hz * args.dt))) == 0:
                    print(f"t={elapsed:7.2f}s speed={row['speed_kmh']:6.2f} km/h ay={ay:+6.2f} m/s2", flush=True)
                    if (args.mode == "pad" and not args.no_hud and
                            snapshot.frame - last_hud_frame >= max(1, round(1 / (args.log_hz * args.dt)))):
                        location = transform.location
                        label = (getattr(args, "scenario", None) or
                                 ("custom" if args.profile == "custom" else getattr(args, "maneuver", "basic")))
                        hud = (f"TEST: {label} | PHASE: {phase}\n"
                               f"TIME {elapsed:.1f} s   SPEED {row['speed_kmh']:.1f} km/h\n"
                               f"VX {vx:.2f}   VY {vy:.2f} m/s   YAW RATE {row['r_radps']:.3f} rad/s\n"
                               f"AX {ax:.2f}   AY {ay:.2f} m/s2   BETA {row['beta_rad']:.3f} rad\n"
                               f"STEER {row['steer_applied_norm']:.2f}   THROTTLE {row['throttle_applied']:.2f}   BRAKE {row['brake_applied']:.2f}")
                        world.debug.draw_string(
                            carla.Location(x=location.x, y=location.y, z=location.z + 4.0), hud,
                            draw_shadow=True, color=carla.Color(80, 255, 100),
                            life_time=max(.1, .95 / args.log_hz))
                        if last_path_point is not None:
                            path_location = carla.Location(x=location.x, y=location.y, z=location.z + .25)
                            world.debug.draw_line(last_path_point, path_location, thickness=.08,
                                color=carla.Color(255, 210, 30), life_time=2.0)
                        else:
                            path_location = carla.Location(x=location.x, y=location.y, z=location.z + .25)
                        last_path_point = path_location
                        last_hud_frame = snapshot.frame
                    if plotter is not None:
                        plotter.update(row)
                if frame_count % max(1, round(args.param_refresh / args.dt)) == 0:
                    live_physics = vehicle.get_physics_control()
                    mass = float(live_physics.mass)
                    friction = sum(float(w.tire_friction) for w in live_physics.wheels) / 4

        collisions = []
        while not collision_queue.empty():
            collisions.append(collision_queue.get())
        result = "cutoff_collision_event" if collisions else termination_reason
        write_json(out / "run_status.json", {"result": result, "rows": frame_count,
                    "simulated_s": elapsed, "wall_elapsed_s": time.monotonic() - wall_start,
                    "telemetry_source": ("state_estimate" if args.no_telemetry else
                                         api.get_capabilities()["native_telemetry_method"] or "state_estimate")})
        write_json(out / "collision_events.json", collisions)
        model_status = {"enabled": not args.no_model,
                        "status": "disabled" if args.no_model else "waiting_for_lateral_excitation",
                        "wheelbase_m": params["wheelbase_m"], "mass_kg": mass,
                        "lf_fraction_assumed": args.lf_fraction,
                        "parameter_source": "online least-squares from CARLA body state; yaw inertia rectangular-box assumption"}
        if online_model is not None and online_model.coeff is not None:
            rmse = lambda key: float(np.sqrt(np.mean([e[key] ** 2 for e in online_model.errors]))) if online_model.errors else None
            model_status.update(status="identified", cf_over_mass=float(online_model.coeff[0]),
                                cr_over_mass=float(online_model.coeff[1]), iz_over_mass=online_model.iz_m,
                                regression_samples=len(online_model.samples), scored_samples=len(online_model.errors),
                                resets=int(math.ceil(elapsed / max(getattr(args, "reset_s", 1.0), .2))),
                                rmse_vy_mps=rmse("err_vy_mps"), rmse_r_radps=rmse("err_r_radps"),
                                rmse_psi_rad=rmse("err_psi_rad"), rmse_pos_m=rmse("err_pos_m"))
        write_json(out / "mo_hinh_truc_tuyen.json", model_status)
        return 0 if result == "duration_complete" else 2
    except BaseException as exc:
        write_json(out / "run_status.json", {"result": "interrupted" if isinstance(exc, KeyboardInterrupt) else "error",
                    "rows": locals().get("frame_count", 0), "error_type": type(exc).__name__, "message": str(exc)})
        (out / "error_traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        raise
    finally:
        if args.mode == "pad":
            keep = vehicle if getattr(args, "keep_vehicle", False) and locals().get("vehicle") is not None else None
            cleanup(world, old_settings, actors, keep_vehicle=keep, carla_module=carla)
