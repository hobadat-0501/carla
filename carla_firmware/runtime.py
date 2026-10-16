"""Fresh CARLA 0.9.15 runtime utilities used by the simulation firmware."""
from __future__ import annotations

import json
import math
from pathlib import Path


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_carla(client):
    versions = client.get_client_version(), client.get_server_version()
    if any(not str(v).startswith("0.9.15") for v in versions):
        raise RuntimeError(f"CARLA client/server 0.9.15 required, got {versions}")
    return versions


def configure_world(world, dt):
    old = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    settings.substepping = True
    settings.max_substep_delta_time = 0.005
    settings.max_substeps = min(10, max(1, math.ceil(dt / 0.005)))
    world.apply_settings(settings)
    return old


def tick_checked(world, previous_frame, dt, timeout=30.0):
    before = world.get_snapshot()
    if previous_frame is not None and before.frame != previous_frame:
        raise RuntimeError("World advanced outside the simulation firmware; stop other ticking clients")
    frame = world.tick(timeout)
    after = world.get_snapshot()
    if after.frame != frame or (previous_frame is not None and frame != previous_frame + 1):
        raise RuntimeError("Unexpected CARLA frame sequence")
    actual = after.timestamp.elapsed_seconds - before.timestamp.elapsed_seconds
    if abs(actual - dt) > max(1e-6, dt * 0.01):
        raise RuntimeError(f"CARLA timestep mismatch: requested {dt:g}s, received {actual:g}s")
    return frame


def wheelbase_from_physics(physics, override=None):
    if len(physics.wheels) != 4:
        raise ValueError("The selected vehicle must have exactly four wheels")
    # CARLA wheel positions are reported in centimetres, unlike world transforms.
    positions = [float(w.position.x) * 0.01 for w in physics.wheels]
    front = sorted(positions, reverse=True)[:2]
    rear = sorted(positions)[:2]
    inferred = (sum(front) / 2.0) - (sum(rear) / 2.0)
    if override is not None:
        override = finite(override, "wheelbase")
        if override <= 0:
            raise ValueError("wheelbase must be positive")
        return override, "user_verified_override", inferred
    if inferred <= 0:
        raise ValueError("Cannot infer positive wheelbase from wheel locations; pass --wheelbase")
    return inferred, "physics_wheel_positions", inferred


def physics_report(vehicle, wheelbase_override=None):
    physics = vehicle.get_physics_control()
    if len(physics.wheels) != 4 or not math.isfinite(float(physics.mass)) or physics.mass <= 0:
        raise ValueError("Vehicle physics must have four wheels and positive finite mass")
    front_steer = [float(w.max_steer_angle) for w in list(physics.wheels)[:2]]
    if not all(math.isfinite(v) for v in front_steer) or max(front_steer) <= 0:
        raise ValueError("Vehicle physics has no valid front wheel steering range")
    wheelbase, source, inferred = wheelbase_from_physics(physics, wheelbase_override)
    # Serialize the complete public CARLA control object without depending on a
    # version-specific attribute list.
    from .api import _attrs, WHEEL_NAMES
    values = _attrs(physics, ("mass", "drag_coefficient", "center_of_mass", "max_rpm", "moi",
                               "damping_rate_full_throttle", "damping_rate_zero_throttle_clutch_engaged",
                               "damping_rate_zero_throttle_clutch_disengaged", "final_ratio", "gear_switch_time",
                               "clutch_strength", "use_gear_autobox", "use_sweep_wheel_collision"))
    for field in ("torque_curve", "steering_curve"):
        curve = getattr(physics, field, None)
        if curve is not None:
            values[field] = [[float(p.x), float(p.y)] for p in curve]
    values["forward_gears"] = [_attrs(gear, ("ratio", "down_ratio", "up_ratio"))
                               for gear in getattr(physics, "forward_gears", ())]
    values["wheels"] = []
    for name, wheel in zip(WHEEL_NAMES, physics.wheels):
        item = _attrs(wheel, ("tire_friction", "damping_rate", "max_steer_angle", "radius",
                               "max_brake_torque", "max_handbrake_torque", "lat_stiff_max_load",
                               "lat_stiff_value", "long_stiff_value", "position"))
        if item.get("position") is not None:
            item["position"] = [coordinate * 0.01 for coordinate in item["position"]]
        item["wheel"] = name
        values["wheels"].append(item)
    extent = vehicle.bounding_box.extent
    values.update(wheelbase_m=wheelbase, wheelbase_source=source, wheelbase_inferred_m=inferred,
                  bounding_box_extent_m=[float(extent.x), float(extent.y), float(extent.z)])
    values["units"] = {"mass": "kg", "center_of_mass": "m", "moi": "kg*m^2",
                       "gear_switch_time": "s", "clutch_strength": "kg*m^2/s",
                       "max_rpm": "rpm", "drag_coefficient": "CARLA scalar",
                       "wheel.position": "m", "wheel.radius": "cm", "wheel.max_steer_angle": "deg",
                       "wheel.max_brake_torque": "N*m", "wheel.max_handbrake_torque": "N*m",
                       "wheel.tire_friction": "CARLA scalar", "wheel.damping_rate": "CARLA scalar",
                       "wheel.lat_stiff_value": "CARLA custom parameter",
                       "wheel.lat_stiff_max_load": "CARLA normalized load",
                       "wheel.long_stiff_value": "kg/rad"}
    return physics, values


def actor_state(world, vehicle):
    snapshot = world.get_snapshot()
    actor = snapshot.find(vehicle.id)
    if actor is None:
        raise RuntimeError("Vehicle disappeared from CARLA world snapshot")
    transform = actor.get_transform()
    velocity, accel, angular = actor.get_velocity(), actor.get_acceleration(), actor.get_angular_velocity()
    return snapshot, actor, transform, velocity, accel, angular


def cleanup(world, old_settings, actors, *, keep_vehicle=None, carla_module=None):
    for actor in reversed(actors):
        try:
            if keep_vehicle is not None and actor.id == keep_vehicle.id and actor.is_alive:
                actor.apply_control(carla_module.VehicleControl(brake=1.0))
                actor.set_simulate_physics(False)
                continue
            if actor.is_alive:
                if actor.type_id.startswith("sensor."):
                    actor.stop()
                actor.destroy()
        except Exception:
            pass
    if world is not None and old_settings is not None:
        world.apply_settings(old_settings)
