"""CARLA 0.9.15 compatibility API for vehicle simulation and telemetry.

This is a Python client-side adapter, not a server firmware replacement. It
reports values exposed by the connected CARLA server and adds explicitly
labelled kinematic estimates where 0.9.15 has no wheel telemetry API.
"""
from __future__ import annotations

import math


WHEEL_NAMES = ("FL", "FR", "RL", "RR")
WHEEL_TELEMETRY_FIELDS = (
    "tire_friction", "lat_slip", "long_slip", "omega", "tire_load",
    "normalized_tire_load", "torque", "long_force", "lat_force",
)


def _value(value):
    """Convert CARLA's pybind values into JSON-safe Python primitives."""
    if value is None:
        return None
    if all(hasattr(value, axis) for axis in ("x", "y")):
        result = [float(value.x), float(value.y)]
        if hasattr(value, "z"):
            result.append(float(value.z))
        return result
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (str, bool)):
        return value
    return None


def _attrs(obj, names):
    return {name: _value(getattr(obj, name, None)) for name in names
            if getattr(obj, name, None) is not None}


class CarlaFirmware:
    """Stable read API over CARLA 0.9.15, with optional newer telemetry."""

    def __init__(self, client, world, vehicle):
        self.client = client
        self.world = world
        self.vehicle = vehicle

    def get_versions(self):
        return {"client": self.client.get_client_version(),
                "server": self.client.get_server_version()}

    def get_capabilities(self):
        names = ("get_telemetry_data", "get_telemetry")
        method = next((getattr(self.vehicle, n) for n in names
                       if callable(getattr(self.vehicle, n, None))), None)
        return {
            "physics_control": callable(getattr(self.vehicle, "get_physics_control", None)),
            "snapshot_state": callable(getattr(self.world, "get_snapshot", None)),
            "wheel_steer_angle": callable(getattr(self.vehicle, "get_wheel_steer_angle", None)),
            "debug_telemetry_overlay": callable(getattr(self.vehicle, "show_debug_telemetry", None)),
            "native_wheel_telemetry": method is not None,
            "native_telemetry_method": getattr(method, "__name__", None),
            "wheel_telemetry_fallback": "kinematic_estimates_and_unavailable_fields",
        }

    def get_vehicle_parameters(self, wheelbase_m=None):
        physics = self.vehicle.get_physics_control()
        wheels = list(physics.wheels)
        if len(wheels) != 4:
            raise ValueError("This firmware API requires exactly four vehicle wheels.")
        report = _attrs(physics, (
            "mass", "drag_coefficient", "center_of_mass", "max_rpm", "moi",
            "damping_rate_full_throttle", "damping_rate_zero_throttle_clutch_engaged",
            "damping_rate_zero_throttle_clutch_disengaged", "final_ratio", "gear_switch_time",
            "clutch_strength", "use_gear_autobox", "use_sweep_wheel_collision",
        ))
        for name in ("torque_curve", "steering_curve"):
            curve = getattr(physics, name, None)
            if curve is not None:
                report[name] = [[float(p.x), float(p.y)] for p in curve]
        report["forward_gears"] = [_attrs(gear, ("ratio", "down_ratio", "up_ratio"))
                                   for gear in getattr(physics, "forward_gears", ())]
        report["wheels"] = []
        for name, wheel in zip(WHEEL_NAMES, wheels):
            item = _attrs(wheel, (
                "tire_friction", "damping_rate", "max_steer_angle", "radius",
                "max_brake_torque", "max_handbrake_torque", "lat_stiff_max_load",
                "lat_stiff_value", "long_stiff_value", "position",
            ))
            if item.get("position") is not None:
                item["position"] = [coordinate * 0.01 for coordinate in item["position"]]
            item["wheel"] = name
            report["wheels"].append(item)
        box = getattr(self.vehicle, "bounding_box", None)
        extent = getattr(box, "extent", None)
        if extent is not None:
            report["bounding_box_extent_m"] = [float(extent.x), float(extent.y), float(extent.z)]
        if wheelbase_m is not None:
            report["wheelbase_m"] = float(wheelbase_m)
        return report

    def get_state(self):
        """Return the latest snapshot-backed actor state and current controls."""
        snap = self.world.get_snapshot()
        actor = snap.find(self.vehicle.id)
        if actor is None:
            raise RuntimeError("Vehicle is absent from the current world snapshot.")
        transform = actor.get_transform()
        velocity = actor.get_velocity()
        acceleration = actor.get_acceleration()
        angular = actor.get_angular_velocity()
        control = self.vehicle.get_control()
        return {
            "frame": int(snap.frame),
            "timestamp_s": float(snap.timestamp.elapsed_seconds),
            "transform": {"location_m": [float(transform.location.x), float(transform.location.y), float(transform.location.z)],
                          "rotation_deg": [float(transform.rotation.roll), float(transform.rotation.pitch), float(transform.rotation.yaw)]},
            "velocity_world_mps": [float(velocity.x), float(velocity.y), float(velocity.z)],
            "acceleration_world_mps2": [float(acceleration.x), float(acceleration.y), float(acceleration.z)],
            "angular_velocity_world_rad_s": [float(angular.x), float(angular.y), float(angular.z)],
            "control": _attrs(control, ("throttle", "steer", "brake", "hand_brake", "reverse", "gear", "manual_gear_shift")),
        }

    def get_map_info(self):
        map_obj = self.world.get_map()
        return {"name": map_obj.name, "opendrive": map_obj.to_opendrive()}

    def get_weather(self):
        weather = self.world.get_weather()
        return _attrs(weather, ("cloudiness", "precipitation", "precipitation_deposits",
                                "wind_intensity", "sun_azimuth_angle", "sun_altitude_angle",
                                "fog_density", "fog_distance", "fog_falloff", "wetness",
                                "scattering_intensity", "mie_scattering_scale",
                                "rayleigh_scattering_scale", "dust_storm"))

    def get_wheel_steering(self, carla_module):
        locations = carla_module.VehicleWheelLocation
        return {name: float(self.vehicle.get_wheel_steer_angle(getattr(locations, f"{name}_Wheel")))
                for name in WHEEL_NAMES}

    def show_debug_telemetry(self, enabled=True):
        """Toggle CARLA's spectator overlay; this does not return wheel data."""
        method = getattr(self.vehicle, "show_debug_telemetry", None)
        if not callable(method):
            return False
        method(bool(enabled))
        return True

    def apply_control(self, **values):
        """Apply validated normalized control fields to the attached vehicle."""
        import carla
        allowed = {"throttle", "steer", "brake", "hand_brake", "reverse", "manual_gear_shift", "gear"}
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"Unsupported control field(s): {sorted(unknown)}")
        for name in ("throttle", "steer", "brake"):
            value = float(values.get(name, 0.0))
            low = -1.0 if name == "steer" else 0.0
            if not math.isfinite(value) or not low <= value <= 1.0:
                raise ValueError(f"Invalid normalized {name} command")
        self.vehicle.apply_control(carla.VehicleControl(**values))

    def apply_physics(self, *, mass_kg=None, tire_friction=None):
        """Apply explicit physics overrides and return simulator readback."""
        physics = self.vehicle.get_physics_control()
        if mass_kg is not None:
            mass = float(mass_kg)
            if not math.isfinite(mass) or mass <= 0:
                raise ValueError("mass_kg must be positive and finite")
            physics.mass = mass
        if tire_friction is not None:
            friction = float(tire_friction)
            if not math.isfinite(friction) or not 0 < friction <= 10:
                raise ValueError("tire_friction must be in (0, 10]")
            for wheel in physics.wheels:
                wheel.tire_friction = friction
        self.vehicle.apply_physics_control(physics)
        updated = self.vehicle.get_physics_control()
        return {"mass_kg": float(updated.mass),
                "tire_friction": [float(w.tire_friction) for w in updated.wheels]}

    def get_telemetry(self, *, state=None, wheelbase_m=None, lf_fraction=0.45, native=True):
        """Return native wheel telemetry or documented estimates for 0.9.15.

        Stock 0.9.15/0.9.16 Python APIs do not document tire loads, forces,
        slip ratios or wheel angular velocities as getters. Those fields stay null; only
        axle slip-angle estimates may be populated from rigid-body state.
        """
        native_method = next((getattr(self.vehicle, n) for n in ("get_telemetry_data", "get_telemetry")
                              if native and callable(getattr(self.vehicle, n, None))), None)
        values = {f"{wheel}_{field}": None for wheel in WHEEL_NAMES
                  for field in WHEEL_TELEMETRY_FIELDS}
        source = "unavailable"
        if native_method is not None:
            native = native_method()
            native_wheels = list(getattr(native, "wheels", ()))
            if len(native_wheels) == 4:
                for wheel_name, wheel in zip(WHEEL_NAMES, native_wheels):
                    for field in WHEEL_TELEMETRY_FIELDS:
                        values[f"{wheel_name}_{field}"] = _value(getattr(wheel, field, None))
                source = "native"
        estimates = {"front_slip_rad": None, "rear_slip_rad": None}
        if state is not None and wheelbase_m and wheelbase_m > 0:
            vx, vy = state["vx_mps"], state["vy_mps"]
            yaw_rate, delta = state["r_radps"], state["delta_equiv_rad"]
            lf = float(lf_fraction) * float(wheelbase_m)
            lr = float(wheelbase_m) - lf
            speed_floor = max(vx, 0.8)
            estimates = {
                "front_slip_rad": delta - math.atan2(vy + lf * yaw_rate, speed_floor),
                "rear_slip_rad": -math.atan2(vy - lr * yaw_rate, speed_floor),
            }
            if source == "unavailable":
                source = "state_estimate"
        return {"source": source, "native_available": native_method is not None,
                "wheel_fields": values, "estimates": estimates,
                "unavailable_fields": [] if source == "native" else list(WHEEL_TELEMETRY_FIELDS),
                "note": ("Wheel loads/forces/slips/angular velocity require native server telemetry; "
                "Published 0.9.15/0.9.16 APIs do not expose them as getters. State-derived axle slip angles are estimates.")}
