"""Run the MATLAB kinematic MPC against a live CARLA vehicle."""
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from .runtime import configure_world, physics_report, tick_checked, validate_carla


def _reference(kind: str, length: float, ds: float = .05):
    s = np.arange(0.0, length + ds, ds)
    if kind in ("sine10", "sine15"):
        x = s.copy()
        y = 4.0 * np.sin(2 * math.pi * x / 100.0)
        psi = np.arctan2(np.gradient(y, ds), np.ones_like(y))
        vx = 10.0 if kind == "sine10" else 15.0
        speed = vx * np.sqrt(1 + (4 * 2 * math.pi / 100 * np.cos(2 * math.pi * x / 100)) ** 2)
    else:
        k = (.012 * np.sin(2 * math.pi * s / 180) + .008 * np.sin(2 * math.pi * s / 97 + 1)
             + .006 * np.sin(2 * math.pi * s / 260 + 2))
        psi = np.r_[0.0, np.cumsum((k[:-1] + k[1:]) * .5 * ds)]
        x = np.r_[0.0, np.cumsum(np.cos(psi[:-1]) * ds)]
        y = np.r_[0.0, np.cumsum(np.sin(psi[:-1]) * ds)]
        smooth = np.convolve(np.abs(k), np.ones(max(1, round(40 / ds))) / max(1, round(40 / ds)), mode="same")
        v0 = np.clip(np.sqrt(2.5 / np.maximum(smooth, 1e-4)), 6, 22)
        speed = (.7 if kind == "wind07" else 1.0) * v0
    return s, x, y, psi, speed


class KinematicMPC:
    def __init__(self, lf, lr, steer_limit, ts=.1, td=.2, hp=8):
        self.lf, self.lr, self.L = lf, lr, lf + lr
        self.ts, self.td, self.hp = ts, td, hp
        self.delta_max = min(math.atan(math.tan(math.radians(37)) * self.L / lr), steer_limit)
        self.ddelta_max = math.radians(10) * self.L / lr
        self.q = np.array([10., 10., 5., 5.])
        self.r = np.array([.1, 1.])
        self.rb = np.array([1., 500.])
        self.u = np.zeros((hp, 2))
        self.previous = np.zeros(2)

    def _step(self, z, u, dt):
        beta = math.atan(self.lr / self.L * math.tan(u[1]))
        return z + dt * np.array([z[3] * math.cos(z[2] + beta), z[3] * math.sin(z[2] + beta),
                                  z[3] / self.lr * math.sin(beta), u[0]])

    def solve(self, z0, ref):
        hp, ts, td = self.hp, self.ts, self.td
        guess = np.vstack((self.u[1:], self.u[-1]))
        bounds = [( -1.5, 1.0), (-self.delta_max, self.delta_max)] * hp
        dt = np.r_[ts, np.full(hp - 1, td)]
        lower = dt[:, None] * np.array([-3., -self.ddelta_max])
        upper = dt[:, None] * np.array([1.5, self.ddelta_max])
        def objective(flat):
            controls = flat.reshape(hp, 2)
            z, prev, cost = z0.copy(), self.previous.copy(), 0.0
            for i, u in enumerate(controls):
                e = z - ref[:, i]; e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
                du = u - prev
                cost += np.dot(self.q, e * e) + np.dot(self.rb, du * du) + np.dot(self.r, u * u)
                z = self._step(z, u, td); prev = u
            e = z - ref[:, hp]; e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
            return cost + np.dot(self.q, e * e)
        def slew(flat):
            u = flat.reshape(hp, 2)
            prev = np.vstack((self.previous, u[:-1]))
            d = u - prev
            return np.r_[(d - lower).ravel(), (upper - d).ravel()]
        result = minimize(objective, guess.ravel(), method="SLSQP", bounds=bounds,
                          constraints=[{"type": "ineq", "fun": slew}],
                          options={"maxiter": 100, "ftol": 1e-6, "disp": False})
        controls = result.x.reshape(hp, 2)
        self.u, self.previous = controls, controls[0].copy()
        return controls[0], bool(result.success), float(result.fun)


class _LivePlot:
    """Live path, speed, and tracking-error view for a CARLA MPC run."""
    def __init__(self, ref_x, ref_y):
        import matplotlib.pyplot as plt
        self.plt = plt
        plt.ion()
        self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
        self.fig.suptitle("CARLA live kinematic MPC", fontweight="bold")
        ax = self.axes[0]
        ax.plot(ref_x, ref_y, "--", color="#9aa6b2", lw=1, label="Reference")
        self.path, = ax.plot([], [], color="#1769aa", lw=1.7, label="CARLA vehicle")
        self.car, = ax.plot([], [], "o", color="#c24135", ms=7)
        ax.set(title="Trajectory", xlabel="World X (m)", ylabel="World Y (m)"); ax.axis("equal")
        ax.legend(loc="best"); ax.grid(True, alpha=.7)
        self.speed, = self.axes[1].plot([], [], color="#1769aa", lw=1.6, label="Measured")
        self.speed_ref, = self.axes[1].plot([], [], "--", color="#e28a1b", lw=1.3, label="Reference")
        self.axes[1].set(title="Speed", xlabel="Time (s)", ylabel="m/s"); self.axes[1].legend(loc="best"); self.axes[1].grid(True, alpha=.7)
        self.error, = self.axes[2].plot([], [], color="#7c3aed", lw=1.5)
        self.axes[2].set(title="Cross-track error", xlabel="Time (s)", ylabel="m"); self.axes[2].grid(True, alpha=.7)
        self.t, self.x, self.y, self.v, self.vref, self.err = [], [], [], [], [], []
        self.fig.show(); plt.pause(.001)

    def update(self, t, x, y, speed, speed_ref, error):
        self.t.append(t); self.x.append(x); self.y.append(y)
        self.v.append(speed); self.vref.append(speed_ref); self.err.append(error)
        self.path.set_data(self.x, self.y); self.car.set_data([x], [y])
        self.speed.set_data(self.t, self.v); self.speed_ref.set_data(self.t, self.vref)
        self.error.set_data(self.t, self.err)
        for ax in self.axes[1:]: ax.relim(); ax.autoscale_view()
        self.fig.suptitle(f"CARLA live kinematic MPC | t={t:.1f} s | speed={speed:.1f} m/s | error={error:.2f} m",
                          fontweight="bold")
        self.fig.canvas.draw_idle(); self.plt.pause(.001)

    def save_close(self, path):
        self.fig.savefig(path, dpi=180, facecolor="white")
        self.plt.close(self.fig)


def run(argv=None):
    p = argparse.ArgumentParser(description="MATLAB kinematic MPC in CARLA 0.9.15")
    p.add_argument("--host", default="127.0.0.1"); p.add_argument("--port", type=int, default=2000)
    p.add_argument("--map", default="Town06"); p.add_argument("--spawn-index", type=int, default=0)
    p.add_argument("--vehicle", default="vehicle.tesla.model3")
    p.add_argument("--scenario", choices=("wind07", "wind10", "sine10", "sine15"), default="wind10")
    p.add_argument("--duration", type=float, default=60); p.add_argument("--dt", type=float, default=.01)
    p.add_argument("--out", type=Path, default=Path("ket_qua/mpc_carla"))
    p.add_argument("--timeout", type=float, default=30); p.add_argument("--fast", action="store_true")
    p.add_argument("--plot", action="store_true", help="show live path, speed, and cross-track plots")
    p.add_argument("--no-spectator-follow", action="store_true", help="disable the CARLA camera following the MPC vehicle")
    args = p.parse_args(argv)
    if args.dt <= 0 or args.duration <= 0: p.error("--dt and --duration must be positive")
    import carla
    args.out.mkdir(parents=True, exist_ok=False)
    client = carla.Client(args.host, args.port); client.set_timeout(args.timeout)
    validate_carla(client)
    world = client.load_world(args.map)
    old_settings = configure_world(world, args.dt)
    actors = []
    try:
        spawn_points = world.get_map().get_spawn_points()
        if args.spawn_index >= len(spawn_points): raise ValueError("spawn index is out of range")
        bp = world.get_blueprint_library().find(args.vehicle)
        bp.set_attribute("role_name", "mpc_ego")
        vehicle = world.try_spawn_actor(bp, spawn_points[args.spawn_index])
        if vehicle is None: raise RuntimeError("Could not spawn vehicle at selected point")
        actors.append(vehicle)
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        # CARLA snapshots do not include a newly spawned actor until the world ticks.
        old_frame = tick_checked(world, None, args.dt, args.timeout)
        physics, _ = physics_report(vehicle)
        # The source MATLAB file uses lf=wheelbase/2-COM_x and lr=wheelbase/2+COM_x.
        wheels = list(physics.wheels)
        front_x = sum(float(w.position.x) for w in wheels[:2]) / 200
        rear_x = sum(float(w.position.x) for w in wheels[2:]) / 200
        wheelbase = front_x - rear_x
        com_x = float(physics.center_of_mass.x)
        lf, lr = wheelbase / 2 - com_x, wheelbase / 2 + com_x
        steer_limit = math.radians(min(float(w.max_steer_angle) for w in wheels[:2]))
        controller = KinematicMPC(lf, lr, steer_limit)
        s, rx, ry, rpsi, rv = _reference(args.scenario, 820 if args.scenario.startswith("wind") else 290)
        origin = spawn_points[args.spawn_index]
        yaw = math.radians(origin.rotation.yaw)
        c, sn = math.cos(yaw), math.sin(yaw)
        def global_ref(x, y, psi):
            return (origin.location.x + c*x - sn*y, -origin.location.y - sn*x - c*y,
                    -yaw - psi)
        live_plot = _LivePlot(*global_ref(rx, ry, rpsi)[:2]) if args.plot else None
        frame = float(world.get_snapshot().timestamp.elapsed_seconds)
        elapsed = 0.; last_control_t = -1e9; progress_idx = 0
        last_u = np.zeros(2); rows = 0; last_wall = time.monotonic()
        last_camera_frame = -10; last_hud_frame = -10
        columns = ["frame", "t_s", "X_m", "Y_m", "psi_rad", "vx_mps", "vy_mps", "speed_mps",
                   "ref_s_m", "ref_x_m", "ref_y_m", "ref_psi_rad", "ref_speed_mps", "accel_cmd_mps2",
                   "delta_cmd_rad", "steer_norm", "throttle", "brake", "mpc_success", "cost", "cross_track_m"]
        with (args.out / "states_mpc.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader()
            while elapsed < args.duration and s[-1] > 0:
                snapshot = world.get_snapshot()
                state_actor = snapshot.find(vehicle.id)
                if state_actor is None:
                    raise RuntimeError(
                        f"Spawned vehicle {vehicle.id} is missing from CARLA snapshot at frame {snapshot.frame}"
                    )
                tf = state_actor.get_transform(); vel = state_actor.get_velocity()
                if not args.no_spectator_follow and snapshot.frame - last_camera_frame >= 5:
                    camera_location = tf.transform(carla.Location(x=-14.0, z=7.0))
                    world.get_spectator().set_transform(carla.Transform(
                        camera_location,
                        carla.Rotation(pitch=-22.0, yaw=tf.rotation.yaw)))
                    last_camera_frame = snapshot.frame
                yaw_world = math.radians(tf.rotation.yaw)
                psi = -yaw_world
                vx = vel.x * math.cos(yaw_world) + vel.y * math.sin(yaw_world)
                vy = -(-vel.x * math.sin(yaw_world) + vel.y * math.cos(yaw_world))
                x, y = tf.location.x, -tf.location.y
                # Progress is the nearest point in a bounded forward window.
                begin, end = max(0, progress_idx - 20), min(len(s), progress_idx + 1000)
                gx, gy, _ = global_ref(rx[begin:end], ry[begin:end], rpsi[begin:end])
                idx = begin + int(np.argmin((gx-x)**2 + (gy-y)**2))
                progress_idx = max(progress_idx, idx)
                refs = np.zeros((4, controller.hp + 1))
                for j in range(controller.hp + 1):
                    ri = min(len(s)-1, idx + round(j * controller.td * max(rv[idx], 1.) / .05))
                    qx, qy, qpsi = global_ref(rx[ri], ry[ri], rpsi[ri])
                    refs[:, j] = [qx, qy, qpsi, rv[ri]]
                success, cost = True, float("nan")
                if elapsed - last_control_t >= controller.ts - 1e-6:
                    last_u, success, cost = controller.solve(np.array([x, y, psi, max(vx, 0.)]), refs)
                    last_control_t = elapsed
                # CARLA's normalized steering has opposite sign to this MATLAB frame.
                steer_norm = max(-1., min(1., -last_u[1] / max(steer_limit, .01)))
                target_speed = refs[3, 0]
                accel = last_u[0] + 1.4 * (target_speed - vx)
                throttle = max(0., min(.65, accel / 3.0))
                brake = max(0., min(.8, -accel / 6.0))
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer_norm))
                ref_x, ref_y, ref_psi = refs[:3, 0]
                writer.writerow({"frame": snapshot.frame, "t_s": elapsed, "X_m": x, "Y_m": y,
                    "psi_rad": psi, "vx_mps": vx, "vy_mps": vy, "speed_mps": math.hypot(vx, vy),
                    "ref_s_m": s[idx], "ref_x_m": ref_x, "ref_y_m": ref_y, "ref_psi_rad": ref_psi,
                    "ref_speed_mps": target_speed, "accel_cmd_mps2": last_u[0], "delta_cmd_rad": last_u[1],
                    "steer_norm": steer_norm, "throttle": throttle, "brake": brake, "mpc_success": int(success),
                    "cost": cost, "cross_track_m": math.hypot(x-ref_x, y-ref_y)})
                if live_plot is not None and (rows % max(1, round(controller.ts/args.dt)) == 0):
                    live_plot.update(elapsed, x, y, math.hypot(vx, vy), target_speed,
                                     math.hypot(x-ref_x, y-ref_y))
                if snapshot.frame - last_hud_frame >= 5:
                    world.debug.draw_string(
                        carla.Location(x=tf.location.x, y=tf.location.y, z=tf.location.z + 3.5),
                        f"KINEMATIC MPC | {args.scenario} | t={elapsed:.1f}s | "
                        f"v={math.hypot(vx, vy):.1f} m/s | e={math.hypot(x-ref_x, y-ref_y):.2f} m",
                        draw_shadow=True, color=carla.Color(80, 255, 100), life_time=.2)
                    last_hud_frame = snapshot.frame
                rows += 1
                old_frame = tick_checked(world, old_frame, args.dt, args.timeout)
                # Use CARLA elapsed time from initial frame rather than accumulated wall time.
                elapsed = float(world.get_snapshot().timestamp.elapsed_seconds) - frame
                if not args.fast:
                    delay = args.dt - (time.monotonic() - last_wall)
                    if delay > 0: time.sleep(delay)
                    last_wall = time.monotonic()
                if rows % 100 == 0: stream.flush()
        (args.out / "run_status.txt").write_text(f"duration_complete; rows={rows}; simulated_s={elapsed:.3f}\n", encoding="utf-8")
        if live_plot is not None:
            live_plot.save_close(args.out / "mpc_live_plot.png")
        print(f"MPC CARLA complete: {rows} samples in {args.out}")
        return 0
    finally:
        for actor in reversed(actors):
            if actor.is_alive: actor.destroy()
        world.apply_settings(old_settings)


if __name__ == "__main__":
    raise SystemExit(run())
