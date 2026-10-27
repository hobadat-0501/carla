"""Run the MATLAB huong_4 residual-learning MPC against CARLA."""
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from .runtime import (configure_world, physics_report, tick_checked,
                      validate_carla, wheelbase_from_physics)
from .mpc_prediction import forecast_plan, write_prediction_errors

ROOT = Path(__file__).resolve().parent.parent


def _movmean(values, window):
    """MATLAB-style centered moving mean with shrinking endpoint windows."""
    values = np.asarray(values, dtype=float)
    window = max(1, int(window))
    left = (window - 1) // 2
    right = window - left
    cumulative = np.r_[0.0, np.cumsum(values)]
    indices = np.arange(values.size)
    starts = np.maximum(0, indices - left)
    stops = np.minimum(values.size, indices + right)
    return (cumulative[stops] - cumulative[starts]) / (stops - starts)


def _reference(kind: str, length: float, ds: float = .05):
    if kind == "lowspeed":
        length = 30.0 + 20.0 * math.pi / 2.0 + 50.0
    s = np.arange(0.0, length + ds, ds)
    if kind in ("sine10", "sine15"):
        x = s.copy()
        y = 4.0 * np.sin(2 * math.pi * x / 100.0)
        psi = np.arctan2(np.gradient(y, ds), np.ones_like(y))
        vx = 10.0 if kind == "sine10" else 15.0
        speed = vx * np.sqrt(1 + (4 * 2 * math.pi / 100 * np.cos(2 * math.pi * x / 100)) ** 2)
    elif kind in ("wind07", "wind10"):
        k = (.012 * np.sin(2 * math.pi * s / 180) + .008 * np.sin(2 * math.pi * s / 97 + 1)
             + .006 * np.sin(2 * math.pi * s / 260 + 2))
        psi = np.r_[0.0, np.cumsum((k[:-1] + k[1:]) * .5 * ds)]
        x = np.r_[0.0, np.cumsum(np.cos(psi[:-1]) * ds)]
        y = np.r_[0.0, np.cumsum(np.sin(psi[:-1]) * ds)]
        smooth = _movmean(np.abs(k), round(40 / ds))
        v0 = np.clip(np.sqrt(2.5 / np.maximum(smooth, 1e-4)), 6, 22)
        v0 = _movmean(v0, round(30 / ds))
        speed = (.7 if kind == "wind07" else 1.0) * v0
    elif kind == "lowspeed":
        straight, radius = 30.0, 20.0
        k = np.where((s >= straight) & (s < straight + radius * math.pi / 2.0),
                     -1.0 / radius, 0.0)
        psi = np.r_[0.0, np.cumsum((k[:-1] + k[1:]) * .5 * ds)]
        x = np.r_[0.0, np.cumsum(np.cos(psi[:-1]) * ds)]
        y = np.r_[0.0, np.cumsum(np.sin(psi[:-1]) * ds)]
        speed = np.zeros_like(s)
    else:
        raise ValueError(f"Unsupported reference scenario: {kind}")
    return s, x, y, psi, speed


def _front_wheel_angles(carla, vehicle):
    """Return measured front-wheel steering angles in degrees, when exposed by CARLA."""
    locations = getattr(carla, "VehicleWheelLocation", None)
    getter = getattr(vehicle, "get_wheel_steer_angle", None)
    if locations is None or not callable(getter):
        return float("nan"), float("nan")
    try:
        return (float(getter(locations.FL_Wheel)), float(getter(locations.FR_Wheel)))
    except (AttributeError, RuntimeError):
        return float("nan"), float("nan")


class ResidualModel:
    """Huong 4 online BLR with forgetting and sparse residual GP."""
    def __init__(self, lf, lr, td):
        self.lf, self.lr, self.L, self.td = lf, lr, lf+lr, td
        self.A0=1e-3*np.eye(8); self.A=self.A0.copy(); self.B=np.zeros((8,4)); self.W=np.zeros((8,4))
        self.gx=np.empty((0,3)); self.gy=np.empty((0,4)); self.ga=self.gy.copy(); self.next=0
        self.ell=np.array([.5,.15,1.]); self.clip=np.array([.5,.5,.2,.5]); self.samples=0
    @staticmethod
    def features(z,u):
        v=max(float(z[3]),0); a,d=map(float,u); n=v/10
        return np.array([d,n*d,n*n*d,n**3*d,a,n*a,n,n*n]),np.array([n,d,a])
    def predict(self,z,u):
        f,x=self.features(z,u); y=self.W.T@f
        if len(self.gx): y += np.exp(-.5*np.sum(((self.gx-x)/self.ell)**2,axis=1))@self.ga
        return np.clip(y,-self.clip,self.clip)
    def update(self,z,u,zn):
        if z[3]<.5 or not np.all(np.isfinite(np.r_[z,u,zn])): return
        b=math.atan(self.lr/self.L*math.tan(u[1])); psi=z[2]; v=z[3]
        nom=z+self.td*np.array([v*math.cos(psi+b),v*math.sin(psi+b),v/self.lr*math.sin(b),u[0]])
        e=zn-nom; e[2]=math.atan2(math.sin(e[2]),math.cos(e[2])); c,s=math.cos(psi),math.sin(psi)
        y=np.clip([c*e[0]+s*e[1],-s*e[0]+c*e[1],e[2],e[3]],-self.clip,self.clip)
        f,x=self.features(z,u); self.A=.998*(self.A-self.A0)+self.A0+np.outer(f,f); self.B=.998*self.B+np.outer(f,y); self.W=np.linalg.solve(self.A,self.B)
        gx=self.gx; gy=self.gy
        if len(gx):
            k=np.exp(-.5*np.sum(((gx-x)/self.ell)**2,axis=1)); j=int(np.argmax(k))
        else: k=[]; j=0
        residual=y-self.W.T@f
        if len(gx) and k[j]>=.9: gy[j]=.8*gy[j]+.2*residual
        elif len(gx)<30: self.gx=np.vstack((gx,x)); self.gy=np.vstack((gy,residual))
        else:
            j=self.next; self.gx[j]=x; self.gy[j]=residual; self.next=(j+1)%30
        q=self.gx/self.ell; d=np.maximum(np.sum(q*q,1)[:,None]+np.sum(q*q,1)[None,:]-2*q@q.T,0)
        self.ga=np.linalg.solve(np.exp(-.5*d)+.1*np.eye(len(q)),self.gy); self.samples+=1


class KinematicMPC:
    def __init__(self, lf, lr, steer_limit, ts=.1, td=.2, hp=8, steer_slew_weight=50.,
                 compensation_mode="none", understeer_k=0.0, curvature_feedforward=False):
        self.lf, self.lr, self.L = lf, lr, lf + lr
        self.ts, self.td, self.hp = ts, td, hp
        self.compensation_mode = compensation_mode
        self.understeer_k = float(understeer_k)
        self.curvature_feedforward = curvature_feedforward
        self.delta_max = min(math.atan(math.tan(math.radians(37)) * self.L / lr), steer_limit)
        self.ddelta_max = math.radians(10) * self.L / lr
        self.q = np.array([10., 10., 5., 5.])
        self.r = np.array([.1, 1.])
        # CARLA reacts more slowly than the ideal MATLAB plant. A large
        # steering-slew penalty kept the controller almost straight in CARLA.
        self.rb = np.array([1., float(steer_slew_weight)])
        self.u = np.zeros((hp, 2))
        self.previous = np.zeros(2)
        self.residual = ResidualModel(lf, lr, td)

    def compensation_lambda(self, speed):
        if self.compensation_mode not in ("lambda", "lambda_free"):
            return 0.0
        eps = self.understeer_k * speed * speed / (self.L + self.understeer_k * speed * speed)
        lam = (0.75 * (self.hp * self.td) * eps - self.td) / 2.0
        return max(0.0, lam) if self.compensation_mode == "lambda" else lam

    def _step(self, z, u, dt):
        beta = math.atan(self.lr / self.L * math.tan(u[1]))
        yaw_rate = z[3] / self.lr * math.sin(beta)
        eps = self.understeer_k * z[3] * z[3] / (self.L + self.understeer_k * z[3] * z[3])
        if self.compensation_mode == "leff":
            yaw_rate *= 1.0 - eps
        lam = self.compensation_lambda(z[3])
        heading = z[2] - lam * yaw_rate + beta
        result = z + dt * np.array([z[3] * math.cos(heading), z[3] * math.sin(heading), yaw_rate, u[0]])
        r = self.residual.predict(z,u)*(dt/self.td); c,s=math.cos(z[2]),math.sin(z[2])
        return result + np.array([c*r[0]-s*r[1],s*r[0]+c*r[1],r[2],r[3]])

    def solve(self, z0, ref):
        hp, ts, td = self.hp, self.ts, self.td
        # The reference heading already contains the planned path curvature.
        # Convert it to a steady-state bicycle steering angle, and let MPC
        # optimize a correction around that feed-forward value.
        if self.curvature_feedforward:
            dpsi = np.arctan2(np.sin(np.diff(ref[2])), np.cos(np.diff(ref[2])))
            ds = np.hypot(np.diff(ref[0]), np.diff(ref[1]))
            curvature = np.divide(dpsi, ds, out=np.zeros_like(dpsi), where=ds > 1e-4)
            beta_ff = np.arcsin(np.clip(self.lr * curvature, -.95, .95))
            delta_ff = np.arctan((self.L / self.lr) * np.tan(beta_ff))
            delta_ff = np.clip(delta_ff[:hp], -self.delta_max, self.delta_max)
        else:
            delta_ff = np.zeros(hp)

        guess = np.vstack((self.u[1:], self.u[-1]))
        guess[:, 1] -= delta_ff
        bounds = []
        for i in range(hp):
            bounds.extend(((-1.5, 1.0),
                           (-self.delta_max - delta_ff[i], self.delta_max - delta_ff[i])))
        guess = np.clip(guess.ravel(), [b[0] for b in bounds], [b[1] for b in bounds])
        dt = np.r_[ts, np.full(hp - 1, td)]
        lower = dt[:, None] * np.array([-3., -self.ddelta_max])
        upper = dt[:, None] * np.array([1.5, self.ddelta_max])
        def objective(flat):
            controls = flat.reshape(hp, 2).copy()
            controls[:, 1] += delta_ff
            z, prev, cost = z0.copy(), self.previous.copy(), 0.0
            for i, u in enumerate(controls):
                e = z - ref[:, i]; e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
                du = u - prev
                cost += np.dot(self.q, e * e) + np.dot(self.rb, du * du) + np.dot(self.r, u * u)
                z = self._step(z, u, td); prev = u
            e = z - ref[:, hp]; e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
            return cost + np.dot(self.q, e * e)
        def slew(flat):
            u = flat.reshape(hp, 2).copy()
            u[:, 1] += delta_ff
            prev = np.vstack((self.previous, u[:-1]))
            d = u - prev
            return np.r_[(d - lower).ravel(), (upper - d).ravel()]
        result = minimize(objective, guess, method="SLSQP", bounds=bounds,
                          constraints=[{"type": "ineq", "fun": slew}],
                          options={"maxiter": 100, "ftol": 1e-6, "disp": False})
        feedback = result.x.reshape(hp, 2)
        controls = feedback.copy()
        controls[:, 1] += delta_ff
        self.u, self.previous = controls, controls[0].copy()
        return controls[0], bool(result.success), float(result.fun), float(delta_ff[0])


class _LivePlot:
    """Live path, speed, and tracking-error view for a CARLA MPC run."""
    def __init__(self, ref_x, ref_y):
        import matplotlib.pyplot as plt
        self.plt = plt
        plt.ion()
        self.fig, self.axes = plt.subplots(1, 4, figsize=(19, 4.5), constrained_layout=True)
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
        self.delta, = self.axes[3].plot([], [], color="#1769aa", lw=1.5, label="MPC total")
        self.delta_ff, = self.axes[3].plot([], [], "--", color="#e28a1b", lw=1.2, label="Feed-forward")
        self.delta_measured, = self.axes[3].plot([], [], color="#16803c", lw=1.1, label="Front wheels measured")
        self.axes[3].set(title="Steering angle", xlabel="Time (s)", ylabel="degrees")
        self.axes[3].legend(loc="best"); self.axes[3].grid(True, alpha=.7)
        self.t, self.x, self.y, self.v, self.vref, self.err, self.delta_values, self.ff_values, self.measured_values = (
            [], [], [], [], [], [], [], [], [])
        self.fig.show(); plt.pause(.001)

    def update(self, t, x, y, speed, speed_ref, error, delta_deg, delta_ff_deg, measured_delta_deg):
        self.t.append(t); self.x.append(x); self.y.append(y)
        self.v.append(speed); self.vref.append(speed_ref); self.err.append(error)
        self.delta_values.append(delta_deg); self.ff_values.append(delta_ff_deg)
        self.measured_values.append(measured_delta_deg)
        self.path.set_data(self.x, self.y); self.car.set_data([x], [y])
        self.speed.set_data(self.t, self.v); self.speed_ref.set_data(self.t, self.vref)
        self.error.set_data(self.t, self.err)
        self.delta.set_data(self.t, self.delta_values); self.delta_ff.set_data(self.t, self.ff_values)
        self.delta_measured.set_data(self.t, self.measured_values)
        for ax in self.axes[1:]: ax.relim(); ax.autoscale_view()
        self.fig.suptitle(f"CARLA live kinematic MPC | t={t:.1f} s | speed={speed:.1f} m/s | error={error:.2f} m",
                          fontweight="bold")
        self.fig.canvas.draw_idle(); self.plt.pause(.001)

    def save_close(self, path):
        self.fig.savefig(path, dpi=180, facecolor="white")
        self.plt.close(self.fig)


def run(argv=None):
    p = argparse.ArgumentParser(description="MATLAB huong_4 residual-learning MPC in CARLA 0.9.15")
    p.add_argument("--host", default="127.0.0.1"); p.add_argument("--port", type=int, default=2000)
    p.add_argument("--map-file", type=Path, default=ROOT / "map" / "scene.xodr",
                   help="custom OpenDRIVE map file (defaults to map/scene.xodr)")
    p.add_argument("--road-id", type=int, default=1)
    p.add_argument("--lane-id", type=int, default=-1)
    p.add_argument("--spawn-s", type=float, default=100.0,
                   help="longitudinal position on the selected OpenDRIVE road, in metres")
    p.add_argument("--vehicle", default="vehicle.tesla.model3")
    p.add_argument("--scenario", choices=("lowspeed", "wind07", "wind10", "sine10", "sine15"), default="wind10")
    p.add_argument("--duration", type=float, default=60); p.add_argument("--dt", type=float, default=.01)
    p.add_argument("--steer-slew-weight", type=float, default=50.,
                   help="MPC steering-change penalty; lower values make CARLA tracking more responsive")
    p.add_argument("--compensation-mode", choices=("none",), default="none",
                   help="Huong 4 uses online learned residual compensation")
    p.add_argument("--k-scale", type=float, default=1.0,
                   help="scale understeer coefficient K; robustness sweep: 0.7, 1.0, 1.3")
    p.add_argument("--cornering-stiffness-front", type=float, default=60000.0,
                   help="front per-tire cornering stiffness in N/rad, from matlab/huong_1/params.m")
    p.add_argument("--cornering-stiffness-rear", type=float, default=60000.0,
                   help="rear per-tire cornering stiffness in N/rad, from matlab/huong_1/params.m")
    p.add_argument("--curvature-feedforward", choices=("on", "off"), default="off",
                   help="Huong 4 reference controller has no curvature feed-forward")
    p.add_argument("--out", type=Path, default=Path("ket_qua/mpc_carla"))
    p.add_argument("--timeout", type=float, default=30); p.add_argument("--fast", action="store_true")
    p.add_argument("--plot", action="store_true", help="show live path, speed, and cross-track plots")
    p.add_argument("--camera-mode", choices=("overhead", "follow", "follow-free", "manual"),
                   default="follow-free",
                   help="spectator: start by car then use CARLA controls (default), fixed overhead, locked rear follow, or free-look follow")
    p.add_argument("--camera-height", type=float, default=80.0,
                   help="height above spawn for overhead spectator view, in metres")
    p.add_argument("--no-spectator-follow", action="store_true",
                   help="legacy alias for --camera-mode manual")
    p.add_argument("--no-trail", action="store_true",
                   help="disable the persistent vehicle path drawn in the CARLA world")
    p.add_argument("--video-fps", type=int, default=10,
                   help="FPS for the CARLA camera recording saved with this run")
    args = p.parse_args(argv)
    if args.dt <= 0 or args.duration <= 0: p.error("--dt and --duration must be positive")
    if args.camera_height <= 0: p.error("--camera-height must be positive")
    if args.video_fps <= 0: p.error("--video-fps must be positive")
    if args.k_scale <= 0: p.error("--k-scale must be positive")
    if args.cornering_stiffness_front <= 0 or args.cornering_stiffness_rear <= 0:
        p.error("cornering stiffness values must be positive")
    if args.no_spectator_follow: args.camera_mode = "manual"
    import carla
    args.out.mkdir(parents=True, exist_ok=False)
    client = carla.Client(args.host, args.port); client.set_timeout(args.timeout)
    validate_carla(client)
    map_file = args.map_file.expanduser().resolve()
    if not map_file.is_file():
        raise FileNotFoundError(f"OpenDRIVE map file not found: {map_file}")
    xodr = map_file.read_text(encoding="utf-8")
    world = client.generate_opendrive_world(xodr, carla.OpendriveGenerationParameters(
        vertex_distance=2.0, max_road_length=50.0, wall_height=0.0, additional_width=.6,
        smooth_junctions=True, enable_mesh_visibility=True))
    old_settings = configure_world(world, args.dt)
    actors = []
    video_writer = None
    video_sensor = None
    video_errors = []
    try:
        if args.spawn_s < 0:
            raise ValueError("--spawn-s must be nonnegative")
        spawn = world.get_map().get_waypoint_xodr(args.road_id, args.lane_id, args.spawn_s)
        if spawn is None:
            raise RuntimeError(
                f"No waypoint at road {args.road_id}, lane {args.lane_id}, s={args.spawn_s:g} m in {map_file}"
            )
        spawn = spawn.transform
        spawn.location.z += .5
        if args.camera_mode == "overhead":
            overhead = carla.Transform(
                carla.Location(x=spawn.location.x, y=spawn.location.y,
                               z=spawn.location.z + args.camera_height),
                carla.Rotation(pitch=-90.0, yaw=spawn.rotation.yaw))
            world.get_spectator().set_transform(overhead)
        elif args.camera_mode == "manual":
            # Put the initial spectator near the ego vehicle, then leave it to
            # CARLA's native controls; do not update it as the vehicle moves.
            camera_location = spawn.transform(carla.Location(x=-18.0, z=10.0))
            world.get_spectator().set_transform(carla.Transform(
                camera_location,
                carla.Rotation(pitch=-25.0, yaw=spawn.rotation.yaw)))
        bp = world.get_blueprint_library().find(args.vehicle)
        bp.set_attribute("role_name", "mpc_ego")
        vehicle = world.try_spawn_actor(bp, spawn)
        if vehicle is None: raise RuntimeError("Could not spawn vehicle at selected point")
        actors.append(vehicle)
        import imageio.v2 as imageio
        video_writer = imageio.get_writer(
            args.out / "carla_vehicle_follow.mp4", fps=args.video_fps,
            codec="libx264", quality=7, macro_block_size=16)
        camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "1280")
        camera_bp.set_attribute("image_size_y", "720")
        camera_bp.set_attribute("fov", "90")
        camera_bp.set_attribute("sensor_tick", f"{1.0 / args.video_fps:.6f}")
        video_sensor = world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=-8.0, z=3.5),
                            carla.Rotation(pitch=-12.0)),
            attach_to=vehicle)
        actors.append(video_sensor)

        def record_camera(image):
            try:
                pixels = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4))
                video_writer.append_data(pixels[:, :, :3][:, :, ::-1])
            except Exception as exc:
                video_errors.append(exc)

        video_sensor.listen(record_camera)
        vehicle.apply_control(carla.VehicleControl(brake=1.0))
        # CARLA snapshots do not include a newly spawned actor until the world ticks.
        old_frame = tick_checked(world, None, args.dt, args.timeout)
        physics, physics_data = physics_report(vehicle)
        wheels = list(physics.wheels)
        vehicle_transform = vehicle.get_transform()
        wheelbase, wheelbase_source, _ = wheelbase_from_physics(physics, vehicle_transform)
        # wheel.position is world-space cm; project relative to the vehicle
        # origin into its local forward axis before comparing with local COM.x.
        origin = vehicle_transform.location
        forward = vehicle_transform.get_forward_vector()
        wheel_x_local = [
            ((float(w.position.x) * .01 - float(origin.x)) * float(forward.x)
             + (float(w.position.y) * .01 - float(origin.y)) * float(forward.y)
             + (float(w.position.z) * .01 - float(origin.z)) * float(forward.z))
            for w in wheels
        ]
        front_x = sum(sorted(wheel_x_local, reverse=True)[:2]) / 2.0
        rear_x = sum(sorted(wheel_x_local)[:2]) / 2.0
        com_x = float(physics.center_of_mass.x)
        lf, lr = front_x - com_x, com_x - rear_x
        if not all(math.isfinite(v) and v > 0 for v in (wheelbase, lf, lr)):
            raise RuntimeError(
                f"Invalid vehicle geometry from CARLA: wheelbase={wheelbase:g} m, "
                f"lf={lf:g} m, lr={lr:g} m, COM_x={com_x:g} m"
            )
        if abs((lf + lr) - wheelbase) > max(0.15, 0.08 * wheelbase):
            raise RuntimeError(
                f"Inconsistent CARLA axle/COM geometry: lf+lr={lf + lr:g} m, "
                f"wheelbase={wheelbase:g} m (source={wheelbase_source})"
            )
        steer_limit = math.radians(min(float(w.max_steer_angle) for w in wheels[:2]))
        mass = float(physics.mass)
        understeer_k = mass / wheelbase * (
            lr / (2.0 * args.cornering_stiffness_front)
            - lf / (2.0 * args.cornering_stiffness_rear))
        understeer_k *= args.k_scale
        controller = KinematicMPC(lf, lr, steer_limit,
                                  steer_slew_weight=args.steer_slew_weight,
                                  compensation_mode=args.compensation_mode,
                                  understeer_k=understeer_k,
                                  curvature_feedforward=args.curvature_feedforward == "on")
        ref_length = 800 if args.scenario.startswith("wind") else (290 if args.scenario.startswith("sine") else 112)
        s, rx, ry, rpsi, rv = _reference(args.scenario, ref_length)
        low_speed_times = np.array([0., 3., 6., 20., 24., 30., 33., 38., 41., 60.])
        low_speed_values = np.array([0., 0., 3.5, 3.5, 0., 0., 6., 6., 0., 0.])
        origin = spawn
        yaw = math.radians(origin.rotation.yaw)
        c, sn = math.cos(yaw), math.sin(yaw)
        def global_ref(x, y, psi):
            # CARLA uses +Y to the right; the MPC frame uses +Y to the left.
            # State measurements use Y=-world_y and psi=-CARLA_yaw, so rotate
            # references with that same convention (and preserve local heading).
            return (origin.location.x + c*x + sn*y, -origin.location.y - sn*x + c*y,
                    -yaw + psi)
        live_plot = _LivePlot(*global_ref(rx, ry, rpsi)[:2]) if args.plot else None
        (args.out / "map_used.txt").write_text(
            f"OpenDRIVE: {map_file}\nroad_id={args.road_id}\nlane_id={args.lane_id}\n"
            f"spawn_s_m={args.spawn_s:g}\ncamera_mode={args.camera_mode}\n"
            f"camera_height_m={args.camera_height:g}\nvideo_fps={args.video_fps}\n"
            f"compensation_mode={args.compensation_mode}\nundersteer_K_s2_per_m={understeer_k:.8g}\n"
            f"K_scale={args.k_scale:g}\ncornering_stiffness_front_N_per_rad={args.cornering_stiffness_front:g}\n"
            f"cornering_stiffness_rear_N_per_rad={args.cornering_stiffness_rear:g}\n"
            f"video_file=carla_vehicle_follow.mp4\nsteer_slew_weight={args.steer_slew_weight:g}\n"
            f"wheelbase_m={wheelbase:.6g}\nwheelbase_source={wheelbase_source}\n"
            f"lf_m={lf:.6g}\nlr_m={lr:.6g}\ncom_x_m={com_x:.6g}\n"
            f"steer_limit_deg={math.degrees(steer_limit):.6g}\n"
            f"delta_max_deg={math.degrees(controller.delta_max):.6g}\n"
            f"delta_rate_limit_deg_s={math.degrees(controller.ddelta_max):.6g}\n",
            encoding="utf-8")
        frame = float(world.get_snapshot().timestamp.elapsed_seconds)
        elapsed = 0.; last_control_t = -1e9; progress_idx = 0
        learned_history = []
        prediction_rows = []; actual_states = []
        cross_track_values = []; solve_times = []
        last_u = np.zeros(2); last_delta_ff = 0.; rows = 0; last_wall = time.monotonic()
        last_camera_frame = -10; last_hud_frame = -10
        last_follow_vehicle_location = None
        last_trail_frame = -10; last_trail_location = None
        trail_every_frames = max(1, round(.1 / args.dt))
        columns = ["frame", "t_s", "sample_period_s", "mpc_updated", "mpc_compute_time_s",
                   "understeer_K_s2_per_m", "yaw_delay_lambda_s",
                   "X_m", "Y_m", "psi_rad", "vx_mps", "vy_mps", "speed_mps",
                   "ref_s_m", "ref_x_m", "ref_y_m", "ref_psi_rad", "ref_speed_mps", "accel_cmd_mps2",
                   "delta_cmd_rad", "delta_ff_rad", "delta_feedback_rad", "delta_measured_rad",
                   "steer_fl_deg", "steer_fr_deg", "steer_limit_rad", "steer_norm",
                   "throttle", "brake", "mpc_success", "cost", "cross_track_m"]
        with (args.out / "states_mpc.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns); writer.writeheader()
            while elapsed < args.duration and progress_idx < len(s) - 1:
                snapshot = world.get_snapshot()
                state_actor = snapshot.find(vehicle.id)
                if state_actor is None:
                    raise RuntimeError(
                        f"Spawned vehicle {vehicle.id} is missing from CARLA snapshot at frame {snapshot.frame}"
                    )
                tf = state_actor.get_transform(); vel = state_actor.get_velocity()
                if not args.no_trail and snapshot.frame - last_trail_frame >= trail_every_frames:
                    trail_location = carla.Location(
                        x=tf.location.x, y=tf.location.y, z=tf.location.z + .12)
                    if last_trail_location is not None:
                        world.debug.draw_line(
                            last_trail_location, trail_location, thickness=.16,
                            color=carla.Color(0, 220, 255), life_time=0.0)
                    last_trail_location = trail_location
                    last_trail_frame = snapshot.frame
                if args.camera_mode == "follow" and snapshot.frame - last_camera_frame >= 5:
                    camera_location = tf.transform(carla.Location(x=-14.0, z=7.0))
                    world.get_spectator().set_transform(carla.Transform(
                        camera_location,
                        carla.Rotation(pitch=-22.0, yaw=tf.rotation.yaw)))
                    last_camera_frame = snapshot.frame
                elif args.camera_mode == "follow-free" and snapshot.frame - last_camera_frame >= 5:
                    spectator = world.get_spectator()
                    if last_follow_vehicle_location is None:
                        camera_location = tf.transform(carla.Location(x=-18.0, z=10.0))
                        spectator.set_transform(carla.Transform(
                            camera_location,
                            carla.Rotation(pitch=-25.0, yaw=tf.rotation.yaw)))
                    else:
                        # Follow only vehicle translation. Preserve spectator rotation
                        # so the user can freely look around instead of being yaw-locked.
                        camera_transform = spectator.get_transform()
                        vehicle_delta = tf.location - last_follow_vehicle_location
                        camera_location = camera_transform.location + vehicle_delta
                        spectator.set_transform(carla.Transform(
                            camera_location, camera_transform.rotation))
                    last_follow_vehicle_location = tf.location
                    last_camera_frame = snapshot.frame
                yaw_world = math.radians(tf.rotation.yaw)
                psi = -yaw_world
                vx = vel.x * math.cos(yaw_world) + vel.y * math.sin(yaw_world)
                vy = -(-vel.x * math.sin(yaw_world) + vel.y * math.cos(yaw_world))
                x, y = tf.location.x, -tf.location.y
                yaw_rate = -math.radians(state_actor.get_angular_velocity().z)
                actual_states.append((elapsed, x, y, psi, vx, vy, yaw_rate))
                # This runs at 100 Hz, so bound progress to this frame's
                # physical travel. The MATLAB 30 m window runs only at 10 Hz;
                # applying it every frame let a distant projection jump ahead.
                begin = max(0, progress_idx - 60)
                max_forward = max(1, int(math.ceil(max(vx, 0.) * args.dt / .05)))
                end = min(len(s), progress_idx + max_forward + 1)
                gx, gy, _ = global_ref(rx[begin:end], ry[begin:end], rpsi[begin:end])
                idx = begin + int(np.argmin((gx-x)**2 + (gy-y)**2))
                progress_idx = max(progress_idx, idx)
                refs = np.zeros((4, controller.hp + 1))
                ref_s = float(s[idx])
                for j in range(controller.hp + 1):
                    ri = min(len(s)-1, int(round(ref_s / .05)))
                    qx, qy, qpsi = global_ref(rx[ri], ry[ri], rpsi[ri])
                    target_v = (float(np.interp(elapsed + j * controller.td,
                                                low_speed_times, low_speed_values))
                                if args.scenario == "lowspeed" else float(rv[ri]))
                    refs[:, j] = [qx, qy, qpsi, target_v]
                    ref_s = min(float(s[-1]), ref_s + target_v * controller.td)
                success, cost = True, float("nan")
                mpc_updated = False
                mpc_compute_time = ""
                if elapsed - last_control_t >= controller.ts - 1e-6:
                    solve_started = time.perf_counter()
                    z_now = np.array([x, y, psi, max(vx, 0.)])
                    if len(learned_history) >= 2 and len(learned_history) % 2 == 0:
                        z_prev, u0 = learned_history[-2]
                        _, u1 = learned_history[-1]
                        controller.residual.update(z_prev, (u0+u1)/2, z_now)
                    last_u, success, cost, last_delta_ff = controller.solve(z_now, refs)
                    learned_history.append((z_now.copy(), last_u.copy()))
                    d_now = np.array([x, y, psi, vx, vy, yaw_rate])
                    prediction_rows.extend(forecast_plan(elapsed, z_now, d_now, controller.u,
                                                         controller, mass, lf, lr))
                    mpc_compute_time = time.perf_counter() - solve_started
                    solve_times.append(mpc_compute_time)
                    mpc_updated = True
                    last_control_t = elapsed
                # CARLA's normalized steering has opposite sign to this MATLAB frame.
                steer_norm = max(-1., min(1., -last_u[1] / max(steer_limit, .01)))
                target_speed = refs[3, 0]
                accel = last_u[0] + 1.4 * (target_speed - vx)
                throttle = max(0., min(.65, accel / 3.0))
                brake = max(0., min(.8, -accel / 6.0))
                vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=steer_norm))
                steer_fl_deg, steer_fr_deg = _front_wheel_angles(carla, vehicle)
                delta_measured = -math.radians((steer_fl_deg + steer_fr_deg) / 2.)
                ref_x, ref_y, ref_psi = refs[:3, 0]
                cross_track = math.hypot(x-ref_x, y-ref_y)
                cross_track_values.append(cross_track)
                writer.writerow({"frame": snapshot.frame, "t_s": elapsed,
                    "sample_period_s": args.dt, "mpc_updated": int(mpc_updated),
                    "mpc_compute_time_s": mpc_compute_time,
                    "understeer_K_s2_per_m": understeer_k,
                    "yaw_delay_lambda_s": controller.compensation_lambda(max(vx, 0.)),
                    "X_m": x, "Y_m": y,
                    "psi_rad": psi, "vx_mps": vx, "vy_mps": vy, "speed_mps": math.hypot(vx, vy),
                    "ref_s_m": s[idx], "ref_x_m": ref_x, "ref_y_m": ref_y, "ref_psi_rad": ref_psi,
                    "ref_speed_mps": target_speed, "accel_cmd_mps2": last_u[0], "delta_cmd_rad": last_u[1],
                    "delta_ff_rad": last_delta_ff, "delta_feedback_rad": last_u[1] - last_delta_ff,
                    "delta_measured_rad": delta_measured,
                    "steer_fl_deg": steer_fl_deg, "steer_fr_deg": steer_fr_deg,
                    "steer_limit_rad": steer_limit,
                    "steer_norm": steer_norm, "throttle": throttle, "brake": brake, "mpc_success": int(success),
                    "cost": cost, "cross_track_m": cross_track})
                if live_plot is not None and (rows % max(1, round(controller.ts/args.dt)) == 0):
                    live_plot.update(elapsed, x, y, math.hypot(vx, vy), target_speed,
                                     cross_track, math.degrees(last_u[1]),
                                     math.degrees(last_delta_ff), math.degrees(delta_measured))
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
                if video_errors:
                    raise RuntimeError(f"CARLA video recording failed: {video_errors[0]}")
                if not args.fast:
                    delay = args.dt - (time.monotonic() - last_wall)
                    if delay > 0: time.sleep(delay)
                    last_wall = time.monotonic()
                if rows % 100 == 0: stream.flush()
        write_prediction_errors(args.out / "mpc_prediction_errors.csv", prediction_rows, actual_states)
        status = "path_complete" if progress_idx >= len(s) - 1 else "duration_complete"
        metric_values = np.asarray(cross_track_values, dtype=float)
        metrics = {
            "scenario": args.scenario,
            "compensation_mode": args.compensation_mode,
            "curvature_feedforward": args.curvature_feedforward,
            "understeer_K_s2_per_m": understeer_k,
            "k_scale": args.k_scale,
            "samples": len(metric_values),
            "cross_track_mean_m": float(np.mean(metric_values)) if metric_values.size else float("nan"),
            "cross_track_std_m": float(np.std(metric_values)) if metric_values.size else float("nan"),
            "cross_track_max_m": float(np.max(metric_values)) if metric_values.size else float("nan"),
            "mpc_solve_mean_s": float(np.mean(solve_times)) if solve_times else float("nan"),
            "mpc_solve_max_s": float(np.max(solve_times)) if solve_times else float("nan"),
        }
        with (args.out / "metrics.csv").open("w", newline="", encoding="utf-8") as metric_stream:
            metric_writer = csv.DictWriter(metric_stream, fieldnames=list(metrics))
            metric_writer.writeheader(); metric_writer.writerow(metrics)
        if video_sensor is not None:
            video_sensor.stop()
        if video_writer is not None:
            video_writer.close()
            video_writer = None
        (args.out / "run_status.txt").write_text(
            f"{status}; rows={rows}; simulated_s={elapsed:.3f}; "
            f"cross_track_mean={metrics['cross_track_mean_m']:.3f} m; "
            f"cross_track_max={metrics['cross_track_max_m']:.3f} m\n", encoding="utf-8")
        if live_plot is not None:
            live_plot.save_close(args.out / "mpc_live_plot.png")
        print(f"MPC CARLA complete: {rows} samples in {args.out}")
        return 0
    finally:
        for actor in reversed(actors):
            if actor.is_alive:
                if actor.type_id.startswith("sensor."):
                    try: actor.stop()
                    except Exception: pass
                actor.destroy()
        if video_writer is not None:
            video_writer.close()
        world.apply_settings(old_settings)


if __name__ == "__main__":
    raise SystemExit(run())
