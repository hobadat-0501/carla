"""Realtime CARLA 0.9.15: environment + vehicle + telemetry + online shadow model.

Everything is read from the running simulator:
  * vehicle parameters   : get_physics_control()
  * body states          : world snapshot (pose, velocity, acceleration, yaw rate)
  * wheel angle          : get_wheel_steer_angle()
  * tire load/force/slip : Vehicle.get_telemetry()  (per wheel, engine values)
The linear 3-DOF model of mo_hinh_xe.py runs ONLINE beside CARLA. Its parameters
are identified from the live telemetry (no tham_so.json, no Python-only
simulation of the vehicle). Longitudinal motion is conditioned on measured ax,
so the comparison is a lateral/yaw study only.

Modes
  pad    : replaces the world with a flat pad, spawns a vehicle, drives a scripted
           speed hold + J-turn/double-pulse in synchronous mode paced to wall clock.
  attach : read-only; attaches to an existing vehicle (e.g. manual_control.py,
           role_name 'hero'). Never ticks and never changes world settings.

No ISO conformity, real-vehicle validation or tire-force truth is claimed. The
telemetry units/sign are checked by the script (see report) but not assumed.
"""
import argparse
import csv
import math
import queue
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from carla_runtime import (ObservationClock, checked_tick, cleanup, cli_entry, run_record,
                           save_json, snapshot_state, validate_connection, validate_physics)
from buoc1_carla import body_components, clamp, equivalent_steer, wheelbase_from_geometry
from kiem_thu_maneuver import cutoff, road_xml, steering_profile
from mo_hinh_xe import dynamic_rhs, validate_parameters

ROOT = Path(__file__).resolve().parent
WHEELS = ['FL', 'FR', 'RL', 'RR']
WHEEL_FIELDS = ['tire_friction', 'lat_slip', 'long_slip', 'omega', 'tire_load',
                'normalized_tire_load', 'torque', 'long_force', 'lat_force']
BASE_KEYS = ['t_sim_s', 'X_m', 'Y_m', 'psi_rad', 'vx_mps', 'vy_mps', 'r_radps', 'ax_mps2',
             'ay_mps2', 'delta_equiv_rad']
MODEL_KEYS = ['model_active', 'model_age_s', 'model_X_m', 'model_Y_m', 'model_psi_rad',
              'model_vx_mps', 'model_vy_mps', 'model_r_radps',
              'err_vy_mps', 'err_r_radps', 'err_psi_rad', 'err_pos_m']
NAN = float('nan')


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- CARLA reads
def fields(obj, names):
    out = {}
    for n in names:
        v = getattr(obj, n, None)
        if v is None:
            continue
        if hasattr(v, 'x') and hasattr(v, 'y'):
            v = [float(v.x), float(v.y)] + ([float(v.z)] if hasattr(v, 'z') else [])
        elif isinstance(v, float) and not math.isfinite(v):
            v = None
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            v = float(v)
        out[n] = v
    return out


def physics_report(ego, wheelbase_override):
    ph = ego.get_physics_control()
    validate_physics(ph)
    rep = fields(ph, ['mass', 'drag_coefficient', 'center_of_mass', 'max_rpm', 'moi', 'final_ratio',
                      'gear_switch_time', 'clutch_strength', 'use_gear_autobox'])
    rep['torque_curve'] = [[float(p.x), float(p.y)] for p in ph.torque_curve]
    rep['steering_curve'] = [[float(p.x), float(p.y)] for p in ph.steering_curve]
    rep['wheels'] = []
    for name, w in zip(WHEELS, ph.wheels):
        d = fields(w, ['tire_friction', 'damping_rate', 'max_steer_angle', 'radius', 'max_brake_torque',
                       'max_handbrake_torque', 'lat_stiff_max_load', 'lat_stiff_value',
                       'long_stiff_value', 'position'])
        d['wheel'] = name
        rep['wheels'].append(d)
    ext = ego.bounding_box.extent
    rep['bounding_box_extent_m'] = [float(ext.x), float(ext.y), float(ext.z)]
    L, source, details = wheelbase_from_geometry(ph, 2 * ext.x, wheelbase_override)
    rep.update(wheelbase_m=float(L), wheelbase_source=source,
               wheelbase_details=details if isinstance(details, (dict, list, str, int, float)) else str(details))
    return ph, rep


def refresh_params(ego):
    ph = ego.get_physics_control()
    fr = [float(w.tire_friction) for w in ph.wheels]
    return float(ph.mass), sum(fr) / len(fr)


def enable_telemetry(ego):
    try:
        ego.enable_telemetry(True)
    except AttributeError as exc:
        raise RuntimeError('This CARLA client has no Vehicle.enable_telemetry; check '
                           "[n for n in dir(vehicle) if 'telem' in n.lower()]. Use --no-telemetry "
                           'to log states only.') from exc


def read_telemetry(ego):
    out = {f'{w}_{k}': NAN for w in WHEELS for k in WHEEL_FIELDS}
    tel = ego.get_telemetry()
    if len(tel.wheels) == 4:
        for name, w in zip(WHEELS, tel.wheels):
            for k in WHEEL_FIELDS:
                out[f'{name}_{k}'] = float(getattr(w, k, NAN))
    return out


def read_state(ego, world, carla, mass, mu, use_tel):
    snapshot, s = snapshot_state(ego, world)
    tr = s.get_transform()
    v = s.get_velocity()
    a = s.get_acceleration()
    w = s.get_angular_velocity()
    yaw = math.radians(tr.rotation.yaw)
    vx, vy = body_components(v.x, v.y, yaw)
    ax, ay = body_components(a.x, a.y, yaw)
    fl = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    fr = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    delta, _ = equivalent_steer(fl, fr)
    c = ego.get_control()
    speed = math.hypot(vx, vy)
    row = {'frame': snapshot.frame, 't_sim_s': snapshot.timestamp.elapsed_seconds,
           'X_m': tr.location.x, 'Y_m': -tr.location.y, 'Z_m': tr.location.z,
           'psi_rad': -yaw, 'roll_deg': tr.rotation.roll, 'pitch_deg': tr.rotation.pitch,
           'vx_mps': vx, 'vy_mps': vy, 'speed_kmh': 3.6 * speed, 'r_radps': -math.radians(w.z),
           'ax_mps2': ax, 'ay_mps2': ay, 'beta_rad': math.atan2(vy, vx) if speed > .5 else 0.0,
           'delta_equiv_rad': delta, 'steer_fl_deg': fl, 'steer_fr_deg': fr,
           'throttle': c.throttle, 'brake': c.brake, 'steer_norm': c.steer, 'gear': c.gear,
           'mass_kg': mass, 'tire_friction_mean': mu,
           'Fx_total_est_N': mass * ax, 'Fy_total_est_N': mass * ay}
    if not all(math.isfinite(row[k]) for k in BASE_KEYS):
        raise ValueError('Nonfinite CARLA state at frame %s' % row['frame'])
    row.update(read_telemetry(ego) if use_tel else {f'{x}_{k}': NAN for x in WHEELS for k in WHEEL_FIELDS})
    row['Fy_front_tel_N'] = row['FL_lat_force'] + row['FR_lat_force']
    row['Fy_rear_tel_N'] = row['RL_lat_force'] + row['RR_lat_force']
    row['front_slip_tel'] = .5 * (row['FL_lat_slip'] + row['FR_lat_slip'])
    row['rear_slip_tel'] = .5 * (row['RL_lat_slip'] + row['RR_lat_slip'])
    return row


def find_ego(world, args):
    deadline = time.monotonic() + args.wait
    announced = False
    while True:
        if args.actor_id is not None:
            actor = world.get_actor(args.actor_id)
            if actor is not None and actor.type_id.startswith('vehicle.'):
                return actor
        else:
            for actor in world.get_actors().filter('vehicle.*'):
                if actor.attributes.get('role_name') == args.role_name:
                    return actor
        if time.monotonic() >= deadline:
            raise RuntimeError('Khong tim thay xe (role_name=%s). Chay manual_control.py truoc '
                               'hoac dung --mode pad.' % args.role_name)
        if not announced:
            print('Dang cho xe xuat hien trong CARLA...', flush=True)
            announced = True
        time.sleep(.5)


# ------------------------------------------------------- online identification
class OnlineIdentification:
    """Parameters of mo_hinh_xe.py from live CARLA telemetry. No fixed file.

    lf_fraction : static load split (near-static samples, |ax|,|ay|<0.3).
    cf, cr      : zero-intercept regression of telemetry lateral force on slip
                  angles computed from measured states (vx>3 m/s). Sign taken
                  from the data, never assumed.
    Iz          : regression of the yaw equation with telemetry axle forces;
                  falls back to a rectangular-box assumption (flagged).
    """
    def __init__(self, L, ext, g=9.81):
        self.L, self.g = L, g
        self.box_iz_over_m = ((2 * ext[0]) ** 2 + (2 * ext[1]) ** 2) / 12.0
        self.ff_sum = 0.0
        self.ff_n = 0
        self.load_over_weight = []
        self.f = [0.0, 0.0, 0]
        self.r = [0.0, 0.0, 0]
        self.iz = [0.0, 0.0, 0]
        self.reason = 'cho_tai_tinh'

    @property
    def lf_fraction(self):
        return 1.0 - self.ff_sum / self.ff_n if self.ff_n >= 100 else None

    def update(self, row, prev):
        ax, ay = row['ax_mps2'], row['ay_mps2']
        loads = [row[f'{w}_tire_load'] for w in WHEELS]
        if not all(math.isfinite(x) for x in loads + [row['FL_lat_force'], row['RL_lat_force']]):
            return
        total = sum(loads)
        if abs(ax) < .3 and abs(ay) < .3 and total > 0:
            self.ff_sum += (loads[0] + loads[1]) / total
            self.ff_n += 1
            self.load_over_weight.append(total / (row['mass_kg'] * self.g))
        lf_frac = self.lf_fraction
        if lf_frac is None or row['vx_mps'] < 3.0:
            return
        lf = lf_frac * self.L
        lr = self.L - lf
        vx, vy, r, d = row['vx_mps'], row['vy_mps'], row['r_radps'], row['delta_equiv_rad']
        af = d - math.atan2(vy + lf * r, vx)
        ar = -math.atan2(vy - lr * r, vx)
        ff, fr = row['Fy_front_tel_N'], row['Fy_rear_tel_N']
        if abs(af) > .005:
            self.f[0] += ff * af
            self.f[1] += af * af
            self.f[2] += 1
        if abs(ar) > .005:
            self.r[0] += fr * ar
            self.r[1] += ar * ar
            self.r[2] += 1
        if prev is not None:
            dt = row['t_sim_s'] - prev['t_sim_s']
            if dt > 0:
                rdot = (r - prev['r_radps']) / dt
                m_raw = (lf * ff - lr * fr) / row['mass_kg']
                self.iz[0] += m_raw * rdot
                self.iz[1] += rdot * rdot
                self.iz[2] += 1

    def parameters(self, mass):
        lf_frac = self.lf_fraction
        if lf_frac is None:
            self.reason = 'cho_tai_tinh (xe gan dung yen/gia toc nho)'
            return None
        if self.f[2] < 200 or self.r[2] < 200 or self.f[1] <= 0 or self.r[1] <= 0:
            self.reason = 'cho_kich_thich_ngang (lai khi vx>3 m/s)'
            return None
        kf, kr = self.f[0] / self.f[1], self.r[0] / self.r[1]
        if kf * kr <= 0:
            self.reason = 'dau_luc_ngang_truc_truoc_sau_khong_nhat_quan'
            return None
        sign = 1.0 if kf > 0 else -1.0
        iz, iz_source = self.box_iz_over_m, 'rectangular_box_assumption'
        if self.iz[2] >= 200 and self.iz[1] > 1e-3:
            candidate = sign * self.iz[0] / self.iz[1]
            if math.isfinite(candidate) and .2 < candidate < 10:
                iz, iz_source = candidate, 'yaw_equation_regression'
        p = {'wheelbase_m': self.L, 'mass_kg': mass, 'low_speed_end_mps': .8,
             'longitudinal': dict(traction_gain=0., brake_gain=0., rolling_accel=0., linear_drag=0.),
             'lateral': dict(cf_over_mass=abs(kf) / mass, cr_over_mass=abs(kr) / mass,
                             lf_fraction=lf_frac, iz_over_mass=iz)}
        try:
            validate_parameters(p)
        except (ValueError, KeyError) as exc:
            self.reason = 'tham_so_khong_hop_le: ' + str(exc)
            return None
        p['_sources'] = {'lf_fraction': 'telemetry static load split',
                         'cf_cr': 'telemetry lat_force regression', 'iz_over_mass': iz_source,
                         'lat_force_sign_vs_body_y': sign, 'samples_front': self.f[2],
                         'samples_rear': self.r[2], 'samples_static_load': self.ff_n}
        self.reason = 'san_sang'
        return p


# ----------------------------------------------------------------- shadow model
class ShadowModel:
    """Linear model started from measured CARLA state; replays measured delta and ax.

    Parameters are frozen at each re-initialisation (every reset_s of sim time),
    so errors are horizon-bounded prediction errors, not drifting open-loop runs.
    """
    def __init__(self, reset_s, max_step=.002):
        self.reset_s, self.max_step = reset_s, max_step
        self.state = None
        self.p = None
        self.t_reset = 0.0
        self.resets = 0
        self.failures = 0
        self.errors = {'vy': [], 'r': [], 'psi': [], 'pos': []}

    def _rhs(self, s, delta, ax):
        out = dynamic_rhs(s, 0.0, 0.0, delta, self.p)
        out[3] = ax + s[5] * s[4]  # measured ax conditions the longitudinal channel
        return out

    def _advance(self, prev, row):
        dt = row['t_sim_s'] - prev['t_sim_s']
        if dt <= 0:
            raise ValueError('nonpositive dt')
        n = math.ceil(dt / self.max_step)
        h = dt / n
        s = self.state.copy()

        def f(x, u):
            d = prev['delta_equiv_rad'] + (row['delta_equiv_rad'] - prev['delta_equiv_rad']) * u
            a = prev['ax_mps2'] + (row['ax_mps2'] - prev['ax_mps2']) * u
            return self._rhs(x, d, a)
        for j in range(n):
            u, m, e = j / n, (j + .5) / n, (j + 1) / n
            k1 = f(s, u)
            k2 = f(s + .5 * h * k1, m)
            k3 = f(s + .5 * h * k2, m)
            k4 = f(s + h * k3, e)
            s = s + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
            if not np.all(np.isfinite(s)) or np.linalg.norm(s[3:]) > 300:
                raise FloatingPointError('model divergence')
            if s[3] < .8:
                raise ValueError('model left forward-driving domain')
        return s

    def step(self, row, prev, params):
        out = {k: NAN for k in MODEL_KEYS}
        out['model_active'] = 0
        reset = self.state is None or row['t_sim_s'] - self.t_reset >= self.reset_s
        if not reset and prev is not None:
            try:
                self.state = self._advance(prev, row)
            except (FloatingPointError, ValueError):
                self.failures += 1
                reset = True
        if reset:
            if params is None or row['vx_mps'] < 1.0:
                self.state = None
                return out
            self.p = params
            self.state = np.array([row['X_m'], row['Y_m'], row['psi_rad'], row['vx_mps'],
                                   row['vy_mps'], row['r_radps']], dtype=float)
            self.t_reset = row['t_sim_s']
            self.resets += 1
        s = self.state
        age = row['t_sim_s'] - self.t_reset
        e_psi = wrap(s[2] - row['psi_rad'])
        e_pos = math.hypot(s[0] - row['X_m'], s[1] - row['Y_m'])
        out.update(model_active=1, model_age_s=age, model_X_m=s[0], model_Y_m=s[1], model_psi_rad=s[2],
                   model_vx_mps=s[3], model_vy_mps=s[4], model_r_radps=s[5],
                   err_vy_mps=s[4] - row['vy_mps'], err_r_radps=s[5] - row['r_radps'],
                   err_psi_rad=e_psi, err_pos_m=e_pos)
        if age > 1e-9:
            self.errors['vy'].append(out['err_vy_mps'])
            self.errors['r'].append(out['err_r_radps'])
            self.errors['psi'].append(e_psi)
            self.errors['pos'].append(e_pos)
        return out

    def summary(self):
        def rmse(x):
            return float(np.sqrt(np.mean(np.square(x)))) if x else None
        return {'resets': self.resets, 'divergence_or_domain_failures': self.failures,
                'samples_scored': len(self.errors['vy']),
                'rmse_vy_mps': rmse(self.errors['vy']), 'rmse_r_radps': rmse(self.errors['r']),
                'rmse_psi_rad': rmse(self.errors['psi']), 'rmse_pos_m': rmse(self.errors['pos']),
                'scope': 'Lateral/yaw only; vx is conditioned on measured ax. Horizon = reset_s.'}


# -------------------------------------------------------------------- driving
class Driver:
    """pad mode only: settle -> speed warm-up -> maneuver -> cruise (-> repeat)."""
    def __init__(self, args, maxsteer_deg):
        self.a = args
        self.target = args.speed_kmh / 3.6
        self.range = math.radians(maxsteer_deg)
        self.phase = 'settle'
        self.t_start = None
        self.t_phase = None
        self.stable = 0
        self.integral = 0.0
        self.steer_int = 0.0
        self.direction = args.direction
        self.warm_t0 = None

    def control(self, row, dt):
        t = row['t_sim_s']
        if self.t_start is None:
            self.t_start = t
        if self.phase == 'settle':
            if t - self.t_start < 2.0:
                return 0.0, 1.0, 0.0
            self.phase, self.warm_t0 = 'warmup', t
        err = self.target - row['vx_mps']
        self.integral = clamp(self.integral + err * dt, -3, 3)
        effort = .25 * err + .08 * self.integral
        throttle, brake = clamp(effort, 0, .6), clamp(-effort, 0, .3)
        desired = 0.0
        if self.phase == 'warmup':
            self.stable = self.stable + 1 if abs(err) < .5 / 3.6 else 0
            if self.stable >= math.ceil(1 / dt):
                self.phase, self.t_phase = 'maneuver', t
            elif t - self.warm_t0 > 40:
                raise RuntimeError('Entry speed did not settle within 40 s / pad limits')
        if self.phase == 'maneuver':
            tm = t - self.t_phase
            if self.a.maneuver != 'none':
                desired = steering_profile(tm, self.a.maneuver, math.radians(self.a.delta_deg), self.direction)
            if tm >= 6.0:
                self.phase, self.t_phase = 'cruise', t
        elif self.phase == 'cruise' and self.a.repeat_s > 0 and t - self.t_phase >= self.a.repeat_s \
                and self.a.maneuver != 'none':
            self.direction *= -1
            self.phase, self.t_phase = 'maneuver', t
        e = desired - row['delta_equiv_rad']
        self.steer_int = clamp(self.steer_int + e * dt, -.1, .1)
        steer = -clamp((desired + .25 * e + .8 * self.steer_int) / self.range, -1, 1)
        return throttle, brake, steer


# ------------------------------------------------------------------ live plot
class LivePlot:
    PANELS = [('speed (km/h)', ['speed_kmh']), ('vy CARLA / model (m/s)', ['vy_mps', 'model_vy_mps']),
              ('yaw rate CARLA / model (rad/s)', ['r_radps', 'model_r_radps']),
              ('wheel angle (rad)', ['delta_equiv_rad']),
              ('Fy telemetry front/rear (N)', ['Fy_front_tel_N', 'Fy_rear_tel_N']),
              ('slip telemetry front/rear', ['front_slip_tel', 'rear_slip_tel'])]

    def __init__(self, window_s):
        import matplotlib.pyplot as plt
        self.plt = plt
        plt.ion()
        self.fig, axes = plt.subplots(3, 2, figsize=(11, 8))
        self.axes = list(axes.flat)
        self.window = window_s
        self.buf = {}
        self.fig.suptitle('CARLA realtime - telemetry va mo hinh truc tuyen')

    def update(self, row):
        for k in {'t_sim_s'} | {k for _, ks in self.PANELS for k in ks}:
            self.buf.setdefault(k, []).append(row[k])
        t = self.buf['t_sim_s']
        while t and t[-1] - t[0] > self.window:
            for series in self.buf.values():
                series.pop(0)
        for ax, (title, keys) in zip(self.axes, self.PANELS):
            ax.clear()
            for k in keys:
                ax.plot(t, self.buf[k], label=k)
            ax.set_title(title, fontsize=9)
            ax.grid(alpha=.3)
            if len(keys) > 1:
                ax.legend(fontsize=7)
        self.fig.tight_layout(rect=(0, 0, 1, .95))
        self.plt.pause(.001)


# ----------------------------------------------------------------------- main
def _run(args, status):
    import carla
    world = None
    old = None
    owned = []
    events = queue.Queue()
    try:
        status['phase'] = 'connecting'
        client = carla.Client(args.host, args.port)
        client.set_timeout(args.timeout)
        versions = validate_connection(client)
        dt = args.dt
        clock = None
        if args.mode == 'pad':
            print('Tao duong phang 1200 x 200 m; the gioi hien tai se bi thay the.', flush=True)
            xml = road_xml()
            gen = carla.OpendriveGenerationParameters(vertex_distance=2.0, max_road_length=50.0,
                                                      wall_height=0.0, additional_width=.6,
                                                      smooth_junctions=True, enable_mesh_visibility=True)
            status['phase'] = 'creating_world'
            world = client.generate_opendrive_world(xml, gen)
            old = world.get_settings()
            s = world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = dt
            s.substepping = True
            s.max_substep_delta_time = .005
            s.max_substeps = 10
            world.apply_settings(s)
            world.set_weather(carla.WeatherParameters.WetNoon if args.rain_visual
                              else carla.WeatherParameters.ClearNoon)
            (args.out / 'test_road.xodr').write_text(xml, encoding='utf-8')
            wp = world.get_map().get_waypoint_xodr(1, -1, 100.0)
            if wp is None:
                raise RuntimeError('Generated waypoint unavailable')
            spawn = wp.transform
            spawn.location.y = 0.0
            spawn.location.z += .5
            bp = world.get_blueprint_library().find(args.vehicle)
            bp.set_attribute('role_name', 'realtime_validation')
            ego = world.try_spawn_actor(bp, spawn)
            if ego is None:
                raise RuntimeError('Cannot spawn vehicle asset on generated pad.')
            owned.append(ego)
            ego.set_autopilot(False)
            if args.tire_friction is not None:
                ph = ego.get_physics_control()
                validate_physics(ph)
                wheels = list(ph.wheels)
                for w in wheels:
                    w.tire_friction = args.tire_friction
                ph.wheels = wheels
                ego.apply_physics_control(ph)
            sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),
                                       carla.Transform(), attach_to=ego)
            owned.append(sensor)
            sensor.listen(lambda e: events.put({'frame': e.frame, 'other_actor_id': e.other_actor.id}))
            ego.apply_control(carla.VehicleControl(brake=1.0))
            clock = ObservationClock(not args.fast)
        else:
            world = client.get_world()
            ego = find_ego(world, args)
        status.update(vehicle_actor_id=ego.id, vehicle_type=ego.type_id, mode=args.mode)
        print(f'Xe: actor={ego.id} {ego.type_id}', flush=True)

        use_tel = not args.no_telemetry
        if use_tel:
            enable_telemetry(ego)
        if args.mode == 'pad':  # let suspension/telemetry initialise before reading physics
            for _ in range(5):
                checked_tick(world, None, args.timeout, dt)
        physics, report = physics_report(ego, args.wheelbase)
        L = report['wheelbase_m']
        if args.tire_friction is not None:
            readback = [float(w.tire_friction) for w in physics.wheels]
            if any(abs(x - args.tire_friction) > 1e-4 for x in readback):
                raise RuntimeError('Friction scalar read-back mismatch')
        report.update(client_version=versions[0], server_version=versions[1],
                      map_name=world.get_map().name, mode=args.mode, telemetry_used=use_tel,
                      data_source='CARLA simulator state read live; not a physical vehicle experiment',
                      mu_note='tire_friction is a CARLA scalar, not a verified effective mu')
        save_json(args.out / 'tham_so_carla.json', report)
        mass, mu = refresh_params(ego)
        ident = OnlineIdentification(L, report['bounding_box_extent_m']) if use_tel else None
        shadow = ShadowModel(args.reset_s) if use_tel else None
        maxsteer = max(w.max_steer_angle for w in list(physics.wheels)[:2])
        driver = Driver(args, maxsteer) if args.mode == 'pad' else None
        plot = LivePlot(args.plot_window) if args.plot else None
        print(f'Khoi luong={mass:.0f} kg; co so={L:.3f} m; telemetry={"bat" if use_tel else "tat"}', flush=True)

        status['phase'] = 'realtime'
        status['result'] = 'duration_complete'
        print('Dang chay thoi gian thuc. Ctrl+C de dung.', flush=True)
        next_refresh = time.monotonic() + args.param_refresh
        last_print = 0.0
        prev = None
        t0 = None
        gaps = 0
        params = None
        ready_announced = False
        writer = None
        with (args.out / 'states_realtime.csv').open('w', newline='', encoding='utf-8') as f:
            try:
                while True:
                    if args.mode == 'attach':
                        world.wait_for_tick(args.timeout)
                    if time.monotonic() >= next_refresh:
                        mass, mu = refresh_params(ego)
                        next_refresh = time.monotonic() + args.param_refresh
                    row = read_state(ego, world, carla, mass, mu, use_tel)
                    if prev is not None and row['frame'] == prev['frame']:
                        continue
                    if t0 is None:
                        t0 = row['t_sim_s']
                    row['t_s'] = row['t_sim_s'] - t0
                    row['phase'] = driver.phase if driver else 'attach'
                    if prev is not None and row['frame'] != prev['frame'] + 1:
                        gaps += 1
                    if use_tel:
                        ident.update(row, prev)
                        params = ident.parameters(mass)
                        if params is not None and not ready_announced:
                            ready_announced = True
                            print('Tham so mo hinh da nhan dang tu telemetry: '
                                  f'cf/m={params["lateral"]["cf_over_mass"]:.2f} '
                                  f'cr/m={params["lateral"]["cr_over_mass"]:.2f} '
                                  f'lf_frac={params["lateral"]["lf_fraction"]:.3f} '
                                  f'iz/m={params["lateral"]["iz_over_mass"]:.2f} '
                                  f'({params["_sources"]["iz_over_mass"]})', flush=True)
                        row.update(shadow.step(row, prev, params))
                    else:
                        row.update({k: NAN for k in MODEL_KEYS})
                    reason = None
                    if args.mode == 'pad':
                        reason = cutoff(row)
                        if not events.empty():
                            reason = 'cutoff_collision_event'
                    if writer is None:
                        writer = csv.DictWriter(f, fieldnames=list(row))
                        writer.writeheader()
                    writer.writerow(row)
                    status['rows'] += 1
                    if status['rows'] % 20 == 0:
                        f.flush()
                    now = time.monotonic()
                    if now - last_print >= 1.0 / args.print_hz:
                        last_print = now
                        tail = (f' | mo hinh err vy={row["err_vy_mps"]:+.3f} r={row["err_r_radps"]:+.4f}'
                                if row['model_active'] else
                                f' | mo hinh: {ident.reason if use_tel else "tat"}')
                        print(f'[{row["phase"]:8s}] t={row["t_s"]:6.2f}s v={row["speed_kmh"]:5.1f}km/h '
                              f'ax={row["ax_mps2"]:5.2f} ay={row["ay_mps2"]:5.2f} '
                              f'r={math.degrees(row["r_radps"]):6.1f}deg/s '
                              f'delta={math.degrees(row["delta_equiv_rad"]):5.2f}deg{tail}', flush=True)
                        if plot is not None:
                            plot.update(row)
                    if reason:
                        status['result'] = reason
                        break
                    if args.duration > 0 and row['t_s'] >= args.duration:
                        break
                    prev = row
                    if args.mode == 'pad':
                        thr, brk, steer = driver.control(row, dt)
                        ego.apply_control(carla.VehicleControl(throttle=thr, brake=brk, steer=steer))
                        clock.update(world, ego, carla)
                        checked_tick(world, row['frame'], args.timeout, dt)
            except KeyboardInterrupt:
                status['result'] = 'stopped_by_user'
            finally:
                f.flush()
        status['frame_gaps'] = gaps
        if use_tel:
            lw = ident.load_over_weight
            final = ident.parameters(mass)
            save_json(args.out / 'mo_hinh_truc_tuyen.json', {
                'identified_parameters': final,
                'identification_status': ident.reason,
                'static_tire_load_over_weight_mean': float(np.mean(lw)) if lw else None,
                'static_load_note': 'Close to 1 means telemetry tire_load is in newtons; otherwise check units.',
                'lat_force_vs_m_ay': 'Compare Fy_front_tel_N+Fy_rear_tel_N with Fy_total_est_N in states_realtime.csv.',
                'shadow_model': shadow.summary(),
                'assumptions': ['Linear tires; vx conditioned on measured ax; frozen parameters per reset window.',
                                'Iz may be a rectangular-box assumption: see _sources.iz_over_mass.',
                                'No ISO conformity or real-vehicle validation.']})
    finally:
        if world is not None:
            cleanup(world, old, owned, events, args.out, status, carla, keep_vehicle=args.keep_vehicle)


def run(args):
    with run_record(args.out) as status:
        _run(args, status)
    print(f'{args.out}: {status["result"]}; {status["rows"]} mau', flush=True)
    return 0 if status['result'] in ('duration_complete', 'stopped_by_user') else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=['pad', 'attach'], default='pad')
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--timeout', type=float, default=30.0)
    p.add_argument('--vehicle', default='vehicle.tesla.model3')
    p.add_argument('--wheelbase', type=float)
    p.add_argument('--role-name', default='hero', help='attach mode')
    p.add_argument('--actor-id', type=int, help='attach mode')
    p.add_argument('--wait', type=float, default=60.0, help='attach mode: seconds to wait for the vehicle')
    p.add_argument('--maneuver', choices=['jturn', 'double_pulse', 'none'], default='double_pulse')
    p.add_argument('--speed-kmh', type=float, default=30.0)
    p.add_argument('--delta-deg', type=float, default=2.0, help='front wheel angle target (not handwheel)')
    p.add_argument('--direction', type=int, choices=[-1, 1], default=1)
    p.add_argument('--repeat-s', type=float, default=0.0, help='repeat the maneuver every N s (alternating side)')
    p.add_argument('--duration', type=float, default=60.0, help='sim seconds; 0 = until Ctrl+C')
    p.add_argument('--dt', type=float, default=.01)
    p.add_argument('--fast', action='store_true', help='pad mode: do not pace to wall clock')
    p.add_argument('--tire-friction', type=float, help='pad mode: CARLA wheel friction scalar')
    p.add_argument('--rain-visual', action='store_true')
    p.add_argument('--reset-s', type=float, default=1.0, help='shadow model re-initialisation period (sim s)')
    p.add_argument('--param-refresh', type=float, default=1.0)
    p.add_argument('--print-hz', type=float, default=5.0)
    p.add_argument('--no-telemetry', action='store_true', help='log states only; no identification/model')
    p.add_argument('--plot', action='store_true')
    p.add_argument('--plot-window', type=float, default=20.0)
    p.add_argument('--keep-vehicle', action='store_true', help='pad mode: freeze and keep the vehicle at the end')
    p.add_argument('--out', type=Path)
    a = p.parse_args()
    nums = [a.timeout, a.speed_kmh, a.delta_deg, a.repeat_s, a.duration, a.dt, a.reset_s,
            a.param_refresh, a.print_hz, a.wait, a.plot_window]
    if not all(math.isfinite(x) for x in nums):
        p.error('Finite numeric arguments required')
    if not (1 <= a.port <= 65535 and 1 <= a.timeout <= 600 and 0 < a.dt <= .02 and 5 <= a.speed_kmh <= 100
            and 0 < a.delta_deg <= 10 and a.repeat_s >= 0 and a.duration >= 0 and .2 <= a.reset_s <= 10
            and a.param_refresh > 0 and 0 < a.print_hz <= 60 and a.wait >= 0):
        p.error('Invalid argument range')
    if a.tire_friction is not None and not (0 < a.tire_friction <= 10):
        p.error('tire-friction in (0,10]')
    if a.wheelbase is not None and not (math.isfinite(a.wheelbase) and a.wheelbase > 0):
        p.error('Positive finite wheelbase required')
    a.out = (a.out or ROOT / 'ket_qua_v4' / f'thoi_gian_thuc_{datetime.now():%Y%m%d_%H%M%S_%f}').resolve()
    return run(a)


if __name__ == '__main__':
    raise SystemExit(cli_entry(main))
