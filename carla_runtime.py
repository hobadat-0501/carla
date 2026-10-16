"""Small, dependency-free runtime helpers for the CARLA experiment scripts."""
import contextlib
import json
import math
import re
import sys
import time
import traceback
from pathlib import Path


def configure_console():
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, 'reconfigure'):
            stream.reconfigure(encoding='utf-8', errors='replace')


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False,
                                    allow_nan=False), encoding='utf-8')


def error_hint(error):
    message = str(error).lower()
    if isinstance(error, ModuleNotFoundError):
        return 'Chay 00_CAI_DAT_WINDOWS.bat; dung Python trong .venv_0915 cua bo ma.'
    if isinstance(error, FileExistsError):
        return 'Thu muc/tep da ton tai. Chon --out moi; chuong trinh khong ghi de du lieu.'
    if '0.9.15' in message or 'version' in message:
        return 'Dung CARLA server 0.9.15 va goi Python carla==0.9.15.'
    if 'timeout' in message or 'time-out' in message or 'connect' in message:
        return ('Mo CarlaUE4.exe, doi hien ban do; kiem tra host/port va firewall. '
                'Chay lai 01_KIEM_TRA_WINDOWS.bat. Chi mot chuong trinh duoc tick server.')
    if 'tick' in message or 'timing' in message:
        return 'Dong manual_control/automatic_control/traffic va cac client khac dang tick CARLA.'
    if 'blueprint' in message or 'vehicle asset' in message:
        return 'Kiem tra ten --vehicle va cac asset trong ban CARLA da cai.'
    if 'wheelbase' in message:
        return 'Kiem tra hinh hoc xe; chi dung --wheelbase khi da biet chieu dai co so theo met.'
    return 'Doc message va error_traceback.txt; giu nguyen thu muc ket qua de kiem tra.'


def validate_connection(client):
    versions = (client.get_client_version(), client.get_server_version())
    for value in versions:
        if not re.match(r'^0\.9\.15(?:$|[-+])', value):
            raise RuntimeError('Matching CARLA client/server 0.9.15 required; got ' + str(versions))
    return versions


def checked_tick(world, previous=None, timeout=30.0, expected_dt=None):
    before = world.get_snapshot()
    previous = before.frame if previous is None else previous
    if before.frame != previous:
        raise RuntimeError('Another client is ticking this world before our tick.')
    frame = world.tick(timeout)
    after = world.get_snapshot()
    if frame != previous + 1 or after.frame != frame:
        raise RuntimeError('Another client is ticking this world.')
    if expected_dt is not None:
        dt = after.timestamp.elapsed_seconds - before.timestamp.elapsed_seconds
        if abs(dt - expected_dt) > max(1e-6, .01 * expected_dt):
            raise RuntimeError('Unexpected simulator timing; check synchronous settings and other clients.')
    return frame


def snapshot_state(ego, world):
    snapshot = world.get_snapshot()
    state = snapshot.find(ego.id)
    if state is None:
        raise RuntimeError('Vehicle missing from snapshot; server world changed or actor was destroyed.')
    return snapshot, state


def validate_physics(physics):
    if len(physics.wheels) != 4:
        raise ValueError('This experiment requires a four-wheel vehicle asset.')
    if not math.isfinite(physics.mass) or physics.mass <= 0:
        raise ValueError('Invalid vehicle mass read from simulator.')
    if not all(math.isfinite(w.max_steer_angle) for w in physics.wheels):
        raise ValueError('Invalid steering geometry read from simulator.')
    if max(w.max_steer_angle for w in list(physics.wheels)[:2]) <= 0:
        raise ValueError('Vehicle asset has no front steering range.')


def follow_vehicle(world, ego, carla):
    tr=ego.get_transform();yaw=math.radians(tr.rotation.yaw)
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(x=tr.location.x-8*math.cos(yaw),
                       y=tr.location.y-8*math.sin(yaw),z=tr.location.z+6),
        carla.Rotation(pitch=-math.degrees(math.atan2(6,8)),yaw=tr.rotation.yaw)))


class ObservationClock:
    """Camera follow throughout acquisition; optional wall-clock pacing only."""
    def __init__(self, enabled=False):
        self.enabled=enabled;self.sim_origin=None;self.wall_origin=None;self.last_camera=None

    def update(self,world,ego,carla):
        sim=world.get_snapshot().timestamp.elapsed_seconds;now=time.perf_counter()
        if self.sim_origin is None:self.sim_origin=sim;self.wall_origin=now
        if self.enabled:
            target=self.wall_origin+sim-self.sim_origin
            if now-target>.5:self.wall_origin=now-(sim-self.sim_origin);target=now
            if target>now:time.sleep(target-now)
        if self.last_camera is None or sim-self.last_camera>=.05-1e-6:
            follow_vehicle(world,ego,carla);self.last_camera=sim


def cleanup(world, old_settings, owned, events, out, status, carla, keep_vehicle=False):
    warnings = []
    kept=None
    if world is not None:
        if owned:
            try:
                tr=owned[0].get_transform()
                status['last_vehicle_pose']={'x_m':tr.location.x,'y_carla_m':tr.location.y,
                    'z_m':tr.location.z,'yaw_deg':tr.rotation.yaw,
                    'pitch_deg':tr.rotation.pitch,'roll_deg':tr.rotation.roll}
                status['vehicle_actor_id']=owned[0].id
                if keep_vehicle and owned[0].is_alive:
                    owned[0].set_simulate_physics(False);kept=owned[0]
                    status['vehicle_cleanup']='kept_frozen_for_inspection'
                    status['inspection_note']='Physics frozen only after data acquisition ended; next world replacement removes the vehicle.'
            except Exception as exc:warnings.append('Inspection setup: '+str(exc))
            try:
                owned[0].apply_control(carla.VehicleControl(brake=1.0))
                checked_tick(world)
            except Exception as exc:
                warnings.append('Final brake/tick: ' + str(exc))
        for actor in reversed(owned):
            try:
                if actor is kept:continue
                if actor.is_alive:
                    if actor.type_id.startswith('sensor.'):
                        actor.stop()
                    actor.destroy()
            except Exception as exc:
                warnings.append('Actor cleanup: ' + str(exc))
        if kept is not None:
            try:follow_vehicle(world,kept,carla)
            except Exception as exc:warnings.append('Inspection camera: '+str(exc))
            print(f'Xe actor {kept.id} duoc giu lai de quan sat; du lieu da dong.',flush=True)
        elif owned:status['vehicle_cleanup']='destroyed_on_exit'
        if old_settings is not None:
            try:
                world.apply_settings(old_settings)
            except Exception as exc:
                warnings.append('Settings cleanup: ' + str(exc))
    received = []
    while not events.empty():
        received.append(events.get_nowait())
    status['collision_events'] = len(received)
    # A callback can arrive after the final sample. Never report this as success.
    if received and status.get('result') == 'duration_complete':
        status['result'] = 'cutoff_collision_event'
    if warnings:
        status['cleanup_warnings'] = warnings
        if status.get('result') == 'duration_complete':
            status['result'] = 'cleanup_error'
    save_json(out / 'collision_events.json', received)
    save_json(out / 'run_status.json', status)


@contextlib.contextmanager
def run_record(out):
    """Create a fresh run and record even import/connect/map-generation failures."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=False)
    wall_start=time.perf_counter()
    status = {'result': 'incomplete', 'rows': 0}
    save_json(out / 'run_status.json', status)
    try:
        yield status
    except BaseException as exc:
        status.update(result='interrupted' if isinstance(exc, KeyboardInterrupt) else 'error',
                      error_type=type(exc).__name__, message=str(exc), hint=error_hint(exc))
        (out / 'error_traceback.txt').write_text(traceback.format_exc(), encoding='utf-8')
        raise
    finally:
        status['wall_elapsed_s']=round(time.perf_counter()-wall_start,6)
        save_json(out / 'run_status.json', status)


def cli_entry(function):
    configure_console()
    try:
        result = function()
        return result if isinstance(result, int) else 0
    except KeyboardInterrupt:
        print('Da dung theo yeu cau. Du lieu da ghi duoc giu lai.', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'LOI [{type(exc).__name__}]: {exc}', file=sys.stderr)
        print(error_hint(exc), file=sys.stderr)
        return 1
