"""CARLA 0.9.15 research maneuvers; no ISO/NHTSA conformity claim.

Replaces the world. J-turn-style ramp/hold or a two-cycle steering pulse.
CARLA takes normalized commands, not tire forces. Actual wheel angle is logged
for identical offline replay in the body models. Friction scalar != verified mu.
"""
import argparse
import csv
import json
import math
import queue
from pathlib import Path
from carla_runtime import (checked_tick, cleanup, cli_entry, run_record,
                           snapshot_state, validate_connection, validate_physics, ObservationClock)

from buoc1_carla import (TEST_ROAD, body_components, clamp, equivalent_steer,
                        wheelbase_from_geometry, wrap)


def road_xml():
    return (TEST_ROAD.replace('FlatIdentificationRoad','WideManeuverPad')
            .replace('north="20" south="-20" east="500"',
                     'north="100" south="-100" east="1200"')
            .replace('length="500"','length="1200"')
            .replace('a="20" b="0" c="0" d="0"','a="100" b="0" c="0" d="0"')
            .replace('max="15"','max="35"'))


def steering_profile(t, maneuver, amplitude_rad, direction=1):
    """Research-only wheel-angle target; same waveform at each test speed."""
    tau=t-1.0
    if tau<0:return 0.0
    if maneuver=='jturn':
        if tau<.5:value=tau/.5
        elif tau<2.5:value=1.0
        elif tau<3.0:value=(3.0-tau)/.5
        else:value=0.0
    else:
        # + - - + curvature lobes form an outward/return excitation.
        # This does NOT define or guarantee a lane-change path or ISO gates.
        if tau<2.0:value=math.sin(math.pi*tau)
        elif tau<4.0:value=-math.sin(math.pi*(tau-2.0))
        else:value=0.0
    return direction*amplitude_rad*value


def sample(ego, world, carla):
    snapshot,s=snapshot_state(ego,world)
    tr=s.get_transform();v=s.get_velocity();a=s.get_acceleration()
    vx,vy=body_components(v.x,v.y,math.radians(tr.rotation.yaw))
    ax,ay=body_components(a.x,a.y,math.radians(tr.rotation.yaw))
    fl=ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    fr=ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    delta,conversion=equivalent_steer(fl,fr)
    if not all(math.isfinite(x) for x in (vx,vy,ax,ay,fl,fr)):
        raise ValueError('Nonfinite CARLA state/wheel data.')
    return {'frame':snapshot.frame,'t_sim_s':snapshot.timestamp.elapsed_seconds,
            'X_m':tr.location.x,'Y_m':-tr.location.y,'Z_m':tr.location.z,'vz_mps':v.z,
            'psi_rad':-math.radians(tr.rotation.yaw),'vx_mps':vx,'vy_mps':vy,
            'ax_mps2':ax,'ay_mps2':ay,'roll_deg':tr.rotation.roll,
            'pitch_deg':tr.rotation.pitch,'delta_equiv_rad':delta,
            'steer_fl_deg_carla':fl,'steer_fr_deg_carla':fr,
            'delta_conversion':conversion}


def tick(world, previous, timeout=90.0, expected_dt=None):
    return checked_tick(world,previous,timeout,expected_dt)


def cutoff(row):
    if row.get('Z_m',0)<-1:return 'cutoff_below_road'
    if not (10<row['X_m']<1190 and abs(row['Y_m'])<85):return 'cutoff_test_area'
    if abs(row['roll_deg'])>5 or abs(row['pitch_deg'])>5:return 'cutoff_nonplanar_motion'
    if math.hypot(row['vx_mps'],row['vy_mps'])>35:return 'cutoff_speed'
    return None


def save_json(path,data):
    path.write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def collect(args):
    with run_record(args.out) as status:
        _collect(args,status)
    print(f'{args.out.resolve()}: {status["result"]}; {status["rows"]} rows',flush=True)
    return 0 if status['result']=='duration_complete' else 2


def _collect(args,status):
    import carla
    world=None;old=None;owned=[];events=queue.Queue()
    view=ObservationClock(getattr(args,'observe',False))
    status['phase']='connecting'
    try:
        client=carla.Client(args.host,args.port);client.set_timeout(args.timeout)
        validate_connection(client)
        print('Creating a 1200 x 200 m flat pad; current world/actors will be replaced.',flush=True)
        xml=road_xml()
        params=carla.OpendriveGenerationParameters(vertex_distance=2.0,max_road_length=50.0,
            wall_height=0.0,additional_width=.6,smooth_junctions=True,enable_mesh_visibility=True)
        status['phase']='creating_world'
        world=client.generate_opendrive_world(xml,params);old=world.get_settings()
        s=world.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=args.dt
        s.substepping=True;s.max_substep_delta_time=.005;s.max_substeps=10
        world.apply_settings(s)
        world.set_weather(carla.WeatherParameters.WetNoon if args.rain_visual
                          else carla.WeatherParameters.ClearNoon)
        wp=world.get_map().get_waypoint_xodr(1,-1,100.0)
        if wp is None:raise RuntimeError('Generated waypoint unavailable')
        spawn=wp.transform;spawn.location.y=0.0;spawn.location.z+=.5
        bp=world.get_blueprint_library().find(args.vehicle)
        bp.set_attribute('role_name','maneuver_validation')
        ego=world.try_spawn_actor(bp,spawn)
        if ego is None:raise RuntimeError('Cannot spawn vehicle asset on generated pad.')
        owned.append(ego);ego.set_autopilot(False)
        status['phase']='initial_settling';status['vehicle_actor_id']=ego.id
        print(f'Xe da tao: actor={ego.id}. Camera bam xe; dang on dinh vat ly.',flush=True)
        physics=ego.get_physics_control();validate_physics(physics)
        wheels=list(physics.wheels);original_friction=[float(w.tire_friction) for w in wheels]
        for w in wheels:w.tire_friction=args.tire_friction
        physics.wheels=wheels;ego.apply_physics_control(physics)
        sensor=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),
                                 carla.Transform(),attach_to=ego);owned.append(sensor)
        sensor.listen(lambda e:events.put({'frame':e.frame,'timestamp':e.timestamp,
                                          'other_actor_id':e.other_actor.id}))
        ego.apply_control(carla.VehicleControl(brake=1.0))
        for _ in range(math.ceil(2/args.dt)):
            tick(world,world.get_snapshot().frame,args.timeout,args.dt)
            view.update(world,ego,carla)
        if ego.get_transform().location.z < -1:raise RuntimeError('Vehicle fell below the test road during settling.')
        if not events.empty():raise RuntimeError('Collision during initial settling.')
        physics=ego.get_physics_control();validate_physics(physics)
        readback=[float(w.tire_friction) for w in physics.wheels]
        if any(abs(x-args.tire_friction)>1e-4 for x in readback):
            raise RuntimeError('Friction scalar read-back mismatch')
        L,source,details=wheelbase_from_geometry(physics,2*ego.bounding_box.extent.x,args.wheelbase)
        meta={'schema_version':4,'vehicle_id':ego.type_id,'mass_kg':float(physics.mass),
            'wheelbase_m':L,'wheelbase_source':source,'wheelbase_details':details,
            'client_version':client.get_client_version(),'server_version':client.get_server_version(),
            'data_source':'CARLA simulator state; not a physical vehicle experiment',
            'case':args.maneuver,'speed_target_kmh':args.speed_kmh,
            'mu_nominal':args.mu_label,'effective_mu_verified':False,
            'tire_friction_requested':args.tire_friction,'tire_friction_readback':readback,
            'tire_friction_original':original_friction,'rain_visual':args.rain_visual,
            'physics_control_text':str(physics),'dt_s':args.dt,
            'maneuver_arguments':{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
            'iso_conformity':'not_assessed; steering excitation, no ISO gate track implemented',
            'nhtsa_conformity':'not_assessed; illustrative wheel-angle ramp/hold only',
            'steering_boundary':'measured actual equivalent wheel angle replayed offline',
            'longitudinal_boundary':'normalized commands; actual tire force equality not verified',
            'reference_point':'CARLA actor reference; verify/transform to model CoM before physical interpretation',
            'convention':'X=x_CARLA, Y=-y_CARLA, psi=-yaw_CARLA; body y left'}
        save_json(args.out/'metadata.json',meta)
        (args.out/'test_road.xodr').write_text(xml,encoding='utf-8')
        status['phase']='warmup'
        target=args.speed_kmh/3.6;stable=0;integral=0.0;ready=False
        with (args.out/'warmup.csv').open('w',newline='',encoding='utf-8') as f:
            writer=None
            for k in range(math.ceil(40/args.dt)):
                row=sample(ego,world,carla)
                view.update(world,ego,carla)
                if writer is None:writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);f.flush()
                reason=cutoff(row)
                if reason or not events.empty():raise RuntimeError(reason or 'warmup_collision')
                err=target-row['vx_mps'];integral=clamp(integral+err*args.dt,-3,3)
                effort=.25*err+.08*integral
                ego.apply_control(carla.VehicleControl(throttle=clamp(effort,0,.6),
                                  brake=clamp(-effort,0,.3),steer=0.0))
                stable=stable+1 if abs(err)<.5/3.6 else 0
                if k%max(1,round(2/args.dt))==0:
                    print(f'Warmup: {row["vx_mps"]*3.6:.1f}/{args.speed_kmh:g} km/h',flush=True)
                if stable>=math.ceil(1/args.dt):ready=True;break
                tick(world,row['frame'],args.timeout,args.dt)
        if not ready:raise RuntimeError('Entry speed did not settle within 40 s / pad limits')
        tick(world,row['frame'],args.timeout,args.dt)
        row=sample(ego,world,carla);t0=row['t_sim_s'];psi_old=row['psi_rad'];time_old=t0
        steer_int=0.0;maxsteer=max(w.max_steer_angle for w in list(physics.wheels)[:2])
        status['phase']='maneuver'
        result='duration_complete';steps=round(6/args.dt)
        status['entry_speed_actual_kmh']=3.6*row['vx_mps']
        with (args.out/'states.csv').open('w',newline='',encoding='utf-8') as f:
            writer=None
            for k in range(steps+1):
                row=sample(ego,world,carla);t=row['t_sim_s']-t0
                if abs(t-k*args.dt)>max(1e-6,args.dt*.01):raise RuntimeError('Unexpected simulator timing')
                desired=steering_profile(t,args.maneuver,math.radians(args.delta_deg),args.direction)
                e=desired-row['delta_equiv_rad'];steer_int=clamp(steer_int+e*args.dt,-.1,.1)
                # Normalize the feedback by the same physical steering range.
                # A modest proportional gain avoids alternating/saturated commands
                # when the simulator wheel response is faster than one sample.
                angle_range=math.radians(maxsteer)
                steer=-clamp((desired+.25*e+.8*steer_int)/angle_range,-1,1)
                err=target-row['vx_mps'];integral=clamp(integral+err*args.dt,-3,3)
                effort=.25*err+.08*integral
                throttle=clamp(effort,0,.6);brake=clamp(-effort,0,.3)
                if args.brake_pulse and 2.0<=t<2.5:throttle=0.0;brake=args.brake_pulse
                applied=ego.get_control();reason=cutoff(row)
                if not events.empty():reason='cutoff_collision_event'
                if reason:result=reason
                row.update({'t_s':t,'delta_target_next_rad':desired,
                    'throttle_applied':applied.throttle,'brake_applied':applied.brake,
                    'steer_applied_norm':applied.steer,'throttle_next':throttle,
                    'brake_next':brake,'steer_next_norm':steer,
                    'r_yaw_difference_radps':wrap(row['psi_rad']-psi_old)/(row['t_sim_s']-time_old)
                        if k else 0.0})
                if writer is None:writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);f.flush();status['rows']+=1
                if reason or k==steps:break
                ego.apply_control(carla.VehicleControl(throttle=throttle,brake=brake,steer=steer))
                view.update(world,ego,carla)
                if k%max(1,round(1/args.dt))==0:
                    print(f'Maneuver t={t:.1f}s v={row["vx_mps"]*3.6:.1f}km/h',flush=True)
                psi_old=row['psi_rad'];time_old=row['t_sim_s']
                tick(world,row['frame'],args.timeout,args.dt)
        status.update(result=result,iso_pass=None,note='Completion is not scientific acceptance or ISO conformity.')
    finally:
        cleanup(world,old,owned,events,args.out,status,carla,keep_vehicle=getattr(args,'observe',False))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--maneuver',choices=['jturn','double_pulse'],required=True)
    p.add_argument('--speed-kmh',type=float,choices=[30,50,80],required=True)
    p.add_argument('--mu-label',type=float,choices=[1.0,.4],required=True,
                   help='Nominal model mu label, not a verified CARLA effective coefficient')
    p.add_argument('--tire-friction',type=float,required=True,help='CARLA wheel friction scalar')
    p.add_argument('--delta-deg',type=float,default=2.0,help='Front wheel angle target, not handwheel')
    p.add_argument('--direction',type=int,choices=[-1,1],default=1)
    p.add_argument('--brake-pulse',type=float,default=0.0,help='Normalized brake pulse from 2 to 2.5 s')
    p.add_argument('--rain-visual',action='store_true',help='Visual wetness preset only')
    p.add_argument('--observe',action='store_true',help='Wall-clock pacing, chase camera and keep frozen vehicle after acquisition')
    p.add_argument('--timeout',type=float,default=90.0)
    p.add_argument('--dt',type=float,default=.01)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--vehicle',default='vehicle.tesla.model3')
    p.add_argument('--wheelbase',type=float);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    vals=[a.tire_friction,a.delta_deg,a.brake_pulse,a.dt,a.timeout]
    if not (1<=a.port<=65535 and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    if not all(math.isfinite(v) for v in vals):p.error('Finite numeric arguments required')
    if not (0<a.tire_friction<=10 and 0<a.delta_deg<=10 and 0<=a.brake_pulse<=1):
        p.error('Invalid excitation/friction parameters')
    if not (0<a.dt<=.02 and abs(6/a.dt-round(6/a.dt))<1e-6):
        p.error('Use dt <= .02 that divides 6 seconds exactly, e.g. .01')
    if a.wheelbase is not None and (not math.isfinite(a.wheelbase) or a.wheelbase<=0):
        p.error('Positive finite wheelbase required')
    return collect(a)


if __name__=='__main__':raise SystemExit(cli_entry(main))
