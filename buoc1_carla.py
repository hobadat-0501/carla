"""CARLA 0.9.15: collect longitudinal/lateral experiments or analyze a saved run.

By default --case REPLACES the world with an empty flat OpenDRIVE road.
--world-mode auto/reuse can reuse an identical road. Use a dedicated session.
No NMPC, UKF or fitted longitudinal/lateral dynamic model is implemented here.
"""
import argparse
import csv
import json
import math
import platform
import queue
import sys
from pathlib import Path
from tien_do_carla import StageTrace
from quan_ly_duong_thu import prepare_world
from carla_runtime import (checked_tick, cleanup, cli_entry, run_record,
                           snapshot_state, validate_connection, validate_physics, ObservationClock)

TEST_ROAD = '''<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
 <header revMajor="1" revMinor="4" name="FlatIdentificationRoad" version="1.00"
         date="2026-10-15" north="20" south="-20" east="500" west="0">
  <geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs]]></geoReference>
 </header>
 <road name="FlatIdentificationRoad" length="500" id="1" junction="-1">
  <link/>
  <type s="0" type="rural"><speed max="15" unit="m/s"/></type>
  <planView><geometry s="0" x="0" y="0" hdg="0" length="500"><line/></geometry></planView>
  <elevationProfile><elevation s="0" a="0" b="0" c="0" d="0"/></elevationProfile>
  <lateralProfile/>
  <lanes>
   <laneOffset s="0" a="0" b="0" c="0" d="0"/>
   <laneSection s="0">
    <left><lane id="1" type="driving" level="false"><link/>
     <width sOffset="0" a="20" b="0" c="0" d="0"/>
     <roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.12"/>
    </lane></left>
    <center><lane id="0" type="none" level="false"><link/>
     <roadMark sOffset="0" type="broken" weight="standard" color="white" width="0.12"/>
    </lane></center>
    <right><lane id="-1" type="driving" level="false"><link/>
     <width sOffset="0" a="20" b="0" c="0" d="0"/>
     <roadMark sOffset="0" type="solid" weight="standard" color="white" width="0.12"/>
    </lane></right>
   </laneSection>
  </lanes>
  <objects/><signals/>
 </road>
</OpenDRIVE>
'''


def clamp(x, low, high):
    return max(low, min(high, x))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def body_components(world_x, world_y, yaw_carla_rad):
    """Project a world vector into x-forward/y-left components on a flat road."""
    c, s = math.cos(yaw_carla_rad), math.sin(yaw_carla_rad)
    return c*world_x+s*world_y, s*world_x-c*world_y


def equivalent_steer(left_deg, right_deg):
    """Bicycle-equivalent front axle angle; internal positive points left.

    For ideal Ackermann angles: cot(delta) = [cot(left)+cot(right)]/2.
    Small or inconsistent angles use their mean and are marked accordingly.
    """
    left, right = -math.radians(left_deg), -math.radians(right_deg)
    t1, t2 = math.tan(left), math.tan(right)
    if abs(t1)+abs(t2) < 1e-8:
        return 0.0, 'zero'
    if t1*t2 > 0 and abs(t1+t2) > 1e-8:
        return math.atan(2*t1*t2/(t1+t2)), 'ackermann_equivalent'
    return (left+right)/2, 'angle_mean_fallback'


def wheelbase_from_geometry(physics, vehicle_length, override=None):
    """Resolve wheel-position length units against actor size, with provenance.

    CARLA documentation/wheel-position conventions can differ between versions.
    Only accept a unique plausible scale; the user may explicitly supply L.
    This is simulator geometry, not identification of the physical Hunter SE.
    """
    if override is not None:
        return override, 'user_supplied', {}
    wheels = list(physics.wheels)
    if len(wheels) != 4:
        raise ValueError('This experiment requires a four-wheel vehicle.')
    front = [(getattr(wheels[0].position, a)+getattr(wheels[1].position, a))/2
             for a in ('x','y','z')]
    rear = [(getattr(wheels[2].position, a)+getattr(wheels[3].position, a))/2
            for a in ('x','y','z')]
    raw = math.sqrt(sum((a-b)**2 for a,b in zip(front,rear)))
    candidates = [(scale, raw*scale) for scale in (1.0, .01)
                  if .3*vehicle_length <= raw*scale <= .9*vehicle_length]
    if len(candidates) != 1:
        raise ValueError('Cannot resolve wheelbase units unambiguously. '
                         'Specify --wheelbase METERS after checking vehicle geometry.')
    scale, value = candidates[0]
    return value, 'wheel_geometry_with_size_consistency_check', {
        'front_midpoint_raw':front, 'rear_midpoint_raw':rear,
        'raw_axle_distance':raw, 'scale_to_m':scale,
        'actor_bbox_length_m':vehicle_length}


def experiment_command(case, t, vx, speed, drive, steer_amp, frequency):
    """Return throttle, brake, normalized CARLA steer, phase description."""
    if case == 'doc':
        if t < 1: return 0.0, 1.0, 0.0, 'stationary'
        if t < 5: return drive, 0.0, 0.0, 'traction_1'
        if t < 9: return min(1.5*drive, .5), 0.0, 0.0, 'traction_2'
        if t < 12: return 0.0, 0.0, 0.0, 'coasting'
        if t < 16: return 0.0, .2, 0.0, 'braking'
        return 0.0, 1.0, 0.0, 'end_braking'
    target = speed
    if case == 'phoi_hop' and t >= 10:
        target = 1.25*speed
    effort = .4*(target-vx)
    throttle = clamp(effort, 0.0, .4)
    brake = clamp(-effort, 0.0, .3)
    if t < 5:
        return throttle, brake, 0.0, 'speed_settling'
    if case == 'ngang':
        # Ramp into a steering step, then hold. No path-following feedback.
        steer = steer_amp*clamp((t-5)/.5, 0, 1) if t < 11 else 0.0
        if t >= 11: return 0.0, .25, 0.0, 'end_braking'
        return throttle, brake, steer, 'steering_step'
    steer = steer_amp*math.sin(2*math.pi*frequency*(t-5))
    if t >= 15: return 0.0, .25, 0.0, 'end_braking'
    return throttle, brake, steer, 'sine_steering_and_speed_change'


def parse_args(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--case', choices=['doc','ngang','phoi_hop'])
    mode.add_argument('--analyze', type=Path, metavar='RUN_DIRECTORY')
    p.add_argument('--out', type=Path, help='New output directory; never overwritten')
    p.add_argument('--model-file', type=Path,
                   help='Optional independent model predictions CSV, only with --analyze')
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=2000)
    p.add_argument('--observe',action='store_true',help='Wall-clock pacing, chase camera and keep frozen vehicle after acquisition')
    p.add_argument('--trace-stages',action='store_true',help='Persist stages before RPC calls and print a wall-clock heartbeat every 5 seconds')
    p.add_argument('--world-mode',choices=['create','auto','reuse'],default='create',
                   help='create: new world; auto: reuse identical test road; reuse: require identical test road')
    p.add_argument('--timeout', type=float, default=90.0, help='RPC timeout, seconds')
    p.add_argument('--dt', type=float, default=.02)
    p.add_argument('--seconds', type=float, default=18.0)
    p.add_argument('--speed', type=float, default=2.0, help='Lateral test speed target, m/s')
    p.add_argument('--drive', type=float, default=.18, help='Longitudinal test throttle')
    p.add_argument('--steer-amplitude', type=float, default=.06,
                   help='Normalized CARLA steer, NOT radian or degree')
    p.add_argument('--frequency', type=float, default=.2, help='Combined test sine frequency, Hz')
    p.add_argument('--max-speed', type=float, default=10.0, help='Simulation cutoff, m/s')
    p.add_argument('--wheelbase', type=float, default=None, help='Optional verified wheelbase in meters')
    p.add_argument('--mass-scale', type=float, default=1.0)
    p.add_argument('--vehicle', default='vehicle.tesla.model3')
    args=p.parse_args(argv)
    numeric=[args.dt,args.seconds,args.speed,args.drive,args.steer_amplitude,args.frequency,
             args.max_speed,args.mass_scale,args.timeout]
    if not all(math.isfinite(x) for x in numeric):p.error('Finite numeric arguments required')
    if not (1<=args.port<=65535 and 1<=args.timeout<=600):p.error('Invalid port/timeout')
    if args.case and args.out is None: p.error('--case requires --out')
    if args.model_file and not args.analyze: p.error('--model-file requires --analyze')
    if not (0 < args.dt <= .05 and args.seconds > 0): p.error('Require 0 < dt <= .05, seconds > 0')
    if not (0 < args.speed <= args.max_speed and args.mass_scale > 0): p.error('Invalid speed/mass scale')
    if not (0 < args.drive <= .5 and abs(args.steer_amplitude) <= .15 and args.frequency > 0):
        p.error('Invalid excitation amplitude or frequency')
    if args.wheelbase is not None and (not math.isfinite(args.wheelbase) or args.wheelbase <= 0): p.error('Wheelbase must be positive')
    return args


def collect(args):
    with run_record(args.out) as status:
        with StageTrace(args.out,status,getattr(args,'trace_stages',False)) as trace:
            trace.stage('importing_client','Nap thu vien Python carla.')
            _collect(args, status, trace)
    print(f'Data: {args.out.resolve()} | {status["result"]} | {status["rows"]} rows')
    return 0 if status['result']=='duration_complete' else 2


def _collect(args, status, trace):
    import carla
    world=None;old=None;owned=[];events=queue.Queue()
    view=ObservationClock(getattr(args,'observe',False))
    trace.stage('connecting','Kiem tra client/server 0.9.15.')
    try:
        client=carla.Client(args.host,args.port);client.set_timeout(args.timeout)
        validate_connection(client)
        parameters=carla.OpendriveGenerationParameters(vertex_distance=2.0,max_road_length=50.0,
            wall_height=0.0,additional_width=.6,smooth_junctions=True,enable_mesh_visibility=True)
        world,old=prepare_world(client,TEST_ROAD,parameters,getattr(args,'world_mode','create'),status,trace)
        trace.stage('configuring_world','Duong thu da san sang; cai dat synchronous va dt.')
        settings=world.get_settings();settings.synchronous_mode=True
        settings.fixed_delta_seconds=args.dt;settings.substepping=True
        settings.max_substep_delta_time=.005;settings.max_substeps=10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        trace.stage('locating_spawn','Doc ban do va vi tri tao xe.')
        wp=world.get_map().get_waypoint_xodr(1,-1,50.0)
        if wp is None:raise RuntimeError('Generated OpenDRIVE waypoint is missing.')
        spawn=wp.transform;spawn.location.y=0.0;spawn.location.z+=.5
        trace.stage('loading_vehicle_blueprint','Tim blueprint '+args.vehicle)
        blueprint=world.get_blueprint_library().find(args.vehicle)
        blueprint.set_attribute('role_name','model_validation_ego')
        trace.stage('spawning_vehicle','Tao xe tai x=50 m, y=0 m.')
        ego=world.try_spawn_actor(blueprint,spawn)
        if ego is None:raise RuntimeError('Cannot spawn experiment vehicle asset.')
        owned.append(ego);ego.set_autopilot(False)
        status['vehicle_actor_id']=ego.id
        trace.stage('configuring_vehicle','Xe da tao; doc va cai dat vat ly.')
        print(f'Xe da tao: actor={ego.id}. Camera bam xe; dang on dinh vat ly.',flush=True)
        physics=ego.get_physics_control();validate_physics(physics)
        physics.mass*=args.mass_scale;ego.apply_physics_control(physics)
        # Read back geometry only after physics has settled.
        trace.stage('spawning_collision_sensor','Tao cam bien va cham va dang ky callback.')
        collision=world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'),
                                    carla.Transform(),attach_to=ego)
        owned.append(collision)
        collision.listen(lambda e:events.put({'frame':e.frame,'timestamp':e.timestamp,
                                              'other_actor_id':e.other_actor.id}))
        ego.apply_control(carla.VehicleControl(brake=1.0))
        trace.stage('initial_settling','Giu phanh 2 giay mo phong truoc khi thu du lieu.')
        for j in range(math.ceil(2.0/args.dt)):
            checked_tick(world,timeout=args.timeout,expected_dt=args.dt)
            view.update(world,ego,carla)
            if j%max(1,round(1/args.dt))==0:trace.pulse(settling_tick=j+1,rows=0)
        if ego.get_transform().location.z < -1:raise RuntimeError('Vehicle fell below the test road during settling.')
        if not events.empty():raise RuntimeError('Collision during initial settling.')
        trace.stage('reading_geometry','Vat ly da on dinh; doc hinh hoc va luu metadata.')
        physics=ego.get_physics_control();validate_physics(physics)
        L,L_source,L_details=wheelbase_from_geometry(physics,2*ego.bounding_box.extent.x,args.wheelbase)
        metadata={**{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
                  'schema_version':4,'client_version':client.get_client_version(),
                  'server_version':client.get_server_version(),'python_version':platform.python_version(),
                  'map_name':world.get_map().name,'vehicle_id':ego.type_id,
                  'mass_kg':physics.mass,'wheelbase_m':L,'wheelbase_source':L_source,
                  'wheelbase_details':L_details,'physics_control_text':str(physics),
                  'frame_convention':'X=x_CARLA, Y=-y_CARLA, psi=-yaw_CARLA; body y points left',
                  'data_source':'CARLA simulator state; not a physical vehicle experiment',
                  'world_setup':status['world_setup'],
                  'test_road':'500 m straight road; two wide 20 m lanes; spawn at y=0',
                  'scope':'Response acquisition. Dynamic comparison requires independent model predictions.'}
        (args.out/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
        (args.out/'test_road.xodr').write_text(TEST_ROAD,encoding='utf-8')
        fields=['frame','t_s','t_sim_s','X_m','Y_m','Z_m','vz_mps','psi_rad','vx_mps','vy_mps','ax_mps2','ay_mps2',
                'pitch_deg','roll_deg','angular_z_snapshot_raw','r_yaw_difference_radps',
                'steer_fl_deg_carla','steer_fr_deg_carla','delta_equiv_rad','delta_conversion',
                'throttle_applied','brake_applied','steer_applied_norm',
                'throttle_next','brake_next','steer_next_norm','phase_next']
        trace.stage('acquiring','Thu mau; ngang: 0-5 s tang toc, 5-11 s danh lai, 11-18 s phanh.')
        previous_psi=None;previous_time=None
        last_phase=None
        t0=world.get_snapshot().timestamp.elapsed_seconds;result='duration_complete'
        steps=math.ceil(args.seconds/args.dt)
        with (args.out/'states.csv').open('w',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            for k in range(steps+1):
                snapshot,state=snapshot_state(ego,world)
                tr=state.get_transform();v=state.get_velocity();a=state.get_acceleration()
                now=snapshot.timestamp.elapsed_seconds;t=now-t0
                psi=-math.radians(tr.rotation.yaw)
                vx,vy=body_components(v.x,v.y,-psi);ax,ay=body_components(a.x,a.y,-psi)
                rate=(wrap(psi-previous_psi)/(now-previous_time)
                      if previous_psi is not None and now>previous_time else float('nan'))
                left=ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
                right=ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
                delta,conversion=equivalent_steer(left,right);applied=ego.get_control()
                if not all(math.isfinite(x) for x in [vx,vy,ax,ay,psi,left,right]):
                    raise ValueError('Nonfinite CARLA state/wheel data.')
                throttle,brake,steer,phase=experiment_command(args.case,t,vx,args.speed,args.drive,
                    args.steer_amplitude,args.frequency)
                if tr.location.z < -1:result='cutoff_below_road'
                if math.hypot(vx,vy)>args.max_speed:result='cutoff_speed'
                if not (10<tr.location.x<490 and abs(tr.location.y)<16):result='cutoff_test_area'
                if abs(tr.rotation.roll)>5 or abs(tr.rotation.pitch)>5:result='cutoff_nonplanar_motion'
                if not events.empty():result='cutoff_collision_event'
                writer.writerow(dict(zip(fields,[snapshot.frame,t,now,tr.location.x,-tr.location.y,
                    tr.location.z,v.z,psi,vx,vy,ax,ay,tr.rotation.pitch,tr.rotation.roll,
                    state.get_angular_velocity().z,rate,left,right,delta,conversion,
                    applied.throttle,applied.brake,applied.steer,throttle,brake,steer,phase])))
                f.flush();status['rows']+=1
                if phase!=last_phase or k%max(1,round(1/args.dt))==0 or k==steps or result!='duration_complete':
                    trace.pulse(t_sim_s=round(t,3),rows=status['rows'],command_phase=phase,
                                vx_mps=round(vx,4),vy_mps=round(vy,4),
                                delta_rad=round(delta,6),cutoff=result if result!='duration_complete' else None)
                    last_phase=phase
                if result!='duration_complete' or k==steps:break
                ego.apply_control(carla.VehicleControl(throttle=throttle,brake=brake,steer=steer))
                view.update(world,ego,carla)
                if k%max(1,round(2/args.dt))==0:print(f't={t:.1f}s vx={vx:.2f}m/s',flush=True)
                previous_psi,previous_time=psi,now
                checked_tick(world,snapshot.frame,args.timeout,args.dt)
        status.update(result=result,note='Completion is not scientific acceptance.')
    finally:
        error=sys.exc_info()[1]
        if error is not None:
            status['failure_phase']=status.get('phase')
            status['failure_message']=str(error)
        status['last_acquisition_phase']=status.get('phase')
        trace.stage('cleanup','Dong du lieu, go cam bien va phuc hoi cai dat the gioi.')
        cleanup(world,old,owned,events,args.out,status,carla,keep_vehicle=getattr(args,'observe',False))


def analyze(args):
    import numpy as np
    import pandas as pd
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    run=args.analyze
    df=pd.read_csv(run/'states.csv')
    meta=json.loads((run/'metadata.json').read_text(encoding='utf-8'))
    if len(df)<3:raise ValueError('Need at least three samples to analyze.')
    if not np.all(np.diff(df.t_s)>0):raise ValueError('Nonmonotonic or duplicate timestamps.')
    # Compare interval yaw-rate to the interval average of the no-slip relation.
    df['r_kinematic_radps']=df.vx_mps*np.tan(df.delta_equiv_rad)/meta['wheelbase_m']
    df['r_kinematic_interval_radps']=.5*(df.r_kinematic_radps+df.r_kinematic_radps.shift(1))
    active=(df.t_s>.25) if meta.get('case') in ('jturn','double_pulse') else (df.t_s>5)
    mask=active&(df.vx_mps>.5)&np.isfinite(df.r_yaw_difference_radps)
    error=(df.r_yaw_difference_radps-df.r_kinematic_interval_radps)[mask]
    diagnostic={'wheelbase_m':meta['wheelbase_m'],'samples':len(df),
        'kinematic_yaw_rate_samples':int(mask.sum()),
        'kinematic_yaw_rate_rmse_radps':float(np.sqrt(np.mean(error**2))) if len(error) else None,
        'meaning':'Agreement of a no-slip yaw-rate relation with CARLA. '
                  'This is not validation against a real vehicle or direct tire-force measurement.'}
    if (run/'run_status.json').is_file():
        diagnostic['run_status']=json.loads((run/'run_status.json').read_text(encoding='utf-8'))
    df.to_csv(run/'processed.csv',index=False)
    fig,axes=plt.subplots(3,2,figsize=(12,9))
    axes[0,0].plot(df.t_s,df.throttle_applied,label='Throttle')
    axes[0,0].plot(df.t_s,df.brake_applied,label='Brake')
    axes[0,0].set(ylabel='Normalized command')
    axes[0,1].plot(df.t_s,np.degrees(df.delta_equiv_rad),label='Equivalent wheel angle')
    axes[0,1].set(ylabel='Internal steering angle (deg)')
    axes[1,0].plot(df.t_s,df.vx_mps,label='vx')
    axes[1,0].plot(df.t_s,df.vy_mps,label='vy')
    axes[1,0].set(ylabel='Body velocity (m/s)')
    axes[1,1].plot(df.t_s,df.ax_mps2,label='ax')
    axes[1,1].plot(df.t_s,df.ay_mps2,label='ay')
    axes[1,1].set(ylabel='Body acceleration (m/s2)')
    axes[2,0].plot(df.t_s,df.r_yaw_difference_radps,label='CARLA yaw difference')
    axes[2,0].plot(df.t_s,df.r_kinematic_interval_radps,'--',label='Kinematic relation')
    axes[2,0].set(ylabel='Yaw rate (rad/s)')
    axes[2,1].plot(df.X_m-df.X_m.iloc[0],df.Y_m-df.Y_m.iloc[0],label='CARLA trajectory')
    axes[2,1].axis('equal');axes[2,1].set(xlabel='X displacement (m)',ylabel='Y displacement (m)')
    for i,ax in enumerate(axes.flat):
        if i<5:ax.set_xlabel('Simulation time (s)')
        ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(run/'responses.png',dpi=180);plt.close(fig)
    if args.model_file:
        predictions=pd.read_csv(args.model_file)
        if 't_s' not in predictions:raise ValueError('Model CSV needs t_s.')
        if len(predictions)<2 or not np.all(np.diff(predictions.t_s)>0):
            raise ValueError('Model timestamps must increase strictly.')
        if predictions.t_s.iloc[0]>df.t_s.iloc[0]+1e-6 or predictions.t_s.iloc[-1]<df.t_s.iloc[-1]-1e-6:
            raise ValueError('Model predictions must cover the whole experiment; no extrapolation.')
        metrics=[]
        mapping={'vx_model_mps':'vx_mps','vy_model_mps':'vy_mps',
                 'r_model_radps':'r_yaw_difference_radps','ax_model_mps2':'ax_mps2',
                 'ay_model_mps2':'ay_mps2','X_model_m':'X_m','Y_model_m':'Y_m'}
        available=[(a,b) for a,b in mapping.items() if a in predictions]
        if not available:raise ValueError('No recognized prediction columns; see README.')
        fig,axes=plt.subplots(len(available),1,figsize=(10,2.5*len(available)),squeeze=False)
        for (source,target),ax in zip(available,axes[:,0]):
            predicted=np.interp(df.t_s,predictions.t_s,predictions[source])
            observed=df[target].to_numpy();valid=np.isfinite(predicted)&np.isfinite(observed)
            e=predicted[valid]-observed[valid]
            if not len(e):raise ValueError(f'No finite paired values for {source}.')
            metrics.append({'quantity':target,'samples':int(valid.sum()),
                            'rmse':float(np.sqrt(np.mean(e**2))),
                            'max_abs_error':float(np.max(np.abs(e)))})
            ax.plot(df.t_s,observed,label='CARLA');ax.plot(df.t_s,predicted,'--',label='User model')
            ax.set(xlabel='Simulation time (s)',ylabel=target);ax.legend();ax.grid(alpha=.25)
        pd.DataFrame(metrics).to_csv(run/'model_comparison_metrics.csv',index=False)
        fig.tight_layout();fig.savefig(run/'model_comparison.png',dpi=180);plt.close(fig)
        diagnostic['model_comparison']=metrics
    (run/'diagnostics.json').write_text(json.dumps(diagnostic,indent=2),encoding='utf-8')
    print(json.dumps(diagnostic,indent=2))
    print(f'Open {run.resolve()/"responses.png"}')


def main():
    arguments=parse_args()
    return analyze(arguments) if arguments.analyze else collect(arguments)


if __name__=='__main__':
    raise SystemExit(cli_entry(main))
