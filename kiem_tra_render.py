"""Acquire real CARLA RGB frames on the dedicated pad after a maneuver run.

Camera output verifies rendering, while hardware GPU attribution is checked
separately from the renderer's log. No image is synthesized by this script.
"""
import argparse
import hashlib
import queue
import time
from pathlib import Path
from carla_runtime import (checked_tick, cleanup, cli_entry, run_record,
                           save_json, validate_connection)


def probe(args):
    import carla
    import numpy as np
    with run_record(args.out) as status:
        world=None;old=None;owned=[];events=queue.Queue();images=queue.Queue(maxsize=12)
        frames={};details=[];warnings=[]
        try:
            client=carla.Client(args.host,args.port);client.set_timeout(args.timeout)
            versions=validate_connection(client);world=client.get_world();old=world.get_settings()
            s=world.get_settings();s.synchronous_mode=True;s.fixed_delta_seconds=.05
            s.substepping=True;s.max_substeps=10;s.max_substep_delta_time=.005
            s.no_rendering_mode=False;world.apply_settings(s)
            if world.get_settings().no_rendering_mode:
                raise RuntimeError('Server still has no_rendering_mode=True.')
            wp=world.get_map().get_waypoint_xodr(1,-1,100.)
            if wp is None:raise RuntimeError('Run the J-turn workflow first to create the dedicated pad.')
            tr=wp.transform;tr.location.y=0.;tr.location.z+=.5
            ego=world.try_spawn_actor(world.get_blueprint_library().find(args.vehicle),tr)
            if ego is None:raise RuntimeError('Cannot spawn render-probe vehicle asset.')
            owned.append(ego);ego.apply_control(carla.VehicleControl(brake=1.))
            for _ in range(40):checked_tick(world,timeout=args.timeout,expected_dt=.05)
            bp=world.get_blueprint_library().find('sensor.camera.rgb')
            for key,value in [('image_size_x','640'),('image_size_y','360'),('fov','90'),('sensor_tick','0.0')]:
                bp.set_attribute(key,value)
            camera=world.spawn_actor(bp,carla.Transform(carla.Location(x=-6,z=3),
                carla.Rotation(pitch=-15)),attach_to=ego);owned.append(camera)
            def receive(im):
                try:images.put_nowait(im)
                except queue.Full:
                    try:images.get_nowait()
                    except queue.Empty:pass
                    try:images.put_nowait(im)
                    except queue.Full:pass
            camera.listen(receive)
            first_frame=world.get_snapshot().frame;last_frame=first_frame
            for k in range(80):
                last_frame=checked_tick(world,timeout=args.timeout,expected_dt=.05)
                # GPU camera delivery can lag simulator snapshots. Keep its own
                # frame ID, accept delayed frames, and never fabricate a match.
                try:im=images.get(timeout=.1)
                except queue.Empty:continue
                if first_frame<im.frame<=last_frame and im.frame not in frames:
                    frames[im.frame]=im
                    if len(frames)>8:del frames[min(frames)]
                if len(frames)>=5 and k>=20:break
            deadline=time.monotonic()+3
            while len(frames)<3 and time.monotonic()<deadline:
                try:im=images.get(timeout=.2)
                except queue.Empty:continue
                if first_frame<im.frame<=last_frame:frames[im.frame]=im
            for frame,im in sorted(frames.items()):
                raw=bytes(im.raw_data)
                expected=int(im.width)*int(im.height)*4
                if len(raw)!=expected or expected==0:
                    warnings.append(f'Invalid RGB buffer for frame {frame}');continue
                rgb=np.frombuffer(raw,dtype=np.uint8).reshape(im.height,im.width,4)[:,:,:3]
                stats={'frame':int(frame),'timestamp_s':float(im.timestamp),
                    'width':int(im.width),'height':int(im.height),
                    'rgb_std':float(rgb.astype(float).std(axis=(0,1)).mean()),
                    'raw_sha256':hashlib.sha256(raw).hexdigest()}
                # Nonuniform pixels are a diagnostic, not an image quality score.
                if stats['rgb_std']<=1.0:stats['usable']=False
                else:
                    stats['usable']=True;path=args.out/f'camera_{frame:08d}.png'
                    im.save_to_disk(str(path));stats['file']=path.name
                    stats['png_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
                details.append(stats)
            usable=[x for x in details if x['usable']]
            ok=len(usable)>=3
            report={'result':'rendering_observed' if ok else 'rendering_not_confirmed',
                'data_source':'live CARLA RGB camera, not generated test fixtures',
                'client_version':versions[0],'server_version':versions[1],
                'world_id':int(world.id),'no_rendering_mode':False,
                'valid_frame_min':first_frame+1,'valid_frame_max':last_frame,
                'frames':details,'usable_frame_count':len(usable),'warnings':warnings,
                'hardware_gpu_attribution':'requires separate selected-adapter log evidence'}
            save_json(args.out/'render_report.json',report)
            status.update(result='duration_complete' if ok else 'rendering_not_confirmed',rows=len(details))
        finally:
            cleanup(world,old,owned,events,args.out,status,carla)
    return 0 if status['result']=='duration_complete' else 2


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.);p.add_argument('--vehicle',default='vehicle.tesla.model3')
    p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    if not (1<=a.port<=65535 and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    return probe(a)


if __name__=='__main__':raise SystemExit(cli_entry(main))
