"""Reuse only an identical test road, without reloading the CARLA world.

Intended for recovering/diagnosing a stopped run, not deterministic repetitions.
Fresh vehicle actors are still required for every acquisition.
"""
import hashlib

ROLE='model_validation_ego'


def prepare_world(client, road, parameters, mode, status, trace):
    if mode not in ('create','auto','reuse'):
        raise ValueError('Unknown world mode: '+str(mode))
    info={'requested_mode':mode,'reused':False,'removed_previous_actor_ids':[],
          'expected_opendrive_sha256':hashlib.sha256(road.strip().encode('utf-8')).hexdigest()}
    status['world_setup']=info
    if mode!='create':
        trace.stage('checking_current_road','Kiem tra noi dung OpenDRIVE cua ban do dang mo.')
        current=client.get_world()
        observed=current.get_map().to_opendrive().strip()
        info['current_opendrive_sha256']=hashlib.sha256(observed.encode('utf-8')).hexdigest()
        if observed==road.strip():
            trace.stage('reusing_test_road','Duong thu trung khop; bo qua tao lai ban do.')
            actors=list(current.get_actors())
            previous=[a for a in actors if a.type_id.startswith('vehicle.')
                      and a.attributes.get('role_name')==ROLE]
            previous_ids={a.id for a in previous}
            foreign=[a.id for a in actors if a.type_id.startswith(('vehicle.','walker.'))
                     and a.id not in previous_ids]
            if foreign:
                raise RuntimeError('Cannot reuse occupied test road; foreign vehicle/walker actors: '
                                   +str(foreign)+'. Use a dedicated session or --world-mode create.')
            trace.stage('removing_previous_test_vehicle','Don xe thu cu va cam bien gan voi xe do.')
            sensors=[a for a in actors if a.type_id.startswith('sensor.')
                     and a.parent is not None and a.parent.id in previous_ids]
            for actor in sensors+previous:
                if not actor.is_alive:continue
                if actor.type_id.startswith('sensor.'):actor.stop()
                if not actor.destroy():
                    raise RuntimeError('Cannot remove previous test actor '+str(actor.id))
                info['removed_previous_actor_ids'].append(actor.id)
            info.update(reused=True,
                        repetition_scope='Reused world with new vehicle; deterministic world reload not performed')
            return current,current.get_settings()
        if mode=='reuse':
            raise RuntimeError('Current OpenDRIVE does not exactly match the test road. '
                               'No actors or settings changed. Use --world-mode auto or create.')
    print('Creating flat road; the current world and its actors will be replaced.',flush=True)
    trace.stage('creating_world','Dang tao lai duong OpenDRIVE; chua tao xe, chua thu mau.')
    world=client.generate_opendrive_world(road,parameters)
    return world,world.get_settings()
