"""Plan/run research maneuvers sequentially on a dedicated CARLA server."""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from carla_runtime import cli_entry, validate_connection


def make_plan(args):
    plan=[]
    for maneuver in ['jturn','double_pulse']:
        for speed in [30,50,80]:
            for mu,friction in [(1.0,args.dry_friction),(.4,args.wet_friction)]:
                for direction in args.directions:
                    for repeat in range(1,args.repeats+1):
                        name=f'{maneuver}_v{speed}_mu{mu:g}_dir{direction}_r{repeat:02d}'
                        cmd=[sys.executable,str(Path(__file__).with_name('kiem_thu_maneuver.py')),
                            '--maneuver',maneuver,'--speed-kmh',str(speed),'--mu-label',str(mu),
                            '--tire-friction',str(friction),'--delta-deg',str(args.delta_deg),
                            '--direction',str(direction),'--host',args.host,'--port',str(args.port),
                            '--vehicle',args.vehicle,'--timeout',str(getattr(args,'timeout',90.0)),'--out',str(args.out/name)]
                        if args.wheelbase is not None:cmd+=['--wheelbase',str(args.wheelbase)]
                        if mu<1 and args.rain_visual:cmd+=['--rain-visual']
                        plan.append({'name':name,'maneuver':maneuver,'speed_kmh':speed,
                            'mu_nominal':mu,'tire_friction_scalar':friction,
                            'effective_mu_verified':False,'direction':direction,
                            'repeat':repeat,'command':cmd,'result':'not_run'})
    return plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--dry-friction',type=float,required=True)
    p.add_argument('--wet-friction',type=float,required=True)
    p.add_argument('--repeats',type=int,default=3)
    p.add_argument('--directions',type=int,nargs='+',choices=[-1,1],default=[1])
    p.add_argument('--delta-deg',type=float,default=2)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.0)
    p.add_argument('--continue-on-error',action='store_true',help='Keep running after a failed experiment')
    p.add_argument('--vehicle',default='vehicle.tesla.model3');p.add_argument('--wheelbase',type=float)
    p.add_argument('--rain-visual',action='store_true')
    p.add_argument('--execute',action='store_true',help='Actually run CARLA; otherwise save plan only')
    a=p.parse_args();a.out=a.out.resolve()
    if not (1<=a.port<=65535 and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    if not (0<a.repeats<=100 and all(math.isfinite(f) and 0<f<=10 for f in [a.dry_friction,a.wet_friction])):
        p.error('Positive repeats <= 100 and finite friction scalars in (0,10] required')
    if not (math.isfinite(a.delta_deg) and 0<a.delta_deg<=10):p.error('delta-deg in (0,10]')
    if len(set(a.directions))!=len(a.directions):p.error('Duplicate directions')
    if a.wheelbase is not None and (not math.isfinite(a.wheelbase) or a.wheelbase<=0):
        p.error('Positive finite wheelbase required')
    a.out.mkdir(parents=True,exist_ok=False);plan=make_plan(a)
    file=a.out/'ma_tran.json'
    def save():file.write_text(json.dumps(plan,indent=2),encoding='utf-8')
    save();print(f'{len(plan)} planned runs; effective mu and ISO conformity remain unverified.')
    if not a.execute:
        print('Plan only. Choose a NEW --out and add --execute to run.');return
    # Check connection BEFORE the first world replacement.
    import carla
    client=carla.Client(a.host,a.port);client.set_timeout(a.timeout);validate_connection(client)
    failed=0;completed=0
    for item in plan:
        print(f'Running {item["name"]}',flush=True)
        result=subprocess.run(item['command'],check=False)
        item['exit_code']=result.returncode;item['result']='process_success' if result.returncode==0 else 'failed'
        status=a.out/item['name']/'run_status.json'
        if status.is_file():item['run_status']=json.loads(status.read_text(encoding='utf-8'))
        save();completed+=1;failed+=int(result.returncode!=0)
        if result.returncode and not a.continue_on_error:
            print('Stopped after failed run. Inspect run_status; remaining runs are not_run.');break
    print(f'Finished {completed}/{len(plan)} processes; {failed} failed. Inspect each run_status before comparison.')
    return 2 if failed else 0


if __name__=='__main__':raise SystemExit(cli_entry(main))
