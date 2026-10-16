"""One-command workflows with fresh result folders and complete process logs."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
from carla_runtime import cli_entry, save_json

ROOT=Path(__file__).resolve().parent


def run_step(script,arguments,log_dir):
    log_dir.mkdir(parents=True,exist_ok=True)
    log=log_dir/f'{len(list(log_dir.glob("*.log")))+1:02d}_{script}.log'
    command=[sys.executable,'-u',str(ROOT/script),*map(str,arguments)]
    print('\nCHAY: '+subprocess.list2cmdline(command),flush=True)
    with log.open('w',encoding='utf-8') as output:
        output.write(subprocess.list2cmdline(command)+'\n');output.flush()
        with subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                              text=True,encoding='utf-8',errors='replace',cwd=ROOT) as process:
            try:
                for line in process.stdout:
                    print(line,end='',flush=True);output.write(line);output.flush()
                code=process.wait()
            except BaseException:
                process.terminate()
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:process.kill();process.wait()
                raise
    if code:
        raise RuntimeError(f'{script} dung voi exit code {code}. Doc {log}')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('workflow',choices=['smoke','baseline','matrix'])
    p.add_argument('--out',type=Path)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.0);p.add_argument('--vehicle',default='vehicle.tesla.model3')
    p.add_argument('--dry-friction',type=float,default=1.0);p.add_argument('--wet-friction',type=float,default=.4)
    p.add_argument('--repeats',type=int,default=3)
    a=p.parse_args()
    if not (1<=a.port<=65535 and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    if not (0<a.dry_friction<=10 and 0<a.wet_friction<=10 and 1<=a.repeats<=100):
        p.error('Invalid friction/repeats')
    out=(a.out or ROOT/'ket_qua_v4'/f'{a.workflow}_{datetime.now():%Y%m%d_%H%M%S_%f}').resolve()
    out.mkdir(parents=True,exist_ok=False)
    report={'workflow':a.workflow,'result':'incomplete','python':sys.executable}
    logdir=out/'logs'
    try:
        common=['--host',a.host,'--port',a.port,'--timeout',a.timeout,'--vehicle',a.vehicle]
        run_step('kiem_tra_moi_truong.py',common+['--out',out/'moi_truong.json'],logdir)
        if a.workflow=='smoke':
            run=out/'jturn_30_dry'
            run_step('kiem_thu_maneuver.py',common+['--maneuver','jturn','--speed-kmh',30,
                '--mu-label',1.0,'--tire-friction',a.dry_friction,'--delta-deg',2,'--out',run],logdir)
            run_step('buoc1_carla.py',['--analyze',run],logdir)
            report['responses']=str(run/'responses.png')
        elif a.workflow=='baseline':
            train=[];test=[]
            for kind,drive,speed,amp,freq in [('train',.12,2,.04,.15),('test',.16,3,.035,.25)]:
                runs=[]
                for case in ['doc','ngang','phoi_hop']:
                    run=out/f'{kind}_{case}';runs.append(run)
                    run_step('buoc1_carla.py',common+['--case',case,'--drive',drive,'--speed',speed,
                        '--steer-amplitude',amp,'--frequency',freq,'--out',run],logdir)
                    run_step('buoc1_carla.py',['--analyze',run],logdir)
                if kind=='train':
                    train=runs
                    run_step('so_sanh_mo_hinh.py',['fit','--train',*train,'--out',out/'tham_so.json'],logdir)
                else:test=runs
            run_step('so_sanh_mo_hinh.py',['compare','--parameters',out/'tham_so.json',
                     '--runs',*test,'--out',out/'so_sanh'],logdir)
        else:
            run_step('chay_ma_tran.py',common+['--out',out/'ma_tran','--dry-friction',a.dry_friction,
                '--wet-friction',a.wet_friction,'--directions',1,-1,'--repeats',a.repeats,'--execute'],logdir)
        report['result']='completed'
    except BaseException as exc:
        report.update(result='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',message=str(exc))
        raise
    finally:
        save_json(out/'workflow_status.json',report)
        print(f'KET QUA: {out}',flush=True)
    print('Hoan thanh. Doc run_status va bao cao sai so truoc khi ket luan.',flush=True)
    return 0


if __name__=='__main__':raise SystemExit(cli_entry(main))
