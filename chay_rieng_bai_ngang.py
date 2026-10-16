"""Run only the original baseline lateral training case, with diagnostic logs."""
import argparse
import contextlib
from datetime import datetime
import faulthandler
import math
from pathlib import Path
import platform
import sys
import zipfile
from carla_runtime import cli_entry, save_json

ROOT=Path(__file__).resolve().parent


class Tee:
    def __init__(self, terminal, log):self.terminal=terminal;self.log=log
    def write(self, text):
        self.terminal.write(text);self.log.write(text);self.flush()
        return len(text)
    def flush(self):self.terminal.flush();self.log.flush()


def execute(a, out):
    import buoc1_carla as base
    run=out/'thu_ngang'
    args=base.parse_args(['--case','ngang','--speed','2','--steer-amplitude','0.04',
        '--drive','0.12','--frequency','0.15','--seconds','18','--dt','0.02',
        '--observe','--trace-stages','--host',a.host,'--port',str(a.port),
        '--world-mode',a.world_mode,
        '--timeout',str(a.timeout),'--out',str(run)])
    print('CHI CHAY BAI NGANG: target=2 m/s, steer=0.04 (chuan hoa), dt=0.02 s, 18 s.',flush=True)
    print('Giu CARLA 0.9.15 dang mo; dong cac client khac dang tick.',flush=True)
    print(f'Che do duong thu: {a.world_mode}.',flush=True)
    if a.world_mode!='create':
        print('Dung lai duong neu OpenDRIVE trung khop; don xe thu cu va tao xe moi.',flush=True)
    print('Neu phai tao lai ban do, cac actor cua the gioi cu se bi xoa.',flush=True)
    print(f'Python: {platform.python_version()} | {sys.executable}',flush=True)
    print(f'Server: {a.host}:{a.port} | RPC timeout: {a.timeout:g} s',flush=True)
    print('0-5 s: tang toc; 5-5.5 s: tang lai; 5.5-11 s: giu lai; 11-18 s: phanh.',flush=True)
    code=base.collect(args)
    if code==0:
        print('[BUOC plotting] Ve dap ung tu du lieu vua thu.',flush=True)
        base.analyze(argparse.Namespace(analyze=run,model_file=None))
    return code


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,help='New diagnostic folder; existing data is never overwritten')
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.0)
    p.add_argument('--world-mode',choices=['auto','reuse','create'],default='auto',
                   help='auto: reuse identical road; reuse: never regenerate; create: new world')
    a=p.parse_args()
    if not(1<=a.port<=65535 and math.isfinite(a.timeout) and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    out=(a.out or ROOT/'runs'/f'ngang_quan_sat_{datetime.now():%Y%m%d_%H%M%S_%f}').resolve()
    out.mkdir(parents=True,exist_ok=False)
    with (out/'console.log').open('w',encoding='utf-8') as log, \
         (out/'python_stacks.log').open('w',encoding='utf-8') as stacks:
        with contextlib.redirect_stdout(Tee(sys.stdout,log)),contextlib.redirect_stderr(Tee(sys.stderr,log)):
            print(f'THU MUC CHAN DOAN: {out}',flush=True)
            # Snapshot every 15 wall-clock seconds, also when a C-extension RPC holds the GIL.
            # A snapshot alone does not indicate a deadlock or server failure.
            faulthandler.dump_traceback_later(15,repeat=True,file=stacks)
            try:code=cli_entry(lambda:execute(a,out))
            finally:faulthandler.cancel_dump_traceback_later()
            print(f'MA THOAT: {code}; xem thu_ngang/run_status.json va stage_trace.jsonl.',flush=True)
    summary={'case':'ngang','purpose':'Isolate original baseline training case; diagnose interruption',
             'exit_code':code,'model_validated':False,'python':sys.executable,
             'python_version':platform.python_version(),'server':f'{a.host}:{a.port}',
             'world_mode':a.world_mode}
    save_json(out/'chan_doan_status.json',summary)
    archive=out.with_name(out.name+'.zip')
    with archive.open('xb') as dest,zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as z:
        for path in sorted(out.rglob('*')):
            if path.is_file():z.write(path,path.relative_to(out.parent))
    print(f'TEP GUI LAI: {archive}',flush=True)
    print('Thu xong bai ngang chua phai ket luan kiem chung mo hinh 3-DOF.',flush=True)
    return code


if __name__=='__main__':raise SystemExit(cli_entry(main))
