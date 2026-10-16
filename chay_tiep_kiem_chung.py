"""Resume the six-run baseline, preserve old/partial runs, and compare models.

Completion means data were collected and comparisons computed, not that the
model passed scientific acceptance. Requires a reachable CARLA 0.9.15 server.
"""
import argparse
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import traceback
import zipfile
from carla_runtime import cli_entry,save_json
from chay_quy_trinh import ROOT,run_step


def classify_run(path,case):
    if not path.exists():return False,'missing'
    try:
        from so_sanh_mo_hinh import read_run
        df,meta=read_run(path)
        source=meta.get('data_source','').lower()
        if 'carla' not in source or any(w in source for w in ('synthetic','fixture','mock','fake')):
            return False,'source_not_live_carla'
        if meta.get('case')!=case:return False,'case_mismatch'
        if len(df)<800 or float(df.t_s.iloc[-1]-df.t_s.iloc[0])<17.9:
            return False,'too_short_for_baseline'
        if any(not str(meta.get(k,'')).startswith('0.9.15') for k in ('client_version','server_version')):
            return False,'version_mismatch'
        return True,'complete_valid_log'
    except Exception as exc:return False,f'{type(exc).__name__}: {exc}'


def quality_report(runs):
    from so_sanh_mo_hinh import read_run,consistent_vehicle
    loaded=[read_run(path) for path in runs]
    consistent_vehicle(loaded)
    checks=[]
    for path,(df,meta) in zip(runs,loaded):
        moving=df.vx_mps>.8
        braking=moving&(df.brake_applied>0)
        steering=(df.vx_mps>1)&(df.delta_equiv_rad.abs()>.002)
        flags=[]
        if meta.get('case')=='doc' and int(braking.sum())<50:
            flags.append('few_braking_samples_in_identification_domain')
        if meta.get('case')!='doc' and int(steering.sum())<50:
            flags.append('few_steering_samples')
        if float(df.ax_mps2.abs().max())>9.81:
            flags.append('longitudinal_transients_above_1g_inspect_raw_data')
        checks.append({'run':path.name,'samples':len(df),'state_sha256':df.attrs['state_sha256'],
            'braking_samples_vx_gt_0_8':int(braking.sum()),'steering_samples_vx_gt_1':int(steering.sum()),
            'vx_min_mps':float(df.vx_mps.min()),'vx_max_mps':float(df.vx_mps.max()),'flags':flags})
    return {'runs':checks,'screen_note':'Counts and 1g are software diagnostics, not acceptance criteria or grounds to silently remove samples.'}


def execute(a):
    root=a.baseline.resolve()
    if not root.is_dir():raise FileNotFoundError(root)
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out=root/f'kiem_chung_tiep_{stamp}';out.mkdir()
    report={'result':'incomplete','model_validated':False,'reused_runs':[],
        'collected_runs':[],'preserved_partial_runs':[],
        'acceptance':'No scientific acceptance criteria were specified; no automatic validation claim.'}
    code=1;all_runs=[];parameters=out/'tham_so.json'
    try:
        common=['--host',a.host,'--port',a.port,'--timeout',a.timeout,'--vehicle',a.vehicle]
        run_step('kiem_tra_moi_truong.py',common+['--out',out/'moi_truong.json'],out/'logs')
        train=[];tests=[]
        for kind,drive,speed,amp,freq in [('train',.12,2.,.04,.15),('test',.16,3.,.035,.25)]:
            group=[]
            for case in ['doc','ngang','phoi_hop']:
                path=root/f'{kind}_{case}';ok,reason=classify_run(path,case)
                if ok:
                    print('TAI SU DUNG: '+str(path),flush=True);report['reused_runs'].append(path.name)
                else:
                    if path.exists():
                        saved=root/'_luot_chua_xong'/f'{path.name}_{stamp}'
                        saved.parent.mkdir(exist_ok=True);shutil.move(str(path),str(saved))
                        report['preserved_partial_runs'].append({'run':path.name,'reason':reason,'saved':str(saved)})
                    arguments=common+['--case',case,'--drive',drive,'--speed',speed,
                        '--steer-amplitude',amp,'--frequency',freq,'--out',path,
                        '--world-mode',getattr(a,'world_mode','create'),'--trace-stages']
                    if a.observe:arguments.append('--observe')
                    run_step('buoc1_carla.py',arguments,out/'logs')
                    ok,reason=classify_run(path,case)
                    if not ok:raise ValueError(f'{path.name}: {reason}')
                    report['collected_runs'].append(path.name)
                group.append(path);all_runs.append(path)
                if not (path/'responses.png').is_file():
                    run_step('buoc1_carla.py',['--analyze',path],out/'logs')
            if kind=='train':
                train=group
                run_step('so_sanh_mo_hinh.py',['fit','--train',*train,'--out',parameters],out/'logs')
            else:tests=group
        quality=quality_report(all_runs);save_json(out/'data_quality.json',quality)
        comparison=out/'so_sanh'
        try:
            run_step('so_sanh_mo_hinh.py',['compare','--parameters',parameters,
                     '--runs',*tests,'--out',comparison],out/'logs')
        except RuntimeError:
            if not (comparison/'validation_report.json').is_file():raise
            report['comparison_process_failed']=True
        validation=json.loads((comparison/'validation_report.json').read_text(encoding='utf-8'))
        p=json.loads(parameters.read_text(encoding='utf-8'))
        report.update(result='comparison_ready_for_review',fit_warnings=p['fit_diagnostics']['warnings'],
            model_failures=[x for run in validation['runs'] for x in run.get('model_failures',[])],
            data_quality_flags=[{'run':x['run'],'flags':x['flags']} for x in quality['runs'] if x['flags']],
            comparison_directory=str(comparison),
            scope='Low-speed baseline only, nominal 2/3 m/s. Does not validate 30/50/80 km/h or low-friction operation.')
        if report['model_failures'] or report['fit_warnings'] or report['data_quality_flags']:
            report['result']='comparison_has_limitations'
        code=2
    except BaseException as exc:
        report.update(result='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',
                      error_type=type(exc).__name__,message=str(exc))
        (out/'error_traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
        print(f'CHUA HOAN THANH: {type(exc).__name__}: {exc}',flush=True)
        code=130 if isinstance(exc,KeyboardInterrupt) else 1
    finally:
        save_json(out/'tiep_tuc_status.json',report)
        (out/'KET_LUAN.txt').write_text(
            f'Trang thai: {report["result"]}\nMo hinh da duoc kiem chung: CHUA THE KET LUAN\n'
            'Can xem sai so doc lap, pham vi toc do, canh bao du lieu va tieu chi chap nhan.\n'
            'Khong dung du lieu nhan dang lam bang chung kiem thu doc lap.\n',encoding='utf-8')
        bundle=root.with_name(root.name+f'_TIEP_{stamp}.zip')
        files=set(p for p in out.rglob('*') if p.is_file())
        for path in all_runs:
            files.update(p for p in path.rglob('*') if p.is_file())
        for name in ['train_doc','train_ngang','train_phoi','test_doc','test_ngang','test_phoi']:
            path=root/name
            if path.is_dir():files.update(p for p in path.rglob('*') if p.is_file())
        with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
            for p in sorted(files):z.write(p,Path(root.name)/p.relative_to(root))
        print('KET QUA: '+str(out),flush=True)
        print('GUI LAI TEP: '+str(bundle),flush=True)
    return code


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.)
    p.add_argument('--vehicle',default='vehicle.tesla.model3');p.add_argument('--observe',action='store_true')
    p.add_argument('--world-mode',choices=['create','auto','reuse'],default='create',
                   help='Use auto to bypass regeneration when recovering on an identical test road')
    a=p.parse_args()
    if not (1<=a.port<=65535 and math.isfinite(a.timeout) and 1<=a.timeout<=600):p.error('Invalid port/timeout')
    return execute(a)


if __name__=='__main__':raise SystemExit(cli_entry(main))
