"""Run a real J-turn plus RGB probe; bundle evidence without assuming success.

Use --server-exe to launch a LOCAL CARLA 0.9.15 instance with a fresh renderer
log. Without it, attach to an existing server; GPU attribution stays unverified.
This script does not rent cloud GPU resources or download a CARLA server.
"""
import argparse
import csv
from datetime import datetime,timezone
import hashlib
import json
import math
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
import traceback
from pathlib import Path
import zipfile

from carla_runtime import cli_entry,save_json,validate_connection
from chay_quy_trinh import ROOT,run_step


def parse_renderer(text):
    """Conservative: an enumerated GPU alone is never a selected GPU."""
    devices={};selected=None;backend=None;lines=[];selection=None
    for line in text.splitlines():
        if not re.search(r'Log(?:Vulkan|D3D11|D3D12|RHI)',line,re.I):continue
        if re.search(r'adapter|device\s+\d|adapter name',line,re.I):lines.append(line)
        m=re.search(r'LogVulkanRHI:.*?\bDevice\s+(\d+):\s*(.+)',line,re.I)
        if m and not re.search(r'\bUsing\s+Device',line,re.I):devices[('Vulkan',m[1])]=m[2].strip()
        m=re.search(r'Found\s+(D3D11|D3D12)\s+adapter\s+(\d+):\s*(.+)',line,re.I)
        if m:devices[(m[1].upper(),m[2])]=m[3].strip()
        m=re.search(r'Using\s+Device\s+(\d+):',line,re.I)
        if m and 'VulkanRHI' in line:selection=('Vulkan',m[1])
        m=re.search(r'Chosen\s+(D3D11|D3D12)\s+Adapter(?:\s+Id)?\s*[:=]\s*(\d+)',line,re.I)
        if m:selection=(m[1].upper(),m[2])
        m=re.search(r'Adapter Name:\s*(.+)',line,re.I)
        if m and selection:devices[selection]=m[1].strip()
    if selection:backend=selection[0];selected=devices.get(selection)
    software=bool(selected and re.search(r'llvmpipe|lavapipe|swiftshader|software|basic render|\bwarp\b',selected,re.I))
    hardware=bool(selected and not software and re.search(r'NVIDIA|GeForce|Quadro|Tesla|RTX|GTX|Radeon|AMD|Intel.*(?:Graphics|Iris|Arc|UHD)',selected,re.I))
    return {'selected_adapter':selected,'backend':backend,'hardware_adapter_recognized':hardware,
        'software_adapter_recognized':software,'selected_index':selection[1] if selection else None,
        'evidence_lines':lines[-100:],'interpretation':'Only a chosen/using adapter is considered; listing an available GPU is insufficient.'}


def inspect_motion(run):
    status=json.loads((run/'run_status.json').read_text(encoding='utf-8'))
    meta=json.loads((run/'metadata.json').read_text(encoding='utf-8'))
    with (run/'states.csv').open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
    failures=[]
    if status.get('result')!='duration_complete':failures.append('run_not_complete')
    if not str(meta.get('server_version','')).startswith('0.9.15'):failures.append('wrong_server_version')
    if not str(meta.get('client_version','')).startswith('0.9.15'):failures.append('wrong_client_version')
    source=meta.get('data_source','').lower()
    if 'carla' not in source or re.search(r'synthetic|fixture|mock|fake',source):failures.append('not_live_carla_provenance')
    if len(rows)<300:failures.append('too_few_samples')
    required=['t_s','X_m','Y_m','psi_rad','vx_mps','delta_equiv_rad','frame']
    values={key:[float(row[key]) for row in rows] for key in required}
    if not all(math.isfinite(x) for seq in values.values() for x in seq):failures.append('nonfinite_data')
    if any(b<=a for a,b in zip(values['t_s'],values['t_s'][1:])):failures.append('nonmonotonic_time')
    if any(b-a!=1 for a,b in zip(values['frame'],values['frame'][1:])):failures.append('nonconsecutive_frames')
    dt=float(meta.get('dt_s',.01))
    if any(abs((b-a)-dt)>max(1e-6,dt*.01) for a,b in zip(values['t_s'],values['t_s'][1:])):
        failures.append('time_step_mismatch')
    duration=values['t_s'][-1]-values['t_s'][0] if rows else 0.
    distance=math.hypot(values['X_m'][-1]-values['X_m'][0],values['Y_m'][-1]-values['Y_m'][0]) if rows else 0.
    delta=max(map(abs,values['delta_equiv_rad']),default=0.)
    entry=values['vx_mps'][0]*3.6 if rows else 0.
    if duration<5.9:failures.append('maneuver_duration_too_short')
    if abs(entry-30)>.75:failures.append('entry_speed_outside_diagnostic_tolerance')
    if distance<20:failures.append('vehicle_did_not_move_enough')
    if delta<math.radians(.5):failures.append('wheel_angle_did_not_respond')
    return {'ok':not failures,'failures':failures,'samples':len(rows),'duration_s':duration,
        'displacement_m':distance,'entry_speed_kmh':entry,'max_abs_wheel_angle_deg':math.degrees(delta),
        'note':'Software execution diagnostics for one 30 km/h case; not model accuracy or ISO acceptance.'}


def gpu_inventory():
    exe=shutil.which('nvidia-smi')
    if not exe:return {'available':False,'note':'nvidia-smi unavailable; this does not rule out AMD/Intel hardware.'}
    try:
        r=subprocess.run([exe,'--query-gpu=name,driver_version,memory.total,utilization.gpu,memory.used',
            '--format=csv,noheader'],capture_output=True,text=True,timeout=8)
        return {'available':r.returncode==0,'output':r.stdout.strip(),'error':r.stderr.strip(),
                'scope':'local client host only; inventory alone does not prove CARLA GPU use'}
    except Exception as exc:return {'available':False,'error':str(exc)}


def port_open(host,port):
    try:
        with socket.create_connection((host,port),timeout=.5):return True
    except OSError:return False


def make_bundle(out):
    files=[p for p in out.rglob('*') if p.is_file() and p.name not in
           ('server_engine.log','server_console.log','file_hashes.json')]
    hashes={str(p.relative_to(out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    save_json(out/'file_hashes.json',hashes);files.append(out/'file_hashes.json')
    target=out.with_name(out.name+'_MINH_CHUNG.zip')
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files):z.write(p,Path(out.name)/p.relative_to(out))
    return target


def execute(a):
    a.out=(a.out or ROOT/'ket_qua_v4'/f'xac_nhan_{datetime.now():%Y%m%d_%H%M%S_%f}').resolve()
    a.out.mkdir(parents=True,exist_ok=False)
    started=time.time();process=None;console=None;engine_log=None;bound=False
    report={'created_utc':datetime.now(timezone.utc).isoformat(),'client_host_os':platform.platform(),
        'python_version':platform.python_version(),'server_host':a.host,'server_port':a.port,
        'server_simulation_confirmed':False,'camera_rendering_confirmed':False,
        'hardware_gpu_confirmed':False,'result':'not_completed','scope':'One 30 km/h J-turn and a separate RGB rendering probe.'}
    code=1
    try:
        from kiem_tra_moi_truong import check
        offline=check(a.host,a.port,min(a.timeout,15),a.vehicle,True)
        save_json(a.out/'client_environment.json',offline)
        if not offline['ok']:raise RuntimeError('Client environment failed; run 00_CAI_DAT_WINDOWS.bat first.')
        report['gpu_inventory_before']=gpu_inventory()
        if a.server_exe:
            if a.host not in ('127.0.0.1','localhost'):raise ValueError('--server-exe can only launch a local server.')
            exe=a.server_exe.resolve()
            if not exe.is_file():raise FileNotFoundError(exe)
            if exe.suffix.lower() not in ('.exe','.sh'):raise ValueError('Choose CarlaUE4.exe or CarlaUE4.sh.')
            if port_open(a.host,a.port):raise RuntimeError('CARLA port already open. Close the existing local server before --server-exe launch.')
            engine_log=a.out/'server_engine.log'
            command=[str(exe),'-quality-level=Low','-ResX=1280','-ResY=720',f'-carla-rpc-port={a.port}',
                     f'-abslog={engine_log}','-log']
            if a.offscreen:command.append('-RenderOffScreen')
            console=(a.out/'server_console.log').open('wb')
            process=subprocess.Popen(command,cwd=exe.parent,stdout=console,stderr=subprocess.STDOUT)
            bound=True;report['server_launch']={'pid':process.pid,'executable':str(exe),'command':command}
            print('Dang cho CARLA server khoi dong...',flush=True)
        import carla
        deadline=time.monotonic()+a.startup_timeout;last_progress=0;last_error=None
        while True:
            try:
                client=carla.Client(a.host,a.port);client.set_timeout(min(a.timeout,3.))
                versions=validate_connection(client);client.get_world();break
            except RuntimeError as exc:
                last_error=str(exc)
                if 'Matching CARLA' in last_error:raise
                if process is not None and process.poll() not in (None,0):
                    raise RuntimeError('CARLA server exited; inspect server_console.log/server_engine.log.')
                if time.monotonic()>=deadline:raise RuntimeError('CARLA server connection timeout: '+last_error)
                if time.monotonic()-last_progress>10:
                    print('Chua ket noi duoc server; dang doi...',flush=True);last_progress=time.monotonic()
                time.sleep(.5)
        report.update(client_version=versions[0],server_version=versions[1])
        common=['--host',a.host,'--port',a.port,'--timeout',a.timeout,'--vehicle',a.vehicle]
        run_step('chay_quy_trinh.py',['smoke',*common,'--out',a.out/'smoke'],a.out/'logs')
        report['motion']=inspect_motion(a.out/'smoke/jturn_30_dry')
        report['server_simulation_confirmed']=report['motion']['ok']
        if not report['motion']['ok']:raise RuntimeError('Motion diagnostics failed: '+str(report['motion']['failures']))
        run_step('kiem_tra_render.py',[*common,'--out',a.out/'camera'],a.out/'logs')
        camera=json.loads((a.out/'camera/render_report.json').read_text(encoding='utf-8'))
        report['camera_rendering_confirmed']=(camera['result']=='rendering_observed' and camera['usable_frame_count']>=3)
        if engine_log is not None and engine_log.is_file():
            text=engine_log.read_text(encoding='utf-8',errors='replace');rhi=parse_renderer(text)
            fresh=engine_log.stat().st_mtime>=started-2
            report['renderer']=rhi
            report['renderer_log_bound_to_launch']=bound and fresh
            report['renderer_log_sha256']=hashlib.sha256(engine_log.read_bytes()).hexdigest()
            (a.out/'renderer_excerpt.txt').write_text('\n'.join(rhi['evidence_lines']),encoding='utf-8')
            report['hardware_gpu_confirmed']=bool(bound and fresh and rhi['hardware_adapter_recognized'] and report['camera_rendering_confirmed'])
        else:report['renderer_note']='Fresh selected-adapter log unavailable. GPU attribution remains unverified.'
        report['gpu_inventory_after']=gpu_inventory()
        report['result']='server_and_gpu_confirmed' if report['hardware_gpu_confirmed'] else 'server_and_camera_confirmed_gpu_unverified'
        code=0 if report['hardware_gpu_confirmed'] else 2
    except BaseException as exc:
        report.update(result='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',
                      error_type=type(exc).__name__,message=str(exc))
        (a.out/'error_traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
        print(f'CHUA XAC NHAN: {exc}',flush=True)
        code=130 if isinstance(exc,KeyboardInterrupt) else 1
    finally:
        # Leave only the server launched by the user-requested check running;
        # users can inspect/close its window. Never kill unrelated processes.
        report['server_started_by_check']=process is not None
        if process is not None:report['server_launcher_exit_code']=process.poll()
        if console is not None:console.close()
        save_json(a.out/'xac_nhan.json',report)
        labels={True:'DAT',False:'CHUA XAC NHAN'}
        text=(f"MO PHONG SERVER: {labels[report['server_simulation_confirmed']]}\n"
              f"CAMERA RENDER: {labels[report['camera_rendering_confirmed']]}\n"
              f"GPU PHAN CUNG: {labels[report['hardware_gpu_confirmed']]}\n"
              f"Trang thai: {report['result']}\n"
              "Pham vi: mot bai J-turn 30 km/h va mot phep thu camera.\n"
              "Khong phai ket luan mo hinh chinh xac hay chung nhan ISO.\n")
        (a.out/'KET_QUA_XAC_NHAN.txt').write_text(text,encoding='utf-8')
        bundle=make_bundle(a.out)
        print(text,flush=True);print('TEP MINH CHUNG: '+str(bundle),flush=True)
        if process is not None:print('CARLA duoc giu mo; dong cua so CARLA khi khong dung nua.',flush=True)
    return code


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--server-exe',type=Path,help='Local CarlaUE4.exe/.sh; creates a fresh renderer log')
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=90.);p.add_argument('--startup-timeout',type=float,default=120.)
    p.add_argument('--vehicle',default='vehicle.tesla.model3');p.add_argument('--out',type=Path)
    p.add_argument('--offscreen',action='store_true');a=p.parse_args()
    if not (1<=a.port<=65535 and 1<=a.timeout<=600 and 1<=a.startup_timeout<=600):p.error('Invalid port/timeout')
    return execute(a)


if __name__=='__main__':raise SystemExit(cli_entry(main))
