"""Read-only preflight: Python, dependencies, client/server versions and assets.

No world replacement, vehicle creation, settings change or simulator tick.
"""
import argparse
import importlib
import json
import platform
import struct
import sys
from pathlib import Path
from carla_runtime import cli_entry, error_hint, save_json, validate_connection


def check(host='127.0.0.1',port=2000,timeout=15.0,vehicle='vehicle.tesla.model3',offline=False):
    checks=[]
    def add(name,ok,details):
        checks.append({'name':name,'ok':bool(ok),'details':str(details)})
    add('Python', sys.version_info[:2]==(3,10) and struct.calcsize('P')==8,
        f'{platform.python_version()} | {struct.calcsize("P")*8} bit | {sys.executable}; require 3.10 x64')
    for module in ['numpy','pandas','matplotlib','scipy','carla']:
        try:
            lib=importlib.import_module(module)
            version=getattr(lib,'__version__','available')
            if module=='numpy':
                add(module,int(version.split('.')[0])<2,version+'; this package requires numpy<2')
            elif module=='carla':
                version=lib.Client(host,port).get_client_version()
                add('CARLA client',version=='0.9.15',version)
                missing=[name for name in ['Client','Map','VehicleWheelLocation','OpendriveGenerationParameters']
                         if not hasattr(lib,name)]
                add('CARLA API',not missing,'missing: '+str(missing))
            else:add(module,True,version)
        except Exception as exc:
            add(module,False,f'{type(exc).__name__}: {exc}. {error_hint(exc)}')
    if not offline and all(c['ok'] for c in checks):
        try:
            carla=importlib.import_module('carla');client=carla.Client(host,port);client.set_timeout(timeout)
            versions=validate_connection(client);world=client.get_world()
            add('CARLA server',True,versions[1])
            lib=world.get_blueprint_library()
            bp=lib.find(vehicle)
            wheels=bp.get_attribute('number_of_wheels').as_int() if bp.has_attribute('number_of_wheels') else None
            add('Vehicle asset',wheels in (4,None),f'{bp.id}; wheels={wheels}')
            collision=lib.find('sensor.other.collision')
            add('Collision sensor',True,collision.id)
            s=world.get_settings()
            add('World',True,f'{world.get_map().name}; synchronous_mode={s.synchronous_mode}. Stop other ticking clients before running.')
        except Exception as exc:
            add('CARLA connection/assets',False,f'{type(exc).__name__}: {exc}. {error_hint(exc)}')
    return {'ok':all(c['ok'] for c in checks),'mode':'offline' if offline else 'server_read_only',
            'checks':checks,'server_maneuver_tested':False}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=2000)
    p.add_argument('--timeout',type=float,default=15.0);p.add_argument('--vehicle',default='vehicle.tesla.model3')
    p.add_argument('--offline',action='store_true');p.add_argument('--out',type=Path)
    a=p.parse_args()
    if not (1<=a.port<=65535 and 0<a.timeout<=600):p.error('Invalid port/timeout')
    report=check(a.host,a.port,a.timeout,a.vehicle,a.offline)
    for item in report['checks']:print(f'{"[OK]" if item["ok"] else "[LOI]"} {item["name"]}: {item["details"]}')
    if a.out:
        if a.out.exists():raise FileExistsError(a.out)
        a.out.parent.mkdir(parents=True,exist_ok=True);save_json(a.out,report)
    print('KIEM TRA DAT' if report['ok'] else 'CHUA DAT: sua cac dong [LOI] truoc khi chay.')
    return 0 if report['ok'] else 1


if __name__=='__main__':raise SystemExit(cli_entry(main))
