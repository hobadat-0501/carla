"""Offline model comparison of independent maneuver logs with frozen parameters.

Linear baseline and an illustrative static-load friction-circle 3-DOF model.
No tire-force equality, ISO conformity, or real-vehicle validation is claimed.
Optional measured-ax conditioning is a lateral-only study, not longitudinal validation.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
from carla_runtime import cli_entry

import numpy as np
import pandas as pd

from mo_hinh_xe import dynamic_rhs, initial_state, tire_terms, validate_parameters
from so_sanh_mo_hinh import metric, read_run, save_json


def project_circle(fx,fy,limit):
    norm=math.hypot(fx,fy)
    scale=min(1.0,limit/max(norm,1e-12))
    return fx*scale,fy*scale,scale<1.0


def force_terms(state,throttle,brake,delta,p):
    """All forces divided by mass (m/s2). Assumed static axle loads.

    This extends a grey-box longitudinal map; front drive/brake allocation is
    an explicit assumption. The V2 fit did NOT identify these fractions or mu.
    No load transfer, roll, tire relaxation or physical wheel slip is modeled.
    """
    _,_,_,vx,vy,r=state
    fyf,fyr,af,ar,lf,lr=tire_terms(vx,vy,r,delta,p)
    q=p['longitudinal'];c=p['maneuver_model'];L=p['wheelbase_m']
    traction=q['traction_gain']*throttle;braking=q['brake_gain']*brake
    fx_f=c['drive_front_fraction']*traction-c['brake_front_fraction']*braking
    fx_r=(1-c['drive_front_fraction'])*traction-(1-c['brake_front_fraction'])*braking
    fz_f=9.81*lr/L;fz_r=9.81*lf/L
    fx_f,fyf,sf=project_circle(fx_f,fyf,c['mu_nominal']*fz_f)
    fx_r,fyr,sr=project_circle(fx_r,fyr,c['mu_nominal']*fz_r)
    resist=q['rolling_accel']*math.tanh(max(vx,0)/.2)+q['linear_drag']*max(vx,0)
    ax=fx_f*math.cos(delta)-fyf*math.sin(delta)+fx_r-resist
    front_y=fx_f*math.sin(delta)+fyf*math.cos(delta)
    ay=front_y+fyr
    rdot=(lf*front_y-lr*fyr)/p['lateral']['iz_over_mass']
    return ax,ay,rdot,af,ar,sf,sr,fx_f,fyf,fx_r,fyr,fz_f,fz_r


def rhs(state,throttle,brake,delta,p,model,ax_input=None):
    if model=='linear':out=dynamic_rhs(state,throttle,brake,delta,p)
    else:
        X,Y,psi,vx,vy,r=state
        ax,ay,rdot,*_=force_terms(state,throttle,brake,delta,p)
        c,s=math.cos(psi),math.sin(psi)
        out=np.array([vx*c-vy*s,vx*s+vy*c,r,ax+r*vy,ay-r*vx,rdot])
    if ax_input is not None:
        # Known measured output conditions longitudinal motion. Do NOT score ax/vx.
        out[3]=ax_input+state[5]*state[4]
    return out


def rollout_maneuver(df,p,model,condition_ax=False,max_step=.001):
    rows=df.to_dict('records');state=initial_state(rows[0],'dynamic');output=[]
    for i,row in enumerate(rows):
        _,_,af,ar,_,_=tire_terms(state[3],state[4],state[5],row['delta_equiv_rad'],p)
        output.append(dict(zip(['X_m','Y_m','psi_rad','vx_mps','vy_mps','r_radps'],map(float,state)),
            t_s=row['t_s'],beta_rad=math.atan2(state[4],state[3]),
            front_slip_rad=float(af),rear_slip_rad=float(ar)))
        if i==len(rows)-1:break
        nxt=rows[i+1];dt=nxt['t_s']-row['t_s'];n=math.ceil(dt/max_step);h=dt/n
        for j in range(n):
            def f(s,u):
                delta=row['delta_equiv_rad']+(nxt['delta_equiv_rad']-row['delta_equiv_rad'])*u
                ax=(row['ax_mps2']+(nxt['ax_mps2']-row['ax_mps2'])*u) if condition_ax else None
                return rhs(s,row['throttle_next'],row['brake_next'],delta,p,model,ax)
            u=j/n;mid=(j+.5)/n;end=(j+1)/n
            k1=f(state,u);k2=f(state+.5*h*k1,mid)
            k3=f(state+.5*h*k2,mid);k4=f(state+h*k3,end)
            state=state+h*(k1+2*k2+2*k3+k4)/6
            if not np.all(np.isfinite(state)) or np.linalg.norm(state[3:])>300:
                raise FloatingPointError('Divergence or unsupported speed')
            if state[3]<.8:raise ValueError('Model left forward-driving domain vx >= 0.8 m/s')
    out=pd.DataFrame(output);t=out.t_s.to_numpy();r=out.r_radps.to_numpy()
    # Derivatives of predicted velocities for net body acceleration, identical
    # processing for both models. Endpoint samples excluded from scoring.
    out['ax_mps2']=np.gradient(out.vx_mps,t)-r*out.vy_mps
    out['ay_mps2']=np.gradient(out.vy_mps,t)+r*out.vx_mps
    return out


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--parameters',type=Path,required=True)
    a.add_argument('--runs',type=Path,nargs='+',required=True)
    a.add_argument('--out',type=Path,required=True)
    a.add_argument('--drive-front-fraction',type=float,required=True)
    a.add_argument('--brake-front-fraction',type=float,required=True)
    a.add_argument('--condition-ax',action='store_true')
    a.add_argument('--max-step',type=float,default=.001)
    args=a.parse_args()
    if not (0<=args.drive_front_fraction<=1 and 0<=args.brake_front_fraction<=1
            and math.isfinite(args.max_step) and 0<args.max_step<=.01):
        a.error('Finite fractions in [0,1] and max step in (0,.01] required')
    return compare(args)


def compare(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if args.out.exists():raise FileExistsError('Choose a new output directory')
    params=json.loads(args.parameters.read_text(encoding='utf-8'))
    validate_parameters(params)
    training={r['state_sha256'] for r in params['training_runs']}
    runs=[read_run(path) for path in args.runs]
    hashes=[df.attrs['state_sha256'] for df,_ in runs]
    if len(hashes)!=len(set(hashes)):raise ValueError('Duplicate test logs')
    for df,meta in runs:
        if df.attrs['state_sha256'] in training:raise ValueError('Training/test data leakage')
        if meta['vehicle_id']!=params['vehicle_id']:raise ValueError('Different vehicle asset')
        for key in ['mass_kg','wheelbase_m']:
            if abs(meta[key]/params[key]-1)>.005:raise ValueError(f'Different {key}')
        if not (.05<=meta['mu_nominal']<=2):raise ValueError('Unsupported nominal mu')
        if df.vx_mps.min()<.8:raise ValueError('Reference left forward-driving domain')
    args.out.mkdir(parents=True);metrics=[];reports=[]
    for idx,(df,meta) in enumerate(runs):
        name=f'{idx+1:02d}_{Path(df.attrs["path"]).name}'
        dst=args.out/name;dst.mkdir()
        p=copy.deepcopy(params)
        p['maneuver_model']={'mu_nominal':meta['mu_nominal'],
            'drive_front_fraction':args.drive_front_fraction,
            'brake_front_fraction':args.brake_front_fraction}
        mode='measured_ax_conditioned_lateral_only' if args.condition_ax else 'command_replay'
        report={'run':name,'mode':mode,'data_source':meta.get('data_source'),
            'state_sha256':df.attrs['state_sha256'],'failures':[],
            'iso_conformity':'not_assessed','effective_mu_verified':meta.get('effective_mu_verified',False),
            'mu_nominal':meta['mu_nominal'],'tire_friction_readback':meta.get('tire_friction_readback'),
            'force_equality_verified':False,'scientific_acceptance':'not_automatically_assessed',
            'longitudinal_force_balance_enforced':not args.condition_ax,
            'model_assumptions':p['maneuver_model'],
            'entry_speed_actual_kmh':float(df.vx_mps.iloc[0]*3.6),
            'speed_min_kmh':float(df.vx_mps.min()*3.6),'speed_max_kmh':float(df.vx_mps.max()*3.6)}
        if 'delta_target_next_rad' in df:
            # Actuation delay means this is a same-time tracking diagnostic only.
            e=np.degrees(df.delta_equiv_rad-df.delta_target_next_rad)
            report['steering_tracking_same_time_rmse_deg']=float(np.sqrt(np.mean(e**2)))
        df['beta_rad']=np.arctan2(df.vy_mps,df.vx_mps)
        predictions={}
        for model in ['linear','friction_circle']:
            try:pred=rollout_maneuver(df,p,model,args.condition_ax,args.max_step)
            except (FloatingPointError,ValueError,OverflowError) as exc:
                report['failures'].append({'model':model,'error':str(exc)});continue
            pred.to_csv(dst/f'{model}_predictions.csv',index=False);predictions[model]=pred
            report[model+'_max_slip_rad']=float(max(pred.front_slip_rad.abs().max(),pred.rear_slip_rad.abs().max()))
            columns=['X_m','Y_m','vx_mps','vy_mps','ax_mps2','ay_mps2','beta_rad']
            if args.condition_ax:columns=[c for c in columns if c not in ['ax_mps2','vx_mps']]
            for quantity in columns+['r_radps','psi_rad']:
                actual=df['r_reference_radps'] if quantity=='r_radps' else df[quantity]
                error=pred[quantity].to_numpy()-actual.to_numpy()
                if quantity in ['psi_rad','beta_rad']:error=np.arctan2(np.sin(error),np.cos(error))
                for domain,mask in {'whole_run':(df.t_s>.1)&(df.t_s<df.t_s.iloc[-1]-.1),
                                    'steering_and_recovery':(df.t_s>=1)&(df.t_s<df.t_s.iloc[-1]-.1)}.items():
                    metrics.append({'run':name,'model':model,'mode':mode,'domain':domain,
                        'speed_target_kmh':meta.get('speed_target_kmh'),'mu_nominal':meta['mu_nominal'],
                        'quantity':quantity,**metric(np.zeros(int(mask.sum())),error[mask])})
        synthetic='synthetic' in meta.get('data_source','').lower()
        reference='Synthetic software fixture' if synthetic else 'CARLA state reference'
        fig,axs=plt.subplots(4,2,figsize=(12,12))
        fig.suptitle(('SYNTHETIC SOFTWARE CHECK - NOT CARLA RESULTS' if synthetic
            else f'{meta.get("case")} | {meta.get("speed_target_kmh")} km/h | nominal mu={meta["mu_nominal"]}')
            +'\n'+mode+' | ISO conformity and effective mu NOT verified',fontsize=11)
        for ax,q in zip(axs.flat,['vx_mps','vy_mps','r_radps','beta_rad','ax_mps2','ay_mps2']):
            truth='r_reference_radps' if q=='r_radps' else q
            ax.plot(df.t_s,df[truth],label=reference)
            for model,pred in predictions.items():ax.plot(pred.t_s,pred[q],'--',label=model)
            ax.set(xlabel='Simulation time (s)',ylabel=q);ax.legend(fontsize=7);ax.grid(alpha=.2)
        ax=axs[3,0];ax.plot(df.X_m-df.X_m.iloc[0],df.Y_m-df.Y_m.iloc[0],label=reference)
        for model,pred in predictions.items():ax.plot(pred.X_m-df.X_m.iloc[0],pred.Y_m-df.Y_m.iloc[0],'--',label=model)
        ax.set(xlabel='dX (m)',ylabel='dY (m)');ax.axis('equal');ax.legend(fontsize=7);ax.grid(alpha=.2)
        ax=axs[3,1];ax.plot(df.t_s,np.degrees(df.delta_equiv_rad),label='Actual wheel angle replayed')
        if 'delta_target_next_rad' in df:ax.plot(df.t_s,np.degrees(df.delta_target_next_rad),'--',label='Requested wheel angle')
        ax.set(xlabel='Simulation time (s)',ylabel='Front wheel angle (deg)');ax.legend(fontsize=7);ax.grid(alpha=.2)
        fig.tight_layout(rect=(0,0,1,.95));fig.savefig(dst/'comparison.png',dpi=150);plt.close(fig)
        save_json(dst/'run_report.json',report);reports.append(report)
    pd.DataFrame(metrics).to_csv(args.out/'metrics.csv',index=False)
    save_json(args.out/'validation_report.json',{'runs':reports,'real_vehicle_validated':False,
        'parameters_sha256':hashlib.sha256(args.parameters.read_bytes()).hexdigest(),
        'fit_warnings':params['fit_diagnostics']['warnings'],
        'note':'Measured actual steering replay; no force equality claim. Circle model uses nominal mu, assumed axle force split and static loads. Conditioning on measured ax excludes longitudinal verification. No parameter updates on test data.'})
    print(f'Wrote {args.out}; read all failures, fit warnings and unverified friction status.')
    return 2 if any(r['failures'] for r in reports) else 0


if __name__=='__main__':raise SystemExit(cli_entry(main))
