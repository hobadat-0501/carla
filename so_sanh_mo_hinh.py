"""Fit effective parameters on training runs and compare on distinct test runs.

The fitted parameters describe this illustrative model structure and are not
automatically the physical tire stiffness, inertia or CoM of a CARLA asset.
No automatic scientific acceptance threshold or real-vehicle claim is made.
"""
import argparse
import hashlib
import json
import math
import platform
from pathlib import Path
from carla_runtime import cli_entry

import numpy as np
import pandas as pd
import scipy
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter
from mo_hinh_xe import rollout, tire_terms, validate_parameters


def save_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')


def read_run(path):
    path=Path(path).resolve()
    status=json.loads((path/'run_status.json').read_text(encoding='utf-8'))
    if status.get('result')!='duration_complete':
        raise ValueError(f'{path.name}: incomplete/cutoff experiment: {status.get("result")}')
    metadata=json.loads((path/'metadata.json').read_text(encoding='utf-8'))
    raw=(path/'states.csv').read_bytes()
    df=pd.read_csv(path/'states.csv')
    required=['t_s','X_m','Y_m','psi_rad','vx_mps','vy_mps','ax_mps2','ay_mps2',
              'delta_equiv_rad','throttle_next','brake_next','throttle_applied','brake_applied']
    missing=set(required)-set(df.columns)
    if missing:raise ValueError(f'{path.name}: missing columns {sorted(missing)}')
    if len(df)<30:raise ValueError(f'{path.name}: fewer than 30 samples')
    if not np.isfinite(df[required].to_numpy()).all():raise ValueError('Nonfinite required state/input data')
    intervals=np.diff(df.t_s)
    if np.min(intervals)<=0:raise ValueError('Timestamps must increase strictly')
    dt=float(np.median(intervals))
    if np.max(np.abs(intervals-dt))>dt*.01:raise ValueError('Use uniformly sampled logs for derivative processing')
    # Reference derivatives are computed offline from yaw, never fed to a controller.
    window=max(5,int(round(.18/dt))|1)
    window=min(window,len(df) if len(df)%2 else len(df)-1)
    angle=np.unwrap(df.psi_rad.to_numpy())
    df['r_reference_radps']=savgol_filter(angle,window,3,deriv=1,delta=dt)
    df['rdot_reference_radps2']=savgol_filter(angle,window,3,deriv=2,delta=dt)
    df.attrs.update({'path':str(path),'state_sha256':hashlib.sha256(raw).hexdigest(),
                     'dt':dt,'yaw_derivative_window_samples':window})
    collisions=path/'collision_events.json'
    if collisions.is_file() and json.loads(collisions.read_text(encoding='utf-8')):
        raise ValueError(f'{path.name}: received collision event(s), exclude from free-body identification')
    return df,metadata


def condition_report(matrix):
    # Column-normalized local sensitivity, not a global identifiability proof.
    a=np.asarray(matrix,dtype=float)
    scales=np.linalg.norm(a,axis=0)
    normalized=a/np.where(scales>1e-14,scales,1.0)
    singular=np.linalg.svd(normalized,compute_uv=False)
    rank=int(np.linalg.matrix_rank(normalized))
    cond=float(singular[0]/singular[-1]) if singular[-1]>1e-14 else None
    return {'rank':rank,'columns':a.shape[1],'column_normalized_condition':cond,
            'singular_values':[float(x) for x in singular],
            'interpretation':'Local sensitivity diagnostic only; not physical parameter verification.'}


def consistent_vehicle(runs):
    first=runs[0][1]
    for _,meta in runs:
        if meta['vehicle_id']!=first['vehicle_id']:
            raise ValueError('Do not mix vehicle assets in one parameter fit')
        for key in ['wheelbase_m','mass_kg']:
            if abs(meta[key]/first[key]-1)>.005:
                raise ValueError(f'Do not mix {key} during parameter identification')
    return first


def fit(args):
    if args.out.exists():raise FileExistsError('Parameter file exists; choose a new --out')
    runs=[read_run(p) for p in args.train]
    meta=consistent_vehicle(runs)
    hashes=[df.attrs['state_sha256'] for df,_ in runs]
    if len(set(hashes))!=len(hashes):raise ValueError('Duplicate training data content')
    longitudinal=[];lateral=[]
    for df,m in runs:
        if m.get('case')=='doc':
            mask=(df.vx_mps>.8)&(df.t_s>1)&(df.delta_equiv_rad.abs()<.005)
            longitudinal.append(df[mask])
        else:
            mask=(df.vx_mps>1.0)&(df.t_s>2)&(df.t_s<df.t_s.iloc[-1]-.2)
            mask&=(df.delta_equiv_rad.abs()>.002)|(df.r_reference_radps.abs()>.005)
            lateral.append(df[mask])
    if not longitudinal or not lateral:raise ValueError('Need longitudinal and steering training runs')
    lo=pd.concat(longitudinal,ignore_index=True);la=pd.concat(lateral,ignore_index=True)
    if len(lo)<50 or len(la)<50:raise ValueError('Too few excited samples: collect more data above the speed gate')
    v=lo.vx_mps.to_numpy()
    design=np.column_stack([lo.throttle_applied,-lo.brake_applied,-np.tanh(v/.2),-v])
    target=lo.ax_mps2.to_numpy()
    # Four effective coefficients. Numerical bounds are design assumptions, not vehicle specifications.
    lower=[.01,.01,0.0,0.0];upper=[60.0,60.0,5.0,3.0]
    result=lsq_linear(design,target,bounds=(lower,upper))
    if not result.success:raise RuntimeError('Longitudinal parameter fit failed')
    L=float(meta['wheelbase_m'])
    params={'schema_version':2,'model_name':'example_effective_planar_bicycle',
        'vehicle_id':meta['vehicle_id'],'mass_kg':float(meta['mass_kg']),'wheelbase_m':L,
        'low_speed_end_mps':.8,
        'longitudinal':dict(zip(['traction_gain','brake_gain','rolling_accel','linear_drag'],
                                [float(x) for x in result.x]))}
    vx=la.vx_mps.to_numpy();vy=la.vy_mps.to_numpy();r=la.r_reference_radps.to_numpy()
    delta=la.delta_equiv_rad.to_numpy();ay=la.ay_mps2.to_numpy();rdot=la.rdot_reference_radps2.to_numpy()
    ay_scale=max(float(np.std(ay)),.05);rdot_scale=max(float(np.std(rdot)),.05)
    def unpack(z):
        return {'cf_over_mass':float(np.exp(z[0])),'cr_over_mass':float(np.exp(z[1])),
                'lf_fraction':float(z[2]),'iz_over_mass':float(np.exp(z[3]))}
    def residual(z):
        p=unpack(z);lf=L*p['lf_fraction'];lr=L-lf
        af=delta-np.arctan2(vy+lf*r,vx)
        ar=-np.arctan2(vy-lr*r,vx)
        ff=p['cf_over_mass']*af;fr=p['cr_over_mass']*ar
        ay_model=ff*np.cos(delta)+fr
        rdot_model=(lf*ff*np.cos(delta)-lr*fr)/p['iz_over_mass']
        return np.r_[(ay_model-ay)/ay_scale,(rdot_model-rdot)/rdot_scale]
    bounds=([math.log(2),math.log(2),.2,math.log(.15*L*L)],
            [math.log(200),math.log(200),.8,math.log(1.5*L*L)])
    initial=[math.log(40),math.log(40),.5,math.log(.35*L*L)]
    lat_result=least_squares(residual,initial,bounds=bounds,max_nfev=800,
                             loss='linear',x_scale='jac',ftol=1e-10,xtol=1e-10,gtol=1e-10)
    if not lat_result.success:raise RuntimeError('Lateral parameter fit failed: '+lat_result.message)
    params['lateral']=unpack(lat_result.x)
    lo_condition=condition_report(design);la_condition=condition_report(lat_result.jac)
    lo_at_bound=bool(np.any(np.isclose(result.x,lower,atol=1e-5))
                     or np.any(np.isclose(result.x,upper,atol=1e-5)))
    lat_at_bound=bool(np.any(lat_result.active_mask!=0))
    warnings=[]
    if lo_condition['rank']<4 or (lo_condition['column_normalized_condition'] or 1e100)>1e4:
        warnings.append('longitudinal_excitation_or_conditioning_insufficient')
    if la_condition['rank']<4 or (la_condition['column_normalized_condition'] or 1e100)>1e4:
        warnings.append('lateral_excitation_or_local_conditioning_insufficient')
    if lo_at_bound:warnings.append('longitudinal_parameter_at_assumed_bound')
    if lat_at_bound:warnings.append('lateral_parameter_at_assumed_bound')
    params['fit_diagnostics']={'longitudinal_samples':len(lo),'lateral_samples':len(la),
        'longitudinal':lo_condition,'lateral':la_condition,
        'longitudinal_training_acceleration_rmse':float(np.sqrt(np.mean((design@result.x-target)**2))),
        'warnings':warnings,'physical_parameters_verified':False,
        'thresholds_note':'Conditioning/bounds screens are software diagnostics, not statistical confidence guarantees.'}
    params['training_runs']=[{'path':df.attrs['path'],'state_sha256':df.attrs['state_sha256'],
        'case':m.get('case'),'data_source':m.get('data_source','unspecified'),
        'yaw_derivative_window_samples':df.attrs['yaw_derivative_window_samples']}
        for df,m in runs]
    params['scope']={
        'longitudinal':'Static effective acceleration map from normalized throttle/brake; no drivetrain lag or gear model.',
        'lateral':'Linear tire cornering-force law with geometric slip angles; body-equivalent longitudinal actuation.',
        'reference_point':'CoM and CARLA state reference are assumed coincident; verify or transform before interpreting pose errors.',
        'steering':'Actual measured wheel angle is replayed. Steering actuator mapping/prediction is outside the boundary.',
        'low_speed':'Kinematic continuation below 0.8 m/s is numerical handling, not a validated tire model.',
        'parameters':'Effective grey-box estimates; Cf/m, Cr/m, lf/L and Iz/m may absorb omitted dynamics.',
        'acceptance':'No model is declared reliable until independent test errors satisfy separately defined requirements.'}
    params['software']={'python':platform.python_version(),'numpy':np.__version__,
                        'pandas':pd.__version__,'scipy':scipy.__version__}
    args.out.parent.mkdir(parents=True,exist_ok=True);save_json(args.out,params)
    print(f'Wrote {args.out}; fit warnings: {warnings or "none from numerical screens"}')
    print('Parameters are effective estimates; physical parameter verification remains false.')


def metric(observed,predicted):
    e=np.asarray(predicted)-np.asarray(observed)
    good=np.isfinite(e)
    if not np.any(good):return {'samples':0,'rmse':None,'max_abs':None}
    return {'samples':int(good.sum()),'rmse':float(np.sqrt(np.mean(e[good]**2))),
            'max_abs':float(np.max(np.abs(e[good])))}


def compare(args):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if args.out.exists():raise FileExistsError('Output directory exists; choose a new --out')
    p=json.loads(args.parameters.read_text(encoding='utf-8'))
    validate_parameters(p)
    training_hashes={r['state_sha256'] for r in p['training_runs']}
    runs=[read_run(path) for path in args.runs]
    for df,m in runs:
        if df.attrs['state_sha256'] in training_hashes:
            raise ValueError('Training/test leakage: this state log was used to identify parameters')
        if m['vehicle_id']!=p['vehicle_id'] or abs(m['wheelbase_m']/p['wheelbase_m']-1)>.005:
            raise ValueError('Asset/geometry mismatch; use the same vehicle for this validation stage')
    if len({d.attrs['state_sha256'] for d,_ in runs})!=len(runs):
        raise ValueError('Duplicate test runs')
    args.out.mkdir(parents=True)
    metrics=[];endpoint_errors=[];run_reports=[]
    for index,(df,m) in enumerate(runs):
        name=f'{index+1:02d}_{Path(df.attrs["path"]).name}'
        destination=args.out/name;destination.mkdir()
        reports={'run':df.attrs['path'],'state_sha256':df.attrs['state_sha256'],
                 'data_source':m.get('data_source','unspecified'),
                 'test_mass_changed':abs(m['mass_kg']/p['mass_kg']-1)>.005,
                 'model_failures':[],'domain_speed_min_mps':.8}
        # Use the same measured-state mask for both models; never exclude errors
        # because a model prediction itself looks poor.
        domains={'whole_run':np.ones(len(df),dtype=bool),
                 'moving_domain':(df.vx_mps.to_numpy()>=.8)&(df.t_s.to_numpy()>=2)}
        predictions={}
        for model in ['kinematic','dynamic']:
            try:
                prediction=pd.DataFrame(rollout(df,p,model,max_step=args.max_step))
            except (FloatingPointError,OverflowError,ValueError) as error:
                reports['model_failures'].append({'model':model,'error':str(error)})
                continue
            # Kinematic accelerations follow from the model's own predicted velocities.
            if model=='kinematic':
                t=prediction.t_s.to_numpy();r=prediction.r_radps.to_numpy()
                prediction['ax_mps2']=np.gradient(prediction.vx_mps,t)-r*prediction.vy_mps
                prediction['ay_mps2']=np.gradient(prediction.vy_mps,t)+r*prediction.vx_mps
            if model=='dynamic':
                prediction['Fx_net_model_N']=p['mass_kg']*prediction.ax_mps2
                prediction['Fy_net_model_N']=p['mass_kg']*prediction.ay_mps2
                # These are model-implied NET forces, not independently observed tire forces.
                reports['dynamic_max_abs_slip_rad']=float(max(
                    prediction.front_slip_rad.abs().max(),prediction.rear_slip_rad.abs().max()))
                reports['slip_exceeds_illustrative_small_angle_screen']=reports['dynamic_max_abs_slip_rad']>.15
            predictions[model]=prediction
            prediction.to_csv(destination/f'{model}_predictions.csv',index=False)
            columns={'vx_mps':'vx_mps','vy_mps':'vy_mps','r_radps':'r_reference_radps',
                     'ax_mps2':'ax_mps2','ay_mps2':'ay_mps2','X_m':'X_m','Y_m':'Y_m'}
            for domain,mask in domains.items():
                for predicted,observed in columns.items():
                    metrics.append({'run':name,'model':model,'domain':domain,'quantity':predicted,
                        **metric(df[observed].to_numpy()[mask],prediction[predicted].to_numpy()[mask])})
                angle_error=np.arctan2(np.sin(prediction.psi_rad-df.psi_rad),
                                      np.cos(prediction.psi_rad-df.psi_rad))
                metrics.append({'run':name,'model':model,'domain':domain,'quantity':'psi_rad',
                    **metric(np.zeros(mask.sum()),angle_error.to_numpy()[mask])})
            # Moving-window prediction is initialized from truth at each start,
            # unlike the single-initialization whole-run simulation above.
            interval=max(1,int(round(args.stride/df.attrs['dt'])))
            for start in range(0,len(df),interval):
                if df.t_s.iloc[start]<2 or df.vx_mps.iloc[start]<.8:continue
                end=int(np.searchsorted(df.t_s,df.t_s.iloc[start]+max(args.horizons)))
                if end>=len(df):continue
                try:
                    future=pd.DataFrame(rollout(df,p,model,start=start,end=end,max_step=args.max_step))
                except (FloatingPointError,OverflowError,ValueError) as error:
                    reports['model_failures'].append({'model':model,'window_start':float(df.t_s.iloc[start]),'error':str(error)})
                    continue
                for horizon in args.horizons:
                    target_t=float(df.t_s.iloc[start]+horizon)
                    k=int(np.argmin(np.abs(future.t_s.to_numpy()-target_t)))
                    observed=df.iloc[start+k];pred=future.iloc[k]
                    endpoint_errors.append({'run':name,'model':model,'horizon_s':horizon,
                        'start_t_s':float(df.t_s.iloc[start]),'actual_horizon_s':float(pred.t_s-df.t_s.iloc[start]),
                        'position_error_m':float(math.hypot(pred.X_m-observed.X_m,pred.Y_m-observed.Y_m)),
                        'vx_error_mps':float(pred.vx_mps-observed.vx_mps),
                        'vy_error_mps':float(pred.vy_mps-observed.vy_mps),
                        'r_error_radps':float(pred.r_radps-observed.r_reference_radps)})
        synthetic='synthetic' in m.get('data_source','').lower()
        reference_label='Synthetic software fixture' if synthetic else 'CARLA state reference'
        fig,axes=plt.subplots(3,2,figsize=(12,9))
        fig.suptitle('SYNTHETIC SOFTWARE CHECK - NOT CARLA RESULTS' if synthetic
                     else 'Recorded CARLA data and model predictions',fontsize=13)
        variables=[('vx_mps','vx_mps'),('vy_mps','vy_mps'),('r_radps','r_reference_radps'),
                   ('ax_mps2','ax_mps2'),('ay_mps2','ay_mps2')]
        for ax,(variable,reference) in zip(axes.flat,variables):
            ax.plot(df.t_s,df[reference],label=reference_label,linewidth=1.5)
            for model,prediction in predictions.items():ax.plot(prediction.t_s,prediction[variable],'--',label=model)
            ax.set(xlabel='Simulation time (s)',ylabel=variable);ax.legend(fontsize=8);ax.grid(alpha=.25)
        ax=axes[2,1];ax.plot(df.X_m,df.Y_m,label=reference_label)
        for model,prediction in predictions.items():ax.plot(prediction.X_m,prediction.Y_m,'--',label=model)
        ax.set(xlabel='X (m)',ylabel='Y (m)');ax.axis('equal');ax.legend(fontsize=8);ax.grid(alpha=.25)
        fig.tight_layout(rect=(0,0,1,.96));fig.savefig(destination/'comparison.png',dpi=180);plt.close(fig)
        save_json(destination/'run_report.json',reports);run_reports.append(reports)
    pd.DataFrame(metrics).to_csv(args.out/'metrics.csv',index=False)
    future_df=pd.DataFrame(endpoint_errors)
    future_df.to_csv(args.out/'prediction_errors.csv',index=False)
    if len(future_df):
        summary=[]
        for (model,horizon),group in future_df.groupby(['model','horizon_s']):
            for q in ['position_error_m','vx_error_mps','vy_error_mps','r_error_radps']:
                summary.append({'model':model,'horizon_s':horizon,'quantity':q,
                                **metric(np.zeros(len(group)),group[q])})
        pd.DataFrame(summary).to_csv(args.out/'prediction_summary.csv',index=False)
    save_json(args.out/'validation_report.json',{
        'scientific_acceptance':'not_automatically_assessed','real_vehicle_validated':False,
        'fit_warnings':p['fit_diagnostics']['warnings'],'runs':run_reports,
        'model_scope':p['scope'],'parameters_sha256':hashlib.sha256(args.parameters.read_bytes()).hexdigest(),
        'evaluation_note':'No test-data parameter updates. Known measured wheel-angle replay and command inputs condition the predictions. Whole-run and moving-window metrics are separate. Thresholds must be defined from the control task before scientific acceptance.'})
    print(f'Wrote model predictions, figures, metrics and report to {args.out.resolve()}')
    print('Read fit warnings and model failures before interpreting any error table.')
    return 2 if any(r['model_failures'] for r in run_reports) else 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    a=sub.add_parser('fit');a.add_argument('--train',nargs='+',type=Path,required=True)
    a.add_argument('--out',type=Path,required=True)
    b=sub.add_parser('compare');b.add_argument('--parameters',type=Path,required=True)
    b.add_argument('--runs',nargs='+',type=Path,required=True)
    b.add_argument('--out',type=Path,required=True)
    b.add_argument('--max-step',type=float,default=.001)
    b.add_argument('--horizons',type=float,nargs='+',default=[.1,.5,1.0])
    b.add_argument('--stride',type=float,default=1.0)
    args=parser.parse_args()
    if args.action=='fit':fit(args)
    else:
        if (not all(math.isfinite(x) for x in [args.max_step,args.stride,*args.horizons])
                or args.max_step<=0 or min(args.horizons)<=0 or args.stride<=0):
            parser.error('Integration step, horizons and stride must be positive')
        return compare(args)


if __name__=='__main__':raise SystemExit(cli_entry(main))
