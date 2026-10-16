"""Identify the existing longitudinal map and test on a distinct straight run.

This does NOT identify/validate the lateral or yaw equations of a 3-DOF model.
Measured test velocities are used for scoring only, never to update parameters.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import lsq_linear
from carla_runtime import cli_entry,save_json
from mo_hinh_xe import longitudinal_acceleration
from so_sanh_mo_hinh import read_run,consistent_vehicle,condition_report,metric


def evaluate(args):
    if args.out.exists():raise FileExistsError(args.out)
    loaded=[read_run(args.train),read_run(args.test)]
    meta=consistent_vehicle(loaded);train,test=[x[0] for x in loaded]
    if train.attrs['state_sha256']==test.attrs['state_sha256']:raise ValueError('Training/test leakage')
    for df,m in loaded:
        source=m.get('data_source','').lower()
        if 'carla' not in source or any(w in source for w in ('synthetic','fixture','mock','fake')):
            raise ValueError('This evaluator requires recorded CARLA data, not software fixtures.')
        if df.delta_equiv_rad.abs().max()>.005:raise ValueError('Use straight-driving runs for this limited 1-D check.')
    lo=train[(train.vx_mps>.8)&(train.t_s>1)&(train.delta_equiv_rad.abs()<.005)]
    if len(lo)<50:raise ValueError('Insufficient moving training samples')
    v=lo.vx_mps.to_numpy()
    design=np.column_stack([lo.throttle_applied,-lo.brake_applied,-np.tanh(v/.2),-v])
    lower=np.array([.01,.01,0.,0.]);upper=np.array([60.,60.,5.,3.])
    fitted=lsq_linear(design,lo.ax_mps2.to_numpy(),bounds=(lower,upper))
    if not fitted.success:raise RuntimeError('Longitudinal fit failed')
    q=dict(zip(['traction_gain','brake_gain','rolling_accel','linear_drag'],map(float,fitted.x)))
    p={'longitudinal':q};results=[];predictions={}
    for name,df in [('train',train),('independent_test',test)]:
        rows=df.to_dict('records');speed=max(float(rows[0]['vx_mps']),0.);records=[]
        for k,row in enumerate(rows):
            records.append({'t_s':row['t_s'],'vx_carla_mps':row['vx_mps'],
                            'vx_model_mps':speed,'vx_error_mps':speed-row['vx_mps']})
            if k==len(rows)-1:break
            dt=rows[k+1]['t_s']-row['t_s'];n=math.ceil(dt/args.max_step);h=dt/n
            def f(x):return longitudinal_acceleration(x,row['throttle_next'],row['brake_next'],p)
            for _ in range(n):
                a=f(speed);b=f(speed+.5*h*a);c=f(speed+.5*h*b);d=f(speed+h*c)
                speed=max(0.,speed+h*(a+2*b+2*c+d)/6)
        pred=pd.DataFrame(records);predictions[name]=pred
        for domain,mask in [('whole_run',np.ones(len(df),dtype=bool)),
                            ('moving_domain',(df.vx_mps.to_numpy()>=.8)&(df.t_s.to_numpy()>=2))]:
            results.append({'run':name,'domain':domain,
                **metric(pred.vx_carla_mps.to_numpy()[mask],pred.vx_model_mps.to_numpy()[mask]),
                'observed_vmax_mps':float(df.vx_mps.max()),'predicted_vmax_mps':float(pred.vx_model_mps.max())})
    args.out.mkdir(parents=True)
    pd.DataFrame(results).to_csv(args.out/'metrics.csv',index=False)
    for name,pred in predictions.items():pred.to_csv(args.out/f'{name}_predictions.csv',index=False)
    at_bound=[key for i,key in enumerate(q) if np.isclose(fitted.x[i],lower[i],atol=1e-5) or np.isclose(fitted.x[i],upper[i],atol=1e-5)]
    report={'scope':'Longitudinal submodel only; lateral/yaw parameters are not identified.',
        'model_3dof_validated':False,'longitudinal':q,'vehicle_id':meta['vehicle_id'],
        'mass_kg':meta['mass_kg'],'wheelbase_m':meta['wheelbase_m'],
        'training_brake_samples_vx_gt_0_8':int((lo.brake_applied>0).sum()),
        'parameters_at_assumed_bounds':at_bound,'design':condition_report(design),'metrics':results,
        'sources':[{'role':name,'state_sha256':df.attrs['state_sha256'],
                   'case':m.get('case'),'data_source':m.get('data_source'),
                   'drive_requested':m.get('drive')} for name,(df,m) in zip(['train','independent_test'],loaded)],
        'protocol':'Fit once on training moving samples; initialize once per run and replay throttle_next/brake_next. No resets or test-data tuning.',
        'acceptance':'No scientific acceptance criteria specified; errors must be reviewed before any validation claim.'}
    save_json(args.out/'tham_so_CHI_PHAN_DOC.json',report)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for ax,name in zip(axes[:2],['train','independent_test']):
        pred=predictions[name]
        ax.plot(pred.t_s,pred.vx_carla_mps,label='Recorded CARLA');ax.plot(pred.t_s,pred.vx_model_mps,'--',label='Longitudinal approximation')
        ax.set(ylabel='vx (m/s)',title=name);ax.legend(fontsize=8);ax.grid(alpha=.3)
    pred=predictions['independent_test'];axes[2].plot(pred.t_s,pred.vx_error_mps)
    axes[2].axhline(0,color='gray',linewidth=.8);axes[2].set(xlabel='Simulation time (s)',ylabel='Test vx error (m/s)');axes[2].grid(alpha=.3)
    fig.suptitle('Independent longitudinal check - full 3-DOF NOT validated')
    fig.tight_layout();fig.savefig(args.out/'doi_chieu_phan_doc.png',dpi=150);plt.close(fig)
    lines=['KET QUA DOI CHIEU NGOAI TUYEN — CHI PHAN DOC',
           'Du lieu la cac luot CARLA do nguoi dung cung cap, khong phai du lieu nhan tao.',
           'Mo hinh 3-DOF da duoc kiem chung: CHUA DU BANG CHUNG.',
           f'So mau phanh trong mien nhan dang: {report["training_brake_samples_vx_gt_0_8"]}',
           f'Tham so cham bien gia thiet: {at_bound}']
    for r in results:lines.append(f'{r["run"]}/{r["domain"]}: RMSE vx={r["rmse"]:.6f} m/s; max_abs={r["max_abs"]:.6f} m/s')
    lines+=['Can them du lieu ngang/phoi hop, kiem thu doc lap va tieu chi chap nhan.',
            'Bao cao nay khong phai tep tham so day du de nap vao bo doi chieu 3-DOF.']
    (args.out/'BAO_CAO.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('\n'.join(lines),flush=True)
    return 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--train',type=Path,required=True);p.add_argument('--test',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--max-step',type=float,default=.001)
    a=p.parse_args()
    if not (math.isfinite(a.max_step) and 0<a.max_step<=.01):p.error('Invalid integration step')
    return evaluate(a)


if __name__=='__main__':raise SystemExit(cli_entry(main))
