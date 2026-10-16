"""Illustrative grey-box vehicle models with explicitly fitted effective parameters.

SI, body x-forward/y-left, positive yaw and steer to the left. These models
are authored examples, not an identified Hunter SE model or a copy of CARLA.
Measured steering is replayed as an input: steering actuator prediction is
outside this comparison boundary. CoM/reference-point coincidence must be
verified before position errors can be interpreted physically.
"""
import math
import numpy as np


def validate_parameters(p):
    """Fail before integration if a supplied JSON has invalid SI parameters."""
    positive=[p['wheelbase_m'],p['mass_kg'],p['lateral']['cf_over_mass'],
              p['lateral']['cr_over_mass'],p['lateral']['iz_over_mass']]
    if not all(math.isfinite(x) and x>0 for x in positive):
        raise ValueError('Model mass, geometry, stiffness and inertia must be positive finite values.')
    fraction=p['lateral']['lf_fraction'];low=p.get('low_speed_end_mps',.8)
    if not (math.isfinite(fraction) and 0<fraction<1 and math.isfinite(low) and low>.3):
        raise ValueError('Require 0<lf_fraction<1 and low_speed_end_mps>0.3.')
    if not all(math.isfinite(v) and v>=0 for v in p['longitudinal'].values()):
        raise ValueError('Longitudinal gains must be finite and nonnegative.')


def longitudinal_acceleration(vx, throttle, brake, p):
    q=p['longitudinal']
    forward=max(float(vx),0.0)
    a=(q['traction_gain']*throttle-q['brake_gain']*brake
       -q['rolling_accel']*math.tanh(forward/.2)-q['linear_drag']*forward)
    return max(a,0.0) if forward<=0 else a


def tire_terms(vx,vy,r,delta,p):
    lat=p['lateral'];L=p['wheelbase_m']
    lf=lat['lf_fraction']*L;lr=L-lf
    denominator=max(vx,p.get('low_speed_end_mps',.8))
    alpha_f=delta-math.atan2(vy+lf*r,denominator)
    alpha_r=-math.atan2(vy-lr*r,denominator)
    return (lat['cf_over_mass']*alpha_f,lat['cr_over_mass']*alpha_r,
            alpha_f,alpha_r,lf,lr)


def dynamic_rhs(state,throttle,brake,delta,p):
    X,Y,psi,vx,vy,r=state
    fyf,fyr,af,ar,lf,lr=tire_terms(vx,vy,r,delta,p)
    speed=max(vx,0.0)
    blend=np.clip((speed-.3)/(p.get('low_speed_end_mps',.8)-.3),0.0,1.0)
    base=longitudinal_acceleration(speed,throttle,brake,p)
    ax=base-blend*fyf*math.sin(delta)
    ay=fyf*math.cos(delta)+fyr
    dvx=ax+r*vy
    dvy_high=ay-r*vx
    dr_high=(lf*fyf*math.cos(delta)-lr*fyr)/p['lateral']['iz_over_mass']
    r_kin=speed*math.tan(delta)/p['wheelbase_m']
    # Low-speed numerical continuation, not a tire model verified from data.
    dvy_low=(lr*r_kin-vy)/.1
    dr_low=(r_kin-r)/.1
    dvy=blend*dvy_high+(1-blend)*dvy_low
    dr=blend*dr_high+(1-blend)*dr_low
    if vx<=0 and dvx<0:dvx=0.0
    c,s=math.cos(psi),math.sin(psi)
    return np.array([vx*c-vy*s,vx*s+vy*c,r,dvx,dvy,dr],dtype=float)


def kinematic_rhs(state,throttle,brake,delta,p):
    X,Y,psi,v=state
    lr=p['wheelbase_m']*(1-p['lateral']['lf_fraction'])
    beta=math.atan(lr/p['wheelbase_m']*math.tan(delta))
    speed=max(v,0.0)
    yaw_rate=speed*math.cos(beta)*math.tan(delta)/p['wheelbase_m']
    return np.array([speed*math.cos(psi+beta),speed*math.sin(psi+beta),yaw_rate,
                     longitudinal_acceleration(speed,throttle,brake,p)])


def output(state,model,throttle,brake,delta,p):
    if model=='dynamic':
        X,Y,psi,vx,vy,r=state
        derivative=dynamic_rhs(state,throttle,brake,delta,p)
        ax=derivative[3]-r*vy;ay=derivative[4]+r*vx
        _,_,af,ar,_,_=tire_terms(vx,vy,r,delta,p)
    else:
        X,Y,psi,v=state
        lr=p['wheelbase_m']*(1-p['lateral']['lf_fraction'])
        beta=math.atan(lr/p['wheelbase_m']*math.tan(delta))
        vx=max(v,0)*math.cos(beta);vy=max(v,0)*math.sin(beta)
        r=vx*math.tan(delta)/p['wheelbase_m']
        ax=ay=float('nan');af=ar=0.0
    return {'X_m':X,'Y_m':Y,'psi_rad':psi,'vx_mps':vx,'vy_mps':vy,
            'r_radps':r,'ax_mps2':ax,'ay_mps2':ay,
            'front_slip_rad':af,'rear_slip_rad':ar}


def initial_state(row,model):
    if model=='dynamic':
        return np.array([row['X_m'],row['Y_m'],row['psi_rad'],row['vx_mps'],
                         row['vy_mps'],row['r_reference_radps']],dtype=float)
    return np.array([row['X_m'],row['Y_m'],row['psi_rad'],
                     math.hypot(row['vx_mps'],row['vy_mps'])],dtype=float)


def advance(state,model,dt,throttle,brake,delta0,delta1,p,max_step=.002):
    if dt<=0:raise ValueError('Positive time step required.')
    if max_step<=0:raise ValueError('Positive integration substep required.')
    n=math.ceil(dt/max_step);h=dt/n
    rhs=dynamic_rhs if model=='dynamic' else kinematic_rhs
    state=state.copy()
    for j in range(n):
        d0=delta0+(delta1-delta0)*j/n
        dm=delta0+(delta1-delta0)*(j+.5)/n
        d1=delta0+(delta1-delta0)*(j+1)/n
        k1=rhs(state,throttle,brake,d0,p)
        k2=rhs(state+.5*h*k1,throttle,brake,dm,p)
        k3=rhs(state+.5*h*k2,throttle,brake,dm,p)
        k4=rhs(state+h*k3,throttle,brake,d1,p)
        state+=(h/6)*(k1+2*k2+2*k3+k4)
        # Forward-only stopping continuation; no reverse driving in this example.
        state[3]=max(state[3],0.0)
        if not np.all(np.isfinite(state)) or np.linalg.norm(state[3:])>1000:
            raise FloatingPointError('Model integration diverged; inspect parameters and substep.')
    return state


def rollout(frame,p,model,start=0,end=None,max_step=.002):
    """Initialize from measured state ONCE, then replay inputs without resets.

    Normalized throttle/brake at row k act on [t_k,t_{k+1}]. Actual measured
    steering is linearly replayed across that interval for offline body-model
    comparison; it is not a forecast of future actuator output.
    """
    end=len(frame)-1 if end is None else end
    rows=frame.iloc[start:end+1].to_dict('records')
    state=initial_state(rows[0],model)
    result=[]
    for i,row in enumerate(rows):
        record=output(state,model,row['throttle_next'],row['brake_next'],row['delta_equiv_rad'],p)
        record['t_s']=row['t_s'];result.append(record)
        if i+1<len(rows):
            other=rows[i+1]
            state=advance(state,model,other['t_s']-row['t_s'],
                          row['throttle_next'],row['brake_next'],row['delta_equiv_rad'],
                          other['delta_equiv_rad'],p,max_step=max_step)
    return result
