"""Offline software checks using the real CARLA Python library; no server needed."""
import math
from pathlib import Path
import tempfile
from argparse import Namespace

from carla_runtime import cli_entry


def main():
    import carla
    import numpy as np
    from buoc1_carla import TEST_ROAD,body_components,wheelbase_from_geometry
    from kiem_thu_maneuver import road_xml,steering_profile
    from chay_ma_tran import make_plan
    from mo_hinh_xe import dynamic_rhs,validate_parameters
    from doi_chieu_maneuver import project_circle
    for name,xml in [('baseline',TEST_ROAD),('maneuver',road_xml())]:
        road=carla.Map(name,xml)
        assert road.get_waypoint_xodr(1,-1,50.0) is not None
    assert body_components(1,0,0)==(1,0)
    assert np.allclose(body_components(0,1,math.pi/2),(1,0),atol=1e-12)
    for scale in (1,100):
        wheels=[carla.WheelPhysicsControl(position=carla.Vector3D(x=x*scale,y=y*scale))
                for x,y in [(1.4,-.8),(1.4,.8),(-1.4,-.8),(-1.4,.8)]]
        physics=carla.VehiclePhysicsControl(mass=1500,wheels=wheels)
        assert abs(wheelbase_from_geometry(physics,4.5)[0]-2.8)<1e-6
    assert abs(steering_profile(1.25,'jturn',1)-.5)<1e-12
    assert project_circle(3,4,2.5)==(1.5,2.0,True)
    p={'wheelbase_m':2.8,'mass_kg':1500,'low_speed_end_mps':.8,
       'longitudinal':dict(traction_gain=6,brake_gain=7,rolling_accel=.05,linear_drag=.08),
       'lateral':dict(cf_over_mass=35,cr_over_mass=45,lf_fraction=.45,iz_over_mass=1.6)}
    validate_parameters(p)
    assert np.allclose(dynamic_rhs(np.array([0,0,0,3,0,0.]),.1,0,0,p),[3,0,0,.31,0,0])
    plan=make_plan(Namespace(dry_friction=1,wet_friction=.4,directions=[1,-1],repeats=3,
        delta_deg=2,host='127.0.0.1',port=2000,vehicle='vehicle.tesla.model3',
        out=Path('not_created'),wheelbase=None,rain_visual=False,timeout=90))
    assert len(plan)==72 and len({r['name'] for r in plan})==72
    print('DAT: doc OpenDRIVE, hinh hoc, he toa do, dang lai, gioi han bam, mo hinh va ke hoach 72 luot.')
    print('Day la kiem tra phan mem offline; chua chay CARLA server/GPU.')
    return 0


if __name__=='__main__':raise SystemExit(cli_entry(main))
