from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m8_motion import RigidMotion

def wall(start=0., y=3.):
    return np.c_[np.linspace(start,start+19,64),np.full(64,y)]

def test_sliding_observation_window_is_not_wall_motion():
    m=RigidMotion()
    out=[m.update(wall(.45*k),k*.1) for k in range(12)]
    assert all(s!='dynamic' and not c for s,c,v in out)
    assert m.evidence.get('surface_translation_valid') is False

def test_common_fixed_wall_overrules_moving_window_ends():
    m=RigidMotion();a=wall();b=wall(.9)
    m.surfaces={0.:a,.2:b};m._surface_tree=None;m._surface_cache={}
    valid,_,ratio,residual,reason=m._surface_pair(0.,.2)
    assert not valid and reason in ('stationary_extended_surface','extended_surface_motion_unobservable')

def test_proven_stationary_wall_retracts_wrong_dynamic_state():
    # Long static face with changed crop, matching most old points exactly.
    m=RigidMotion();a=wall();m.update(a,0.);m.update(a,.1)
    m.state='dynamic';m.last_result=('dynamic',False,np.array([3.,0.]));m.velocity=np.array([3.,0.])
    s,c,v=m.update(a[2:],.2)
    assert s=='static' and not c and np.linalg.norm(v)==0
    assert m.evidence['motion_retracted']=='stationary_extended_surface'

def test_lateral_crossing_next_to_wall_still_detected_without_extra_scan():
    a=np.c_[np.repeat(np.linspace(0,1,8),4),np.tile(np.linspace(0,.4,4),8)]
    m=RigidMotion();out=[m.update(a+[10,3-.3*k],k*.1)for k in range(10)]
    assert any(o[1] for o in out[:4]) and out[-1][0]=='dynamic'

def test_translating_long_surface_normal_to_wall_is_not_rejected():
    m=RigidMotion();out=[m.update(wall(y=3-.45*k),k*.1)for k in range(10)]
    assert any(o[1] for o in out[:4]) and out[-1][0]=='dynamic'

def test_missing_surface_does_not_retract_dynamic_history():
    m=RigidMotion();a=wall();m.update(a,0);m.update(a,.1)
    m.state='dynamic';m.last_result=('dynamic',False,np.array([3.,0.]))
    m.update(a[:2],.2)
    assert not m.evidence.get('motion_retracted')

def test_motion_policy_can_be_rolled_back():
    m=RigidMotion(stationary_surface_priority=False)
    out=[m.update(wall(.45*k),k*.1)for k in range(12)]
    assert any(o[1] or o[0]=='dynamic' for o in out)
