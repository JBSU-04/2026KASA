"""M8 restored-engine regression contracts; no ROS master or bag playback."""
import sys,math,threading
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m8_motion import RigidMotion
from m8_repair_policy import early_risk,static_certificate,owned_risk,verified_motion

PATH=np.c_[np.arange(0.,41.,.2),np.zeros(205)]

def surface():
    r=np.random.RandomState(7)
    return np.c_[r.uniform(-.25,.25,30),r.uniform(-.25,.25,30)]

def test_both_crossing_directions_keep_short_candidate_window():
    for side in [-1,1]:
        model=RigidMotion();p=surface()+[10.,side*3.]
        out=[model.update(p+[0.,-side*.3*k],k*.1)for k in range(8)]
        assert next(k for k,r in enumerate(out)if r[1])<=3
        assert out[-1][0]=='dynamic'
        assert model.evidence['surface_translation_valid']

def test_identical_xy_vertical_returns_cannot_create_moving_votes():
    model=RigidMotion()
    for k in range(8):
        p=np.tile([[10.+(k%2)*.6,0.]],(20,1))
        state,candidate,_=model.update(p,k*.1)
        assert state!='dynamic' and not candidate

def test_stationary_common_surface_wins_over_new_face():
    p=surface()+[10.,0.];m=RigidMotion()
    for k in range(10):m.update(p,k*.1)
    for k in range(10,15):
        q=np.vstack((p,p+[1.0+(k-10)*.2,.4]))
        state,candidate,_=m.update(q,k*.1)
        assert state!='dynamic' and not candidate

def test_duplicate_stale_and_coordinate_changes_do_not_confirm():
    m=RigidMotion();p=surface()+[10.,3.]
    m.update(p,1.,arrival_time=1.)
    m.update(p+[0.,-.4],1.,arrival_time=1.1)
    assert len(m.bounds)==1
    m.update(p,1.1,arrival_time=1.6)
    assert not m.bounds
    m.update(p,2.,coordinate_version=1)
    m.update(p+[0.,-.4],2.1,coordinate_version=2)
    assert len(m.bounds)==1 and not m.last_result[1]

def test_wall_residue_outside_progress_cone_not_preslow_but_side_object_is():
    pts=np.array([[3.,7.1],[25.,4.45],[25.,-4.45],[10.,.5]])
    out=early_risk(pts,PATH,(0.,0.,0.),11.,pts[:,0])
    assert out.tolist()==[False,True,True,True]

def test_static_certificate_does_not_require_owned_geometry():
    o=dict(id=10,motion_state='unknown',confirmed=True,static_certified=True,last_seen=9.9)
    assert static_certificate(o,10.)
    assert not static_certificate(o,10.3)

def test_owned_shape_keeps_s_gap_and_ignores_other_objects_box():
    o=dict(id=10,last_seen=10.,motion_state='static',confirmed=True,
           collision_geometry_valid=True,collision_geometry_stamp=10.,coordinate_version=0,
           occupied_cells_world=[[6.,2.],[7.,2.]],footprint_world=[[2,-5],[12,-5],[12,5],[2,5]])
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is False
    o['occupied_cells_world']=[[6.,.2]]
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is True
    o['collision_geometry_stamp']=9.6
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is None

def test_behind_side_clear_but_current_body_overlap_not_skipped():
    o=dict(id=10,collision_geometry_valid=True,collision_geometry_stamp=10.,occupied_cells_world=[[-2.,2.]])
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is False
    o['occupied_cells_world']=[[-.5,.2]]
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is True

def test_verified_small_crossing_requires_original_scan():
    o=dict(last_seen=10.,observed_motion_valid=True,motion_evidence=dict(observation_stamp=10.,surface_translation_valid=True))
    assert verified_motion(o,10.)
    assert not verified_motion(o,10.36)
    o['motion_evidence']['surface_translation_valid']=False
    assert not verified_motion(o,10.)

# Real consumer tests: the local path must never overwrite a native approval.
import test_tunnel_driven_stop as fixture

def configured():
    f=fixture.DrivenStopTests();f.setUp();n=f.n
    n.m8_repair_path=True;n.m8_verified_motion=True;n.mission_mode='tunnel';n.current_lane_number=3;n._state_lock=threading.RLock()
    n._m8_coordinate_version=0;n._m8_approval=None;n._m8_scene_geometry={}
    return n,dict(f.o)

def receipt(plan=7,coordinate=0,valid=True):
    header=NS(frame_id='map',seq=plan,stamp=NS(to_sec=lambda:10.))
    return NS(valid=valid,header=header,plan_id=plan,coordinate_version=coordinate,path=NS(header=header,poses=[NS(pose=NS(position=NS(x=x,y=0.)))for x in [0.,5.,10.]]))

def test_receipt_is_atomic_and_legacy_path_cannot_overwrite(monkeypatch):
    monkeypatch.setattr(fixture.stop.rospy.Time,'now',lambda:NS(to_sec=lambda:10.))
    n,_=configured();n.m8_approved_path_cb(receipt());n._adopt_m8_approval()
    assert n._m8_approval[2]==7
    n.local_path_callback(receipt(plan=99).path)
    assert n._m8_approval[2]==7
    bad=receipt(plan=8);bad.path=NS(header=NS(seq=9,stamp=bad.header.stamp),poses=bad.path.poses)
    n.m8_approved_path_cb(bad)
    assert n._m8_approval[2]==7

def test_dynamic_candidate_without_surface_proof_does_not_stop_safe_side():
    n,o=configured();n.m8_repair_path=False
    o.update(motion_state='static',confirmed=True,speed=3.,velocity=[0.,3.],observed_velocity=[0.,3.],
             observed_motion_valid=True,dynamic_candidate=True,last_seen=10.,motion_evidence=dict(observation_stamp=10.,surface_translation_valid=False))
    n.shared_detector_policy=True
    n._driven_predictive_conflict=lambda *a,**k:(_ for _ in ()).throw(AssertionError('unverified vector used'))
    n._dynamic_zone_stop(o,True,True,True,10.,tunnel=True)


def test_common_scene_candidate_does_not_bypass_prediction():
    n,o=configured();n._m8_approval=(np.c_[[0.,20.],[2.4,2.4]],10.,7,0)
    n._adopt_m8_approval()
    o.update(perception_source='common_scene',motion_state='unknown',dynamic_candidate=True,
             observed_motion_valid=True,observed_velocity=[0.,2.],last_seen=10.,
             motion_evidence=dict(observation_stamp=10.,surface_translation_valid=True))
    n._driven_predictive_conflict=lambda *a,**k:(True,'driven_swept_t1.0')
    assert n._dynamic_zone_stop(o,False,False,True,10.,tunnel=True)[0]

def test_missing_far_shape_is_not_current_collision_and_cannot_clear_hold():
    from m8_repair_policy import inspection_required
    o=dict(position_world=[30.,0.],occupied_cells_world=[])
    assert not inspection_required(o,PATH,(0.,0.,0.),5.)
    o['position_world']=[6.,0.]
    assert inspection_required(o,PATH,(0.,0.,0.),5.)
    assert owned_risk(o,PATH,(0.,0.,0.),10.) is None

def test_static_parent_unknown_label_clears_only_with_owned_safe_path(monkeypatch):
    monkeypatch.setattr(fixture.stop.rospy.Time,'now',lambda:NS(to_sec=lambda:10.))
    n,o=configured();n._m8_approval=(PATH,10.,7,0);n._adopt_m8_approval()
    n.ego_xy=(0.,0.);n.ego_yaw=0.;n._m8_hold_owners={o['id']}
    o.update(static_certified=True,motion_state='unknown',last_seen=10.,
             collision_geometry_valid=True,collision_geometry_stamp=10.,occupied_cells_world=[[6.,2.]])
    assert n.m8_static_hold_clear([o],10.)
    o['occupied_cells_world']=[[6.,0.]]
    assert not n.m8_static_hold_clear([o],10.)


def test_native_scene_missing_geometry_never_uses_large_box_for_body_collision():
    n,o=configured();n.m8_object_scene=True
    o.update(position_world=[0.,0.],length=8.,width=8.,footprint_world=[[-4.,-4.],[4.,-4.],[4.,4.],[-4.,4.]],collision_geometry_valid=False)
    assert not n.actual_body_risk(o)


def test_native_moving_surface_prediction_ignores_empty_box_interior(monkeypatch):
    monkeypatch.setattr(fixture.stop.rospy.Time,'now',staticmethod(lambda:fixture.stop.rospy.Time.from_sec(10.)))
    n,o=configured();n.m8_object_scene=True;n.tunnel_cells_authoritative_enabled=False
    n.ego_xy=(0.,0.);n.ego_yaw=0.;n.ego_pose_time=10.;n.ego_speed=3.;n.local_path_xy=PATH
    n.ttc_ego_front_m=3.85;n.stop_zone_width=1.9
    o.update(position_world=[8.,0.],radius=6.,footprint_world=[[4.,-6.],[12.,-6.],[12.,6.],[4.,6.]],
             collision_geometry_valid=True,collision_geometry_stamp=10.,occupied_cells_world=[[8.,5.],[8.,-5.]],vel_std=0.)
    result=n._driven_predictive_conflict(o,[0.,0.],allow_prefix=True)
    assert result is not None and not result[0]
    o['occupied_cells_world']=[[8.,.1]]
    assert n._driven_predictive_conflict(o,[0.,0.],allow_prefix=True)[0]
