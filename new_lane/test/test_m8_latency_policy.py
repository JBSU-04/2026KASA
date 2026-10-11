import sys,math,threading
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import rospy
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from m8_geometry_native import available,Route
from obstacle_geometry import RayEvidence
from tunnel_native_scene import packet_from_detector,scene_dict
from tunnel_scene_policy import preslow

def test_certified_parent_survives_unknown_motion_without_certifying_new_track():
    o=dict(id=1000000,observed_track_id=4,confirmed=True,motion_state='unknown',static_certified=True,static_occupancy=True,position_world=[10.,0.],last_seen=10.)
    parent={1000000:dict(support=[[10.,0.,.15],[10.1,0.,.15]],stamp=10.,yaw=0.,box=[9.,11.,-1.,1.])}
    m=packet_from_detector(dict(collision_objects=[o]),{},parent,(0.,0.,0.),10.,1)
    assert m.objects[0].motion_state==0 and m.objects[0].static_certified
    assert all(m.fixed) and len(scene_dict(m)['collision_cells_world'])==2
    o['static_certified']=False
    m=packet_from_detector(dict(collision_objects=[o]),{},parent,(0.,0.,0.),10.,2)
    assert not any(m.fixed) and not m.objects[0].static_certified

def test_native_ray_kernel_matches_returned_beams_not_visibility_cones():
    assert available()
    rng=np.random.RandomState(80);origin=np.array([302500.,4124000.,1.765])
    delta=rng.normal(size=(600,3));delta/=np.linalg.norm(delta,axis=1)[:,None];ranges=rng.uniform(2.,35.,600)
    ends=origin+delta*ranges[:,None]
    scan=dict(stamp=10.,frame='map',origin=origin.tolist(),endpoints=ends.tolist())
    old=RayEvidence(scan,10.);new=RayEvidence(dict(scan,native_geometry=True),10.)
    for i in range(50):
        q=origin+delta[i]*(ranges[i]-.5)
        assert old.clear([q])==new.clear([q])
        q=origin+delta[i]*(ranges[i]+.5)
        assert old.clear([q])==new.clear([q])
    assert not new.clear([[302500.,4124000.,1.765]])
    assert not new.clear([[302503.,4124000.,-2.]])

def test_native_route_index_matches_finite_segments_and_utm_precision():
    assert available()
    rng=np.random.RandomState(4);xy=np.cumsum(rng.normal(size=(500,2)),axis=0)+[302500.,4124000.]
    route=Route(xy);v=np.diff(xy,axis=0);l2=np.sum(v*v,axis=1)
    for _ in range(40):
        p=rng.uniform([-15,-15],[15,15],size=(20,2))+[302500.,4124000.]
        delta=p[:,None,:]-xy[None,:-1,:];u=np.clip(np.sum(delta*v,axis=2)/l2,0.,1.)
        expected=np.linalg.norm(delta-u[:,:,None]*v,axis=2).min()
        assert abs(route.distance(p)-expected)<1e-8

def test_wall_residual_outside_future_path_does_not_preslow_but_crossing_does():
    path=np.array([[0.,0.],[30.,0.]])
    o=dict(id=9,motion_state='unknown',in_trigger_corridor=False,occupied_cells_world=[[10.,-5.],[10.1,-5.],[10.,-4.9]])
    scene=dict(state='valid',geometry_stamp=10.,scan_pose=[0,0,0,0,6],obstacles=[o])
    assert not preslow(scene,(0,0,0),path,10.,10.)['present']
    o.update(dynamic_confirmed=True,velocity_world=[0.,2.])
    assert preslow(scene,(0,0,0),path,10.,10.)['present']

def test_unknown_near_body_risk_survives_even_without_path_or_corridor_tag():
    o=dict(id=2,motion_state='unknown',in_trigger_corridor=False,occupied_cells_world=[[2.,.5],[2.1,.5],[2.,.6]])
    s=dict(state='valid',geometry_stamp=10.,obstacles=[o])
    assert preslow(s,(0,0,0),None,10.,10.)['present']

def test_native_callback_keeps_only_newest_pending_scan():
    from obstacle_info_merger import Merger
    from tunnel_region import read_region
    n=Merger.__new__(Merger);n.common_scene_enabled=True;n.native_worker_enabled=True
    n.route=[(0.,0.,7700,3)];n.tunnel_region=read_region(lambda k,d:d)
    n.native_condition=threading.Condition();n.native_pending=None
    messages=[NS(header=NS(stamp=rospy.Time.from_sec(t)))for t in [10.,10.2,10.1]]
    for m in messages:n.native_near_cb(m)
    assert n.native_pending[0] is messages[1]
