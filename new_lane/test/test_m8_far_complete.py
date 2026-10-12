from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from temporal_components import connected_components, TemporalFarInput
from tunnel_region import approach_bounds, read_region, contains
from m8_object_policy import SurfaceMotion
from tunnel_native_scene import rebase_detector_tracks
from tunnel_scene_policy import preslow

BASE=np.array([[0.,0.,.2],[0.,.3,.2],[.4,0.,.2],[.4,.3,.2]])+np.array([302620.,4123655.,0.])

def test_measured_route_approach_does_not_move_gps_or_highway():
    path=Path(__file__).parents[2]/'waypoint_system/data/default/3.csv'
    sections=[dict(id='M7B',csv=3,idx=[6197,7000]),dict(id='M8',csv=3,idx=[7001,8850])]
    params={'/section_router/sections':sections,'/tunnel_recovery/approach_control_enabled':True,
            '/tunnel_recovery/approach_route_csv':str(path)}
    r=read_region(lambda key,default=None:params.get(key,default))
    points=np.loadtxt(path,delimiter=',')[:,1:3];arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    assert 39.8<arc[7001]-arc[r[1]]<=40.000001
    assert 59.8<arc[7001]-arc[r[1]-r[3]]<=60.000001
    assert not contains(r,3,r[1]-1) and contains(r,3,r[1])
    assert contains(r,3,r[1]-r[3],True) and not contains(r,3,6196,True)
    assert sections[1]['idx'][0]==7001
    params['/tunnel_recovery/approach_control_enabled']=False
    assert read_region(lambda key,default=None:params.get(key,default))[1]==7001

def test_bad_approach_route_is_not_silently_armed(tmp_path):
    path=tmp_path/'bad.csv';path.write_text('0,1,2\n')
    with pytest.raises(ValueError):approach_bounds(path,7001)
    with pytest.raises(ValueError):approach_bounds(path,0,60,40)

def test_sparse_history_connects_current_returns_without_republishing_old_surface():
    far=TemporalFarInput()
    old=np.array([[25.,0.,-.5,.2],[25.,.3,-.5,.2],[25.,.6,-.5,.2]])
    fresh=old.copy();fresh[:,1]+=.1
    assert far.groups(old,(0,0,0),10.,0)==[]
    groups=far.groups(fresh,(0,0,0),10.1,0)
    assert len(groups)==1 and set(groups[0])=={0,1,2}
    assert np.all(fresh[groups[0],1]>0) # prior points cannot become fresh
    assert far.groups(fresh,(0,0,0),10.2,1)==[] # coordinate version resets accumulation

def test_separated_objects_and_duplicate_scans_do_not_bridge_or_confirm():
    far=TemporalFarInput();q=np.array([[25.,0.,-.5,.2],[25.,.2,-.5,.2],[25.,.4,-.5,.2],
                                     [25.,2.,-.5,.2],[25.,2.2,-.5,.2],[25.,2.4,-.5,.2]])
    assert far.groups(q,(0,0,0),10.,0)==[]
    assert far.groups(q,(0,0,0),10.,0)==[]
    assert len(far.groups(q,(0,0,0),10.1,0))==2
    assert len(connected_components(q[:,:2],.65))==2

def test_static_certificate_survives_changed_partial_view_but_expires():
    evidence=SurfaceMotion()
    for t in (1.,1.1,1.2):state,_=evidence.update(BASE,t,[0,0],False,.3)
    assert state=='static'
    # New side surface: enclosure expansion is not rigid translation.
    partial=BASE.copy();partial[:,0]+=.7;partial[:,1]+=np.array([-.4,0,-.4,0])
    state,_=evidence.update(np.vstack([BASE[:1],partial]),1.3,[0,0],False,.3)
    assert state=='static' and evidence.reason=='stationary_certificate_partial_view'
    assert evidence.update(partial,2.,[0,0],False,.3)[0]=='unknown'

def test_coherent_motion_revokes_static_certificate():
    evidence=SurfaceMotion()
    for t in (1.,1.1,1.2):evidence.update(BASE,t,[0,0],False,.3)
    for i in range(1,5):state,_=evidence.update(BASE+[i*.2,0,0],1.2+i*.1,[2.,0],True,.3)
    assert state=='dynamic' and not evidence.static_certificate

def test_coordinate_translation_rebases_surface_history_without_creating_motion():
    evidence=SurfaceMotion()
    for t in (1.,1.1,1.2):evidence.update(BASE,t,[0,0],False,.3)
    track=SimpleNamespace(position=BASE[0,:2].copy(),anchor_pos=BASE[0,:2].copy(),history=[],
                          surface_motion=evidence,support_points_world=BASE.tolist(),footprint_world=BASE[:,:2].tolist())
    rebase_detector_tracks([track],.25,-.3)
    assert evidence.update(BASE+[.25,-.3,0],1.3,[0,0],False,.3)[0]=='static'

def test_reference_intrusion_decelerates_before_any_avoidance_path_exists():
    obj=dict(id=4,occupied_cells_world=[[16.,0.]],static_certified=False,
             observed_surface_count=3,last_surface_count=3,in_trigger_corridor=True)
    scene=dict(state='valid',geometry_stamp=10.,scene_id=1,obstacles=[obj])
    decision=preslow(scene,(0,0,0),None,-1.,10.,reference=[[0,0],[30,0]])
    assert decision['present'] and decision['inpath']==16.
    obj['occupied_cells_world']=[[16.,4.]]
    assert not preslow(scene,(0,0,0),None,-1.,10.,reference=[[0,0],[30,0]])['present']

def test_portal_25m_control_is_independent_of_gps_shadow_start():
    path=Path(__file__).parents[2]/'waypoint_system/data/default/3.csv'
    params={'/section_router/sections':[dict(id='M7B',csv=3,idx=[6197,7000]),dict(id='M8',csv=3,idx=[7001,8850])],
            '/tunnel_recovery/approach_control_enabled':True,'/tunnel_recovery/approach_route_csv':str(path),
            '/tunnel_recovery/approach_anchor_idx':7350,'/tunnel_recovery/control_prepare_distance_m':25.}
    r=read_region(lambda key,default=None:params.get(key,default))
    assert r[1]==7101 and r[1]-r[3]==6751
    assert not contains(r,3,7001) and not contains(r,3,7100)
    assert contains(r,3,7101) and contains(r,3,6751,True)
    assert params['/section_router/sections'][1]['idx'][0]==7001
