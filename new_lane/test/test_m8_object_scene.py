from pathlib import Path
import sys,copy,io
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[1]/'scripts'))
from m8_object_scene import ObjectSceneBuilder,shared_surface
from m8_object_policy import StationaryHandoff
from m8_repair_policy import static_certificate
from tunnel_native_scene import scene_dict,final_root
from occupancy_avoidance.msg import TunnelScene

P=[[302510.,4123999.,.08],[302510.2,4123999.,.08],[302510.4,4123999.,.08],
   [302510.,4124001.8,.08],[302510.2,4124001.8,.08],[302510.4,4124001.8,.08]]
POSE=(302500.,4124000.,0.,0.,5.,.02)
def obj(stamp=10.,state='static',points=P,**kw):
    return dict(id=7,position_world=[302510.,4124000.],confirmed=True,motion_state=state,
                last_seen=stamp,support_points_world=points,in_trigger_corridor=True,**kw)
def scene(builder,o,stamp=10.,points=P):
    root=dict(mission_mode='tunnel',collision_objects=[o],geometry_scan={})
    return builder.build(root,[dict(o,support_points_world=points)] if points is not None else [],POSE,stamp,stamp+.02)

def test_disconnected_surface_preserves_s_gap_and_actual_height():
    m=scene(ObjectSceneBuilder(),obj());assert len(m.cells)==6
    assert all(p.z==.08 for p in m.cells)
    assert all(abs(p.y-4124000.)>.9 for p in m.cells)
    assert m.speed==5. and m.curvature==.02
    assert m.scene_id==1 and len(set(m.owner_ids))==1

def test_repeated_scan_not_processed_or_confirmed_twice():
    b=ObjectSceneBuilder();m=scene(b,obj());assert scene(b,obj()) is None
    assert b.sequence==1

def test_unknown_is_not_certified_and_stale_scene_not_free():
    b=ObjectSceneBuilder();m=scene(b,obj(state='unknown'))
    assert not m.objects[0].static_certified and not any(m.fixed)
    assert b.build({'collision_objects':[]},[],POSE,10.1,10.46) is None

def test_retained_surface_keeps_original_seen_not_latest_parent_time():
    b=ObjectSceneBuilder();m=scene(b,obj())
    m=scene(b,obj(stamp=10.2,points=P[:3]),10.2,P[:3])
    by_y={p.y:(t,v)for p,t,v in zip(m.cells,m.seen,m.observed)}
    assert by_y[4124001.8]==(10.,False) and by_y[4123999.]==(10.2,True)
    m=scene(b,obj(stamp=10.2),10.4,None)
    assert m.cells and max(m.seen)<=10.2 and not any(m.observed)
    root=final_root(scene_dict(m));o=root['collision_objects'][0]
    assert static_certificate(o,10.4)
    assert o['last_seen']==10.2

def test_retention_needs_three_distinct_returned_scans(monkeypatch):
    class Rays:
        def __init__(self,scan,now):self.ok=scan.get('tested_clear',False)
        def clear(self,pts):return self.ok
    monkeypatch.setattr('m8_object_scene.RayEvidence',Rays)
    b=ObjectSceneBuilder();scene(b,obj())
    for t in (10.1,10.2,10.3):
        root=dict(collision_objects=[obj(stamp=t,points=P[:3])],geometry_scan={'tested_clear':True})
        m=b.build(root,root['collision_objects'],POSE,t,t+.01)
        assert len(m.cells)==(3 if t==10.3 else 6)
    assert min(m.seen)==10.3

def test_rebase_preserves_physical_surfaces_and_original_time():
    b=ObjectSceneBuilder();scene(b,obj());b.rebase(8,.03,-.07)
    m=b.build(dict(collision_objects=[obj()]),[],POSE,10.1,10.11,8)
    assert len(m.cells)==6 and all(t==10. for t in m.seen)
    assert abs(m.cells[0].x-P[0][0]-.03)<1e-9 and m.coordinate_version==8

def test_no_source_cannot_fill_large_parent_box():
    b=ObjectSceneBuilder();o=obj();o.update(length=9.,width=8.)
    m=scene(b,o,points=None);assert not m.cells
    o=final_root(scene_dict(m))['collision_objects'][0]
    assert not o['collision_geometry_valid']

def test_matching_requires_bidirectional_surface_not_proximity():
    a=np.array(P);assert shared_surface(a,a+[.03,0,0])
    assert not shared_surface(a,a+[0,.5,0])
    assert not shared_surface(a,np.tile(a[:1],(20,1)))

def test_delayed_confirmed_far_points_match_planner_collision_snapshot():
    m=scene(ObjectSceneBuilder(),obj(stamp=9.8,state='unknown'))
    assert not any(m.observed) and all(t==9.8 for t in m.seen)
    assert len(scene_dict(m)['collision_cells_world'])==len(m.cells)

def test_native_utm_and_id_survive_wire():
    m=scene(ObjectSceneBuilder(),obj());buf=io.BytesIO();m.serialize(buf)
    out=TunnelScene();out.deserialize(buf.getvalue());assert out.cells==m.cells and list(out.owner_ids)==list(m.owner_ids)

def handoff_object(t,**kw):
    pts=[[10.,-.1,.2],[10.2,-.1,.2],[10.4,-.1,.2],[10.6,-.1,.2]]
    return obj(t,points=pts,motion_support_xy=[p[:2]for p in pts],motion_epoch=0,motion_dr_valid=True,
               observed_motion_valid=False,observed_velocity=[0,0],**kw)

def test_three_second_handoff_uses_dr_surface_not_xicp_translation():
    h=StationaryHandoff(require_raw_dr=True);route=[[0.,0.],[30.,0.]]
    first=handoff_object(10.);first['motion_state']='dynamic'
    h.update({'collision_objects':[first]},10.,route,0)
    for k in range(1,32):
        t=10.+.1*k;o=handoff_object(t);o['motion_state']='unknown'
        o['support_points_world']=(np.asarray(o['support_points_world'])+[.2*k/10,0,0]).tolist()
        h.update({'collision_objects':[o]},t,route,0)
        if k<30:assert not o['m8_restatic']
    assert o['m8_restatic'] and o['m8_stationary_seconds']>=3.
    assert not o['observed_motion_valid']
    for k in (32,33):
        o=handoff_object(10+.1*k);o.update(observed_motion_valid=True,observed_velocity=[0,2],motion_state='dynamic')
        h.update({'collision_objects':[o]},10+.1*k,route,0)
    assert not o['m8_restatic']

def test_gap_duplicate_epoch_or_missing_dr_cannot_complete_handoff():
    route=[[0.,0.],[30.,0.]]
    h=StationaryHandoff(require_raw_dr=True)
    o=handoff_object(10.);o['motion_state']='dynamic';h.update({'collision_objects':[o]},10.,route,0)
    for t in (10.1,10.1,10.8):
        o=handoff_object(t);o['motion_state']='unknown';h.update({'collision_objects':[o]},t,route,0)
    assert o.get('m8_stationary_seconds',0)<3.
    o=handoff_object(10.9);o['motion_dr_valid']=False
    h.update({'collision_objects':[o]},10.9,route,0);assert h.states[('lidar_only',7)]['since'] is None

def test_owner_survives_dynamic_static_parent_then_dynamic_again():
    b=ObjectSceneBuilder();o=obj(state='dynamic');a=scene(b,o)
    p=obj(10.1);p.update(id=1000000,observed_track_id=7,m8_restatic=True,static_certified=True)
    m=b.build({'collision_objects':[p]},[obj(10.1)],POSE,10.1,10.12)
    assert m.objects[0].id==a.objects[0].id==7
    m=scene(b,obj(10.2,state='dynamic'),10.2)
    assert m.objects[0].id==7 and not any(m.fixed)

def test_ambiguous_shared_surface_is_not_forced_to_nearest_owner():
    b=ObjectSceneBuilder();a=obj();c=obj(points=(np.asarray(P)+[0,.5,0]).tolist());c['id']=8
    m=b.build({'collision_objects':[a,c]},[a,c],POSE,10.,10.01)
    assert {o.id for o in m.objects}=={7,8}
    mixed=obj(10.1,points=P+(np.asarray(P)+[0,.5,0]).tolist());mixed['id']=9
    m=b.build({'collision_objects':[a,c,mixed]},[a,c,mixed],POSE,10.1,10.12)
    assert 9 in {o.id for o in m.objects}


def test_native_handoff_identity_survives_track_id_and_parent_replacement():
    b=ObjectSceneBuilder();b.handoff=StationaryHandoff(require_raw_dr=True,stable_owner=True)
    route=[[0.,0.],[30.,0.]]
    for k in range(33):
        t=10.+.1*k;o=handoff_object(t)
        if k==0:o['motion_state']='dynamic'
        else:o['motion_state']='static'
        o['id']=7 if k<10 else 40
        root_obj=dict(o)
        if k>=20:root_obj.update(id=1000000,observed_track_id=40)
        m=b.build({'collision_objects':[root_obj]},[o],POSE,t,t+.01,route=route)
        out=final_root(scene_dict(m))['collision_objects'][0]
        assert out['id']==7
        if k<30:assert not out.get('m8_restatic')
    assert out['m8_restatic'] and out['m8_stationary_seconds']>=3.


def test_verified_restart_does_not_wait_two_extra_frames_after_handoff():
    h=StationaryHandoff(require_raw_dr=True,stable_owner=True);route=[[0.,0.],[30.,0.]]
    first=handoff_object(10.);first['motion_state']='dynamic';h.update({'collision_objects':[first]},10.,route,0)
    for k in range(1,32):
        o=handoff_object(10.+.1*k);o['motion_state']='unknown';h.update({'collision_objects':[o]},10.+.1*k,route,0)
    assert o['m8_restatic']
    o=handoff_object(13.2);o.update(dynamic_candidate=True,observed_motion_valid=True,observed_velocity=[0.,2.],
        motion_evidence=dict(surface_translation_valid=True,observation_stamp=13.2))
    h.update({'collision_objects':[o]},13.2,route,0)
    assert not o['m8_restatic'] and o['observed_motion_valid']

def test_verified_wall_motion_retraction_clears_only_its_dynamic_history():
    h=StationaryHandoff(require_raw_dr=True,stable_owner=True);route=[[0.,0.],[30.,0.]]
    a=handoff_object(10.);a.update(motion_state='dynamic',dynamic_confirmed=True)
    other=handoff_object(10.);other.update(id=99,motion_state='dynamic',dynamic_confirmed=True)
    h.update({'collision_objects':[a,other]},10.,route,0)
    a=handoff_object(10.1);a.update(motion_state='static',dynamic_confirmed=True,
        motion_evidence=dict(motion_retracted='stationary_extended_surface',surface_translation_valid=False,observation_stamp=10.1))
    other=handoff_object(10.1);other.update(id=99,motion_state='dynamic',dynamic_confirmed=True)
    h.update({'collision_objects':[a,other]},10.1,route,0)
    assert a['motion_state']=='static' and not a['ever_dynamic'] and not a['dynamic_confirmed']
    assert other['ever_dynamic'] and other['motion_state']=='dynamic'

def test_stale_or_unobserved_wall_retraction_cannot_clear_dynamic_history():
    h=StationaryHandoff(require_raw_dr=True,stable_owner=True);route=[[0.,0.],[30.,0.]]
    a=handoff_object(10.);a.update(motion_state='dynamic',dynamic_confirmed=True)
    h.update({'collision_objects':[a]},10.,route,0)
    a=handoff_object(10.1);a.update(motion_state='static',motion_evidence=dict(
        motion_retracted='stationary_extended_surface',observation_stamp=10.))
    h.update({'collision_objects':[a]},10.1,route,0)
    assert a['ever_dynamic'] and a['motion_state']=='unknown'
