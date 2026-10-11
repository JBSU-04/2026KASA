import numpy as np
from m8_object_policy import SurfaceMotion
from tunnel_native_scene import packet_from_detector, scene_dict

BASE=np.array([[0.,0.,.2],[0.,.3,.2],[.4,0.,.2],[.4,.3,.2]])+np.array([302620.,4123655.,0.])

def test_stationary_surface_admits_after_three_real_scans():
    evidence=SurfaceMotion()
    assert evidence.update(BASE,1.,[0,0],False,.3)[0]=='unknown'
    assert evidence.update(BASE,1.1,[0,0],False,.3)[0]=='unknown'
    assert evidence.update(BASE,1.2,[0,0],False,.3)[0]=='static'

def test_growing_visible_face_does_not_create_dynamic_velocity():
    evidence=SurfaceMotion()
    for i in range(5):
        extra=BASE+np.array([.2*i,0.,0.])
        state,vector=evidence.update(np.vstack([BASE,extra]),1+i*.1,[2.,0.],True,.3)
    assert state=='static' and np.linalg.norm(vector)==0.

def test_real_coherent_translation_is_dynamic():
    evidence=SurfaceMotion()
    for i in range(5):
        state,vector=evidence.update(BASE+[0.,i*.15,0.],1+i*.1,[0.,1.5],True,.3)
    assert state=='dynamic' and np.allclose(vector,[0.,1.5])

def test_missing_and_duplicate_scans_cannot_certify_static():
    evidence=SurfaceMotion()
    for _ in range(8):assert evidence.update(BASE,1.,[0,0],False,.3)[0]=='unknown'
    assert evidence.update(BASE,1.5,[0,0],False,.3)[0]=='unknown'

def test_unrelated_surface_never_becomes_static_from_small_velocity():
    evidence=SurfaceMotion()
    for i in range(5):state,_=evidence.update(BASE+[i*.25,0.,0.],1+i*.1,[0,0],False,.3)
    assert state=='unknown'

def test_confirmed_unknown_occupancy_is_transient_in_same_planner_scene():
    obj=dict(id=7,confirmed=True,motion_state='unknown',last_seen=10.,
             position_world=BASE[0,:2].tolist(),support_points_world=BASE.tolist())
    root=dict(collision_objects=[obj])
    packet=packet_from_detector(root,root,{},(302620.,4123650.,0.),10.,2)
    scene=scene_dict(packet)
    assert len(scene['voxels'])==len(BASE)
    assert not any(v[3] for v in scene['voxels'])
    assert not scene['obstacles'][0]['static_certified']

# Exercise the release decision independently of ROS timers and publishers.
import ast
from pathlib import Path
from types import SimpleNamespace

def hold_node():
    path=Path(__file__).parents[1]/'scripts/obstacle_stop_node.py'
    tree=ast.parse(path.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
                if isinstance(n,ast.FunctionDef) and n.name=='m8_static_hold_clear')
    env={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),env)
    n=SimpleNamespace(_m8_hold_owners={7},actual_body_risk=lambda o:False,
                      tunnel_handoff_conflict=lambda o,t:False)
    n.clear=lambda objects,now:env['m8_static_hold_clear'](n,objects,now)
    return n

def safe_owner(**values):
    o=dict(id=1000000,observed_track_id=7,perception_source='common_scene',
           static_certified=True,motion_state='static',last_seen=10.)
    o.update(values);return o

def test_hold_releases_same_owner_after_parent_id_change_on_clear_path():
    assert hold_node().clear([safe_owner()],10.)

def test_hold_keeps_dynamic_unknown_stale_missing_and_real_body_risk():
    n=hold_node()
    for objects in [[],[safe_owner(motion_state='dynamic')],
                    [safe_owner(static_certified=False,motion_state='unknown')],
                    [safe_owner(last_seen=9.)],[safe_owner(observed_track_id=8)]]:
        assert not n.clear(objects,10.)
    n.actual_body_risk=lambda o:True
    assert not n.clear([safe_owner()],10.)
    n.actual_body_risk=lambda o:False
    for result in [None,True]:
        n.tunnel_handoff_conflict=lambda o,t:result
        assert not n.clear([safe_owner()],10.)

def test_previously_dynamic_owner_cannot_bypass_five_second_handoff_in_merger():
    import importlib.util
    path=Path(__file__).parents[1]/'scripts/obstacle_info_merger.py'
    spec=importlib.util.spec_from_file_location('entry_merger',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    cache=module.StaticOccupancy()
    obj=dict(id=7,source='lidar_only',position_world=[8.,0.],position_ego=[8.,0.,0.],
             length=.4,width=.4,confirmed=True,in_trigger_corridor=True,
             motion_state='unknown',stationary_sec=2.,ever_dynamic=True,m8_restatic=False)
    for t in [10.,10.1,10.2]:
        root=cache.update(dict(mission_mode='tunnel',collision_objects=[dict(obj,last_seen=t)]),
                          (0.,0.,0.),t)
        assert not cache.parents
        assert not root['collision_objects'][0].get('static_certified',False)
    obj.update(m8_restatic=True,motion_state='static')
    root=cache.update(dict(mission_mode='tunnel',collision_objects=[dict(obj,last_seen=10.3)]),
                      (0.,0.,0.),10.3)
    assert cache.parents
