import ast
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from m8_object_policy import StationaryHandoff
import m8_geometry_native as native
from tunnel_occupancy_geometry import swept_owned_cells

ROUTE = [[0.,0.,7201,3],[20.,0.,7401,3]]
POINTS = [[8.,-.2,.15],[8.,0.,.15],[8.,.2,.15],[8.1,0.,.15]]

def frame(t, moving=False, y=0., points=None):
    return dict(collision_objects=[dict(id=7,confirmed=True,source='lidar_only',last_seen=t,
        motion_state='dynamic' if moving else 'unknown',dynamic_confirmed=moving,
        observed_motion_valid=True,observed_velocity=[1. if moving else 0.,0.],
        support_points_world=points if points is not None else [[x,z+y,h] for x,z,h in POINTS])])

def run(policy,end=5.2,y=0.):
    policy.update(frame(0.,True,y),0.,ROUTE,0)
    for t in np.arange(.1,end+.001,.1):
        root=frame(float(t),y=y);policy.update(root,float(t),ROUTE,0)
    return root['collision_objects'][0]

def test_five_seconds_requires_observed_blocker_and_keeps_dynamic_history():
    p=StationaryHandoff();o=run(p,4.9);assert not o['m8_restatic']
    o=frame(5.1);p.update(o,5.1,ROUTE,0);o=o['collision_objects'][0]
    assert o['m8_restatic'] and o['static_certified'] and o['ever_dynamic']

def test_stationary_side_object_is_not_converted():
    assert not run(StationaryHandoff(),y=4.)['m8_restatic']

def test_missing_scan_and_coordinate_change_restart_confirmation():
    p=StationaryHandoff();run(p,4.)
    root=frame(4.5);p.update(root,4.5,ROUTE,0)
    assert root['collision_objects'][0]['m8_stationary_seconds']==0.
    root=frame(4.6);p.update(root,4.6,ROUTE,1)
    assert not root['collision_objects'][0]['ever_dynamic']

def test_new_visible_face_does_not_restart_stationary_timer():
    p=StationaryHandoff();run(p,4.9)
    root=frame(5.1,points=POINTS+[[8.2,.1,.15]])
    p.update(root,5.1,ROUTE,0);assert root['collision_objects'][0]['m8_restatic']

def test_two_moving_scans_revoke_static_handoff():
    p=StationaryHandoff();run(p)
    one=frame(5.3,True);p.update(one,5.3,ROUTE,0)
    assert one['collision_objects'][0]['m8_restatic']
    two=frame(5.4,True);p.update(two,5.4,ROUTE,0)
    assert not two['collision_objects'][0]['m8_restatic']
    assert two['collision_objects'][0]['dynamic_confirmed']

def test_duplicate_scan_cannot_add_stationary_time():
    p=StationaryHandoff();run(p,2.)
    last=p.states[('lidar_only',7)]['last']
    for _ in range(50):p.update(frame(2.),2.,ROUTE,0)
    assert p.states[('lidar_only',7)]['last']==last

def test_native_connectivity_matches_exact_scipy_without_point_thinning():
    rng=np.random.default_rng(29)
    for points in [rng.normal(size=(800,2)),rng.uniform(-3,3,(500,2)),np.array([[0,0],[.3,0],[.600001,0]])]:
        edges=cKDTree(points).query_pairs(.3,output_type='ndarray')
        adj=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(points),len(points)))
        _,expected=connected_components(adj,directed=False)
        actual=native.components(points,.3)
        assert len(actual)==len(points)
        assert np.array_equal(expected[:,None]==expected,actual[:,None]==actual)

def test_native_sweep_matches_python_for_utms_curves_and_side_cells():
    rng=np.random.default_rng(33)
    old=native.enabled
    try:
        for _ in range(40):
            offset=np.array([302600.,4123700.])
            path=np.column_stack([np.linspace(0,25,80),np.sin(np.linspace(0,2,80))])+offset
            cells=rng.uniform([-2,-5],[30,5],(30,2))+offset
            native.enabled=False;expected=swept_owned_cells(cells,path,offset,0.,horizon=20.)
            native.enabled=True;actual=swept_owned_cells(cells,path,offset,0.,horizon=20.)
            assert actual is expected
    finally:native.enabled=old

def test_reverse_requires_fresh_spatial_failure_not_dynamic_or_timeout():
    file=Path(__file__).parents[1]/'scripts/reverse_escape_node.py'
    tree=ast.parse(file.read_text())
    method=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body
                if isinstance(n,ast.FunctionDef) and n.name=='m8_escape_allowed')
    env={};exec(compile(ast.Module(body=[method],type_ignores=[]),str(file),'exec'),env)
    node=SimpleNamespace(forward_failure_stamp=10.,forward_failure_reason='no_connected_corridor',forward_mission_blocked=True)
    assert env['m8_escape_allowed'](node,10.1)
    for reason in ['stale_lidar','stale_pose_reference_or_geometry','search_timeout','dynamic_hold']:
        node.forward_failure_reason=reason;assert not env['m8_escape_allowed'](node,10.1)
    node.forward_failure_reason='no_connected_corridor';assert not env['m8_escape_allowed'](node,11.)


def test_unconfirmed_object_cannot_become_certified_by_waiting():
    p=StationaryHandoff()
    for t in np.arange(0.,5.3,.1):
        root=frame(float(t),moving=t==0.)
        root['collision_objects'][0]['confirmed']=False
        p.update(root,float(t),ROUTE,0)
    assert not root['collision_objects'][0]['m8_restatic']
