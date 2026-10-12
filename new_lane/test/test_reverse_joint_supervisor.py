"""Supervisor regression without a ROS master or simulator commands."""
import importlib.util
import sys
from pathlib import Path
import math
import numpy as np
import pytest

rospy = pytest.importorskip('rospy')
sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
spec = importlib.util.spec_from_file_location('joint_reverse_node', Path(__file__).parents[1] / 'scripts/reverse_escape_node.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(rospy.Time, 'now', staticmethod(lambda: rospy.Time.from_sec(10.)))
    n = r.ReverseEscape.__new__(r.ReverseEscape)
    n.pose = (0., 0., 0.)
    n.memory = {}; n.memory_sec = 10.
    n.bg_ids = set(); n.bg_ids_t = 0.
    n.dyn_ids = set(); n.static_ids = set()
    return n


def test_unclassified_memory_does_not_become_ten_second_static(node):
    o = dict(id=1, position_world=[-3., 0.], length=1., width=1.)
    node.remember(o)
    assert len(node.memory_points(10.4)) > 0
    assert len(node.memory_points(10.6)) == 0


def test_dynamic_reclassification_erases_old_static_memory(node):
    o = dict(id=1, motion_state='static', position_world=[-3., 0.], length=1., width=1.)
    node.remember(o); assert 1 in node.memory
    node.remember(dict(o, motion_state='dynamic'))
    assert 1 not in node.memory


def test_static_memory_fills_object_interior(node):
    o = dict(id=1, motion_state='static', footprint_world=[[-1,-1],[1,-1],[1,1],[-1,1]])
    node.remember(o)
    points = node.memory_points(10.)
    assert np.linalg.norm(points, axis=1).min() < .15


def test_preexisting_near_object_cannot_be_ignored_when_backing(node):
    node.rev_max = 10.; node.rear_margin = 1.
    node.cloud_world = np.array([[-1.15, 0.]])
    node.cloud_pts = np.empty((0, 2)); node.sensor_x = 1.1
    node.front_overhang = 3.85; node.rear_overhang = .79
    node.half_width = .95; node.body_margin = .5
    node.joint_enabled = True
    node.joint_poses = np.array([[0,0,0],[-1,0,0],[-2,0,0]])
    path = (node.joint_poses[:, :2], np.array([0.,1.,2.]), 'joint')
    assert node.rear_free_along(path, 0.) == 0.


def test_reverse_controller_tracks_bicycle_arc_without_saturation(node):
    steering = .2; wheelbase = 3.; ds = .1
    poses = [(0., 0., 0.)]
    for _ in range(80):
        x,y,h = poses[-1]; dh=-ds*math.tan(steering)/wheelbase
        poses.append((x-ds*math.cos(h+dh/2), y-ds*math.sin(h+dh/2), h+dh))
    node.joint_enabled=True; node.joint_controls=np.full(len(poses),steering)
    node.joint_poses=np.array(poses)
    node.rev_path=(node.joint_poses[:, :2],np.arange(len(poses))*ds,'joint')
    node.lookahead_min=1.5;node.lookahead_gain=0.
    node.wheelbase=wheelbase;node.max_steer=.6;node.steer_limit=.3
    node.heading_change_limit=math.radians(60);node.target={'yaw0':0.}
    errors=[]; controls=[]
    # Exclude final lookahead truncation; supervisor handles terminal deceleration.
    for _ in range(100):
        progress=node.path_progress(node.rev_path)
        delta=node.reverse_steer(node.rev_path,progress,1.)
        controls.append(delta)
        x,y,h=node.pose;dh=-.05*math.tan(delta)/wheelbase
        node.pose=(x-.05*math.cos(h+dh/2),y-.05*math.sin(h+dh/2),h+dh)
        errors.append(min(np.hypot(node.joint_poses[:,0]-node.pose[0],node.joint_poses[:,1]-node.pose[1])))
    assert max(errors)<.15
    assert max(abs(c) for c in controls)<.3


def test_corrected_geometry_is_not_overwritten_by_raw_merged_box(node):
    from types import SimpleNamespace
    import json
    node.corrected_obs_t = -1e9
    node.planning_obs_cb(SimpleNamespace(data=json.dumps(dict(mission_mode='tunnel', obstacles=[
        dict(id=9, motion_state='static', position_world=[-3., 0.], footprint_world=[[-3.5,-.5],[-2.5,-.5],[-2.5,.5],[-3.5,.5]])]))))
    node.obs_cb(SimpleNamespace(data=json.dumps(dict(mission_mode='tunnel', obstacles=[
        dict(id=9, motion_state='static', position_world=[-3., 0.], length=8., width=4.)]))))
    points = node.memory_points(10.)
    assert np.ptp(points[:, 0]) <= 1.01
    assert np.ptp(points[:, 1]) <= 1.01


def test_background_exclusion_removes_wall_memory(node):
    from types import SimpleNamespace
    import json
    node.corrected_obs_t = -1e9
    node.remember(dict(id=9,motion_state='static',position_world=[-3.,0.],length=1.,width=1.))
    node.planning_obs_cb(SimpleNamespace(data=json.dumps(dict(mission_mode='tunnel', obstacles=[
        dict(id=9, background_excluded=True)]))))
    assert 9 not in node.memory
    assert not node.obs


def test_wall_sized_footprint_not_remembered(node):
    # 2026-10-08: 벽+턱 25 m 외곽선이 차체를 덮어 후진 시작부터 거절됐다.
    o = dict(id=7, motion_state='static', footprint_world=[[-5,-3],[20,-3],[20,-1],[-5,-1]])
    node.remember(o)
    assert 7 not in node.memory


def test_post_stop_failure_restarts_longer_reverse_search(node):
    # 2026-10-09(14:54:47 백): 3 m 후진 뒤 전진 재검증이 37 s 실패했는데 post_stop 에서 못 빠져나왔다.
    node.state, node.state_t = r.POST_STOP, 5.
    node.joint_enabled, node.joint_result = True, None
    node.post_stop_retry_sec, node.trigger_stop_sec = 3., 8.
    node.min_reverse, node.rev_max = 1.5, 10.
    node.target = dict(dist=3.)
    assert not node.post_stop_retry(7.)
    assert node.post_stop_retry(9.)
    assert node.state == r.IDLE and node.retry_min_reverse == 5.
    assert node.target is None and node.stopped_since == 1.


def test_tunnel_reverse_after_three_seconds_of_spatial_failure(node):
    # 2026-10-12(21:51:17 백): 정지 8 s 타이머가 조금만 움직여도 리셋돼 후진이 안 나왔다.
    node.tunnel_trigger_stop_sec = 3.
    node.forward_failure_reason, node.forward_failure_stamp = 'no_connected_corridor', 10.
    node.forward_mission_blocked = True
    node.spatial_fail_since = 8.
    assert not node.tunnel_trigger_ready(10.)
    node.spatial_fail_since = 7.
    assert node.tunnel_trigger_ready(10.)
    node.forward_failure_reason = 'stale_lidar'
    assert not node.tunnel_trigger_ready(10.)


def test_tunnel_solver_failure_without_mission_object_never_triggers_reverse(node):
    node.forward_failure_reason='no_connected_corridor';node.forward_failure_stamp=10.
    node.forward_mission_blocked=False
    assert not node.m8_escape_allowed(10.)
    node.forward_mission_blocked=True;node.forward_failure_reason='curvature'
    assert not node.m8_escape_allowed(10.)
