"""2026-10-13 벽 제거 전 분기 + 배경 지도 비교 선서행(repaired_preslow_cb) 격리 검사. ROS master 없음."""
import sys,importlib.util,json
from pathlib import Path
import numpy as np
import rospy
import pytest
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
spec=importlib.util.spec_from_file_location('wall_node_prewall',Path(__file__).parents[1]/'scripts/tunnel_wall_detector.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class Pub:
    def __init__(self):self.values=[]
    def publish(self,x):self.values.append(x.data)


@pytest.fixture
def n(monkeypatch):
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(10.)))
    x=m.TunnelWallDetector.__new__(m.TunnelWallDetector)
    for k in ['kick_pub','kick_dist_pub','kick_inpath_pub','preslow_cap_pub','preslow_state_pub']:setattr(x,k,Pub())
    x.independent_preslow=True;x.is_tunnel_zone=lambda:True;x.kick_pose_at=lambda t:(0.,0.,0.)
    xy=np.c_[np.arange(0.,60.,.5),np.zeros(120)];tan=np.c_[np.ones(120),np.zeros(120)]
    x.sensor_x=1.1;x.route=(xy,np.arange(7400,7520),tan);x.filtered_shift=0.
    # 실측 터널 단면: 우측 벽 -3.6, 좌측 벽 7.3
    x.wprof=(np.array([7400.,7520.]),np.array([-3.6,-3.6]),np.array([7.3,7.3]))
    x.preslow_root={};x.preslow_root_stamp=10.;x.preslow_input_stamp=-1e9;x.preslow_scene={}
    x.preslow_start=30.;x.preslow_min=4.;x.preslow_initial=7.;x.accepted_selections={}
    x.repaired_preslow=True;x.preslow_approved=None;x.preslow_side_speed=3.;x.preslow_crossing_window=1.5
    x.preslow_wall_margin=.15;x.preslow_footprint_release=True;x.preslow_ego_speed=10.
    return x


def cloud(rows,stamp=10.):
    """rows: (x_sensor, y_sensor, map_label). 높이 0.3 m 저상 점."""
    p=np.array([[r[0],r[1],-1.465,-1.765,r[2],.3] for r in rows],dtype='<f4').reshape((-1,6))
    h=Header(stamp=rospy.Time.from_sec(stamp),frame_id='velodyne_ground')
    return PointCloud2(header=h,height=1,width=len(p),fields=[PointField(name=k,offset=i*4,datatype=7,count=1)
        for i,k in enumerate(['x','y','z','map_ground','map_label','ground_relative'])],point_step=24,row_step=24*len(p),data=p.tobytes(),is_dense=True)


def state(n):return json.loads(n.preslow_state_pub.values[-1])


def test_empty_wall_surface_never_preslows(n):
    # 지도와 맞는 벽(1)과, 지도 불일치라도 벽면 0.15 m 안의 점은 벽 표면.
    n.repaired_preslow_cb(cloud([[18.,-3.7,1],[20.,-3.5,3],[22.,-3.6,3],[25.,7.25,3]]))
    assert n.preslow_cap_pub.values[-1]==9999.


def test_low_object_beside_wall_preslows_early(n):
    # 17:53 선서행(기준경로 +-1.3 m)이 놓친 벽 옆 저상 물체(20:46:03 id 18/19: 횡 -3.3 m, 약 30 m 앞).
    n.repaired_preslow_cb(cloud([[27.,-3.3,3],[27.2,-3.25,3]]))
    s=state(n);assert s['present'] and abs(s['distance']-28.1)<.6
    assert n.preslow_cap_pub.values[-1]<7.


def test_static_far_side_object_outside_crossing_envelope_ignored(n):
    # 좌측 벽 옆 6.6 m 정지 물체: 3 m/s x 1.5 s 횡단 감시 폭(5.65 m) 밖 -> 감속 안 함.
    n.repaired_preslow_cb(cloud([[20.,6.6,3]]));assert n.preslow_cap_pub.values[-1]==9999.


def test_verified_crossing_object_outside_corridor_preslows(n):
    o=dict(id=3,motion_state='unknown',dynamic_candidate=True,observed_velocity=[0.,-3.],confirmed=True,
           last_seen=10.,footprint_world=[[20.6,6.3],[21.4,6.3],[21.4,6.9],[20.6,6.9]])
    n.preslow_root=dict(stamp=10.,obstacles=[o])
    n.repaired_preslow_cb(cloud([[20.,6.6,3]]))
    assert state(n)['present'] and n.preslow_cap_pub.values[-1]<7.


def test_approved_path_clearing_static_target_releases_only_that_object(n):
    target=dict(id=7,motion_state='static',confirmed=True,last_seen=10.,
                footprint_world=[[15.5,-.5],[16.5,-.5],[16.5,.5],[15.5,.5]])
    n.preslow_root=dict(stamp=10.,obstacles=[target])
    path=np.c_[np.arange(0.,45.,.2),np.r_[np.linspace(0,2.6,40),np.full(185,2.6)]]
    n.preslow_approved=(path,10.,11,0)
    n.repaired_preslow_cb(cloud([[15.,0.,3],[15.2,.3,3]]))
    assert n.preslow_cap_pub.values[-1]==9999.
    assert any(r['id']==7 for r in state(n)['releases'])
    # 다른 후보(승인 경로 근처의 다른 저상 물체)는 계속 감속. 승인 경로가 좌로 비켜 가면 우측 벽 옆 -3.2 m 는
    #   경로에서 5.8 m 로 횡단 감시 폭 밖이다(감시는 기준경로가 아니라 승인 경로 기준).
    n.repaired_preslow_cb(cloud([[15.,0.,3],[26.,1.5,3]]))
    assert state(n)['present'] and n.preslow_cap_pub.values[-1]<7.
    # 승인 경로가 대상을 지나면(차체 겹침) 해제하지 않는다
    n.preslow_approved=(np.c_[np.arange(0.,45.,.2),np.zeros(225)],10.,12,0)
    n.repaired_preslow_cb(cloud([[15.,0.,3]]))
    assert state(n)['present']


def test_native_policy_keeps_unmatched_wall_side_point_and_requires_native_release(n):
    n.object_scene_enabled=True
    # The same physical wall point matched by the map is removed. A nearby
    # unmatched low protrusion remains monitoring evidence, without stop authority.
    n.repaired_preslow_cb(cloud([[27.,-3.5,1]]));assert not state(n)['present']
    n.repaired_preslow_cb(cloud([[27.,-3.5,3]]));assert state(n)['present']
    assert not state(n)['mission_control']


def test_native_policy_never_extends_boundary_into_open_profile(n):
    n.object_scene_enabled=True
    n.wprof=(np.array([7400.,7440.,7500.,7520.]),np.array([0.,0.,-3.6,-3.6]),np.array([0.,0.,7.3,7.3]))
    # x=10 maps to index 7420, an explicitly open span. Later walls cannot
    # exclude its residuals by extrapolating their section across the entrance.
    assert n.in_drivable_area(10.,-4.,margin=0.)
