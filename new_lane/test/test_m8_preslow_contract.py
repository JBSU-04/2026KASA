"""Exercise the production preslow callback without ROS master/control outputs."""
import sys,importlib.util,json
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import pytest
spec=importlib.util.spec_from_file_location('wall_node',Path(__file__).parents[1]/'scripts/tunnel_wall_detector.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Pub:
    def __init__(self):self.values=[]
    def publish(self,x):self.values.append(x.data)
@pytest.fixture
def n(monkeypatch):
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(10.)))
    x=m.TunnelWallDetector.__new__(m.TunnelWallDetector)
    for k in ['kick_pub','kick_dist_pub','kick_inpath_pub','preslow_cap_pub','preslow_state_pub']:setattr(x,k,Pub())
    x.independent_preslow=True;x.is_tunnel_zone=lambda:True;x.kick_pose_at=lambda t:(0,0,0)
    x.sensor_x=1.1;x.route=(np.c_[np.arange(50.),np.zeros(50)],np.arange(50),None)
    x.preslow_root={};x.preslow_root_stamp=10.;x.preslow_input_stamp=-1e9
    x.preslow_start=25.;x.preslow_min=4.;x.preslow_initial=6.;x.accepted_selections={}
    return x

def points(rows,stamp=10.):
    p=np.asarray(rows,dtype='<f4').reshape((-1,6));h=Header(stamp=rospy.Time.from_sec(stamp),frame_id='velodyne_ground')
    return PointCloud2(header=h,height=1,width=len(p),fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(['x','y','z','map_ground','map_label','ground_relative'])],point_step=24,row_step=24*len(p),data=p.tobytes(),is_dense=True)
def test_confirmed_residual_uses_distance_floor(n):
    n.filtered_preslow_cb(points([[13.9,0,-1.4,-1.765,3,.365]]))
    assert n.kick_pub.values[-1] is True;assert n.preslow_cap_pub.values[-1]==5.
    n.filtered_preslow_cb(points([[2.,0,-1.4,-1.765,3,.365]]));assert n.preslow_cap_pub.values[-1]==4.
def test_wall_side_background_and_unknown_never_preslow(n):
    for row in [[12.,6,-1.,-1.765,3,.765],[12.,0,-1.,-1.765,1,.765],[12.,0,-1.,-1.765,2,.765]]:
        n.filtered_preslow_cb(points([row]));assert not n.kick_pub.values[-1];assert n.preslow_cap_pub.values[-1]==9999.
def test_stale_input_cannot_refresh(n):
    n.filtered_preslow_cb(points([[12,0,-1,-1.765,3,.765]],9.6));assert not n.kick_pub.values[-1]
    assert n.preslow_input_stamp<0

def test_preslow_ignores_mission_path_and_objects(n):
    # 2026-10-12 선서행은 미션과 완전 분리: 회피 경로가 정적 물체를 피해 가도 지도 잔차면 감속한다.
    n.preslow_root={'collision_objects':[dict(id=7,motion_state='static',static_certified=True,position_world=[15,0],support_points_world=[[15,0],[15.2,.1],[15.1,-.1]])]}
    n.kick_avoid_path=np.c_[np.arange(25.),np.full(25,2.)];n.kick_avoid_path_t=10.;n.kick_path_stamp=10.
    n.accepted_selections={10.:{'stamp':10.,'valid':True}}
    n.filtered_preslow_cb(points([[13.9,0,-1.4,-1.765,3,.365]]));assert n.kick_pub.values[-1]
    assert '"id":7' in n.preslow_state_pub.values[-1]

def test_far_residual_extends_preslow_to_25m(n):
    n.preslow_initial=5.
    n.far_preslow_cb(points([[22.9,0,-1.4,-1.765,3,.365]]))
    n.filtered_preslow_cb(points([]));assert n.kick_pub.values[-1] is True
    assert abs(n.preslow_cap_pub.values[-1]-(4.+(24.-5.)/20.))<1e-6
    assert 'filtered_far_map_residual' in n.preslow_state_pub.values[-1]

def test_stale_far_residual_ignored(n):
    n.far_preslow_cb(points([[22.9,0,-1.4,-1.765,3,.365]]))
    n.preslow_far=(n.preslow_far[0],9.5)
    n.filtered_preslow_cb(points([]));assert not n.kick_pub.values[-1]
