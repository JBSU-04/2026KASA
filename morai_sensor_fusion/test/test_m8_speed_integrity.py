"""Exercise real adapter input-loss branch without ROS/controller connections."""
import ast
import json
import math
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS, MethodType

class Stamp:
    def __init__(self,x):self.x=x
    def to_sec(self):return self.x
    def __sub__(self,other):return Stamp(self.x-other.x)
    def __le__(self,other):return self.x<=other.x

file=Path(__file__).parents[1]/'scripts/morai_measurement_adapter.py'
tree=ast.parse(file.read_text())
methods=[n for c in tree.body if isinstance(c,ast.ClassDef)and c.name=='MeasurementAdapter'
         for n in c.body if isinstance(n,ast.FunctionDef)and n.name in ('imu_callback','real_speed_fresh')]
env=dict(json=json,math=math,rospy=NS(Time=NS(now=lambda:Stamp(10.)),get_param=lambda k,d:d,logerr_throttle=lambda *a:None))
exec(compile(ast.Module(body=methods,type_ignores=[]),str(file),'exec'),env)

def node():
    valid=[];state=[];published=[]
    n=NS(lock=threading.RLock(),speed=None,speed_stamp=None,cur_speed=None,cur_speed_stamp=None,
         speed_timeout=.35,tunnel_m8_active=True,tunnel_section_stamp=Stamp(9.9),
         speed_state_time=-1e9,speed_valid_pub=NS(publish=valid.append),speed_state_pub=NS(publish=state.append),
         imu_buffer=deque(),quaternion_yaw=lambda q:0.,convert_yaw_rate=lambda y,w:(y,w),
         message_time=lambda t:t,dr_stamp=Stamp(9.9),publish_dead_reckoning_pose=lambda *a:published.append(a))
    n.real_speed_fresh=MethodType(env['real_speed_fresh'],n)
    return n,valid,state,published

def test_zero_measured_speed_is_valid_and_expired_speed_is_not():
    n,*_=node();n.speed=0.;n.speed_stamp=Stamp(9.9)
    assert n.real_speed_fresh(Stamp(10.))
    assert not n.real_speed_fresh(Stamp(10.3))

def test_gps_derivative_cannot_fabricate_outage_twist():
    n,valid,state,published=node();n.gps_derived_speed=12.;n.gps_derived_stamp=Stamp(10.)
    n.get_effective_speed=lambda *_:(_ for _ in ()).throw(AssertionError('must not fall back to GPS or zero'))
    msg=NS(orientation=None,angular_velocity=NS(z=0.),header=NS(stamp=Stamp(10.)),
           orientation_covariance=[0.]*8+[.0004],angular_velocity_covariance=[0.]*8+[.0001])
    env['Bool']=lambda **kwargs:NS(**kwargs);env['String']=lambda **kwargs:NS(**kwargs)
    env['imu_callback'](n,msg)
    assert valid[-1].data is False
    assert json.loads(state[-1].data)['reason']=='speed_input_expired'
    assert published and n.dr_stamp.to_sec()==10.
