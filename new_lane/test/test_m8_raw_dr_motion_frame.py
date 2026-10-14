"""2026-10-14 이동 판정 자차 보상 = 보정 전 순수 DR. 측위 보정 단계가 정지 물체를 이동으로 만들지 않는지."""
import ast,math,sys
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'new_lane/scripts'))
from m8_motion import RigidMotion
from obstacle_geometry import pose_at

src=ast.parse((ROOT/'yolo_test/obstacle_perception_node.py').read_text())
fns=[v for c in src.body if isinstance(c,ast.ClassDef) for v in c.body if isinstance(v,ast.FunctionDef) and v.name in ('_motion_frame_points','dr_raw_callback')]
assert len(fns)==2
ns=dict(math=math,np=np,pose_at=pose_at)
exec(compile(ast.Module(body=fns,type_ignores=[]),'perception_methods','exec'),ns)


def node():
    return NS(m8_dr_motion_frame=True,dr_raw_history=deque(maxlen=300),dr_raw_epoch=0,pose_max_age=.15,native_coordinate_version=0)


def odom_msg(t,x,y,yaw,v=0.):
    return NS(header=NS(stamp=NS(to_sec=lambda:t)),pose=NS(pose=NS(position=NS(x=x,y=y),orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))),twist=NS(twist=NS(linear=NS(x=v))))


def face(cx,cy):
    ys=np.arange(-.3,.31,.05);return np.c_[np.full(len(ys),cx),cy+ys]


def run(use_dr,bias_deg=.54,dist=35.):
    """정지 차량 위 자차(속도 0) 앞 dist m 정지 물체. map yaw 에 X-ICP bias 를 초당 4 deg 로 적용(실제 적용 속도)."""
    n=node();n.m8_dr_motion_frame=use_dr;rm=RigidMotion();out=[]
    for k in range(40):
        t=k*.1
        bias=-math.radians(min(bias_deg,4.*max(0.,t-1.)))  # 1 s 부터 적용
        map_pose=(0.,0.,bias)
        ns['dr_raw_callback'](n,odom_msg(t,0.,0.,0.))
        # 센서 기준 물체 점은 그대로(물체·자차 모두 정지). 세계 좌표는 자세에 따라 달라진다.
        pts,frame,rot=ns['_motion_frame_points'](n,face(dist,0.),map_pose,t)
        out.append(rm.update(pts,t,arrival_time=t+.03))
    return out


def test_map_frame_yaw_bias_step_creates_false_motion_at_35m():
    out=run(use_dr=False)
    assert any(o[1] or o[0]=='dynamic' for o in out)


def test_raw_dr_frame_is_immune_to_yaw_bias_step():
    out=run(use_dr=True)
    assert not any(o[1] or o[0]=='dynamic' for o in out)
    assert out[-1][0]=='static'


def test_velocity_rotated_back_to_map_frame():
    n=node();ns['dr_raw_callback'](n,odom_msg(0.,0.,0.,0.));ns['dr_raw_callback'](n,odom_msg(.1,0.,0.,0.))
    pts,frame,rot=ns['_motion_frame_points'](n,np.array([[10.,0.]]),(5.,5.,math.radians(2.)),.05)
    assert frame==('dr_raw',0) and abs(rot-math.radians(2.))<1e-9 and np.allclose(pts,[[10.,0.]])


def test_missing_dr_falls_back_to_map_and_discontinuity_bumps_epoch():
    n=node()
    pts,frame,rot=ns['_motion_frame_points'](n,np.array([[10.,0.]]),(5.,5.,0.),1.)
    assert frame==('map',0) and rot==0. and np.allclose(pts,[[15.,5.]])
    ns['dr_raw_callback'](n,odom_msg(0.,0.,0.,0.,v=1.));ns['dr_raw_callback'](n,odom_msg(.1,50.,0.,0.,v=1.))
    assert n.dr_raw_epoch==1 and len(n.dr_raw_history)==1
    ns['dr_raw_callback'](n,odom_msg(.05,50.,0.,0.))  # 시간 되감김
    assert n.dr_raw_epoch==2


def test_required_missing_raw_dr_does_not_use_corrected_map_as_motion():
    n=node();n.m8_require_dr=True
    points,frame,rot=ns['_motion_frame_points'](n,np.array([[10.,0.]]),(5.,5.,0.),1.)
    assert points.shape==(0,2) and frame[0]=='dr_raw_unavailable'
