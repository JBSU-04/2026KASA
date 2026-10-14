"""2026-10-13 X-ICP 횡보정(DR 반영) 단위 검사. ROS master 없음, 어댑터 실제 메서드를 AST 로 뽑아 실행."""
import ast,bisect,math,json,threading,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from collections import deque
from morai_sensor_fusion.measurement import GpsIntegrityMonitor

NOW=[10.]
source=ast.parse((Path(__file__).parents[1]/'scripts/morai_measurement_adapter.py').read_text())
names=('_history_at','xicp_status_callback','_xicp_lateral_step')
methods=[v for c in source.body if isinstance(c,ast.ClassDef) and c.name=='MeasurementAdapter' for v in c.body if isinstance(v,ast.FunctionDef) and v.name in names]
assert len(methods)==3
ns=dict(math=math,bisect=bisect,json=json,GpsIntegrityMonitor=GpsIntegrityMonitor,String=lambda data:data,
        staticmethod=staticmethod,rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:NOW[0]))))
exec(compile(ast.Module(body=methods,type_ignores=[]),'adapter_methods','exec'),ns)
history_at=ns['_history_at'].__func__ if hasattr(ns['_history_at'],'__func__') else ns['_history_at']


class Node:
    pass


def node():
    n=Node()
    n.xicp_lateral_enabled=True;n.xicp_lateral_rate=.2;n.xicp_lateral_deadband=.03;n.xicp_lateral_max=.6
    n.xicp_lateral_axis=math.pi/2.;n.xicp_lateral_pending=0.;n.xicp_lateral_stamp=None;n.xicp_lateral_applied=deque(maxlen=400)
    n.dr_history=deque(maxlen=400);n.odom_history=deque(maxlen=200);n.lock=threading.Lock()
    n.gps_integrity=NS(mode=GpsIntegrityMonitor.FROZEN);n.tunnel_m8_active=True;n.tunnel_section_stamp=NS(to_sec=lambda:NOW[0])
    n.dr_position=(0.,0.);n.xicp_lateral_pub=NS(publish=lambda x:None)
    n._history_at=history_at
    return n


def status(t,lat,state=2,obs=True):
    return NS(header=NS(stamp=NS(to_sec=lambda:t)),state=state,lateral_observable=obs,lateral_observability=.998,lateral_correction=lat)


def drive(n,t0,t1,truth_y,odom_lag=.5,meas_every=.1,dt=.01,noise=0.):
    """자차는 x 로 직진. 참 횡위치 truth_y. odom 은 DR 을 1차 지연(odom_lag s)으로 따라간다. X-ICP 는 odom 기준 오차를 낸다."""
    t=t0;odom_y=n.odom_history[-1][2] if n.odom_history else n.dr_position[1];next_m=t0
    while t<t1-1e-9:
        t+=dt;NOW[0]=t
        n.dr_position=(n.dr_position[0]+.0,n.dr_position[1])
        ns['_xicp_lateral_step'](n,t,dt)
        n.dr_history.append((t,n.dr_position[0],n.dr_position[1]))
        odom_y+=(n.dr_position[1]-odom_y)*min(1.,dt/odom_lag)
        n.odom_history.append((t,0.,odom_y))
        if t>=next_m:
            next_m+=meas_every
            ns['xicp_status_callback'](n,status(t-.03,truth_y-n._history_at(n.odom_history,t-.03)[1]+noise))
    return n.dr_position[1],odom_y


class Lateral(unittest.TestCase):
    def setUp(self):
        self.n=node()
        for k in range(30):self.n.dr_history.append((9.7+k*.01,0.,0.));self.n.odom_history.append((9.7+k*.01,0.,0.))

    def test_converges_in_about_one_second_without_overshoot_despite_ekf_lag(self):
        ys=[]
        for k in range(30):
            dr,odom=drive(self.n,10.+k*.1,10.1+k*.1,truth_y=.2);ys.append(dr)
        self.assertLess(abs(ys[11]-.2),.03)           # 약 1.1 s 안에 0.2 m 수렴
        self.assertLess(max(ys),.2+.02)                # EKF 지연(0.5 s)에도 과보정 없음
        self.assertLess(abs(ys[-1]-.2),.03)            # 데드밴드(0.03 m) 안에서 멈춘다

    def test_rate_limit(self):
        drive(self.n,10.,10.5,truth_y=.5)
        self.assertLessEqual(self.n.dr_position[1],.2*.5+1e-6)

    def test_invalid_stale_or_degraded_measurements_ignored(self):
        NOW[0]=10.
        for m in (status(9.95,.2,state=1),status(9.95,.2,obs=False),status(9.5,.2),status(9.95,.9)):
            ns['xicp_status_callback'](self.n,m)
        self.assertEqual(self.n.xicp_lateral_pending,0.)

    def test_gps_normal_ignores_and_clears(self):
        NOW[0]=10.;ns['xicp_status_callback'](self.n,status(9.98,.2))
        self.assertAlmostEqual(self.n.xicp_lateral_pending,.2)
        self.n.gps_integrity.mode=GpsIntegrityMonitor.NORMAL
        self.assertEqual(ns['_xicp_lateral_step'](self.n,10.01,.01),0.)
        self.assertEqual(self.n.xicp_lateral_pending,0.)
        ns['xicp_status_callback'](self.n,status(10.,.2));self.assertEqual(self.n.xicp_lateral_pending,0.)

    def test_deadband_and_stale_pending(self):
        NOW[0]=10.;ns['xicp_status_callback'](self.n,status(9.98,.02))
        self.assertEqual(self.n.xicp_lateral_pending,0.)
        ns['xicp_status_callback'](self.n,status(9.98,.2))
        self.assertEqual(ns['_xicp_lateral_step'](self.n,11.5,.01),0.)  # 1 s 넘게 새 측정이 없으면 밀지 않는다

    def test_disabled_switch(self):
        self.n.xicp_lateral_enabled=False;NOW[0]=10.
        ns['xicp_status_callback'](self.n,status(9.98,.2))
        self.assertEqual(ns['_xicp_lateral_step'](self.n,10.01,.01),0.)


if __name__=='__main__':
    unittest.main()
