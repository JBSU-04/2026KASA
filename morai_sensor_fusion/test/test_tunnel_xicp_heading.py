"""Exercise the actual adapter methods without a ROS master or vehicle control."""
import ast,bisect,math,threading,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from collections import deque
from morai_sensor_fusion.lane_heading import sample_rows
from morai_sensor_fusion.measurement import normalize_angle,interpolate_angle,GpsIntegrityMonitor
class Stamp:
 def __init__(self,x):self.x=x
 def to_sec(self):return self.x
 def __sub__(self,other):return Stamp(self.x-other.x)

source=ast.parse((Path(__file__).parents[1]/'scripts/morai_measurement_adapter.py').read_text())
methods=[v for c in source.body if isinstance(c,ast.ClassDef)and c.name=='MeasurementAdapter'for v in c.body if isinstance(v,ast.FunctionDef)and v.name in ('xicp_yaw_callback','lane_yaw_callback','_tunnel_heading_ref')]
namespace=dict(sample_rows=sample_rows,math=math,bisect=bisect,normalize_angle=normalize_angle,interpolate_angle=interpolate_angle,GpsIntegrityMonitor=GpsIntegrityMonitor,rospy=NS(Time=NS(now=lambda:Stamp(10.))))
exec(compile(ast.Module(body=methods,type_ignores=[]),'adapter_methods','exec'),namespace)
class Heading(unittest.TestCase):
 def setUp(self):
  self.n=NS(xicp_yaw_enabled=True,gps_integrity=NS(mode=GpsIntegrityMonitor.FROZEN),lock=threading.Lock(),imu_buffer=deque([(Stamp(9.95),.05),(Stamp(10.05),.05)]),quaternion_yaw=lambda q:q.yaw,xicp_bias=None,xicp_yaw_stamp=None,tunnel_exit_x_utm=0,tunnel_entry_x_utm=20,tunnel_min_y_utm=0,tunnel_max_y_utm=20,tunnel_nominal_heading=0,tunnel_wall_yaw_enabled=False,tunnel_camera_yaw_enabled=False)
 def message(self,t=10.,yaw=0.):return NS(header=NS(stamp=Stamp(t)),pose=NS(orientation=NS(yaw=yaw)))
 def observe(self,m):namespace['xicp_yaw_callback'](self.n,m)
 def heading(self):return namespace['_tunnel_heading_ref'](self.n,Stamp(10.1),10,10)
 def test_actual_turn_is_preserved_after_bias_correction(self):
  self.observe(self.message());self.n.imu_buffer.append((Stamp(10.1),.25))
  src,h=self.heading();self.assertEqual(src,'xicp');self.assertAlmostEqual(h,.2)
 def test_expired_result_never_corrects_heading(self):
  self.observe(self.message(9.));self.assertIsNone(self.n.xicp_bias)
 def test_missing_synchronized_imu_never_corrects_heading(self):
  self.n.imu_buffer=deque([(Stamp(9.),.05),(Stamp(11.),.05)])
  self.observe(self.message());self.assertIsNone(self.n.xicp_bias)
 def test_other_mission_cannot_accept_tunnel_correction(self):
  self.n.gps_integrity.mode=GpsIntegrityMonitor.NORMAL;self.observe(self.message());self.assertIsNone(self.n.xicp_bias)
 def test_unreasonable_bias_is_rejected(self):
  self.observe(self.message(yaw=.3));self.assertIsNone(self.n.xicp_bias)

 def test_dr_mode_ignores_existing_observer(self):
  self.observe(self.message());self.n.tunnel_heading_mode='dr'
  self.assertEqual(self.heading(),(None,None))
 def test_lane_capture_bias_preserves_future_turn(self):
  self.n.tunnel_heading_mode='ufld';self.n.lane_bias=None;self.n.lane_stamp=None
  namespace['lane_yaw_callback'](self.n,self.message())
  self.n.imu_buffer.append((Stamp(10.1),.25))
  src,yaw=self.heading();self.assertEqual(src,'ufld')
  self.assertAlmostEqual(yaw,.25-math.radians(.2))
 def test_lane_expiry_does_not_fall_back_to_xicp(self):
  self.observe(self.message());self.n.tunnel_heading_mode='ufld'
  self.n.lane_bias=.01;self.n.lane_stamp=Stamp(9.)
  self.assertEqual(self.heading(),(None,None))

class ContinuousHeading(unittest.TestCase):
 def setUp(self):
  Heading.setUp(self)
  import rospy
  self.Time=rospy.Time.from_sec
  self.n.continuous_m8=True;self.n.tunnel_heading_mode='ufld';self.n.lane_bias=math.radians(1.);self.n.lane_stamp=self.Time(10.)
  self.n.lane_applied_bias=0.;self.n.lane_apply_stamp=None
  self.n.imu_buffer=deque([(self.Time(9.95),.05),(self.Time(10.),.05),(self.Time(10.05),.05)])
 def call(self,t):return namespace['_tunnel_heading_ref'](self.n,self.Time(t),10,10)
 def test_expiration_releases_bias_gradually(self):
  self.call(10.);self.call(10.05);old=self.n.lane_applied_bias
  self.n.imu_buffer.append((self.Time(10.4),.25));src,yaw=self.call(10.4)
  self.assertLessEqual(abs(self.n.lane_applied_bias-old),math.radians(.4)+1e-9)
  self.assertLess(abs(self.n.lane_applied_bias),old)
 def test_normal_gps_does_not_instantly_drop_bias(self):
  self.n.lane_applied_bias=math.radians(1.);self.n.lane_apply_stamp=self.Time(10.)
  self.n.gps_integrity.mode=GpsIntegrityMonitor.NORMAL
  src,yaw=self.call(10.05);self.assertEqual(src,'ufld');self.assertAlmostEqual(self.n.lane_applied_bias,math.radians(.8))
 def test_repeated_timestamp_cannot_accumulate_correction(self):
  self.call(10.);old=self.n.lane_applied_bias
  for _ in range(20):self.call(10.)
  self.assertEqual(old,self.n.lane_applied_bias)
 def test_unknown_sensor_time_cannot_supply_heading(self):
  self.n.lane_applied_bias=.01
  self.assertEqual(self.call(11.),(None,None))

class XicpOnlyHeading(unittest.TestCase):
 def setUp(self):
  Heading.setUp(self);import rospy
  self.Time=rospy.Time.from_sec;self.n.tunnel_heading_mode='xicp'
  self.n.xicp_bias=math.radians(1.);self.n.xicp_yaw_stamp=self.Time(10.)
  self.n.xicp_applied_bias=0.;self.n.xicp_apply_stamp=None
  self.n.imu_buffer=deque([(self.Time(9.95),.05),(self.Time(10.),.05),(self.Time(10.05),.25)])
 def call(self,t):return namespace['_tunnel_heading_ref'](self.n,self.Time(t),10.,10.)
 def test_bias_correction_keeps_real_avoidance_turn_and_is_rate_limited(self):
  self.call(10.);src,yaw=self.call(10.05)
  self.assertEqual(src,'xicp');self.assertAlmostEqual(yaw,.25-math.radians(.4))
 def test_stale_result_keeps_corrected_dr_without_wall_or_camera_fallback(self):
  self.call(10.);old=self.n.xicp_applied_bias
  self.n.tunnel_wall_yaw_enabled=True;self.n.wall_yaw_error=.5;self.n.wall_yaw_stamp=self.Time(10.4)
  self.n.imu_buffer.append((self.Time(10.4),.3));src,yaw=self.call(10.4)
  self.assertEqual(src,'xicp_dr');self.assertEqual(self.n.xicp_applied_bias,old);self.assertAlmostEqual(yaw,.3-old)
 def test_missing_registration_does_not_use_wall_fallback(self):
  self.n.xicp_yaw_stamp=None;self.n.xicp_bias=None
  self.assertEqual(self.call(10.),(None,None))
 def test_repeated_timestamp_does_not_apply_bias_twice(self):
  self.call(10.);old=self.n.xicp_applied_bias
  for _ in range(10):self.call(10.)
  self.assertEqual(self.n.xicp_applied_bias,old)
 def test_other_mission_cannot_reuse_held_xicp_bias(self):
  self.n.xicp_applied_bias=.01;self.n.tunnel_m8_active=False
  self.assertEqual(self.call(10.),(None,None))
 def test_stale_mission_state_cannot_enable_heading(self):
  self.n.tunnel_m8_active=True;self.n.tunnel_section_stamp=self.Time(9.)
  self.assertEqual(self.call(10.),(None,None))
