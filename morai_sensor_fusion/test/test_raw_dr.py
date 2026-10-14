"""2026-10-14 보정 전 순수 DR: 차속 + 원래 IMU yaw 만 적분, 측위 보정(X-ICP 횡, GPS 재획득)에 영향 없음."""
import ast,math,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from morai_sensor_fusion.measurement import interpolate_angle
src=ast.parse((Path(__file__).parents[1]/'scripts/morai_measurement_adapter.py').read_text())
fns=[v for c in src.body if isinstance(c,ast.ClassDef) for v in c.body if isinstance(v,ast.FunctionDef) and v.name=='_integrate_raw_dr']
ns=dict(math=math,interpolate_angle=interpolate_angle)
exec(compile(ast.Module(body=fns,type_ignores=[]),'raw_dr','exec'),ns)
step=ns['_integrate_raw_dr']


class RawDR(unittest.TestCase):
    def setUp(self):
        self.n=NS(dr_raw=None,dr_raw_stamp=None,dr_raw_yaw=None,dr_position=(100.,200.),dr_max_step_dt=.05)

    def test_starts_at_dr_then_ignores_corrections(self):
        self.assertEqual(step(self.n,0.,0.,5.),(100.,200.,0.))
        for k in range(1,101):
            self.n.dr_position=(self.n.dr_position[0]+.05,self.n.dr_position[1]+.002)  # 보정된 DR 은 옆으로 밀려도
            out=step(self.n,k*.01,0.,5.)
        self.assertAlmostEqual(out[0],105.,places=6);self.assertAlmostEqual(out[1],200.,places=9)

    def test_waits_for_initial_position_and_skips_old_stamps(self):
        self.n.dr_position=None;self.assertIsNone(step(self.n,0.,0.,5.))
        self.n.dr_position=(0.,0.);step(self.n,1.,0.,5.);self.assertIsNone(step(self.n,.99,0.,5.))

    def test_follows_raw_imu_heading_and_reverse(self):
        step(self.n,0.,math.pi/2,2.);out=step(self.n,.05,math.pi/2,-2.)
        self.assertAlmostEqual(out[0],100.,places=6);self.assertAlmostEqual(out[1],199.9,places=6)


if __name__=='__main__':unittest.main()
