#!/usr/bin/env python3
"""2단계: GroundGrid 장애물 문턱 스윕.

minimum_point_height_obstacle_threshold 만 바꾸며 두 지표를 동시에 본다.

  ① 노면 누출 : 지면이 /nonground 로 올라오는 양
                (전방 경로대에서 ground_relative 가 낮은 점의 수)
  ② 실물 보존 : 실제 장애물의 점이 남는가
                (ground_relative 가 큰 점의 수 + 최대 검출거리)

두 지표는 반대로 움직이므로, 실물을 잃지 않는 가장 높은 문턱이 답이다.

사용: rosrun total_bringup tune_stage2_groundgrid.py 0.05 0.07 0.09 0.11
"""
import subprocess
import sys
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

NODE = '/groundgrid'
KEY = 'minimum_point_height_obstacle_threshold'


def set_thresh(v):
    subprocess.run(['rosrun', 'dynamic_reconfigure', 'dynparam',
                    'set', NODE, KEY, str(v)],
                   capture_output=True, timeout=15)


class Probe(object):
    def __init__(self):
        self.ng = None
        self.gr = None
        self.vel = []
        rospy.Subscriber('/nonground', PointCloud2, self.cb_ng, queue_size=2)
        rospy.Subscriber('/groundgrid/ground', PointCloud2, self.cb_gr,
                         queue_size=2)
        rospy.Subscriber('/odom', Odometry, self.cb_v, queue_size=50)

    def cb_v(self, m):
        self.vel.append(m.twist.twist.linear.x)

    @staticmethod
    def _read(m):
        names = [f.name for f in m.fields]
        want = ('x', 'y', 'z') + (('ground_relative',)
                                  if 'ground_relative' in names else ())
        pts = list(pc2.read_points(m, field_names=want, skip_nans=True))
        return np.array(pts, dtype=float) if pts else None

    def cb_ng(self, m):
        self.ng = self._read(m)

    def cb_gr(self, m):
        self.gr = self._read(m)

    def sample(self, sec):
        """전방 경로대 통계를 sec 동안 모아 평균."""
        rows = []
        t0 = time.time()
        while time.time() - t0 < sec and not rospy.is_shutdown():
            time.sleep(0.3)
            if self.ng is None:
                continue
            P = self.ng
            fwd = (np.abs(P[:, 1]) < 3.0) & (P[:, 0] > 2.0) & (P[:, 0] < 30.0)
            s = P[fwd]
            if len(s) == 0:
                rows.append((0, 0, 0, 0.0))
                continue
            # ground_relative 가 있으면 그걸, 없으면 z 로 대체
            h = s[:, 3] if s.shape[1] > 3 else s[:, 2] + 1.696
            low = int(((h > 0.0) & (h < 0.20)).sum())    # 노면 누출 후보
            obj = int((h >= 0.30).sum())                 # 실물 후보
            far = float(np.hypot(s[h >= 0.30, 0],
                                 s[h >= 0.30, 1]).max()) if obj else 0.0
            rows.append((len(s), low, obj, far))
        if not rows:
            return None
        A = np.array(rows, dtype=float)
        return A[:, 0].mean(), A[:, 1].mean(), A[:, 2].mean(), A[:, 3].max()


def main():
    vals = [float(v) for v in sys.argv[1:]] or [0.05, 0.07, 0.09, 0.11]
    rospy.init_node('tune_stage2', anonymous=True, disable_signals=True)
    p = Probe()
    time.sleep(2.0)
    print('%s 스윕 — 장애물이 보이는 자리에 정지해 두세요\n' % KEY)
    print('  문턱   전방점  노면누출(<0.20)  실물(>=0.30)  최대실물거리')
    out = []
    for v in vals:
        set_thresh(v)
        time.sleep(2.0)
        r = p.sample(5.0)
        if r is None:
            print('  %.2f   (수신 실패)' % v)
            continue
        out.append((v,) + r)
        print('  %.2f   %6.0f      %6.0f          %6.0f        %5.1f m'
              % (v, r[0], r[1], r[2], r[3]))
    if p.vel:
        print('\n측정 중 속도 %.1f ~ %.1f m/s (정지여야 공정)'
              % (min(p.vel), max(p.vel)))
    if len(out) >= 2:
        print('\n판정: 실물 점 수가 떨어지기 시작하는 직전 값이 상한이다.')
        base = out[0][3]
        for v, _t, low, obj, far in out:
            keep = 100.0 * obj / max(base, 1e-6)
            print('  %.2f -> 실물 보존 %3.0f%%  노면누출 %5.0f' % (v, keep, low))
    print('\n확정값은 config/kcity_vlp16.yaml 에 저장할 것.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
