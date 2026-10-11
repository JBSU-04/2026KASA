#!/usr/bin/env python3
"""0단계: IMU/LiDAR 정렬 보정 실측.

두 가지를 분리해서 잰다.

  A. 정지 바이어스  - 평지 정지 상태에서 IMU 가 읽는 roll/pitch 영점.
                     그대로 roll_bias_deg / pitch_bias_deg 에 넣으면 제거된다
                     (GravityAlignNode 는 q_bias_ 를 conjugate 로 적용).
  B. 응답 방향      - 급가감속 때 보정된 점군의 노면 기울기가 '줄어드는가'.
                     늘어나면 bias 가 아니라 imu_to_lidar_* 또는 IMU 축 문제다.
                     그때는 GroundGrid 문턱으로 보상하면 안 된다.

사용:
  rosrun total_bringup tune_stage0_imu.py            # 정지 바이어스 (A)
  rosrun total_bringup tune_stage0_imu.py --dynamic  # 급가감속 응답 (B)
"""
import math
import sys
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2

RAW = '/velodyne_points'
ALIGNED = '/perception/points_gravity'


def quat_rp(q):
    sr = 2.0 * (q.w * q.x + q.y * q.z)
    cr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    return math.degrees(math.atan2(sr, cr)), math.degrees(math.asin(sp))


def plane_slope_deg(P, y_half=3.0, r_lo=3.0, r_hi=20.0):
    """전방 노면 점의 '거리 대비 높이' 기울기(deg). 0 이면 수평."""
    if P is None or len(P) < 60:
        return None
    s = P[(np.abs(P[:, 1]) < y_half) & (P[:, 0] > r_lo) & (P[:, 0] < r_hi)]
    if len(s) < 50:
        return None
    # 물체를 배제하고 노면만 본다. 전역 최저 10% 를 고르면 한 거리 구간에
    # 몰려서 회귀가 불가능해지므로(실측: ptp<1m), '거리 빈별 최저점'을 쓴다.
    d = np.hypot(s[:, 0], s[:, 1])
    edges = np.arange(r_lo, r_hi + 0.5, 1.0)
    xs, zs = [], []
    for i in range(len(edges) - 1):
        m = (d >= edges[i]) & (d < edges[i + 1])
        if m.sum() < 3:
            continue
        g = s[m]
        # 그 빈에서 가장 낮은 몇 점의 중앙값 = 그 거리의 지면 높이
        k = max(1, len(g) // 5)
        low = np.sort(g[:, 2])[:k]
        xs.append(0.5 * (edges[i] + edges[i + 1]))
        zs.append(float(np.median(low)))
    if len(xs) < 5:
        return None
    return math.degrees(math.atan(np.polyfit(xs, zs, 1)[0]))


class Stage0(object):
    def __init__(self, dynamic):
        self.dynamic = dynamic
        self.imu = []
        self.raw = None
        self.aligned = None
        self.vel = []
        rospy.Subscriber('/imu', Imu, self.cb_imu, queue_size=200)
        rospy.Subscriber(RAW, PointCloud2, self.mk('raw'), queue_size=2)
        rospy.Subscriber(ALIGNED, PointCloud2, self.mk('aligned'), queue_size=2)
        rospy.Subscriber('/odom', Odometry, self.cb_odom, queue_size=50)

    def cb_imu(self, m):
        self.imu.append(quat_rp(m.orientation))

    def cb_odom(self, m):
        self.vel.append(m.twist.twist.linear.x)

    def mk(self, key):
        def f(m):
            pts = list(pc2.read_points(m, field_names=('x', 'y', 'z'),
                                       skip_nans=True))
            if pts:
                setattr(self, key, np.array(pts, dtype=float))
        return f

    def run(self, sec):
        t0 = time.time()
        rows = []
        while time.time() - t0 < sec and not rospy.is_shutdown():
            time.sleep(0.25)
            sr = plane_slope_deg(self.raw)
            sa = plane_slope_deg(self.aligned)
            v = self.vel[-1] if self.vel else 0.0
            if sr is not None and sa is not None:
                rows.append((v, sr, sa))
        return rows


def main():
    dynamic = '--dynamic' in sys.argv
    rospy.init_node('tune_stage0_imu', anonymous=True, disable_signals=True)
    node = Stage0(dynamic)
    if dynamic:
        print('>>> 급가속/급감속을 반복하세요 (25초) <<<')
    else:
        print('>>> 평지에 정지한 상태로 두세요 (15초) <<<')
    time.sleep(2.0)
    rows = node.run(25.0 if dynamic else 15.0)

    if not node.imu:
        print('IMU 수신 없음'); return
    if not rows:
        print('점군 비교 실패 - %s 가 나오는지 확인' % ALIGNED); return

    R = np.array([r for r, _ in node.imu])
    P = np.array([p for _, p in node.imu])
    A = np.array(rows)

    print('\n=== IMU 원시 ===')
    print('  roll  %+.3f ~ %+.3f  평균 %+.3f' % (R.min(), R.max(), R.mean()))
    print('  pitch %+.3f ~ %+.3f  평균 %+.3f' % (P.min(), P.max(), P.mean()))
    print('\n=== 노면 기울기 (deg, 0 이 수평) ===')
    print('  raw     %+.3f ~ %+.3f  표준편차 %.3f'
          % (A[:, 1].min(), A[:, 1].max(), A[:, 1].std()))
    print('  aligned %+.3f ~ %+.3f  표준편차 %.3f'
          % (A[:, 2].min(), A[:, 2].max(), A[:, 2].std()))
    print('  속도 %.1f ~ %.1f m/s' % (A[:, 0].min(), A[:, 0].max()))

    if not dynamic:
        print('\n=== A. 정지 바이어스 권장값 ===')
        print('  roll_bias_deg:  %+.3f' % R.mean())
        print('  pitch_bias_deg: %+.3f' % P.mean())
        print('  (GravityAlignNode 가 conjugate 로 적용 - 부호 그대로)')
        print('\n  aligned 노면 기울기 평균 %+.3f deg' % A[:, 2].mean())
        print('  -> |값| < 0.2 이면 영점 OK. 크면 위 bias 를 넣고 재측정.')
    else:
        red = 100.0 * (1.0 - A[:, 2].std() / max(A[:, 1].std(), 1e-6))
        print('\n=== B. 응답 방향 판정 ===')
        print('  변동 억제율 %+.0f%%  (raw 표준편차 -> aligned)' % red)
        if red > 40:
            print('  => 보정 정상. 2단계(GroundGrid 문턱)로 넘어가도 된다.')
        elif red > 0:
            print('  => 부분 보정. imu_to_lidar_pitch_deg 를 ±1 deg 씩 훑어볼 것.')
        else:
            print('  => ★ 악화. bias 가 아니라 축/부호 문제다.')
            print('     imu_to_lidar_roll/pitch_deg 를 먼저 맞출 것.')
            print('     GroundGrid 문턱으로 보상하지 말 것.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
