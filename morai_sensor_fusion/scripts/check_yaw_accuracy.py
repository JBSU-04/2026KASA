#!/usr/bin/env python3
"""EKF yaw 정확도를 MORAI ground truth(heading)와 비교한다.

왜 필요한가.
  음영구간 cte 누적의 원인을 찾을 때, 오차가 센서인지 EKF 인지부터
  갈라야 한다. /Competition_topic 의 heading 이 차량의 진짜 자세이므로
  이것을 기준으로 삼으면 세 가지를 한 번에 분리할 수 있다.
    원시 IMU yaw  - GT : IMU 자체 정확도
    원시 GPS 변위 - GT : GPS 궤적 정확도
    /odom  yaw    - GT : EKF 출력 정확도
  실측(2026-09-17 16:11 직선 백)에서는 앞의 둘이 0.01 deg 이내인데
  /odom 만 -0.79 deg 로 상시 치우쳐 있었다 - EKF 튜닝 문제였다.

사용법
  python3 check_yaw_accuracy.py <bag> [--skip 2.0]
  --skip : 초기 수렴 구간을 제외할 시간(초). 기본 2.0
"""
import argparse
import bisect
import math
import sys

import numpy as np

try:
    import rosbag
except ImportError as exc:  # pragma: no cover
    sys.exit('ROS 환경을 source 한 뒤 실행하세요: %s' % exc)


def quat_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff_deg(a, b):
    return math.degrees(math.atan2(math.sin(a - b), math.cos(a - b)))


def summarize(name, values):
    if not values:
        print('%-28s (샘플 없음)' % name)
        return
    a = np.array(values)
    print('%-28s 중앙 %+8.4f  평균 %+8.4f  std %7.4f  범위 %+.2f~%+.2f  n=%d'
          % (name, np.median(a), a.mean(), a.std(), a.min(), a.max(), len(a)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--skip', type=float, default=2.0)
    ap.add_argument('--min-speed', type=float, default=3.0)
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    t0 = bag.get_start_time()

    gt = [(t.to_sec() - t0, math.radians(m.heading), m.wheel_angle)
          for _, m, t in bag.read_messages('/Competition_topic')]
    if not gt:
        sys.exit('/Competition_topic 이 없습니다 (ground truth 필요).')
    gk = [g[0] for g in gt]

    def gt_at(s):
        i = min(bisect.bisect(gk, s), len(gt) - 1)
        return gt[i]

    imu_err, odom_err = [], []
    for _, m, t in bag.read_messages('/imu'):
        s = t.to_sec() - t0
        if s < args.skip:
            continue
        imu_err.append(ang_diff_deg(quat_yaw(m.orientation), gt_at(s)[1]))

    pts = []
    for _, m, t in bag.read_messages('/odom'):
        s = t.to_sec() - t0
        p = m.pose.pose
        v = m.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        if s >= args.skip:
            odom_err.append(ang_diff_deg(quat_yaw(p.orientation), gt_at(s)[1]))
        pts.append((s, p.position.x, p.position.y, speed))

    course_err = []
    span = 15
    for i in range(span, len(pts) - span):
        s = pts[i][0]
        if s < args.skip or pts[i][3] < args.min_speed:
            continue
        x0, y0 = pts[i - span][1], pts[i - span][2]
        x1, y1 = pts[i + span][1], pts[i + span][2]
        if math.hypot(x1 - x0, y1 - y0) < 2.0:
            continue
        course_err.append(ang_diff_deg(math.atan2(y1 - y0, x1 - x0), gt_at(s)[1]))
    bag.close()

    print('bag: %s   (초기 %.1f s 제외)' % (args.bag, args.skip))
    print()
    summarize('IMU yaw  - GT', imu_err)
    summarize('/odom yaw - GT', odom_err)
    summarize('/odom course - GT', course_err)
    print()
    if odom_err:
        bias = float(np.median(odom_err))
        print('/odom yaw 중앙 편차 %.3f deg 이면,' % bias)
        for d in (100, 120, 150):
            print('   %3d m 주행 시 누적 횡오차 %.2f m'
                  % (d, d * math.tan(math.radians(abs(bias)))))


if __name__ == '__main__':
    main()
