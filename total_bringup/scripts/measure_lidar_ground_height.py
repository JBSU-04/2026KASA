#!/usr/bin/env python3
"""/groundgrid/ground 평면 피팅으로 라이다 장착고(지면 기준)를 실측한다.

방법 (2026-09-14 측정과 동일하게 유지 — 비교 가능해야 한다)
  1. /groundgrid/ground 의 점을 반경 [r_min, r_max] 로 자른다.
     너무 가까우면 자차 차체·타이어 반사가, 너무 멀면 노면 굴곡과
     빔 발산이 섞인다.
  2. 프레임마다 평면 z = a*x + b*y + c 를 최소자승 피팅한다.
  3. 절편 c 가 센서 원점 바로 아래의 노면 높이다(부호 반전 = 장착고).
  4. 프레임별 값의 중앙값을 최종값으로 쓴다(평균은 이상치에 약하다).

사용법
  python3 measure_lidar_ground_height.py <heavy.bag> [--topic /groundgrid/ground]
                                         [--rmin 3.0] [--rmax 12.0] [--max-frames 200]
"""
import argparse
import sys

import numpy as np

try:
    import rosbag
    import sensor_msgs.point_cloud2 as pc2
except ImportError as exc:  # pragma: no cover
    sys.exit('ROS 환경을 source 한 뒤 실행하세요: %s' % exc)


def fit_plane_intercept(pts):
    """z = a*x + b*y + c 피팅 후 c 반환. 실패하면 None."""
    if pts.shape[0] < 50:
        return None
    A = np.c_[pts[:, 0], pts[:, 1], np.ones(pts.shape[0])]
    try:
        coef, *_ = np.linalg.lstsq(A, pts[:, 2], rcond=None)
    except np.linalg.LinAlgError:
        return None
    return float(coef[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/groundgrid/ground')
    ap.add_argument('--rmin', type=float, default=3.0)
    ap.add_argument('--rmax', type=float, default=12.0)
    ap.add_argument('--max-frames', type=int, default=200)
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    if args.topic not in bag.get_type_and_topic_info().topics:
        sys.exit('%s 가 백에 없습니다. record_heavy 로 녹화했는지 확인하세요.'
                 % args.topic)

    intercepts = []
    for _, msg, _ in bag.read_messages(args.topic):
        if len(intercepts) >= args.max_frames:
            break
        pts = np.array([p[:3] for p in pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)])
        if pts.size == 0:
            continue
        r = np.hypot(pts[:, 0], pts[:, 1])
        sel = pts[(r >= args.rmin) & (r <= args.rmax)]
        c = fit_plane_intercept(sel)
        if c is not None:
            intercepts.append(c)
    bag.close()

    if not intercepts:
        sys.exit('유효한 프레임이 없습니다. rmin/rmax 를 조정해 보세요.')

    a = np.array(intercepts)
    height = -np.median(a)   # 절편은 음수(센서 아래) -> 장착고는 부호 반전
    print('프레임 %d개, 반경 %.1f~%.1f m' % (len(a), args.rmin, args.rmax))
    print('  평면 절편  중앙값 %.4f   평균 %.4f   표준편차 %.4f'
          % (np.median(a), a.mean(), a.std()))
    print()
    print('  => z_from_ground = %.4f m' % height)
    print()
    print('lidar_extrinsic.yaml 에 반영:')
    print('  z_from_ground: %.3f' % height)


if __name__ == '__main__':
    main()
