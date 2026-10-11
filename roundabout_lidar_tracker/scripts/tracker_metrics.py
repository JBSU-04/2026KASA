#!/usr/bin/env python3
"""회전교차로 트래커 출력 평가 (2026-10-06).

정답(시뮬레이터 차량 위치)이 없으므로 트랙 자체의 궤적으로 나눈다.
  실제 차량: 링 위 이동 범위 >= real_m (기본 8 m)
  정지 물체: 이동 범위 < static_m (기본 2 m), 관측 10 프레임 이상
지표:
  false_vehicle  정지 물체인데 has_moved 가 한 번이라도 켜진 트랙 수 (진입 판단/앞차를 오염)
  static_out     has_moved 꺼진 채 출력된 정지 트랙 수 (플래너 '움직임 미확인' 보류 원인)
  real           실제 차량 트랙 수, has_moved 켜지기까지 지연(첫 관측부터) p50/p90/max
사용: tracker_metrics.py OUT.bag [topic]
"""
import math
import sys

import numpy as np
import rosbag


def analyze(path, topic='/replay/tracked_objects', static_m=2.0, real_m=8.0):
    tracks = {}
    t0 = None
    for _, m, ts in rosbag.Bag(path).read_messages(topics=[topic]):
        t = ts.to_sec()
        t0 = t if t0 is None else t0
        for o in m.objects:
            tracks.setdefault(o.id, []).append((t - t0, o.position.x, o.position.y, bool(o.has_moved)))
    out = {'false_vehicle': [], 'static_out': 0, 'real': 0, 'delay': [], 'never_moved_real': 0}
    for i, L in tracks.items():
        xy = np.array([(a[1], a[2]) for a in L])
        spread = float(np.max(np.hypot(*(xy - xy[0]).T))) if len(L) > 1 else 0.0
        moved = [a for a in L if a[3]]
        if spread < static_m and len(L) >= 10:
            if moved:
                out['false_vehicle'].append((i, round(L[0][0], 1), round(spread, 2)))
            else:
                out['static_out'] += 1
        elif spread >= real_m:
            out['real'] += 1
            if moved:
                out['delay'].append(moved[0][0] - L[0][0])
            else:
                out['never_moved_real'] += 1
    return out


def main():
    path = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else '/replay/tracked_objects'
    r = analyze(path, topic)
    d = np.array(r['delay']) if r['delay'] else np.array([float('nan')])
    print('%s false_vehicle %d static_out %d real %d (never_moved %d) delay p50 %.2f p90 %.2f max %.2f  %s' % (
        path.split('/')[-1], len(r['false_vehicle']), r['static_out'], r['real'], r['never_moved_real'],
        np.nanpercentile(d, 50), np.nanpercentile(d, 90), np.nanmax(d), r['false_vehicle'][:6]))


if __name__ == '__main__':
    main()
