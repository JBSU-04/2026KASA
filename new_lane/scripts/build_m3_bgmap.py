#!/usr/bin/env python3
"""동적 구간(M3) 배경맵 생성 (2026-10-06).

여러 주행 백의 /obstacle_info 에서 M3(0.csv idx 3700~4900) 정지 물체를 경로 기준 좌표
(idx, 횡거리)로 모아, 도로 방향 along_m(3 m) x 횡 lat_m(0.5 m) 칸 단위로 min_runs 주행 이상
정지 물체가 있던 칸을 배경으로 남긴다. 연석 기둥처럼 등간격으로 서 있는 구조물은 주행마다
잡히는 기둥이 0~3 m 씩 달라서, 세계 좌표 0.5 m 격자로는 같은 자리로 묶이지 않았다.
2 m 넘게 실제로 움직인 물체(횡단 등)가 지나간 칸은 뺀다(트랙 튐 ~1.5 m 는 이동으로 보지 않음).
obstacle_stop_node 의 위험 기반 속도가 이 칸의 물체를 '출발 가능한 길가 물체' 후보에서 뺀다
(충돌/정지 판단에는 영향 없음).

사용: build_m3_bgmap.py out.yaml bag1 bag2 ... [--min-runs 3]
출력: m3_bgmap_cells: [[idx_lo, idx_hi, lat_lo, lat_hi], ...]
"""
import argparse
import csv
import json
import math
import os

import numpy as np
import rosbag
import yaml
from scipy.spatial import cKDTree


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--min-runs', type=int, default=3)
    ap.add_argument('--along-m', type=float, default=3.0)
    ap.add_argument('--lat-m', type=float, default=0.5)
    ap.add_argument('--move-m', type=float, default=2.0)
    ap.add_argument('--idx-min', type=int, default=3700)
    ap.add_argument('--idx-max', type=int, default=4900)
    ap.add_argument('--csv', default=os.path.expanduser('~/catkin_ws/src/waypoint_system/data/default/0.csv'))
    a = ap.parse_args()
    P = np.array([(float(r[1]), float(r[2])) for r in csv.reader(open(a.csv)) if r and r[0][0].isdigit()])
    tree = cKDTree(P)

    def frenet(x, y):
        _, i = tree.query((x, y))
        j, k = min(i + 1, len(P) - 1), max(i - 1, 0)
        tx, ty = P[j] - P[k]
        n = math.hypot(tx, ty) or 1.0
        return int(i), (-(ty / n)) * (x - P[i, 0]) + (tx / n) * (y - P[i, 1])

    step = int(round(a.along_m * 10))       # 0.csv idx 간격 0.1 m
    static_cells, moving_cells = {}, set()
    for run, bag in enumerate(a.bags):
        tracks = {}
        for _, m, _ in rosbag.Bag(bag).read_messages(topics=['/obstacle_info']):
            d = json.loads(m.data)
            for o in d.get('collision_objects', d.get('obstacles', [])):
                pw = o.get('observed_position_world', o.get('position_world'))
                if not pw:
                    continue
                tracks.setdefault(int(o['id']), []).append((float(pw[0]), float(pw[1])))
        for pts in tracks.values():
            pts = np.array(pts)
            spread = np.max(np.hypot(pts[:, 0] - pts[0, 0], pts[:, 1] - pts[0, 1])) if len(pts) > 1 else 0.0
            if spread > a.move_m:
                for x, y in pts:
                    i, l = frenet(x, y)
                    if a.idx_min <= i <= a.idx_max:
                        moving_cells.add((i // step, int(math.floor(l / a.lat_m))))
                continue
            if len(pts) < 5:
                continue
            i, l = frenet(float(np.median(pts[:, 0])), float(np.median(pts[:, 1])))
            if a.idx_min <= i <= a.idx_max:
                static_cells.setdefault((i // step, int(math.floor(l / a.lat_m))), set()).add(run)
    out = []
    for (ci, cl), runs in sorted(static_cells.items()):
        near = set()
        for di in (-1, 0, 1):
            for dl in (-1, 0, 1):
                near |= static_cells.get((ci + di, cl + dl), set())
        # 실제로 움직인 물체가 지나간 칸 + 주변 한 칸(출발 전 서 있던 자리가 다른 트랙으로 잡힌 경우 대비)은 빼낸다.
        if len(near) < a.min_runs or any((ci + di, cl + dl) in moving_cells
                                         for di in (-1, 0, 1) for dl in (-2, -1, 0, 1, 2)):
            continue
        out.append([ci * step, (ci + 1) * step - 1, round(cl * a.lat_m, 2), round((cl + 1) * a.lat_m, 2)])
    yaml.safe_dump({'m3_bgmap_cells': out, 'm3_bgmap_sources': [os.path.basename(b) for b in a.bags]},
                   open(a.out, 'w'), default_flow_style=None)
    print('static cells %d, moving cells %d -> background cells %d' % (len(static_cells), len(moving_cells), len(out)))


if __name__ == '__main__':
    main()
